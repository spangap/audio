/**
 * audio — generic I2S audio engine + ITS packet server. See audio.h.
 *
 * One FreeRTOS task ("audio") owns the ITS server AND both I2S channels, so
 * ITS handles are never sent cross-task and capture/playback stay phase-locked
 * to one block cadence. The task name "audio" is what makes the DataChannel
 * label "audio:1" route here.
 *
 *   RX (mic)     active when client count > 0   → encode + fan out to clients
 *   TX (speaker) active when playback pending   → mix inbound + WAV, write out
 *
 * Both directions are gated independently and the engine is torn down (PM lock
 * released) when fully idle, so the device can light-sleep. I2S is brought up
 * lazily on first need — long after every init hook has run — so a board's
 * audioRegisterCodec() ordering relative to audioInit() does not matter.
 */
#include "audio.h"
#include "codec.h"
#include "wav.h"
#include "av_hdr.h"
#include "its.h"
#include "storage.h"
#include "pm.h"
#include "log.h"
#include "cli.h"
#include "compat.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "driver/i2s_std.h"
#include "driver/i2s_pdm.h"
#include <atomic>
#include <string.h>
#include <sys/time.h>

/* ---- Kconfig fallbacks (bools are undefined when set 'n') ---- */
#ifndef CONFIG_AUDIO_OUT_ENABLE
#define CONFIG_AUDIO_OUT_ENABLE 0
#endif
#ifndef CONFIG_AUDIO_IN_ENABLE
#define CONFIG_AUDIO_IN_ENABLE 0
#endif
#ifndef CONFIG_AUDIO_IN_MODE_PDM
#define CONFIG_AUDIO_IN_MODE_PDM 0
#endif

static const bool OUT_ENABLED = CONFIG_AUDIO_OUT_ENABLE;
static const bool IN_ENABLED  = CONFIG_AUDIO_IN_ENABLE;
static const bool IN_PDM      = CONFIG_AUDIO_IN_MODE_PDM;

/* Config-migration guard (seccam pattern: bump → re-storageDefault). */
#define AUDIO_VERSION 1

/* Largest block we ever buffer: 48 kHz × 40 ms. Engine clamps to this. */
#define AUDIO_MAX_SPB 1920
static const size_t MAXMSG = (size_t)AUDIO_MAX_SPB * 2 + sizeof(av_hdr_t);

/* Aux control port (play/stop), distinct from the client server port. */
static constexpr uint16_t AUDIO_CTRL_PORT = 2;
enum { AUDIO_CMD_PLAY = 1, AUDIO_CMD_STOP };

/* ---- Per-client state ---- */

struct client_t {
  int            handle = -1;        /* ITS handle, -1 = free slot */
  audio_codec_t  codec  = AUDIO_PCM16; /* outbound codec for this client */
  codec_state_t  encState;           /* outbound ADPCM continuity */
  audio_codec_t  inCodec = (audio_codec_t)0xff; /* last inbound codec seen */
  codec_state_t  decState;           /* inbound ADPCM continuity */
  int16_t*       inRing = nullptr;    /* PSRAM device-rate PCM16, mixer input */
  size_t         inCap = 0, inHead = 0, inTail = 0, inCount = 0;
};

struct wavslot_t {
  int          id = -1;              /* >= 0 = active */
  uint32_t     played = 0;           /* samples mixed so far (diagnostic) */
  wav_stream_t st;
};

/* ---- Globals (audio-task owned unless noted) ---- */

static client_t   clients[CONFIG_AUDIO_MAX_CLIENTS];
static wavslot_t  wavs[CONFIG_AUDIO_MAX_WAV];

static pm_lock_handle_t pmLock = nullptr;
static bool             pmHeld = false;
static TaskHandle_t     audioTask = nullptr;
static const audio_codec_ops_t* codecOps = nullptr;   /* set by audioRegisterCodec */
static std::atomic<int> wavIdCounter{0};

static uint32_t currentRate = 0;     /* 0 = engine idle / not resolved */
static uint32_t blockMs = 0;
static size_t   spb = 0;             /* samples per block */

static i2s_chan_handle_t rxChan = nullptr, txChan = nullptr;
static bool rxUp = false, txUp = false;
static int64_t s_playStartUs = 0;   /* diagnostic: wall-clock of current playback */

/* Scratch buffers. inBlk/outBlk are the I2S read/write targets and MUST be
 * internal DMA-capable RAM (a concurrent flash op disables the PSRAM cache
 * mid-copy); they are allocated per-direction at bring-up, sized to the live
 * block, to spare the scarce internal pool. The rest are PSRAM (never touch
 * I2S DMA) and live for the task's lifetime. */
static int16_t* inBlk   = nullptr;   /* DRAM: I2S read target  (sized to spb) */
static int16_t* outBlk  = nullptr;   /* DRAM: I2S write source (sized to spb) */
static int32_t* acc     = nullptr;   /* PSRAM: mixer accumulator         */
static int16_t* decTmp  = nullptr;   /* PSRAM: WAV pull scratch          */
static uint8_t* encBuf  = nullptr;   /* PSRAM: [av_hdr][payload] outbound */
static uint8_t* recvBuf = nullptr;   /* PSRAM: one inbound packet         */
static int16_t* decIn   = nullptr;   /* PSRAM: inbound decode scratch     */

/* Aux play request (struct copied through the ITS inbox). */
struct ctrl_msg_t {
  uint8_t      cmd;
  int32_t      id;
  wav_stream_t st;     /* valid for PLAY: fd + read-ahead buffer handed over */
};

/* ---- Slot helpers ---- */

static int activeClients() {
  int n = 0;
  for (auto& c : clients) if (c.handle >= 0) n++;
  return n;
}
static client_t* findClient(int handle) {
  for (auto& c : clients) if (c.handle == handle) return &c;
  return nullptr;
}
static bool anyInbound() {
  for (auto& c : clients) if (c.inCount > 0) return true;
  return false;
}
static bool anyWav() {
  for (auto& w : wavs) if (w.id >= 0) return true;
  return false;
}

static audio_codec_t pickCodec(const void* data, size_t len) {
  audio_codec_t def = audioCodecFromConfigString(
      storageGetStr("s.audio.codec", "ulaw16k").c_str());
  if (!data || !len) return def;
  char tmp[16];
  size_t m = len < sizeof(tmp) - 1 ? len : sizeof(tmp) - 1;
  memcpy(tmp, data, m);
  tmp[m] = '\0';
  if (!strcmp(tmp, "pcm16"))   return AUDIO_PCM16;
  if (!strcmp(tmp, "ulaw8k"))  return AUDIO_ULAW_8K;
  if (!strcmp(tmp, "ulaw16k")) return AUDIO_ULAW_16K;
  if (!strcmp(tmp, "adpcm"))   return AUDIO_ADPCM;
  uint8_t b = ((const uint8_t*)data)[0];      /* binary single-byte selector */
  if (audioCodecValid(b)) return (audio_codec_t)b;
  return def;
}

/* ---- ITS server callbacks (audio task) ---- */

static int onConnect(int handle, const void* data, size_t len) {
  int s = -1;
  for (int i = 0; i < CONFIG_AUDIO_MAX_CLIENTS; i++)
    if (clients[i].handle < 0) { s = i; break; }
  if (s < 0) return -1;                         /* full → reject */

  client_t* c = &clients[s];
  c->handle = handle;
  c->codec = pickCodec(data, len);
  codecStateReset(&c->encState);
  codecStateReset(&c->decState);
  c->inCodec = (audio_codec_t)0xff;

  if (OUT_ENABLED) {
    uint32_t rate = currentRate ? currentRate
                  : (uint32_t)storageGetInt("s.audio.rate", CONFIG_AUDIO_RATE_DEFAULT);
    size_t cap = (size_t)rate * CONFIG_AUDIO_OUT_MIX_MS / 1000;
    if (cap < 1) cap = 1;
    if (!c->inRing) {
      c->inRing = (int16_t*)heap_caps_malloc(cap * 2, MALLOC_CAP_SPIRAM);
      c->inCap = c->inRing ? cap : 0;
    }
  }
  c->inHead = c->inTail = c->inCount = 0;
  storageSet("audio.clients", activeClients());
  dbg("audio: client %d connected, codec=%d\n", s, c->codec);
  return s;
}

static void onDisconnect(int ref) {
  if (ref < 0 || ref >= CONFIG_AUDIO_MAX_CLIENTS) return;
  client_t* c = &clients[ref];
  if (c->inRing) { heap_caps_free(c->inRing); c->inRing = nullptr; c->inCap = 0; }
  c->handle = -1;
  c->inHead = c->inTail = c->inCount = 0;
  storageSet("audio.clients", activeClients());
}

static void ringPush(client_t* c, const int16_t* s, size_t n) {
  for (size_t i = 0; i < n; i++) {
    if (c->inCount >= c->inCap) break;          /* full → drop the rest */
    c->inRing[c->inTail] = s[i];
    c->inTail = (c->inTail + 1) % c->inCap;
    c->inCount++;
  }
}

static void onRecv(int handle, size_t /*avail*/) {
  size_t n = itsRecv(handle, recvBuf, MAXMSG, 0);
  if (!n) return;
  if (!OUT_ENABLED) return;                     /* nothing to play it into */
  client_t* c = findClient(handle);
  if (!c || !c->inRing) return;

  const uint8_t* p = recvBuf;
  size_t len = n;
  audio_codec_t codec = AUDIO_PCM16;            /* default: raw PCM16 */
  if (len >= sizeof(av_hdr_t)) {
    const av_hdr_t* h = (const av_hdr_t*)recvBuf;
    if (h->type == AV_TYPE_AUDIO && audioCodecValid(h->codec)) {
      codec = (audio_codec_t)h->codec;          /* real header → strip it */
      p += sizeof(av_hdr_t);
      len -= sizeof(av_hdr_t);
    }
  }
  if (c->inCodec != codec) { c->inCodec = codec; codecStateReset(&c->decState); }
  size_t ns = audioDecode(codec, &c->decState, p, len, decIn);
  ringPush(c, decIn, ns);
}

/* ---- Aux control: play / stop (from any task) ---- */

static void audioCtrlHandler(TaskHandle_t /*sender*/, const void* data, size_t len) {
  if (len < 1) return;
  const ctrl_msg_t* m = (const ctrl_msg_t*)data;
  if (m->cmd == AUDIO_CMD_PLAY && len >= sizeof(ctrl_msg_t)) {
    for (auto& w : wavs)
      if (w.id < 0) {
        w.st = m->st; w.id = m->id; w.played = 0;
        s_playStartUs = esp_timer_get_time();
        dbg("play start: id=%d fmt=%u bytes=%u\n",
             m->id, (unsigned)m->st.fmt, (unsigned)m->st.dataLeft);
        return;
      }
    wav_stream_t st = m->st;                     /* no free slot → drop */
    wavClose(&st);
    warn("audio: no free WAV slot, dropping play\n");
  } else if (m->cmd == AUDIO_CMD_STOP) {
    for (auto& w : wavs)
      if (w.id >= 0 && (m->id < 0 || w.id == m->id)) { wavClose(&w.st); w.id = -1; }
  }
}

/* ---- Rate / engine bring-up ---- */

static void resolveRate() {
  if (currentRate) return;
  uint32_t r = storageGetInt("s.audio.rate", CONFIG_AUDIO_RATE_DEFAULT);
  if (r < 8000) r = 8000;
  if (r > 48000) r = 48000;
  uint32_t b = storageGetInt("s.audio.block_ms", CONFIG_AUDIO_BLOCK_MS_DEFAULT);
  if (b < 10) b = 10;
  if (b > 40) b = 40;
  currentRate = r;
  blockMs = b;
  spb = (size_t)currentRate * blockMs / 1000;
  if (spb > AUDIO_MAX_SPB) spb = AUDIO_MAX_SPB;
}

static void rxConfigure() {
  if (IN_PDM) {
    i2s_pdm_rx_config_t cfg = {
      .clk_cfg  = I2S_PDM_RX_CLK_DEFAULT_CONFIG(currentRate),
      .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
      .gpio_cfg = {
        .clk = (gpio_num_t)CONFIG_AUDIO_IN_SCK_GPIO,
        .din = (gpio_num_t)CONFIG_AUDIO_IN_DIN_GPIO,
        .invert_flags = { .clk_inv = false },
      },
    };
    i2s_channel_init_pdm_rx_mode(rxChan, &cfg);
  } else {
    i2s_std_config_t cfg = {
      .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(currentRate),
      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
      .gpio_cfg = {
        .mclk = (gpio_num_t)CONFIG_AUDIO_IN_MCLK_GPIO,
        .bclk = (gpio_num_t)CONFIG_AUDIO_IN_SCK_GPIO,
        .ws   = (gpio_num_t)CONFIG_AUDIO_IN_WS_GPIO,
        .dout = I2S_GPIO_UNUSED,
        .din  = (gpio_num_t)CONFIG_AUDIO_IN_DIN_GPIO,
        .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
      },
    };
    cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;   /* one mic channel */
    i2s_channel_init_std_mode(rxChan, &cfg);
  }
}

static void ensureRx(bool want) {
  if (!IN_ENABLED) return;
  if (want && !rxUp) {
    resolveRate();
    if (codecOps && codecOps->in_init) {
      uint32_t r = codecOps->in_init(currentRate);
      if (!r) warn("audio: input codec init failed\n");
      else if (!txUp && r != currentRate) {       /* adopt clamp if rx is first up */
        currentRate = r;
        spb = (size_t)currentRate * blockMs / 1000;
        if (spb > AUDIO_MAX_SPB) spb = AUDIO_MAX_SPB;
      }
    }
    if (!inBlk) inBlk = (int16_t*)heap_caps_malloc(spb * 2, MALLOC_CAP_INTERNAL);
    if (!inBlk) { warn("audio: rx scratch alloc failed\n"); return; }
    i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG((i2s_port_t)CONFIG_AUDIO_IN_I2S_PORT,
                                                      I2S_ROLE_MASTER);
    cc.dma_desc_num  = CONFIG_AUDIO_DMA_DESC_NUM;
    cc.dma_frame_num = CONFIG_AUDIO_DMA_FRAME_NUM;
    if (i2s_new_channel(&cc, nullptr, &rxChan) != ESP_OK) { warn("audio: rx new_channel\n"); return; }
    rxConfigure();
    if (i2s_channel_enable(rxChan) != ESP_OK) {
      i2s_del_channel(rxChan); rxChan = nullptr;
      warn("audio: rx enable failed\n");
      return;
    }
    rxUp = true;
    storageSet("audio.sample_rate", (int)currentRate);
    dbg("rx up: %u Hz, block %u ms (%u samples)\n",
         (unsigned)currentRate, (unsigned)blockMs, (unsigned)spb);
  } else if (!want && rxUp) {
    i2s_channel_disable(rxChan);
    i2s_del_channel(rxChan);
    rxChan = nullptr;
    rxUp = false;
    if (inBlk) { heap_caps_free(inBlk); inBlk = nullptr; }
    if (codecOps && codecOps->in_deinit) codecOps->in_deinit();
  }
}

static void ensureTx(bool want) {
  if (!OUT_ENABLED) return;
  if (want && !txUp) {
    resolveRate();
    if (codecOps && codecOps->out_init) codecOps->out_init(currentRate);
    if (!outBlk) outBlk = (int16_t*)heap_caps_malloc(spb * 2, MALLOC_CAP_INTERNAL);
    if (!outBlk) { warn("audio: tx scratch alloc failed\n"); return; }
    i2s_chan_config_t cc = I2S_CHANNEL_DEFAULT_CONFIG((i2s_port_t)CONFIG_AUDIO_OUT_I2S_PORT,
                                                      I2S_ROLE_MASTER);
    cc.dma_desc_num  = CONFIG_AUDIO_DMA_DESC_NUM;
    cc.dma_frame_num = CONFIG_AUDIO_DMA_FRAME_NUM;
    /* Clear each DMA buffer once the peripheral has sent it. On a true underrun
     * (the audio task starved of CPU, e.g. an LXMF burst) the driver then
     * re-sends zeros instead of replaying the last buffer — the MAX98357A has no
     * FIFO of its own, so without this a stall audibly loops the stale samples.
     * Silence-on-starve, not a stuck loop. */
    cc.auto_clear = true;
    if (i2s_new_channel(&cc, &txChan, nullptr) != ESP_OK) { warn("audio: tx new_channel\n"); return; }
    i2s_std_config_t cfg = {
      .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(currentRate),
      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
      .gpio_cfg = {
        .mclk = I2S_GPIO_UNUSED,                  /* MAX98357A self-clocks */
        .bclk = (gpio_num_t)CONFIG_AUDIO_OUT_BCK_GPIO,
        .ws   = (gpio_num_t)CONFIG_AUDIO_OUT_WS_GPIO,
        .dout = (gpio_num_t)CONFIG_AUDIO_OUT_DOUT_GPIO,
        .din  = I2S_GPIO_UNUSED,
        .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
      },
    };
    if (i2s_channel_init_std_mode(txChan, &cfg) != ESP_OK) {
      i2s_del_channel(txChan); txChan = nullptr;
      warn("audio: tx init failed\n");
      return;
    }
    /* Preload the DMA with silence before enabling so the clock starts on real
     * data instead of an underrun glitch (the initial "click"). */
    memset(outBlk, 0, spb * 2);
    size_t pl;
    do { pl = 0; i2s_channel_preload_data(txChan, outBlk, spb * 2, &pl); } while (pl == spb * 2);
    if (i2s_channel_enable(txChan) != ESP_OK) {
      i2s_del_channel(txChan); txChan = nullptr;
      warn("audio: tx enable failed\n");
      return;
    }
    txUp = true;
    storageSet("audio.sample_rate", (int)currentRate);
    dbg("tx up: %u Hz\n", (unsigned)currentRate);
  } else if (!want && txUp) {
    i2s_channel_disable(txChan);
    i2s_del_channel(txChan);
    txChan = nullptr;
    txUp = false;
    if (outBlk) { heap_caps_free(outBlk); outBlk = nullptr; }
    if (codecOps && codecOps->out_deinit) codecOps->out_deinit();
  }
}

static inline void applyMicGain(int16_t* s, size_t n, int g) {
  if (g == 1) return;
  if (g == 0) { memset(s, 0, n * 2); return; }
  for (size_t i = 0; i < n; i++) {
    int32_t v = (int32_t)s[i] * g;
    if (v > 32767) v = 32767;
    if (v < -32768) v = -32768;
    s[i] = (int16_t)v;
  }
}

/* ---- Task loop ---- */

static void audioTaskFn(void*) {
  itsServerInit(128);
  itsServerPortOpen(AUDIO_STREAM_PORT, ITS_PACKET, CONFIG_AUDIO_MAX_CLIENTS,
                    /*toCap=*/4 * MAXMSG, /*fromCap=*/4 * MAXMSG,
                    /*depth=*/0, /*maxMsg=*/MAXMSG);
  itsServerOnConnect(AUDIO_STREAM_PORT, onConnect);
  itsServerOnRecv(AUDIO_STREAM_PORT, onRecv);
  itsServerOnDisconnect(AUDIO_STREAM_PORT, onDisconnect);
  itsOnAux(AUDIO_CTRL_PORT, audioCtrlHandler);

  /* PSRAM scratch (never touches I2S DMA), task-lifetime. The internal
   * DMA-capable I2S buffers (inBlk/outBlk) are allocated per-direction at
   * bring-up, sized to the live block. */
  acc     = (int32_t*)heap_caps_malloc(AUDIO_MAX_SPB * sizeof(int32_t), MALLOC_CAP_SPIRAM);
  decTmp  = (int16_t*)heap_caps_malloc(AUDIO_MAX_SPB * 2, MALLOC_CAP_SPIRAM);
  encBuf  = (uint8_t*)heap_caps_malloc(MAXMSG, MALLOC_CAP_SPIRAM);
  recvBuf = (uint8_t*)heap_caps_malloc(MAXMSG, MALLOC_CAP_SPIRAM);
  decIn   = (int16_t*)heap_caps_malloc(MAXMSG * 2 * sizeof(int16_t), MALLOC_CAP_SPIRAM);
  if (!acc || !decTmp || !encBuf || !recvBuf || !decIn) {
    err("audio: scratch alloc failed\n");
    killSelf();
  }

  for (;;) {
    bool rxWant = IN_ENABLED && activeClients() > 0;
    bool txWant = OUT_ENABLED && (anyWav() || anyInbound());

    ensureRx(rxWant);
    ensureTx(txWant);

    if (!rxUp && !txUp) {
      if (pmHeld) { pmLockRelease(pmLock); pmHeld = false; }
      currentRate = 0;                 /* re-read config on next session */
      itsPoll(portMAX_DELAY);          /* wakes on connect / play aux */
      continue;
    }
    if (!pmHeld) { pmLockAcquire(pmLock); pmHeld = true; }

    while (itsPoll(0)) {}              /* drain connects / inbound / ctrl */

    int micGain = storageGetInt("s.audio.mic_gain", 1);
    int outGain = storageGetInt("s.audio.out_gain", 256);

    /* RX: read one block, fan out to every client. The read paces the loop. */
    if (rxUp) {
      size_t got = 0;
      i2s_channel_read(rxChan, inBlk, spb * 2, &got, pdMS_TO_TICKS(blockMs * 2 + 10));
      size_t ns = got / 2;
      if (ns) {
        applyMicGain(inBlk, ns, micGain);
        struct timeval tv;
        gettimeofday(&tv, nullptr);
        uint64_t epoch = (uint64_t)tv.tv_sec * 1000ULL + (uint64_t)tv.tv_usec / 1000ULL;
        int16_t utc = utcOffsetMinutes(tv.tv_sec);
        for (auto& c : clients) {
          if (c.handle < 0 || !itsConnected(c.handle)) continue;
          avHdrFill((av_hdr_t*)encBuf, AV_TYPE_AUDIO, (uint8_t)c.codec, epoch, utc);
          size_t pb = audioEncode(c.codec, &c.encState, inBlk, ns, encBuf + sizeof(av_hdr_t));
          itsSend(c.handle, encBuf, sizeof(av_hdr_t) + pb, 0);   /* non-blocking; drop on backpressure */
        }
      }
    }

    /* TX: mix every inbound ring + every active WAV, saturate, write a block. */
    if (txUp) {
      memset(acc, 0, spb * sizeof(int32_t));
      bool haveOut = false;
      for (auto& c : clients) {
        size_t take = c.inCount < spb ? c.inCount : spb;
        for (size_t i = 0; i < take; i++) {
          acc[i] += c.inRing[c.inHead];
          c.inHead = (c.inHead + 1) % c.inCap;
          c.inCount--;
        }
        if (take) haveOut = true;
      }
      for (auto& w : wavs) {
        if (w.id < 0) continue;
        size_t n = wavRead(&w.st, decTmp, spb);
        if (n == 0) {
          dbg("play done: id=%d samples=%u elapsed=%dms\n", w.id, (unsigned)w.played,
               (int)((esp_timer_get_time() - s_playStartUs) / 1000));
          wavClose(&w.st); w.id = -1; continue;
        }
        for (size_t i = 0; i < n; i++) acc[i] += decTmp[i];
        w.played += n;
        haveOut = true;
      }
      if (haveOut) {
        for (size_t i = 0; i < spb; i++) {
          int32_t v = (acc[i] * outGain) >> 8;
          if (v > 32767) v = 32767;
          if (v < -32768) v = -32768;
          outBlk[i] = (int16_t)v;
        }
      } else {
        memset(outBlk, 0, spb * 2);   /* silence keeps the clock from underrunning */
      }
      /* Push the WHOLE block, blocking on DMA space (this is the pacing). A
       * single short-timeout write dropped its unwritten tail under scheduling
       * load, truncating playback — so loop on the remainder with a generous
       * timeout. i2s_channel_write blocks on the DMA-drain ISR, so a busy CPU
       * just makes it wait, it doesn't lose samples. */
      const uint8_t* p = (const uint8_t*)outBlk;
      size_t off = 0, remain = spb * 2;
      while (remain) {
        size_t w = 0;
        esp_err_t werr = i2s_channel_write(txChan, p + off, remain, &w, pdMS_TO_TICKS(1000));
        off += w; remain -= w;
        if (werr != ESP_OK && w == 0) break;   /* clock truly stalled — abort block */
      }
    }
  }
}

/* ---- CLI ---- */

static void audioCliCmd(const char* args) {
  if (cliWantsHelp(args)) {
    cliPrintf("%-*s mic/speaker status, WAV playback\n", CLI_HELP_COL, "audio");
    if (args && (args[0] == '-')) {
      cliPrintf("  audio              status\n");
      cliPrintf("  audio play <path>  play a device-rate WAV out the speaker\n");
      cliPrintf("  audio stop         stop all WAV playback\n");
    }
    return;
  }
  if (!strncmp(args, "play", 4)) {
    const char* path = args + 4;
    while (*path == ' ') path++;
    if (!*path) { cliPrintf("usage: audio play <path>\n"); return; }
    if (audioPlayWav(path) < 0) cliPrintf("audio: play failed (see log)\n");
    return;
  }
  if (!strncmp(args, "stop", 4)) { audioStopWav(-1); return; }

  /* status (also the bare/no-arg case) */
  cliPrintf("rate        %d Hz (resolved %d)\n",
            storageGetInt("s.audio.rate", CONFIG_AUDIO_RATE_DEFAULT),
            storageGetInt("audio.sample_rate", 0));
  cliPrintf("block_ms    %d\n", storageGetInt("s.audio.block_ms", CONFIG_AUDIO_BLOCK_MS_DEFAULT));
  cliPrintf("clients     %d\n", storageGetInt("audio.clients", 0));
  cliPrintf("def codec   %s\n", storageGetStr("s.audio.codec", "ulaw16k").c_str());
  cliPrintf("out_gain    %d (8.8)\n", storageGetInt("s.audio.out_gain", 256));
  cliPrintf("mic_gain    %d\n", storageGetInt("s.audio.mic_gain", 1));
}

/* ---- Public API ---- */

void audioRegisterCodec(const audio_codec_ops_t* ops) {
  codecOps = ops;
}

int audioPlayWav(const char* path) {
  if (!OUT_ENABLED || !audioTask || !path || !path[0]) return -1;
  uint32_t rate = storageGetInt("audio.sample_rate", 0);
  if (!rate) rate = storageGetInt("s.audio.rate", CONFIG_AUDIO_RATE_DEFAULT);
  size_t ahead = (size_t)rate * CONFIG_AUDIO_WAV_BUF_MS / 1000 * 2;

  ctrl_msg_t m = {};
  m.cmd = AUDIO_CMD_PLAY;
  if (!wavOpen(path, rate, ahead, &m.st)) return -1;
  m.id = wavIdCounter.fetch_add(1) + 1;          /* ids start at 1 */
  if (!itsSendAuxByTaskHandle(audioTask, AUDIO_CTRL_PORT, &m, sizeof(m), pdMS_TO_TICKS(500))) {
    wavClose(&m.st);
    return -1;
  }
  return m.id;
}

void audioStopWav(int id) {
  if (!audioTask) return;
  ctrl_msg_t m = {};
  m.cmd = AUDIO_CMD_STOP;
  m.id = id;
  itsSendAuxByTaskHandle(audioTask, AUDIO_CTRL_PORT, &m, sizeof(m), pdMS_TO_TICKS(500));
}

void AudioService::onInit() {
  if (storageGetInt("s.audio.version", 0) < AUDIO_VERSION) {
    storageDefault("s.audio.rate", CONFIG_AUDIO_RATE_DEFAULT);
    storageDefault("s.audio.block_ms", CONFIG_AUDIO_BLOCK_MS_DEFAULT);
    storageDefault("s.audio.codec", "ulaw16k");
    storageDefault("s.audio.out_gain", 256);
    storageDefault("s.audio.mic_gain", 1);
    storageSet("s.audio.version", AUDIO_VERSION);
  }
  pmLockCreate(PM_NO_LIGHT_SLEEP, "audio", &pmLock);
  cliRegisterCmd("audio", audioCliCmd);
  audioTask = spawnTask(audioTaskFn, "audio", 8192, nullptr, 1, 1);
}
