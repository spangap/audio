/**
 * Audio — PDM mic capture with ring buffer and per-subscriber processing.
 *
 * Raw PCM16 samples go into a shared ring buffer. Per subscriber, a sample
 * counter tracks how many new samples are available. When enough accumulate
 * to fill the subscriber's buffer, processes (HPF → gain/AGC → codec),
 * fills the buffer, sends audio_meta_t via ITS aux. Then re-reads config
 * from consumer's audioSettings pointer for the next chunk.
 */
#include "audio.h"
#include "storage.h"
#include "its.h"
#include "pm.h"
#include "log.h"
#include "Seeed_XIAO_ESP32S3_Sense.h"
#include "esp_heap_caps.h"
#include "driver/i2s_pdm.h"
#include "compat.h"
#include <stdlib.h>
#include <string.h>

static pm_lock_handle_t audioPmLock = nullptr;

/* ---- I2S low-level ---- */

static i2s_chan_handle_t rx_chan = NULL;
static bool i2sInited = false;
static uint32_t currentRate = 16000;

static bool i2sInit(uint32_t rate) {
  if (i2sInited) return true;
  if (rate < 4000 || rate > 48000) rate = 16000;
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
  if (i2s_new_channel(&chan_cfg, NULL, &rx_chan) != ESP_OK) return false;
  i2s_pdm_dsr_t dsr = (rate <= 24000) ? I2S_PDM_DSR_16S : I2S_PDM_DSR_8S;
  i2s_pdm_rx_config_t pdm_cfg = {
    .clk_cfg  = I2S_PDM_RX_CLK_DEFAULT_CONFIG(rate),
    .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
    .gpio_cfg = {
      .clk = (gpio_num_t)PDM_CLK_GPIO,
      .din = (gpio_num_t)PDM_DAT_GPIO,
      .invert_flags = { .clk_inv = false },
    },
  };
  pdm_cfg.clk_cfg.dn_sample_mode = dsr;
  if (i2s_channel_init_pdm_rx_mode(rx_chan, &pdm_cfg) != ESP_OK) {
    i2s_del_channel(rx_chan); rx_chan = NULL; return false;
  }
  if (i2s_channel_enable(rx_chan) != ESP_OK) {
    i2s_del_channel(rx_chan); rx_chan = NULL; return false;
  }
  vTaskDelay(pdMS_TO_TICKS(50));
  currentRate = rate;
  i2sInited = true;
  return true;
}

static void i2sFlush() {
  if (!i2sInited) return;
  uint8_t discard[256];
  size_t got = 0;
  TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(500);
  do {
    if (i2s_channel_read(rx_chan, discard, sizeof(discard), &got, pdMS_TO_TICKS(5)) != ESP_OK) break;
  } while (got > 0 && xTaskGetTickCount() < deadline);
}

static void i2sEnd() {
  if (!i2sInited) return;
  i2s_channel_disable(rx_chan);
  i2s_del_channel(rx_chan);
  rx_chan = NULL;
  i2sInited = false;
}

static size_t i2sRead(uint8_t* buf, size_t len) {
  if (!i2sInited || !buf || len < 2) return 0;
  size_t want = len & ~1u;
  size_t got = 0;
  if (i2s_channel_read(rx_chan, buf, want, &got, pdMS_TO_TICKS(100)) != ESP_OK) return 0;
  return got;
}

/* ---- Ring buffer (raw PCM16 samples) ---- */

static int16_t* ringBuf = nullptr;
static size_t   ringCap = 0;
static size_t   ringHead = 0;

static void ringInit(size_t capSamples) {
  if (ringBuf) return;
  ringCap = capSamples;
  ringBuf = (int16_t*)heap_caps_calloc(ringCap, sizeof(int16_t), MALLOC_CAP_SPIRAM);
  ringHead = 0;
}

static void ringFree() {
  if (ringBuf) { heap_caps_free(ringBuf); ringBuf = nullptr; }
  ringCap = 0; ringHead = 0;
}

static void ringWrite(const int16_t* samples, size_t n) {
  for (size_t i = 0; i < n; i++) {
    ringBuf[ringHead] = samples[i];
    ringHead = (ringHead + 1) % ringCap;
  }
}

static void ringRead(size_t pos, int16_t* out, size_t n) {
  for (size_t i = 0; i < n; i++)
    out[i] = ringBuf[(pos + i) % ringCap];
}

/* ---- Subscriber table ---- */

#define AUDIO_MAX_SUBSCRIBERS 3
/* AUDIO_CMD_PORT and AUDIO_NOTIFY_PORT are now declared in audio.h */

enum { AUDIO_CMD_SUBSCRIBE = 1, AUDIO_CMD_UNSUBSCRIBE };

struct audio_proc_t {
  bool hpfSeeded;
  int32_t hpfPrev_x, hpfPrev_y;
  int32_t agcGain;
  int32_t adpcmPredictor;
  int adpcmStepIndex;
};

struct audio_slot_t {
  TaskHandle_t   task;
  audio_meta_t*  settings;  /* pointer to consumer's audioSettings */
  audio_meta_t*  samples;   /* pointer to consumer's audioSamples */
  audio_cb_t     cb;
  audio_meta_t   cfg;       /* internal copy — refreshed at chunk boundaries */
  audio_proc_t   proc;
  size_t         rawNeeded;
  size_t         rawCount;
  size_t         ringPos;
  int16_t        peak;
  uint32_t       firstMs;       /* millis() at first sample of chunk */
  uint64_t       firstEpochMs;  /* wall clock at first sample of chunk */
  int16_t        firstUtcOffset;
};

static audio_slot_t slots[AUDIO_MAX_SUBSCRIBERS] = {};
static TaskHandle_t taskHandle = nullptr;
static volatile bool running = false;

static int activeCount() {
  int n = 0;
  for (int i = 0; i < AUDIO_MAX_SUBSCRIBERS; i++)
    if (slots[i].task) n++;
  return n;
}

static void procReset(audio_proc_t* p) {
  memset(p, 0, sizeof(*p));
  p->agcGain = 256;
}

/* ---- Codec helpers ---- */

size_t audioRawSamplesForBytes(audio_codec_t codec, size_t outBytes) {
  switch (codec) {
    case AUDIO_PCM16:    return outBytes / 2;
    case AUDIO_ULAW_8K:  return outBytes * 2;
    case AUDIO_ULAW_16K: return outBytes;
    case AUDIO_ADPCM:    return outBytes * 2;
  }
  return outBytes / 2;
}

size_t audioBytesForMs(audio_codec_t codec, int ms) {
  uint32_t rate = audioOutputRate(codec);
  uint32_t samples = rate * (uint32_t)ms / 1000;
  switch (codec) {
    case AUDIO_PCM16:    return samples * 2;
    case AUDIO_ULAW_8K:  return samples;
    case AUDIO_ULAW_16K: return samples;
    case AUDIO_ADPCM:    return (samples + 1) / 2;
  }
  return samples * 2;
}

/* ---- u-law encoding ---- */

static uint8_t linearToUlaw(int16_t sample) {
  const int BIAS = 0x84, CLIP = 32635;
  int sign = (sample >> 8) & 0x80;
  if (sign) sample = -sample;
  if (sample > CLIP) sample = CLIP;
  sample += BIAS;
  int exponent = 7;
  for (int expMask = 0x4000; exponent > 0; exponent--, expMask >>= 1)
    if (sample & expMask) break;
  int mantissa = (sample >> (exponent + 3)) & 0x0F;
  return ~(sign | (exponent << 4) | mantissa);
}

/* ---- IMA ADPCM ---- */

static const int16_t imaStepTable[89] = {
  7,8,9,10,11,12,13,14,16,17,19,21,23,25,28,31,34,37,41,45,50,55,60,66,
  73,80,88,97,107,118,130,143,157,173,190,209,230,253,279,307,337,371,408,
  449,494,544,598,658,724,796,876,963,1060,1166,1282,1411,1552,1707,1878,
  2066,2272,2499,2749,3024,3327,3660,4026,4428,4871,5358,5894,6484,7132,
  7845,8630,9493,10442,11487,12635,13899,15289,16818,18500,20350,22385,
  24623,27086,29794,32767
};
static const int8_t imaIndexTable[16] = {
  -1,-1,-1,-1,2,4,6,8,-1,-1,-1,-1,2,4,6,8
};

/* ---- Processing ---- */

static void hpfApply(audio_proc_t* p, int16_t* samples, size_t n) {
  if (!p->hpfSeeded && n > 0) { p->hpfPrev_x = samples[0]; p->hpfSeeded = true; }
  for (size_t i = 0; i < n; i++) {
    int32_t x = samples[i];
    int32_t y = x - p->hpfPrev_x + ((p->hpfPrev_y * 255) >> 8);
    p->hpfPrev_x = x;
    if (y > 32767) y = 32767;
    if (y < -32768) y = -32768;
    p->hpfPrev_y = y;
    samples[i] = (int16_t)y;
  }
}

static void gainApply(audio_proc_t* p, const audio_meta_t* cfg, int16_t* samples, size_t n) {
  if (cfg->agc_max > 0) {
    int16_t peak = 0;
    for (size_t i = 0; i < n; i++) {
      int16_t s = samples[i] < 0 ? -samples[i] : samples[i];
      if (s < 0) s = 32767;
      if (s > peak) peak = s;
    }
    int32_t targetGain = peak > 0
      ? ((int32_t)cfg->agc_target << 8) / peak
      : (int32_t)cfg->agc_max << 8;
    int32_t maxFp = (int32_t)cfg->agc_max << 8;
    if (targetGain > maxFp) targetGain = maxFp;
    if (targetGain < 256) targetGain = 256;
    size_t chunkMs = n * 1000 / currentRate;
    if (chunkMs < 1) chunkMs = 1;
    if (targetGain < p->agcGain) {
      int32_t step = p->agcGain - targetGain;
      int32_t frac = cfg->agc_attack > 0 ? ((int)chunkMs * 256) / cfg->agc_attack : 256;
      if (frac > 256) frac = 256;
      p->agcGain -= (step * frac) >> 8;
    } else {
      int32_t step = targetGain - p->agcGain;
      int32_t frac = cfg->agc_release > 0 ? ((int)chunkMs * 256) / cfg->agc_release : 256;
      if (frac > 256) frac = 256;
      p->agcGain += (step * frac) >> 8;
    }
    for (size_t i = 0; i < n; i++) {
      int32_t v = ((int32_t)samples[i] * p->agcGain) >> 8;
      if (v > 32767) v = 32767;
      if (v < -32768) v = -32768;
      samples[i] = (int16_t)v;
    }
  } else if (cfg->gain == 0) {
    memset(samples, 0, n * 2);
  } else if (cfg->gain > 1) {
    for (size_t i = 0; i < n; i++) {
      int32_t v = (int32_t)samples[i] * cfg->gain;
      if (v > 32767) v = 32767;
      if (v < -32768) v = -32768;
      samples[i] = (int16_t)v;
    }
  }
}

static size_t encodePcm16(const int16_t* in, size_t n, uint8_t* out) {
  memcpy(out, in, n * 2); return n * 2;
}

static size_t encodeUlaw(const int16_t* in, size_t n, uint8_t* out, bool downsample) {
  if (downsample) {
    size_t outN = n / 2;
    for (size_t i = 0; i < outN; i++) {
      int32_t avg = ((int32_t)in[i*2] + (int32_t)in[i*2+1]) / 2;
      out[i] = linearToUlaw((int16_t)avg);
    }
    return outN;
  }
  for (size_t i = 0; i < n; i++) out[i] = linearToUlaw(in[i]);
  return n;
}

static size_t encodeAdpcm(audio_proc_t* p, const int16_t* in, size_t n, uint8_t* out) {
  size_t outBytes = 0;
  uint8_t nibblePair = 0;
  for (size_t i = 0; i < n; i++) {
    int32_t diff = (int32_t)in[i] - p->adpcmPredictor;
    int sign = 0;
    if (diff < 0) { sign = 8; diff = -diff; }
    int step = imaStepTable[p->adpcmStepIndex];
    int code = 0;
    if (diff >= step) { code |= 4; diff -= step; }
    step >>= 1;
    if (diff >= step) { code |= 2; diff -= step; }
    step >>= 1;
    if (diff >= step) { code |= 1; }
    code |= sign;
    step = imaStepTable[p->adpcmStepIndex];
    int32_t delta = ((code & 7) * step / 4 + step / 8);
    if (code & 8) delta = -delta;
    p->adpcmPredictor += delta;
    if (p->adpcmPredictor > 32767) p->adpcmPredictor = 32767;
    if (p->adpcmPredictor < -32768) p->adpcmPredictor = -32768;
    p->adpcmStepIndex += imaIndexTable[code];
    if (p->adpcmStepIndex < 0) p->adpcmStepIndex = 0;
    if (p->adpcmStepIndex > 88) p->adpcmStepIndex = 88;
    if (i & 1) { nibblePair |= (code << 4); out[outBytes++] = nibblePair; }
    else { nibblePair = code & 0x0F; }
  }
  if (n & 1) out[outBytes++] = nibblePair;
  return outBytes;
}

/* ---- Process and deliver ---- */

static void processAndDeliver(audio_slot_t* slot) {
  audio_meta_t* cfg = &slot->cfg;
  audio_proc_t* p = &slot->proc;
  size_t rawN = slot->rawNeeded;

  int16_t* tmp = (int16_t*)alloca(rawN * 2);
  ringRead(slot->ringPos, tmp, rawN);

  /* Peak on raw PCM */
  int16_t hi = tmp[0], lo = tmp[0];
  for (size_t i = 1; i < rawN; i++) {
    if (tmp[i] > hi) hi = tmp[i];
    if (tmp[i] < lo) lo = tmp[i];
  }
  int16_t rawPeak = (int16_t)(hi - lo);
  if (rawPeak > slot->peak) slot->peak = rawPeak;

  /* Level trigger */
  int lvl = storageGetInt("s.detect.audio.level");
  if (lvl > 0 && rawPeak >= lvl)
    storageSet("audio.last_trigger", (int)time(nullptr));

  /* HPF → gain/AGC → codec (skip encoding for stats-only subscribers) */
  if (cfg->hpf) hpfApply(p, tmp, rawN);
  gainApply(p, cfg, tmp, rawN);

  if (cfg->buf && cfg->len > 0) {
    switch (cfg->codec) {
      case AUDIO_PCM16:    encodePcm16(tmp, rawN, cfg->buf); break;
      case AUDIO_ULAW_8K:  encodeUlaw(tmp, rawN, cfg->buf, true); break;
      case AUDIO_ULAW_16K: encodeUlaw(tmp, rawN, cfg->buf, false); break;
      case AUDIO_ADPCM:    encodeAdpcm(p, tmp, rawN, cfg->buf); break;
    }
  }

  /* Build notification */
  audio_meta_t msg = *cfg;
  msg.timestamp = slot->firstMs;
  msg.epoch_ms = slot->firstEpochMs;
  msg.utc_offset_min = slot->firstUtcOffset;
  msg.peak = slot->peak;
  msg.eff_gain = (cfg->agc_max > 0) ? (int16_t)p->agcGain : (int16_t)(cfg->gain * 256);

  itsSendAuxByTaskHandle(slot->task, AUDIO_NOTIFY_PORT, &msg, sizeof(msg), 0);

  /* Advance ring, re-read consumer's config for next chunk */
  slot->ringPos = (slot->ringPos + rawN) % ringCap;
  slot->rawCount = 0;
  slot->peak = 0;
  slot->firstMs = 0;
  slot->firstEpochMs = 0;
  slot->firstUtcOffset = 0;

  /* Re-read config from consumer's audioSettings */
  audio_meta_t* s = slot->settings;
  slot->cfg.codec       = s->codec;
  slot->cfg.hpf         = s->hpf;
  slot->cfg.gain        = s->gain;
  slot->cfg.agc_target  = s->agc_target;
  slot->cfg.agc_attack  = s->agc_attack;
  slot->cfg.agc_release = s->agc_release;
  slot->cfg.agc_max     = s->agc_max;
  /* Recalc if codec/buf changed */
  if (slot->cfg.buf != s->buf || slot->cfg.len != s->len) {
    slot->cfg.buf = s->buf;
    slot->cfg.len = s->len;
  }
  size_t newRaw = slot->cfg.len > 0
    ? audioRawSamplesForBytes(slot->cfg.codec, (size_t)slot->cfg.len)
    : currentRate / 10;  /* stats-only: 100ms — coarse peaks, less ITS noise */
  if (newRaw != slot->rawNeeded && newRaw > 0)
    slot->rawNeeded = newRaw;
}

/* ---- ITS aux handler (subscribe/unsubscribe on audio task) ---- */

struct audio_sub_msg_t {
  audio_meta_t* settings;
  audio_meta_t* samples;
  audio_cb_t    cb;
};

static void audioCmdHandler(TaskHandle_t sender, const void* data, size_t len) {
  if (len < 1) return;
  uint8_t cmd = *(const uint8_t*)data;

  if (cmd == AUDIO_CMD_SUBSCRIBE) {
    if (len < 1 + sizeof(audio_sub_msg_t)) return;
    auto* msg = (const audio_sub_msg_t*)((const uint8_t*)data + 1);

    audio_slot_t* slot = nullptr;
    for (int i = 0; i < AUDIO_MAX_SUBSCRIBERS; i++)
      if (slots[i].task == sender) { slot = &slots[i]; break; }
    if (!slot)
      for (int i = 0; i < AUDIO_MAX_SUBSCRIBERS; i++)
        if (!slots[i].task) { slot = &slots[i]; break; }
    if (!slot) { err("audio: no free slot\n"); return; }

    slot->task = sender;
    slot->settings = msg->settings;
    slot->samples = msg->samples;
    slot->cb = msg->cb;
    slot->cfg = *msg->settings;  /* initial config copy */
    procReset(&slot->proc);
    /* Chunk size: len (bytes) takes priority, else ms, else 100ms default
     * (stats-only — coarse peaks for level triggers, keeps aux traffic low). */
    if (slot->cfg.len > 0) {
      slot->rawNeeded = audioRawSamplesForBytes(slot->cfg.codec, (size_t)slot->cfg.len);
    } else if (slot->cfg.ms > 0) {
      slot->rawNeeded = (currentRate * (uint32_t)slot->cfg.ms) / 1000;
    } else {
      slot->rawNeeded = currentRate / 10;  /* 100ms */
    }
    if (slot->rawNeeded < 1) slot->rawNeeded = 1;
    slot->rawCount = 0;
    slot->ringPos = ringHead;
    slot->peak = 0;
    slot->firstMs = 0;
    slot->firstEpochMs = 0;
    slot->firstUtcOffset = 0;
    dbg("audio: subscriber %s buf=%u raw=%u\n",
        pcTaskGetName(sender), (unsigned)slot->cfg.len, (unsigned)slot->rawNeeded);

  } else if (cmd == AUDIO_CMD_UNSUBSCRIBE) {
    for (int i = 0; i < AUDIO_MAX_SUBSCRIBERS; i++) {
      if (slots[i].task == sender) {
        dbg("audio: unsubscribe %s\n", pcTaskGetName(sender));
        slots[i] = {};
        break;
      }
    }
  }
}

/* ---- Notification dispatch (runs on consumer's task) ---- */

static void audioNotifyHandler(TaskHandle_t sender, const void* data, size_t len) {
  if (len < sizeof(audio_meta_t)) return;
  TaskHandle_t me = xTaskGetCurrentTaskHandle();
  for (int i = 0; i < AUDIO_MAX_SUBSCRIBERS; i++) {
    if (slots[i].task == me) {
      memcpy(slots[i].samples, data, sizeof(audio_meta_t));
      if (slots[i].cb) slots[i].cb();
      return;
    }
  }
}

/* ---- Task loop ---- */

static void audioTaskFn(void* arg) {
  itsOnAux(AUDIO_CMD_PORT, audioCmdHandler);

  for (;;) {
    while (activeCount() == 0) itsPoll();

    uint32_t sampleRate = (uint32_t)storageGetInt("s.audio.rate");
    i2sInit(sampleRate);
    i2sFlush();
    uint32_t sr = currentRate;
    storageSet("audio.sample_rate", (int)sr);

    size_t readSamples = sr / 100;  /* 10ms */
    size_t readBytes = readSamples * 2;
    uint8_t* readBuf = (uint8_t*)malloc(readBytes);

    /* Ring: 2x largest subscriber's need */
    size_t maxRaw = sr;
    for (int i = 0; i < AUDIO_MAX_SUBSCRIBERS; i++)
      if (slots[i].task && slots[i].rawNeeded > maxRaw)
        maxRaw = slots[i].rawNeeded;
    ringInit(maxRaw * 2);

    running = true;
    pmLockAcquire(audioPmLock);
    info("started %u Hz, ring %u samples\n", (unsigned)sr, (unsigned)ringCap);

    TickType_t lastWake = xTaskGetTickCount();

    while (running) {
      vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(10));
      while (itsPoll(0)) {}
      if (activeCount() == 0) { running = false; break; }

      size_t n = i2sRead(readBuf, readBytes);
      if (n < 2) continue;
      size_t nSamples = n / 2;
      uint32_t nowMs = millis();
      struct timeval nowTv;
      gettimeofday(&nowTv, nullptr);
      uint64_t nowEpochMs = (uint64_t)nowTv.tv_sec * 1000ULL + (uint64_t)nowTv.tv_usec / 1000ULL;
      int16_t nowUtcOff = utcOffsetMinutes(nowTv.tv_sec);

      ringWrite((const int16_t*)readBuf, nSamples);

      for (int i = 0; i < AUDIO_MAX_SUBSCRIBERS; i++) {
        if (!slots[i].task) continue;
        auto* slot = &slots[i];
        if (slot->rawCount == 0) {
          slot->firstMs = nowMs;
          slot->firstEpochMs = nowEpochMs;
          slot->firstUtcOffset = nowUtcOff;
        }
        slot->rawCount += nSamples;
        if (slot->rawCount >= slot->rawNeeded)
          processAndDeliver(slot);
      }
    }

    free(readBuf);
    i2sEnd();
    ringFree();
    running = false;
    pmLockRelease(audioPmLock);
    info("stopped\n");
  }
}

/* ---- Public API ---- */

/* Module config version. Bump when adding/changing defaults. See duckdns.cpp. */
#define AUDIO_VERSION 1

void audioInit() {
  int v = storageGetInt("s.audio.version", 0);
  if (v < AUDIO_VERSION) {
    storageDefault("s.audio.rate", 16000);
    storageSet("s.audio.version", AUDIO_VERSION);
  }

  pmLockCreate(PM_NO_LIGHT_SLEEP, "audio", &audioPmLock);
  taskHandle = spawnTask(audioTaskFn, "audio", 8192, nullptr, 1, 1);
}

int audioMsForBytes(audio_codec_t codec, size_t bytes) {
  if (bytes == 0) return 0;
  uint32_t rate = audioOutputRate(codec);
  size_t rawSamples = audioRawSamplesForBytes(codec, bytes);
  return (int)(rawSamples * 1000 / rate);
}

bool audioSubscribe(audio_meta_t* settings, audio_meta_t* samples, audio_cb_t cb) {
  if (!taskHandle || !cb || !settings || !samples) return false;

  /* Resolve len/ms: fill whichever is -1 from the other + codec */
  if (settings->len == 0 && settings->ms == 0) {
    /* Stats only — no buffer needed */
  } else if (settings->len < 0 && settings->ms < 0) {
    return false;  /* both auto — nothing to derive from */
  } else if (settings->len < 0) {
    settings->len = (int)audioBytesForMs(settings->codec, settings->ms);
  } else if (settings->ms < 0) {
    settings->ms = audioMsForBytes(settings->codec, (size_t)settings->len);
  }

  /* Allocate buffer if not provided */
  if (settings->len > 0 && !settings->buf) {
    settings->buf = (uint8_t*)heap_caps_malloc((size_t)settings->len, MALLOC_CAP_SPIRAM);
    if (!settings->buf) return false;
  }

  /* Register dispatch handler on consumer's task */
  itsOnAux(AUDIO_NOTIFY_PORT, audioNotifyHandler);

  /* Send subscription to audio task */
  uint8_t buf[1 + sizeof(audio_sub_msg_t)];
  buf[0] = AUDIO_CMD_SUBSCRIBE;
  auto* msg = (audio_sub_msg_t*)(buf + 1);
  msg->settings = settings;
  msg->samples = samples;
  msg->cb = cb;
  itsSendAuxByTaskHandle(taskHandle, AUDIO_CMD_PORT, buf, sizeof(buf), pdMS_TO_TICKS(500));
  return true;
}

void audioStop() {
  if (!taskHandle) return;
  uint8_t cmd = AUDIO_CMD_UNSUBSCRIBE;
  itsSendAuxByTaskHandle(taskHandle, AUDIO_CMD_PORT, &cmd, 1, pdMS_TO_TICKS(500));
}

audio_codec_t audioCodecFromConfigString(const char* s) {
  if (!s || !s[0]) return AUDIO_ULAW_16K;
  if (strcmp(s, "ulaw8k") == 0) return AUDIO_ULAW_8K;
  if (strcmp(s, "ulaw16k") == 0) return AUDIO_ULAW_16K;
  if (strcmp(s, "pcm16") == 0) return AUDIO_PCM16;
  return AUDIO_ULAW_16K;
}

/* ---- Output format helpers ---- */

uint32_t audioOutputRate(audio_codec_t codec) {
  return (codec == AUDIO_ULAW_8K) ? 8000 : 16000;
}
uint16_t audioOutputBits(audio_codec_t codec) {
  switch (codec) {
    case AUDIO_PCM16: return 16; case AUDIO_ULAW_8K: case AUDIO_ULAW_16K: return 8;
    case AUDIO_ADPCM: return 4;
  } return 16;
}
uint16_t audioWavFormat(audio_codec_t codec) {
  switch (codec) {
    case AUDIO_PCM16: return 1; case AUDIO_ULAW_8K: case AUDIO_ULAW_16K: return 7;
    case AUDIO_ADPCM: return 0x11;
  } return 1;
}
uint16_t audioBlockAlign(audio_codec_t codec) {
  switch (codec) {
    case AUDIO_PCM16: return 2; default: return 1;
  }
}
