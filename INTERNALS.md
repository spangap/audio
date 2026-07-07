# audio — internals

The design *why* and the platform gotchas behind the engine. The public surface
(wire format, codecs, storage vars, CLI, adding a board) is in [README.md](README.md).

## 1. What this straddle adds

Two pieces come from [seccam](../seccam); the rest is new:

- **`av_hdr_t` — lifted verbatim from seccam** (`av_hdr.h`). The 12-byte
  `[type][codec][epoch_ms][utc_offset_min]` header is byte-identical, so the
  audio blocks this straddle emits demux on the seccam side without change.
- **Codecs — lifted from seccam and extended with decoders.** seccam only
  encoded the mic toward the network; the four codecs (`pcm16`, `ulaw8k`,
  `ulaw16k`, `adpcm`) each gain a matching decoder here because the path is
  full-duplex. The codec enum values still match seccam's `audio_codec_t`, and
  the `ulaw8k` 2:1 decimation is kept for seccam wire-compatibility.
- **Full-duplex I2S engine** — owns both I2S channels (`i2s_std` + `i2s_pdm`)
  and runs capture and playback concurrently on disjoint controllers.
- **ITS-server fan-out** — the mic is streamed to every connected ITS client,
  each with its own outbound codec and encoder state.
- **Speaker mixer** — sums (and saturates) every client's inbound audio plus any
  active WAV into one output block.
- **WAV streaming** — `audioPlayWav(path)` streams a device-rate WAV off the
  filesystem into that mixer via an `fs_*` read-ahead.
- **Lazy I2S bring-up** — the ITS server opens immediately but each I2S
  direction is created only when wanted, so a board's `audioRegisterCodec()`
  ordering relative to `audioInit()` does not matter.

## One task owns everything

A single FreeRTOS task named **`audio`** (core 1, prio 1, 8 KB PSRAM stack) owns
the ITS server *and* both I2S channels. ITS handles are therefore never sent
cross-task, and capture and playback stay phase-locked to one block cadence. The
task name `audio` is also what makes the DataChannel label `audio:1` route here
(the web straddle `itsConnect`s to `task:port`).

The task exposes **two ITS ports**: the client server on
`AUDIO_STREAM_PORT = 1` (`ITS_PACKET`, up to `CONFIG_AUDIO_MAX_CLIENTS` = 4
clients — a connect past the cap is rejected), and an aux handler on
`AUDIO_CTRL_PORT = 2` that receives the in-firmware play/stop control messages
(`itsOnAux`). Port 2 is not a network-reachable client port; it is how
`audioPlayWav`/`audioStopWav` hand work to the audio task from any caller task
(`itsSendAuxByTaskHandle`).

`audioInit()` (auto-dispatched init hook) seeds the `s.audio.*` storage
defaults, registers the CLI, and spawns the task. The task opens its ITS server
immediately but brings I2S up **lazily**, so a board's `audioRegisterCodec()`
ordering relative to `audioInit()` does not matter — the pointer is just stowed
and applied on the first capture. (The `s.audio.version` gate around the default
seeding is a vestigial config-version guard, not a feature — it is a candidate
for code removal under the no-config-migrations policy.)

## Block model

```
samplesPerBlock = rate * block_ms / 1000
```

`rate` is `s.audio.rate` (resolved by the input codec shim on first capture and
republished as `audio.sample_rate`); `block_ms` is `s.audio.block_ms`. Capture,
fan-out, mixing and WAV all operate in these units. The engine clamps to
`rate ∈ [8000, 48000]`, `block_ms ∈ [10, 40]`, and a 1920-sample ceiling
(48 kHz × 40 ms) that bounds every internal buffer.

## Idle / active gating

I2S is APB-clocked and gated in light sleep, so the task holds a
`PM_NO_LIGHT_SLEEP` lock only while a direction is active, and blocks in
`itsPoll(portMAX_DELAY)` when fully idle. The two directions gate independently:

- **RX (mic)** active when client count > 0 (connecting implies wanting the mic).
- **TX (speaker)** active when playback is pending — any client's inbound ring
  has samples, or a WAV stream is active.

Each direction's I2S channel (and its internal DMA scratch buffer) is created on
the iteration its want first goes true and torn down when it goes false; the PM
lock is released when both are down. When fully idle the resolved rate is cleared
so a new session re-reads `s.audio.*` (a rate change takes effect on the next
bring-up, not mid-session).

## Main loop (one iteration ≈ one block)

```
rxWant = IN_ENABLED  && clients > 0
txWant = OUT_ENABLED && (any WAV active || any inbound buffered)
ensureRx(rxWant); ensureTx(txWant)
if neither up → release PM, itsPoll(forever), continue
acquire PM if needed
while (itsPoll(0)) {}                      # drain connects / inbound / ctrl aux
if rxUp:  read one block ── apply mic_gain ── per client: encode [av_hdr][payload], itsSend(…,0)
if txUp:  acc = Σ(each client inbound ring, each active WAV); ×out_gain, saturate; write full block
```

**Pacing comes from the blocking I2S calls.** When RX is up, the
`i2s_channel_read` of one block paces the loop. When only TX is up, the TX write
paces it via DMA backpressure (silence is written when there is nothing to mix,
so the clock never underruns). Outbound `itsSend` is non-blocking (timeout 0) and
drops on backpressure — a slow client never stalls capture.

### The TX write must drain the whole block

A single `i2s_channel_write` with a short timeout is a trap: under scheduling
load on a busy board (WiFi + LoRa + mesh) it returns `ESP_ERR_TIMEOUT` having
queued only part of the block, and the **unwritten tail is silently lost** —
playback degrades to a click or a clipped beep. So the loop writes the *entire*
block, retrying the remainder with a generous (1 s) per-call timeout:
`i2s_channel_write` blocks on the DMA-drain interrupt, so a starved CPU only
makes it *wait*, it never drops samples. The TX channel is also preloaded with
silence (`i2s_channel_preload_data`) before `i2s_channel_enable`, so it starts on
real data instead of an initial underrun glitch.

## Fan-out & mixing

- **Fan-out:** a per-client slot array holds the ITS handle; the loop guards with
  `itsConnected()` then non-blocking `itsSend()`. Each client carries its own
  outbound codec and ADPCM encoder state, so codecs are independent. The outbound
  codec is chosen at connect from the connect payload: an exact codec name
  (`pcm16`/`ulaw8k`/`ulaw16k`/`adpcm`), else a single raw codec byte if it is a
  valid value, else the `s.audio.codec` default.
- **Inbound:** `onRecv` strips the optional `av_hdr` (heuristic: 12+ bytes whose
  first byte is `AV_TYPE_AUDIO` and second is a known codec → real header; else
  treat the whole packet as raw PCM16), decodes per codec into the client's PSRAM
  ring (per-client ADPCM decoder state, reset when the inbound codec changes). The
  ring holds `CONFIG_AUDIO_OUT_MIX_MS` (default 120) ms of device-rate PCM16,
  allocated per client on connect; samples that don't fit are dropped.
- **Mixer:** an `int32` accumulator over one block sums every client's inbound
  plus every active WAV, applies `out_gain` (8.8 fixed) and clamps to int16.

## WAV playback

`audioPlayWav(path)` parses + validates the header on the calling task (mono
PCM16/µ-law, sample rate == device rate — there is no resampler), then hands the open
`fs_*` descriptor and a PSRAM read-ahead buffer to the audio task over an aux
message; it returns a stream id. The audio task streams blocks from the buffer,
refilling it from the file through `fs_*` (proxied to the DRAM-stack worker) so
flash/SD I/O never stalls the audio loop and never runs LittleFS off a PSRAM
stack. `audioStopWav(id)` (or `-1` for all) cancels. WAV slots and the read-ahead
buffer are bounded by `CONFIG_AUDIO_MAX_WAV` / `CONFIG_AUDIO_WAV_BUF_MS`.

## Memory placement (platform hazards)

The ESP32-S3's internal DMA-capable RAM is scarce and heavily contended on a
loaded board (WiFi static RX + SPI DMA), so the engine is frugal with it:

- **I2S DMA scratch (`inBlk`/`outBlk`) is internal DRAM** — a concurrent flash op
  disables the PSRAM cache mid-copy, so an I2S buffer in PSRAM could feed the DMA
  garbage. They are allocated per-direction at bring-up, sized to the *live*
  block (not the 1920-sample max), to spare the pool.
- **The I2S driver's own DMA descriptors** must also come from that pool. The
  driver default (~5.7 KB contiguous) does not fit at steady state, so
  `CONFIG_AUDIO_DMA_DESC_NUM` × `CONFIG_AUDIO_DMA_FRAME_NUM` (default 3 × 128
  frames) keep it small. **Rule:** if full-duplex capture+playback underruns on
  a roomier board, raise `CONFIG_AUDIO_DMA_*` (or the audio task priority) — the
  defaults are tuned for the contended T-Deck pool, not for headroom.
- **Large buffers** (mixer accumulator, inbound rings, WAV read-ahead,
  encode/decode scratch) are **PSRAM** — they never touch I2S DMA.
- The task creates **no FreeRTOS sync objects of its own**; the only cross-task
  channel is ITS (which keeps its own sync objects in internal DRAM). All file
  I/O goes through `fs_*`, never raw LittleFS.

## Full-duplex requires two I2S controllers

Capture and playback run concurrently only because the mic and speaker sit on
**two physically separate I2S controllers** clocking independently (the ESP32-S3
has `SOC_I2S_NUM = 2`). On a part with one controller they cannot overlap. The
board-specific pin sets and any input-codec register sequence (e.g. the ES7210
on the T-Deck) live in that board's audio shim, not here.

## TCP transport (future)

The DataChannel path is the only network transport today. The designed LAN path
is a TCP listener registered with net: one `net_port_msg_t`
(`nvsKey = "audio_port"`, `defaultPort = 0`) sent via
`itsSendAux("net", NET_PORT_REG_PORT, …)`, so the listener opens only when the
user sets `s.net.audio_port`. What makes it non-trivial: TCP is stream-mode, so
the device must length-prefix each `[av_hdr][payload]` unit over the byte
stream — on the packet-native DataChannel path message boundaries come for
free.
