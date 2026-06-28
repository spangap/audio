# spangap/audio

Generic I2S audio engine + ITS packet server. One audio device (single sample
rate, single block size) that:

- **Mic → network:** streams the live microphone to every connected client as
  fixed-size `[av_hdr_t (12B)][payload]` blocks — the header is byte-identical
  to seccam's live DataChannel framing, so existing seccam-side decoders work
  unchanged.
- **Network → speaker:** decodes and **mixes** (sum + saturate) everything any
  client sends to the port into the speaker output. Inbound headers are
  optional and timestamps are ignored.
- **WAV playback:** `audioPlayWav(path)` streams a device-rate WAV from the
  filesystem into the same output mixer.

The straddle **owns the I2S read/write engine** (`i2s_std` + `i2s_pdm`, pins
from Kconfig). A board contributes only Kconfig pin/mode **data** and, for an
I2C/AT-controlled input codec like the ES7210, a small `audio_codec_ops_t`
**control-plane shim** — almost every audio board (MAX98357A, PCM5102A, PDM
mics) is pure Kconfig with no code.

Build for the LilyGo T-Deck Plus (ES7210 mic + MAX98357A speaker):

```
spangap build reticulous/reticulous --with reticulous/hw-tdeck --with spangap/audio
```

## Origins

The `av_hdr_t` packet header is lifted verbatim from
[seccam](../seccam), so the audio blocks this straddle emits are byte-identical
to seccam's live DataChannel framing and existing seccam-side decoders work
unchanged. The codec transforms are also from seccam, extended here with the
matching decoders (seccam only ever encoded the mic toward the network; this
straddle is full-duplex). See [INTERNALS.md](INTERNALS.md) for the full
inventory.

## Reachability

The engine is reachable as the browser DataChannel label **`audio:1`**: the
audio task runs an ITS server on port 1, and the web straddle parses
`task:port`, so a DataChannel labelled `audio:1` bridges straight to it. A
browser opens that channel and immediately starts receiving mic blocks and may
send audio back to be played.

## Wire format

`[av_hdr_t (12B)][payload]`, both directions.

- **Outbound (device → client):** `type=1` (audio), `codec=<the client's
  codec>`, real `epoch_ms`/`utc_offset_min` of the block's first sample.
- **Inbound (client → device):** the header is **optional**. If the first 12
  bytes parse as a valid `av_hdr` (`type==1`, a known codec byte), it is
  stripped and the payload decoded per that codec; otherwise the whole packet
  is treated as raw PCM16 at the device rate. Timestamps are ignored.

A client selects its outbound codec by sending the codec name (`pcm16`,
`ulaw8k`, `ulaw16k`, `adpcm`) as the connect payload — for a browser
DataChannel that is the channel's `protocol` field. With no payload the default
`s.audio.codec` is used.

### Codecs

| codec | wire | notes |
|---|---|---|
| `pcm16`    | signed 16-bit LE        | device rate |
| `ulaw8k`   | G.711 µ-law, 2:1 decimated | half the device rate (seccam-compatible) |
| `ulaw16k`  | G.711 µ-law             | device rate |
| `adpcm`    | IMA ADPCM, 4-bit        | device rate |

There is **no resampler**: WAV files must already be at the device rate, and
inbound network audio is decoded sample-for-sample (assumed device rate).

## Storage vars

| Key | Persist | Default | Meaning |
|---|---|---|---|
| `s.audio.rate`     | flash | `CONFIG_AUDIO_RATE_DEFAULT` (16000) | requested device sample rate |
| `s.audio.block_ms` | flash | `CONFIG_AUDIO_BLOCK_MS_DEFAULT` (20) | global block size (capture/stream/mix unit) |
| `s.audio.codec`    | flash | `ulaw16k` | default outbound codec for clients that don't send one |
| `s.audio.out_gain` | flash | 256 | speaker gain, 8.8 fixed (256 = 1×) |
| `s.audio.mic_gain` | flash | 1 | mic capture gain (integer; >1 amplify, 0 mute) |
| `audio.sample_rate`| ephemeral | — | rate the engine actually clocked (status) |
| `audio.clients`    | ephemeral | — | live connected-client count (status) |

## CLI

- `audio` — status (rate, resolved rate, block, clients, default codec, gains).
- `audio play <path>` — play a device-rate WAV out the speaker.
- `audio stop` — stop all WAV playback.

## Adding a board

1. Set the `CONFIG_AUDIO_*` pin/mode values in the board straddle's `kconfig:`
   block (a non-buildable board's `sdkconfig.defaults` is ignored).
2. If the input codec needs control-bus programming (I2C/SPI/AT), drop a slice
   at `esp-idf/conditional/audio/src/<board>_audio.cpp` that implements
   `audio_codec_ops_t` and calls `audioRegisterCodec()` from a
   `when: spangap/audio` init hook. Pure-I2S parts (MAX98357A, PCM5102A, PDM)
   register nothing.

See [INTERNALS.md](INTERNALS.md) for the engine internals (task loop, gating,
mixer, the blocking TX write, and memory/DMA placement).
