/**
 * Audio — PDM mic with subscriber-based delivery via ITS aux.
 *
 * Consumer declares their own audio_meta_t for settings and samples,
 * sets config fields, calls audioSubscribe(settings, samples, cb).
 * Audio task fills settings->buf, sends audio_meta_t via ITS aux.
 * Client-side handler copies to *samples, calls cb().
 *
 * Config changes: audio task re-reads from settings pointer at chunk boundaries.
 * Shared state: audio.sample_rate, audio.last_trigger (config vars).
 */
#ifndef SECCAM_AUDIO_H
#define SECCAM_AUDIO_H

#include <stdint.h>
#include <stddef.h>

/** Audio task's ITS aux ports. */
static constexpr uint16_t AUDIO_CMD_PORT    = 24;  /* subscribe/unsubscribe */
static constexpr uint16_t AUDIO_NOTIFY_PORT = 25;  /* registered on subscriber's task */

typedef enum {
  AUDIO_PCM16,
  AUDIO_ULAW_8K,
  AUDIO_ULAW_16K,
  AUDIO_ADPCM,
} audio_codec_t;

typedef struct {
  uint8_t*      buf         = nullptr;
  int           len         = -1;    /* buffer bytes. -1 = auto from ms. 0 = stats only */
  int           ms          = -1;    /* buffer duration. -1 = auto from len. 0 = stats only */
  audio_codec_t codec       = AUDIO_PCM16;
  bool          hpf         = true;
  int           gain        = 1;
  int           agc_target  = 0;
  int           agc_attack  = 10;
  int           agc_release = 500;
  int           agc_max     = 0;
  uint32_t      timestamp   = 0;     /* millis() of first sample in buffer (session-relative uptime) */
  uint64_t      epoch_ms    = 0;     /* wall clock (Unix epoch ms) of first sample */
  int16_t       utc_offset_min = 0;  /* utc offset at capture, minutes */
  int16_t       peak        = 0;     /* max raw peak-to-peak across buffer */
  int16_t       eff_gain    = 256;   /* effective gain (8.8 fixed, 256=1x) */
} audio_meta_t;

typedef void (*audio_cb_t)();

void audioInit();

/** Subscribe. Caller owns settings, samples, and optionally buf.
 *  settings: set config fields. len or ms must be >= 0 (other auto-calculated).
 *            If buf is nullptr and len > 0, buffer is auto-allocated.
 *  samples:  filled by audio on each delivery, read from cb().
 *  cb:       called on consumer's task (via itsPoll) when buffer is filled. */
bool audioSubscribe(audio_meta_t* settings, audio_meta_t* samples, audio_cb_t cb);

/** Unsubscribe current task. */
void audioStop();

/** Map settings string to codec: "pcm16", "ulaw8k", "ulaw16k" (default ulaw16k). */
audio_codec_t audioCodecFromConfigString(const char* s);

/* Output format helpers */
uint32_t audioOutputRate(audio_codec_t codec);
uint16_t audioOutputBits(audio_codec_t codec);
uint16_t audioWavFormat(audio_codec_t codec);
uint16_t audioBlockAlign(audio_codec_t codec);
size_t   audioRawSamplesForBytes(audio_codec_t codec, size_t outBytes);
size_t   audioBytesForMs(audio_codec_t codec, int ms);
int      audioMsForBytes(audio_codec_t codec, size_t bytes);

#endif
