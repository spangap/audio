/**
 * codec — audio codec transforms shared by the capture (mic→wire) and
 * playback (wire→speaker) paths.
 *
 * Every codec has both an encoder (mic -> wire) and a decoder (wire ->
 * speaker), since the path is full-duplex. All transforms operate
 * sample-for-sample at the device rate — there is no rate conversion here, with
 * the single exception of ULAW_8K, whose 2:1 average keeps seccam
 * wire-compatibility. Inbound streams are decoded sample-for-sample and assumed
 * to already be at the device rate.
 */
#ifndef AUDIO_CODEC_H
#define AUDIO_CODEC_H

#include <stdint.h>
#include <stddef.h>

/** Wire codecs. Values are stable: they travel in av_hdr_t.codec and match
 *  seccam's audio_codec_t so existing seccam decoders interoperate. */
typedef enum {
  AUDIO_PCM16    = 0,   /* signed 16-bit LE, device rate                       */
  AUDIO_ULAW_8K  = 1,   /* G.711 µ-law, 2:1 decimated (half the device rate)   */
  AUDIO_ULAW_16K = 2,   /* G.711 µ-law at the device rate                      */
  AUDIO_ADPCM    = 3,   /* IMA ADPCM (4-bit), device rate                      */
} audio_codec_t;

/** Per-stream codec state (IMA-ADPCM predictor/step). One per direction per
 *  client; reset on connect / playback start. PCM16 and µ-law are stateless,
 *  but carrying the struct uniformly keeps call sites simple. */
typedef struct {
  int32_t adpcmPredictor;
  int     adpcmStepIndex;
} codec_state_t;

static inline void codecStateReset(codec_state_t* s) {
  s->adpcmPredictor = 0;
  s->adpcmStepIndex = 0;
}

/** Map a settings string to a codec: "pcm16", "ulaw8k", "ulaw16k", "adpcm".
 *  Defaults to ULAW_16K (µ-law at device rate) for empty/unknown input. */
audio_codec_t audioCodecFromConfigString(const char* s);

/** True if `c` is a value this build knows how to decode. Used by the inbound
 *  av_hdr heuristic to tell a real header from raw PCM16. */
bool audioCodecValid(uint8_t c);

/* ---- Encoders: device-rate PCM16 block -> wire payload ----
 * Each returns the number of payload bytes written to `out`. `out` must have
 * room for the worst case (n*2 bytes for PCM16). */
size_t audioEncode(audio_codec_t codec, codec_state_t* st,
                   const int16_t* in, size_t n, uint8_t* out);

/* ---- Decoders: wire payload -> device-rate PCM16 ----
 * Decodes `inBytes` of `codec` payload into `out` (int16 samples). Returns the
 * number of samples produced. `out` must have room for the worst case
 * (inBytes*2 samples for ADPCM). */
size_t audioDecode(audio_codec_t codec, codec_state_t* st,
                   const uint8_t* in, size_t inBytes, int16_t* out);

/* ---- Format helpers (carried over from seccam) ----
 * `deviceRate` is the engine's resolved sample rate; ULAW_8K reports half. */
uint32_t audioOutputRate(audio_codec_t codec, uint32_t deviceRate);
uint16_t audioOutputBits(audio_codec_t codec);
uint16_t audioWavFormat(audio_codec_t codec);

#endif
