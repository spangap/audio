/**
 * codec — µ-law / PCM16 / IMA-ADPCM encode + decode. See codec.h.
 *
 * The encoders and the IMA tables are lifted from seccam's audio.cpp; the
 * decoders (µ-law expand, ADPCM decode) are new, since seccam never played
 * audio back.
 */
#include "codec.h"
#include <string.h>

/* ---- IMA ADPCM tables (shared by encode + decode) ---- */

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

/* ---- µ-law (G.711) ---- */

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

static int16_t ulawToLinear(uint8_t u) {
  u = ~u;
  int sign     = u & 0x80;
  int exponent = (u >> 4) & 0x07;
  int mantissa = u & 0x0F;
  int sample = ((mantissa << 3) + 0x84) << exponent;
  sample -= 0x84;
  return (int16_t)(sign ? -sample : sample);
}

/* ---- ADPCM ---- */

static size_t encodeAdpcm(codec_state_t* p, const int16_t* in, size_t n, uint8_t* out) {
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

static int16_t adpcmDecodeNibble(codec_state_t* p, int code) {
  int step = imaStepTable[p->adpcmStepIndex];
  int32_t delta = ((code & 7) * step / 4 + step / 8);
  if (code & 8) delta = -delta;
  p->adpcmPredictor += delta;
  if (p->adpcmPredictor > 32767) p->adpcmPredictor = 32767;
  if (p->adpcmPredictor < -32768) p->adpcmPredictor = -32768;
  p->adpcmStepIndex += imaIndexTable[code & 0x0F];
  if (p->adpcmStepIndex < 0) p->adpcmStepIndex = 0;
  if (p->adpcmStepIndex > 88) p->adpcmStepIndex = 88;
  return (int16_t)p->adpcmPredictor;
}

/* ---- Public encode/decode ---- */

size_t audioEncode(audio_codec_t codec, codec_state_t* st,
                   const int16_t* in, size_t n, uint8_t* out) {
  switch (codec) {
    case AUDIO_PCM16:
      memcpy(out, in, n * 2);
      return n * 2;
    case AUDIO_ULAW_8K: {        /* 2:1 average decimation, then µ-law */
      size_t outN = n / 2;
      for (size_t i = 0; i < outN; i++) {
        int32_t avg = ((int32_t)in[i * 2] + (int32_t)in[i * 2 + 1]) / 2;
        out[i] = linearToUlaw((int16_t)avg);
      }
      return outN;
    }
    case AUDIO_ULAW_16K:
      for (size_t i = 0; i < n; i++) out[i] = linearToUlaw(in[i]);
      return n;
    case AUDIO_ADPCM:
      return encodeAdpcm(st, in, n, out);
  }
  memcpy(out, in, n * 2);
  return n * 2;
}

size_t audioDecode(audio_codec_t codec, codec_state_t* st,
                   const uint8_t* in, size_t inBytes, int16_t* out) {
  switch (codec) {
    case AUDIO_PCM16: {          /* raw int16 LE passthrough */
      size_t n = inBytes / 2;
      memcpy(out, in, n * 2);
      return n;
    }
    case AUDIO_ULAW_8K:          /* sample-for-sample expand (no upsample) */
    case AUDIO_ULAW_16K:
      for (size_t i = 0; i < inBytes; i++) out[i] = ulawToLinear(in[i]);
      return inBytes;
    case AUDIO_ADPCM: {          /* two samples per byte */
      size_t n = 0;
      for (size_t i = 0; i < inBytes; i++) {
        out[n++] = adpcmDecodeNibble(st, in[i] & 0x0F);
        out[n++] = adpcmDecodeNibble(st, (in[i] >> 4) & 0x0F);
      }
      return n;
    }
  }
  size_t n = inBytes / 2;
  memcpy(out, in, n * 2);
  return n;
}

/* ---- Config + format helpers ---- */

audio_codec_t audioCodecFromConfigString(const char* s) {
  if (!s || !s[0]) return AUDIO_ULAW_16K;
  if (strcmp(s, "pcm16") == 0)    return AUDIO_PCM16;
  if (strcmp(s, "ulaw8k") == 0)   return AUDIO_ULAW_8K;
  if (strcmp(s, "ulaw16k") == 0)  return AUDIO_ULAW_16K;
  if (strcmp(s, "adpcm") == 0)    return AUDIO_ADPCM;
  return AUDIO_ULAW_16K;
}

bool audioCodecValid(uint8_t c) {
  return c <= AUDIO_ADPCM;
}

uint32_t audioOutputRate(audio_codec_t codec, uint32_t deviceRate) {
  return (codec == AUDIO_ULAW_8K) ? deviceRate / 2 : deviceRate;
}
uint16_t audioOutputBits(audio_codec_t codec) {
  switch (codec) {
    case AUDIO_PCM16:    return 16;
    case AUDIO_ULAW_8K:
    case AUDIO_ULAW_16K: return 8;
    case AUDIO_ADPCM:    return 4;
  }
  return 16;
}
uint16_t audioWavFormat(audio_codec_t codec) {
  switch (codec) {
    case AUDIO_PCM16:    return 1;     /* WAVE_FORMAT_PCM        */
    case AUDIO_ULAW_8K:
    case AUDIO_ULAW_16K: return 7;     /* WAVE_FORMAT_MULAW      */
    case AUDIO_ADPCM:    return 0x11;  /* WAVE_FORMAT_DVI_ADPCM  */
  }
  return 1;
}
