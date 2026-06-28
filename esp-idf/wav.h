/**
 * wav — RIFF/WAVE parser + streamed reader feeding the output mixer.
 *
 * There is no resampler, so a file's sample rate MUST equal the engine's
 * resolved device rate (rejected otherwise). Supports mono 16-bit PCM
 * (fmt 1) and mono 8-bit µ-law (fmt 7) — the two formats the device itself
 * produces. File bytes are pulled through the `fs_*` API (proxied to the
 * DRAM-stack worker) into a PSRAM read-ahead buffer so the audio loop never
 * stalls on flash/SD I/O and never runs LittleFS off a PSRAM stack.
 */
#ifndef AUDIO_WAV_H
#define AUDIO_WAV_H

#include <stdint.h>
#include <stddef.h>

struct wav_stream_t {
  int      fd;          /* fs_* descriptor (task-agnostic) */
  uint8_t  fmt;         /* 1 = PCM16, 7 = µ-law */
  uint32_t dataLeft;    /* unread data-chunk bytes still in the file */
  uint8_t* buf;         /* PSRAM read-ahead of raw file bytes */
  size_t   bufCap;
  size_t   bufHead;     /* next unconsumed byte in buf */
  size_t   bufLen;      /* valid bytes in buf */
};

/** Open `path`, parse its header and validate it against `deviceRate`.
 *  Returns true with *out populated on success; logs a reason and returns
 *  false on a missing file, unsupported format, non-mono, or rate mismatch.
 *  `readAheadBytes` sizes the PSRAM refill buffer (0 → a small default). */
bool wavOpen(const char* path, uint32_t deviceRate, size_t readAheadBytes,
             wav_stream_t* out);

/** Pull up to `maxSamples` device-rate PCM16 samples into `out`, refilling
 *  the read-ahead buffer from the file as needed. Returns the sample count;
 *  0 means end-of-stream (caller should wavClose). */
size_t wavRead(wav_stream_t* s, int16_t* out, size_t maxSamples);

/** Close the file and free the read-ahead buffer. Safe to call repeatedly. */
void wavClose(wav_stream_t* s);

#endif
