/**
 * wav — RIFF/WAVE parse + streamed read-ahead. See wav.h.
 */
#include "wav.h"
#include "codec.h"
#include "fs.h"
#include "log.h"
#include "esp_heap_caps.h"
#include <string.h>
#include <cstdio>   /* SEEK_CUR */

static bool readExact(int fd, void* dst, size_t n) {
  return fs_read(dst, 1, n, fd) == n;
}

static uint32_t le32(const uint8_t* p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static uint16_t le16(const uint8_t* p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

bool wavOpen(const char* path, uint32_t deviceRate, size_t readAheadBytes,
             wav_stream_t* out) {
  memset(out, 0, sizeof(*out));
  if (!path || !path[0]) return false;

  int fd = fs_open(path, "rb");
  if (fd < 0) { warn("wav: cannot open %s\n", path); return false; }

  uint8_t hdr[12];
  if (!readExact(fd, hdr, 12) ||
      memcmp(hdr, "RIFF", 4) != 0 || memcmp(hdr + 8, "WAVE", 4) != 0) {
    warn("wav: %s is not a RIFF/WAVE file\n", path);
    fs_close(fd);
    return false;
  }

  uint16_t fmt = 0, channels = 0, bits = 0;
  uint32_t rate = 0, dataBytes = 0;
  bool haveFmt = false, haveData = false;

  /* Walk the chunk list until we have both fmt and data. */
  uint8_t ck[8];
  while (readExact(fd, ck, 8)) {
    uint32_t sz = le32(ck + 4);
    if (memcmp(ck, "fmt ", 4) == 0) {
      uint8_t f[16];
      uint32_t want = sz < sizeof(f) ? sz : sizeof(f);
      if (!readExact(fd, f, want)) break;
      fmt      = le16(f + 0);
      channels = le16(f + 2);
      rate     = le32(f + 4);
      bits     = le16(f + 14);
      haveFmt  = true;
      /* Skip any extension bytes beyond the basic 16-byte fmt body. */
      if (sz > want) fs_seek(fd, sz - want + (sz & 1), SEEK_CUR);
      else if (sz & 1) fs_seek(fd, 1, SEEK_CUR);
    } else if (memcmp(ck, "data", 4) == 0) {
      dataBytes = sz;
      haveData = true;
      break;  /* data is the last thing we read; stream from here */
    } else {
      fs_seek(fd, sz + (sz & 1), SEEK_CUR);  /* chunks are word-aligned */
    }
  }

  if (!haveFmt || !haveData) {
    warn("wav: %s missing fmt/data chunk\n", path);
    fs_close(fd);
    return false;
  }
  if (channels != 1) {
    warn("wav: %s has %u channels (mono only)\n", path, channels);
    fs_close(fd);
    return false;
  }
  if (rate != deviceRate) {
    warn("wav: %s is %u Hz, device is %u Hz (no resampler)\n", path, rate, deviceRate);
    fs_close(fd);
    return false;
  }
  bool okPcm  = (fmt == 1 && bits == 16);
  bool okUlaw = (fmt == 7 && bits == 8);
  if (!okPcm && !okUlaw) {
    warn("wav: %s unsupported fmt=%u bits=%u (need PCM16 or µ-law)\n", path, fmt, bits);
    fs_close(fd);
    return false;
  }

  size_t cap = readAheadBytes ? readAheadBytes : 4096;
  uint8_t* buf = (uint8_t*)heap_caps_malloc(cap, MALLOC_CAP_SPIRAM);
  if (!buf) { fs_close(fd); return false; }

  out->fd = fd;
  out->fmt = (uint8_t)fmt;
  out->dataLeft = dataBytes;
  out->buf = buf;
  out->bufCap = cap;
  out->bufHead = 0;
  out->bufLen = 0;
  return true;
}

/* Ensure at least `need` raw bytes are buffered (or EOF reached). */
static void wavRefill(wav_stream_t* s, size_t need) {
  size_t avail = s->bufLen - s->bufHead;
  if (avail >= need || s->dataLeft == 0) return;
  /* Compact the unread tail to the front, then top up from the file. */
  if (s->bufHead) {
    memmove(s->buf, s->buf + s->bufHead, avail);
    s->bufHead = 0;
    s->bufLen = avail;
  }
  size_t room = s->bufCap - s->bufLen;
  size_t want = room < s->dataLeft ? room : s->dataLeft;
  if (want == 0) return;
  size_t got = fs_read(s->buf + s->bufLen, 1, want, s->fd);
  s->bufLen += got;
  s->dataLeft -= got;
  if (got < want) s->dataLeft = 0;  /* short read = treat as EOF */
}

size_t wavRead(wav_stream_t* s, int16_t* out, size_t maxSamples) {
  if (!s->buf) return 0;
  size_t produced = 0;

  if (s->fmt == 1) {                 /* PCM16: 2 bytes per sample */
    while (produced < maxSamples) {
      if (s->bufLen - s->bufHead < 2) wavRefill(s, 2);
      if (s->bufLen - s->bufHead < 2) break;
      out[produced++] = (int16_t)le16(s->buf + s->bufHead);
      s->bufHead += 2;
    }
  } else {                           /* µ-law: 1 byte per sample */
    codec_state_t dummy;
    codecStateReset(&dummy);
    while (produced < maxSamples) {
      if (s->bufLen - s->bufHead < 1) wavRefill(s, 1);
      if (s->bufLen - s->bufHead < 1) break;
      audioDecode(AUDIO_ULAW_16K, &dummy, s->buf + s->bufHead, 1, &out[produced]);
      s->bufHead += 1;
      produced++;
    }
  }
  return produced;
}

void wavClose(wav_stream_t* s) {
  if (s->fd >= 0) { fs_close(s->fd); s->fd = -1; }
  if (s->buf) { heap_caps_free(s->buf); s->buf = nullptr; }
  s->bufLen = s->bufHead = 0;
  s->dataLeft = 0;
}
