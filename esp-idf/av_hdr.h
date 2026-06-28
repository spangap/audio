/**
 * av_hdr — unified on-the-wire audio/video packet header for live DC paths.
 *
 * Lifted verbatim from seccam so the audio packets this straddle emits are
 * byte-identical to seccam's live DataChannel framing: a sender leaves
 * sizeof(av_hdr_t) bytes of headroom at the front of its buffer, fills the
 * header in place and ships [header][payload] as one packet. The first byte
 * identifies the payload kind (video/audio) so a receiver can demux without
 * branching on the channel name.
 *
 * Layout (12 bytes, little-endian, packed):
 *   type(1)   0 = video, 1 = audio
 *   codec(1)  video: AV_VIDEO_CODEC_* (JPEG today)
 *             audio: audio_codec_t value
 *   epoch_ms(8)       Unix epoch ms of first sample / capture
 *   utc_offset_min(2) minutes east of UTC at capture (DST-aware)
 */
#ifndef AUDIO_AV_HDR_H
#define AUDIO_AV_HDR_H

#include <stdint.h>

#define AV_TYPE_VIDEO  0
#define AV_TYPE_AUDIO  1

#define AV_VIDEO_CODEC_JPEG 0

struct __attribute__((packed)) av_hdr_t {
    uint8_t  type;
    uint8_t  codec;
    uint64_t epoch_ms;
    int16_t  utc_offset_min;
};

static_assert(sizeof(av_hdr_t) == 12, "av_hdr_t must be exactly 12 bytes");

/** Fill in an av_hdr_t at *h. Caller supplies type, codec, and timestamp. */
static inline void avHdrFill(av_hdr_t* h, uint8_t type, uint8_t codec,
                             uint64_t epoch_ms, int16_t utc_offset_min) {
    h->type = type;
    h->codec = codec;
    h->epoch_ms = epoch_ms;
    h->utc_offset_min = utc_offset_min;
}

#endif
