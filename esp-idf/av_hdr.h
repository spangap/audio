/**
 * av_hdr — unified on-the-wire audio/video packet header for live DC paths.
 *
 * A device task leaves sizeof(av_hdr_t) bytes of headroom at the start of
 * its frame/audio buffer, fills the header in place, and ships
 * [header][payload] as one packet. First byte identifies the payload kind
 * so receivers can demux a combined AV channel or dispatch a single channel
 * without branching on the channel name.
 *
 * Layout (12 bytes, little-endian, packed):
 *   type(1)   0 = video, 1 = audio
 *   codec(1)  video: AV_VIDEO_CODEC_* (JPEG today)
 *             audio: audio_codec_t value
 *   epoch_ms(8)       Unix epoch ms of first sample / capture
 *   utc_offset_min(2) minutes east of UTC at capture (DST-aware)
 *
 * Not used by AVI recording (AVI has its own WCLK+fourcc layout) or RTSP
 * (RTP has its own framing). WebRTC DC paths use this format.
 */
#ifndef SECCAM_AV_HDR_H
#define SECCAM_AV_HDR_H

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
