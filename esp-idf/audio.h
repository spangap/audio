/**
 * audio — generic I2S audio engine + ITS packet server.
 *
 * One device (single rate, single block size) that:
 *   - streams the live microphone to every connected ITS client as
 *     [av_hdr_t][payload] packets (seccam-compatible framing), and
 *   - mixes everything any client sends back — plus any WAV playback — into
 *     the speaker output (sum + saturate; inbound timestamps ignored).
 *
 * The straddle owns the I2S read/write engine (i2s_std + i2s_pdm, pins from
 * Kconfig). A board contributes only data (CONFIG_AUDIO_* pin/mode values)
 * and, for an I2C/AT-controlled input codec like the ES7210, an optional
 * control-plane shim registered via audioRegisterCodec().
 *
 * Reachability: the audio task runs an ITS server on port 1, which the web
 * straddle bridges to the browser DataChannel label "audio:1" (it parses
 * "task:port"). Clients connect there for full-duplex audio.
 */
#ifndef AUDIO_H
#define AUDIO_H

#include <stdint.h>
#include "codec.h"   /* audio_codec_t + format helpers */

/* DataChannel label "audio:1": clients connect here for full-duplex audio. */
static constexpr uint16_t AUDIO_STREAM_PORT = 1;

/** Bring-up hook — folded into the generated spangapInitStraddles() in
 *  dependency order. Plain C++ linkage (NOT extern "C"); the generated
 *  dispatcher forward-declares it as `void audioInit(void);`. */
void audioInit();

/* ---- Board codec control plane (optional) ---- */

/** The only board↔generic interface beyond Kconfig: programs a non-I2S input
 *  codec (I2C/SPI/AT) for a sample rate. I2S pins/mode are NOT here — they
 *  come from Kconfig. Amps that self-clock (MAX98357A, PCM5102A) and PDM mics
 *  register nothing and run raw I2S. */
struct audio_codec_ops_t {
  /* Program the input codec for `rate` over its control bus, called by the
   * audio task immediately BEFORE it enables I2S RX (and again if the rate
   * changes). Return the rate the codec actually clocked (may be clamped),
   * or 0 on failure. A no-op codec returns `rate`. */
  uint32_t (*in_init)(uint32_t rate);
  void     (*in_deinit)(void);
  /* Optional output codec control (most amps need none → leave null). */
  uint32_t (*out_init)(uint32_t rate);
  void     (*out_deinit)(void);
};

/** Register a board input/output codec shim. Stores the pointer; the audio
 *  task applies it lazily on first capture, so hook ordering between the
 *  board's init and audioInit does not matter. Call once, from a
 *  `when: spangap/audio` board init hook. */
void audioRegisterCodec(const audio_codec_ops_t* ops);

/* ---- WAV playback (mixed into the speaker output) ---- */

/** Play a WAV from the filesystem into the speaker mixer.
 *   - mono PCM16 (fmt 1) or µ-law (fmt 7); sample rate MUST equal the device
 *     rate (rejected otherwise — there is no resampler).
 *   - Non-blocking: streams in the background via fs_* read-ahead.
 *   - Returns a stream id (>= 0) for audioStopWav(), or < 0 on error
 *     (no output configured / missing file / bad format / wrong rate).
 *  Safe to call from any task. */
int  audioPlayWav(const char* path);

/** Stop a WAV stream by id; -1 stops all. */
void audioStopWav(int id);

#endif
