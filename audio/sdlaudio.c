/*
 * QEMU SDL audio driver
 *
 * Copyright (c) 2004-2005 Vassili Karpov (malc)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "qemu/osdep.h"
#include <SDL.h>
#include <SDL_thread.h>
#include "qemu/module.h"
#include "audio.h"

#ifndef _WIN32
#ifdef __sun__
#define _POSIX_PTHREAD_SEMANTICS 1
#elif defined(__OpenBSD__) || defined(__FreeBSD__) || defined(__DragonFly__)
#include <pthread.h>
#endif
#endif

#define AUDIO_CAP "sdl"
#include "audio_int.h"

typedef struct SDLVoiceOut {
    HWVoiceOut hw;
    int exit;
    int initialized;
    Audiodev *dev;
    SDL_AudioDeviceID devid;
} SDLVoiceOut;

static void GCC_FMT_ATTR (1, 2) sdl_logerr (const char *fmt, ...)
{
    va_list ap;

    va_start (ap, fmt);
    AUD_vlog (AUDIO_CAP, fmt, ap);
    va_end (ap);

    AUD_log (AUDIO_CAP, "Reason: %s\n", SDL_GetError ());
}

static int aud_to_sdlfmt (AudioFormat fmt)
{
    switch (fmt) {
    case AUDIO_FORMAT_S8:
        return AUDIO_S8;

    case AUDIO_FORMAT_U8:
        return AUDIO_U8;

    case AUDIO_FORMAT_S16:
        return AUDIO_S16LSB;

    case AUDIO_FORMAT_U16:
        return AUDIO_U16LSB;

    case AUDIO_FORMAT_S32:
        return AUDIO_S32LSB;

    /* no unsigned 32-bit support in SDL */

    case AUDIO_FORMAT_F32:
        return AUDIO_F32LSB;

    default:
        dolog ("Internal logic error: Bad audio format %d\n", fmt);
#ifdef DEBUG_AUDIO
        abort ();
#endif
        return AUDIO_U8;
    }
}

static int sdl_to_audfmt(int sdlfmt, AudioFormat *fmt, int *endianness)
{
    switch (sdlfmt) {
    case AUDIO_S8:
        *endianness = 0;
        *fmt = AUDIO_FORMAT_S8;
        break;

    case AUDIO_U8:
        *endianness = 0;
        *fmt = AUDIO_FORMAT_U8;
        break;

    case AUDIO_S16LSB:
        *endianness = 0;
        *fmt = AUDIO_FORMAT_S16;
        break;

    case AUDIO_U16LSB:
        *endianness = 0;
        *fmt = AUDIO_FORMAT_U16;
        break;

    case AUDIO_S16MSB:
        *endianness = 1;
        *fmt = AUDIO_FORMAT_S16;
        break;

    case AUDIO_U16MSB:
        *endianness = 1;
        *fmt = AUDIO_FORMAT_U16;
        break;

    case AUDIO_S32LSB:
        *endianness = 0;
        *fmt = AUDIO_FORMAT_S32;
        break;

    case AUDIO_S32MSB:
        *endianness = 1;
        *fmt = AUDIO_FORMAT_S32;
        break;

    case AUDIO_F32LSB:
        *endianness = 0;
        *fmt = AUDIO_FORMAT_F32;
        break;

    case AUDIO_F32MSB:
        *endianness = 1;
        *fmt = AUDIO_FORMAT_F32;
        break;

    default:
        dolog ("Unrecognized SDL audio format %d\n", sdlfmt);
        return -1;
    }

    return 0;
}

static SDL_AudioDeviceID sdl_open(SDL_AudioSpec *req, SDL_AudioSpec *obt,
                                  int rec)
{
    SDL_AudioDeviceID devid;
#ifndef _WIN32
    int err;
    sigset_t new, old;

    /* Make sure potential threads created by SDL don't hog signals.  */
    err = sigfillset (&new);
    if (err) {
        dolog ("sdl_open: sigfillset failed: %s\n", strerror (errno));
        return 0;
    }
    err = pthread_sigmask (SIG_BLOCK, &new, &old);
    if (err) {
        dolog ("sdl_open: pthread_sigmask failed: %s\n", strerror (err));
        return 0;
    }
#endif

    devid = SDL_OpenAudioDevice(NULL, rec, req, obt, 0);
    if (!devid) {
        sdl_logerr("SDL_OpenAudioDevice for %s failed\n",
                   rec ? "recording" : "playback");
    }

#ifndef _WIN32
    err = pthread_sigmask (SIG_SETMASK, &old, NULL);
    if (err) {
        dolog ("sdl_open: pthread_sigmask (restore) failed: %s\n",
               strerror (errno));
        /* We have failed to restore original signal mask, all bets are off,
           so exit the process */
        exit (EXIT_FAILURE);
    }
#endif
    return devid;
}

static void sdl_close_out(SDLVoiceOut *sdl)
{
    if (sdl->initialized) {
        SDL_LockAudioDevice(sdl->devid);
        sdl->exit = 1;
        SDL_UnlockAudioDevice(sdl->devid);
        SDL_PauseAudioDevice(sdl->devid, 1);
        sdl->initialized = 0;
    }
    if (sdl->devid) {
        SDL_CloseAudioDevice(sdl->devid);
        sdl->devid = 0;
    }
}

static void sdl_callback_out(void *opaque, Uint8 *buf, int len)
{
    SDLVoiceOut *sdl = opaque;
    HWVoiceOut *hw = &sdl->hw;

    if (!sdl->exit) {

        /* dolog("callback_out: len=%d avail=%zu\n", len, hw->pending_emul); */

        while (hw->pending_emul && len) {
            size_t write_len;
            ssize_t start = (ssize_t)hw->pos_emul - hw->pending_emul;
            if (start < 0) {
                start += hw->size_emul;
            }
            assert(start >= 0 && start < hw->size_emul);

            write_len = MIN(MIN(hw->pending_emul, len),
                            hw->size_emul - start);

            memcpy(buf, hw->buf_emul + start, write_len);
            hw->pending_emul -= write_len;
            len -= write_len;
            buf += write_len;
        }
    }

    /* clear remaining buffer that we couldn't fill with data */
    if (len) {
        audio_pcm_info_clear_buf(&hw->info, buf,
                                 len / hw->info.bytes_per_frame);
    }
}

#define SDL_WRAPPER_FUNC(name, ret_type, args_decl, args, dir) \
    static ret_type glue(sdl_, name)args_decl                  \
    {                                                          \
        ret_type ret;                                          \
        glue(SDLVoice, dir) *sdl = (glue(SDLVoice, dir) *)hw;  \
                                                               \
        SDL_LockAudioDevice(sdl->devid);                       \
        ret = glue(audio_generic_, name)args;                  \
        SDL_UnlockAudioDevice(sdl->devid);                     \
                                                               \
        return ret;                                            \
    }

SDL_WRAPPER_FUNC(get_buffer_out, void *, (HWVoiceOut *hw, size_t *size),
                 (hw, size), Out)
SDL_WRAPPER_FUNC(put_buffer_out, size_t,
                 (HWVoiceOut *hw, void *buf, size_t size), (hw, buf, size), Out)
SDL_WRAPPER_FUNC(write, size_t,
                 (HWVoiceOut *hw, void *buf, size_t size), (hw, buf, size), Out)
#undef SDL_WRAPPER_FUNC

static void sdl_fini_out(HWVoiceOut *hw)
{
    SDLVoiceOut *sdl = (SDLVoiceOut *)hw;

    sdl_close_out(sdl);
}

static int sdl_init_out(HWVoiceOut *hw, struct audsettings *as,
                        void *drv_opaque)
{
    SDLVoiceOut *sdl = (SDLVoiceOut *)hw;
    SDL_AudioSpec req, obt;
    int endianness;
    int err;
    AudioFormat effective_fmt;
    Audiodev *dev = drv_opaque;
    AudiodevSdlPerDirectionOptions *spdo = dev->u.sdl.out;
    struct audsettings obt_as;

    req.freq = as->freq;
    req.format = aud_to_sdlfmt (as->fmt);
    req.channels = as->nchannels;
    /*
     * This is wrong. SDL samples are QEMU frames. The buffer size will be
     * the requested buffer size multiplied by the number of channels.
     */
    req.samples = audio_buffer_samples(
        qapi_AudiodevSdlPerDirectionOptions_base(spdo), as, 11610);
    req.callback = sdl_callback_out;
    req.userdata = sdl;

    sdl->dev = dev;
    sdl->devid = sdl_open(&req, &obt, 0);
    if (!sdl->devid) {
        return -1;
    }

    err = sdl_to_audfmt(obt.format, &effective_fmt, &endianness);
    if (err) {
        sdl_close_out(sdl);
        return -1;
    }

    obt_as.freq = obt.freq;
    obt_as.nchannels = obt.channels;
    obt_as.fmt = effective_fmt;
    obt_as.endianness = endianness;

    audio_pcm_init_info (&hw->info, &obt_as);
    hw->samples = (spdo->has_buffer_count ? spdo->buffer_count : 4) *
        obt.samples;

    sdl->initialized = 1;
    sdl->exit = 0;
    return 0;
}

static void sdl_enable_out(HWVoiceOut *hw, bool enable)
{
    SDLVoiceOut *sdl = (SDLVoiceOut *)hw;

    SDL_PauseAudioDevice(sdl->devid, !enable);
}

static void *sdl_audio_init(Audiodev *dev)
{
    if (SDL_InitSubSystem (SDL_INIT_AUDIO)) {
        sdl_logerr ("SDL failed to initialize audio subsystem\n");
        return NULL;
    }

    return dev;
}

static void sdl_audio_fini (void *opaque)
{
    SDL_QuitSubSystem (SDL_INIT_AUDIO);
}

static struct audio_pcm_ops sdl_pcm_ops = {
    .init_out = sdl_init_out,
    .fini_out = sdl_fini_out,
  /* wrapper for audio_generic_write */
    .write    = sdl_write,
  /* wrapper for audio_generic_get_buffer_out */
    .get_buffer_out = sdl_get_buffer_out,
  /* wrapper for audio_generic_put_buffer_out */
    .put_buffer_out = sdl_put_buffer_out,
    .enable_out = sdl_enable_out,
};

static struct audio_driver sdl_audio_driver = {
    .name           = "sdl",
    .descr          = "SDL http://www.libsdl.org",
    .init           = sdl_audio_init,
    .fini           = sdl_audio_fini,
    .pcm_ops        = &sdl_pcm_ops,
    .can_be_default = 1,
    .max_voices_out = 1,
    .max_voices_in  = 0,
    .voice_size_out = sizeof (SDLVoiceOut),
    .voice_size_in  = 0
};

static void register_audio_sdl(void)
{
    audio_driver_register(&sdl_audio_driver);
}
type_init(register_audio_sdl);
