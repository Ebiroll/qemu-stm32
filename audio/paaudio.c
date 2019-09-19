/* public domain */

#include "qemu/osdep.h"
#include "qemu/module.h"
#include "audio.h"
#include "qapi/opts-visitor.h"

#include <pulse/pulseaudio.h>

#define AUDIO_CAP "pulseaudio"
#include "audio_int.h"

typedef struct PAConnection {
    char *server;
    int refcount;
    QTAILQ_ENTRY(PAConnection) list;

    pa_threaded_mainloop *mainloop;
    pa_context *context;
} PAConnection;

static QTAILQ_HEAD(PAConnectionHead, PAConnection) pa_conns =
    QTAILQ_HEAD_INITIALIZER(pa_conns);

typedef struct {
    Audiodev *dev;
    PAConnection *conn;
} paaudio;

typedef struct {
    HWVoiceOut hw;
    pa_stream *stream;
    paaudio *g;
    size_t samples;
} PAVoiceOut;

typedef struct {
    HWVoiceIn hw;
    pa_stream *stream;
    const void *read_data;
    size_t read_length;
    paaudio *g;
    size_t samples;
} PAVoiceIn;

static void qpa_conn_fini(PAConnection *c);

static void GCC_FMT_ATTR (2, 3) qpa_logerr (int err, const char *fmt, ...)
{
    va_list ap;

    va_start (ap, fmt);
    AUD_vlog (AUDIO_CAP, fmt, ap);
    va_end (ap);

    AUD_log (AUDIO_CAP, "Reason: %s\n", pa_strerror (err));
}

#ifndef PA_CONTEXT_IS_GOOD
static inline int PA_CONTEXT_IS_GOOD(pa_context_state_t x)
{
    return
        x == PA_CONTEXT_CONNECTING ||
        x == PA_CONTEXT_AUTHORIZING ||
        x == PA_CONTEXT_SETTING_NAME ||
        x == PA_CONTEXT_READY;
}
#endif

#ifndef PA_STREAM_IS_GOOD
static inline int PA_STREAM_IS_GOOD(pa_stream_state_t x)
{
    return
        x == PA_STREAM_CREATING ||
        x == PA_STREAM_READY;
}
#endif

#define CHECK_SUCCESS_GOTO(c, expression, label, msg)           \
    do {                                                        \
        if (!(expression)) {                                    \
            qpa_logerr(pa_context_errno((c)->context), msg);    \
            goto label;                                         \
        }                                                       \
    } while (0)

#define CHECK_DEAD_GOTO(c, stream, label, msg)                          \
    do {                                                                \
        if (!(c)->context || !PA_CONTEXT_IS_GOOD (pa_context_get_state((c)->context)) || \
            !(stream) || !PA_STREAM_IS_GOOD (pa_stream_get_state ((stream)))) { \
            if (((c)->context && pa_context_get_state ((c)->context) == PA_CONTEXT_FAILED) || \
                ((stream) && pa_stream_get_state ((stream)) == PA_STREAM_FAILED)) { \
                qpa_logerr(pa_context_errno((c)->context), msg);        \
            } else {                                                    \
                qpa_logerr(PA_ERR_BADSTATE, msg);                       \
            }                                                           \
            goto label;                                                 \
        }                                                               \
    } while (0)

static size_t qpa_read(HWVoiceIn *hw, void *data, size_t length)
{
    PAVoiceIn *p = (PAVoiceIn *) hw;
    PAConnection *c = p->g->conn;
    size_t l;
    int r;

    pa_threaded_mainloop_lock(c->mainloop);

    CHECK_DEAD_GOTO(c, p->stream, unlock_and_fail,
                    "pa_threaded_mainloop_lock failed\n");

    if (!p->read_length) {
        r = pa_stream_peek(p->stream, &p->read_data, &p->read_length);
        CHECK_SUCCESS_GOTO(c, r == 0, unlock_and_fail,
                           "pa_stream_peek failed\n");
    }

    l = MIN(p->read_length, length);
    memcpy(data, p->read_data, l);

    p->read_data += l;
    p->read_length -= l;

    if (!p->read_length) {
        r = pa_stream_drop(p->stream);
        CHECK_SUCCESS_GOTO(c, r == 0, unlock_and_fail,
                           "pa_stream_drop failed\n");
    }

    pa_threaded_mainloop_unlock(c->mainloop);
    return l;

unlock_and_fail:
    pa_threaded_mainloop_unlock(c->mainloop);
    return 0;
}

static size_t qpa_write(HWVoiceOut *hw, void *data, size_t length)
{
    PAVoiceOut *p = (PAVoiceOut *) hw;
    PAConnection *c = p->g->conn;
    size_t l;
    int r;

    pa_threaded_mainloop_lock(c->mainloop);

    CHECK_DEAD_GOTO(c, p->stream, unlock_and_fail,
                    "pa_threaded_mainloop_lock failed\n");

    l = pa_stream_writable_size(p->stream);

    CHECK_SUCCESS_GOTO(c, l != (size_t) -1, unlock_and_fail,
                       "pa_stream_writable_size failed\n");

    if (l > length) {
        l = length;
    }

    r = pa_stream_write(p->stream, data, l, NULL, 0LL, PA_SEEK_RELATIVE);
    CHECK_SUCCESS_GOTO(c, r >= 0, unlock_and_fail, "pa_stream_write failed\n");

    pa_threaded_mainloop_unlock(c->mainloop);
    return l;

unlock_and_fail:
    pa_threaded_mainloop_unlock(c->mainloop);
    return 0;
}

static pa_sample_format_t audfmt_to_pa (AudioFormat afmt, int endianness)
{
    int format;

    switch (afmt) {
    case AUDIO_FORMAT_S8:
    case AUDIO_FORMAT_U8:
        format = PA_SAMPLE_U8;
        break;
    case AUDIO_FORMAT_S16:
    case AUDIO_FORMAT_U16:
        format = endianness ? PA_SAMPLE_S16BE : PA_SAMPLE_S16LE;
        break;
    case AUDIO_FORMAT_S32:
    case AUDIO_FORMAT_U32:
        format = endianness ? PA_SAMPLE_S32BE : PA_SAMPLE_S32LE;
        break;
    default:
        dolog ("Internal logic error: Bad audio format %d\n", afmt);
        format = PA_SAMPLE_U8;
        break;
    }
    return format;
}

static AudioFormat pa_to_audfmt (pa_sample_format_t fmt, int *endianness)
{
    switch (fmt) {
    case PA_SAMPLE_U8:
        return AUDIO_FORMAT_U8;
    case PA_SAMPLE_S16BE:
        *endianness = 1;
        return AUDIO_FORMAT_S16;
    case PA_SAMPLE_S16LE:
        *endianness = 0;
        return AUDIO_FORMAT_S16;
    case PA_SAMPLE_S32BE:
        *endianness = 1;
        return AUDIO_FORMAT_S32;
    case PA_SAMPLE_S32LE:
        *endianness = 0;
        return AUDIO_FORMAT_S32;
    default:
        dolog ("Internal logic error: Bad pa_sample_format %d\n", fmt);
        return AUDIO_FORMAT_U8;
    }
}

static void context_state_cb (pa_context *c, void *userdata)
{
    PAConnection *conn = userdata;

    switch (pa_context_get_state(c)) {
    case PA_CONTEXT_READY:
    case PA_CONTEXT_TERMINATED:
    case PA_CONTEXT_FAILED:
        pa_threaded_mainloop_signal(conn->mainloop, 0);
        break;

    case PA_CONTEXT_UNCONNECTED:
    case PA_CONTEXT_CONNECTING:
    case PA_CONTEXT_AUTHORIZING:
    case PA_CONTEXT_SETTING_NAME:
        break;
    }
}

static void stream_state_cb (pa_stream *s, void * userdata)
{
    PAConnection *c = userdata;

    switch (pa_stream_get_state (s)) {

    case PA_STREAM_READY:
    case PA_STREAM_FAILED:
    case PA_STREAM_TERMINATED:
        pa_threaded_mainloop_signal(c->mainloop, 0);
        break;

    case PA_STREAM_UNCONNECTED:
    case PA_STREAM_CREATING:
        break;
    }
}

static pa_stream *qpa_simple_new (
        PAConnection *c,
        const char *name,
        pa_stream_direction_t dir,
        const char *dev,
        const pa_sample_spec *ss,
        const pa_channel_map *map,
        const pa_buffer_attr *attr,
        int *rerror)
{
    int r;
    pa_stream *stream;
    pa_stream_flags_t flags;

    pa_threaded_mainloop_lock(c->mainloop);

    stream = pa_stream_new(c->context, name, ss, map);
    if (!stream) {
        goto fail;
    }

    pa_stream_set_state_callback(stream, stream_state_cb, c);

    flags =
        PA_STREAM_INTERPOLATE_TIMING
        | PA_STREAM_AUTO_TIMING_UPDATE
        | PA_STREAM_EARLY_REQUESTS;

    if (dev) {
        /* don't move the stream if the user specified a sink/source */
        flags |= PA_STREAM_DONT_MOVE;
    }

    if (dir == PA_STREAM_PLAYBACK) {
        r = pa_stream_connect_playback(stream, dev, attr, flags, NULL, NULL);
    } else {
        r = pa_stream_connect_record(stream, dev, attr, flags);
    }

    if (r < 0) {
      goto fail;
    }

    pa_threaded_mainloop_unlock(c->mainloop);

    return stream;

fail:
    pa_threaded_mainloop_unlock(c->mainloop);

    if (stream) {
        pa_stream_unref (stream);
    }

    *rerror = pa_context_errno(c->context);

    return NULL;
}

static int qpa_init_out(HWVoiceOut *hw, struct audsettings *as,
                        void *drv_opaque)
{
    int error;
    pa_sample_spec ss;
    pa_buffer_attr ba;
    struct audsettings obt_as = *as;
    PAVoiceOut *pa = (PAVoiceOut *) hw;
    paaudio *g = pa->g = drv_opaque;
    AudiodevPaOptions *popts = &g->dev->u.pa;
    AudiodevPaPerDirectionOptions *ppdo = popts->out;
    PAConnection *c = g->conn;

    ss.format = audfmt_to_pa (as->fmt, as->endianness);
    ss.channels = as->nchannels;
    ss.rate = as->freq;

    ba.tlength = pa_usec_to_bytes(ppdo->latency, &ss);
    ba.minreq = -1;
    ba.maxlength = -1;
    ba.prebuf = -1;

    obt_as.fmt = pa_to_audfmt (ss.format, &obt_as.endianness);

    pa->stream = qpa_simple_new (
        c,
        "qemu",
        PA_STREAM_PLAYBACK,
        ppdo->has_name ? ppdo->name : NULL,
        &ss,
        NULL,                   /* channel map */
        &ba,                    /* buffering attributes */
        &error
        );
    if (!pa->stream) {
        qpa_logerr (error, "pa_simple_new for playback failed\n");
        goto fail1;
    }

    audio_pcm_init_info (&hw->info, &obt_as);
    hw->samples = pa->samples = audio_buffer_samples(
        qapi_AudiodevPaPerDirectionOptions_base(ppdo),
        &obt_as, ppdo->buffer_length);

    return 0;

 fail1:
    return -1;
}

static int qpa_init_in(HWVoiceIn *hw, struct audsettings *as, void *drv_opaque)
{
    int error;
    pa_sample_spec ss;
    pa_buffer_attr ba;
    struct audsettings obt_as = *as;
    PAVoiceIn *pa = (PAVoiceIn *) hw;
    paaudio *g = pa->g = drv_opaque;
    AudiodevPaOptions *popts = &g->dev->u.pa;
    AudiodevPaPerDirectionOptions *ppdo = popts->in;
    PAConnection *c = g->conn;

    ss.format = audfmt_to_pa (as->fmt, as->endianness);
    ss.channels = as->nchannels;
    ss.rate = as->freq;

    ba.fragsize = pa_usec_to_bytes(ppdo->latency, &ss);
    ba.maxlength = pa_usec_to_bytes(ppdo->latency * 2, &ss);
    ba.minreq = -1;
    ba.prebuf = -1;

    obt_as.fmt = pa_to_audfmt (ss.format, &obt_as.endianness);

    pa->stream = qpa_simple_new (
        c,
        "qemu",
        PA_STREAM_RECORD,
        ppdo->has_name ? ppdo->name : NULL,
        &ss,
        NULL,                   /* channel map */
        &ba,                    /* buffering attributes */
        &error
        );
    if (!pa->stream) {
        qpa_logerr (error, "pa_simple_new for capture failed\n");
        goto fail1;
    }

    audio_pcm_init_info (&hw->info, &obt_as);
    hw->samples = pa->samples = audio_buffer_samples(
        qapi_AudiodevPaPerDirectionOptions_base(ppdo),
        &obt_as, ppdo->buffer_length);

    return 0;

 fail1:
    return -1;
}

static void qpa_simple_disconnect(PAConnection *c, pa_stream *stream)
{
    int err;

    pa_threaded_mainloop_lock(c->mainloop);
    /*
     * wait until actually connects. workaround pa bug #247
     * https://gitlab.freedesktop.org/pulseaudio/pulseaudio/issues/247
     */
    while (pa_stream_get_state(stream) == PA_STREAM_CREATING) {
        pa_threaded_mainloop_wait(c->mainloop);
    }

    err = pa_stream_disconnect(stream);
    if (err != 0) {
        dolog("Failed to disconnect! err=%d\n", err);
    }
    pa_stream_unref(stream);
    pa_threaded_mainloop_unlock(c->mainloop);
}

static void qpa_fini_out (HWVoiceOut *hw)
{
    PAVoiceOut *pa = (PAVoiceOut *) hw;

    if (pa->stream) {
        qpa_simple_disconnect(pa->g->conn, pa->stream);
        pa->stream = NULL;
    }
}

static void qpa_fini_in (HWVoiceIn *hw)
{
    PAVoiceIn *pa = (PAVoiceIn *) hw;

    if (pa->stream) {
        qpa_simple_disconnect(pa->g->conn, pa->stream);
        pa->stream = NULL;
    }
}

static void qpa_volume_out(HWVoiceOut *hw, struct mixeng_volume *vol)
{
    PAVoiceOut *pa = (PAVoiceOut *) hw;
    pa_operation *op;
    pa_cvolume v;
    PAConnection *c = pa->g->conn;

#ifdef PA_CHECK_VERSION    /* macro is present in 0.9.16+ */
    pa_cvolume_init (&v);  /* function is present in 0.9.13+ */
#endif

    v.channels = 2;
    v.values[0] = ((PA_VOLUME_NORM - PA_VOLUME_MUTED) * vol->l) / UINT32_MAX;
    v.values[1] = ((PA_VOLUME_NORM - PA_VOLUME_MUTED) * vol->r) / UINT32_MAX;

    pa_threaded_mainloop_lock(c->mainloop);

    op = pa_context_set_sink_input_volume(c->context,
                                          pa_stream_get_index(pa->stream),
                                          &v, NULL, NULL);
    if (!op) {
        qpa_logerr(pa_context_errno(c->context),
                   "set_sink_input_volume() failed\n");
    } else {
        pa_operation_unref(op);
    }

    op = pa_context_set_sink_input_mute(c->context,
                                        pa_stream_get_index(pa->stream),
                                        vol->mute, NULL, NULL);
    if (!op) {
        qpa_logerr(pa_context_errno(c->context),
                   "set_sink_input_mute() failed\n");
    } else {
        pa_operation_unref(op);
    }

    pa_threaded_mainloop_unlock(c->mainloop);
}

static void qpa_volume_in(HWVoiceIn *hw, struct mixeng_volume *vol)
{
    PAVoiceIn *pa = (PAVoiceIn *) hw;
    pa_operation *op;
    pa_cvolume v;
    PAConnection *c = pa->g->conn;

#ifdef PA_CHECK_VERSION
    pa_cvolume_init (&v);
#endif

    v.channels = 2;
    v.values[0] = ((PA_VOLUME_NORM - PA_VOLUME_MUTED) * vol->l) / UINT32_MAX;
    v.values[1] = ((PA_VOLUME_NORM - PA_VOLUME_MUTED) * vol->r) / UINT32_MAX;

    pa_threaded_mainloop_lock(c->mainloop);

    op = pa_context_set_source_output_volume(c->context,
        pa_stream_get_index(pa->stream),
        &v, NULL, NULL);
    if (!op) {
        qpa_logerr(pa_context_errno(c->context),
                   "set_source_output_volume() failed\n");
    } else {
        pa_operation_unref(op);
    }

    op = pa_context_set_source_output_mute(c->context,
        pa_stream_get_index(pa->stream),
        vol->mute, NULL, NULL);
    if (!op) {
        qpa_logerr(pa_context_errno(c->context),
                   "set_source_output_mute() failed\n");
    } else {
        pa_operation_unref(op);
    }

    pa_threaded_mainloop_unlock(c->mainloop);
}

static int qpa_validate_per_direction_opts(Audiodev *dev,
                                           AudiodevPaPerDirectionOptions *pdo)
{
    if (!pdo->has_buffer_length) {
        pdo->has_buffer_length = true;
        pdo->buffer_length = 46440;
    }
    if (!pdo->has_latency) {
        pdo->has_latency = true;
        pdo->latency = 15000;
    }
    return 1;
}

/* common */
static void *qpa_conn_init(const char *server)
{
    PAConnection *c = g_malloc0(sizeof(PAConnection));
    QTAILQ_INSERT_TAIL(&pa_conns, c, list);

    c->mainloop = pa_threaded_mainloop_new();
    if (!c->mainloop) {
        goto fail;
    }

    c->context = pa_context_new(pa_threaded_mainloop_get_api(c->mainloop),
                                server);
    if (!c->context) {
        goto fail;
    }

    pa_context_set_state_callback(c->context, context_state_cb, c);

    if (pa_context_connect(c->context, server, 0, NULL) < 0) {
        qpa_logerr(pa_context_errno(c->context),
                   "pa_context_connect() failed\n");
        goto fail;
    }

    pa_threaded_mainloop_lock(c->mainloop);

    if (pa_threaded_mainloop_start(c->mainloop) < 0) {
        goto unlock_and_fail;
    }

    for (;;) {
        pa_context_state_t state;

        state = pa_context_get_state(c->context);

        if (state == PA_CONTEXT_READY) {
            break;
        }

        if (!PA_CONTEXT_IS_GOOD(state)) {
            qpa_logerr(pa_context_errno(c->context),
                       "Wrong context state\n");
            goto unlock_and_fail;
        }

        /* Wait until the context is ready */
        pa_threaded_mainloop_wait(c->mainloop);
    }

    pa_threaded_mainloop_unlock(c->mainloop);
    return c;

unlock_and_fail:
    pa_threaded_mainloop_unlock(c->mainloop);
fail:
    AUD_log (AUDIO_CAP, "Failed to initialize PA context");
    qpa_conn_fini(c);
    return NULL;
}

static void *qpa_audio_init(Audiodev *dev)
{
    paaudio *g;
    AudiodevPaOptions *popts = &dev->u.pa;
    const char *server;
    PAConnection *c;

    assert(dev->driver == AUDIODEV_DRIVER_PA);

    if (!popts->has_server) {
        char pidfile[64];
        char *runtime;
        struct stat st;

        runtime = getenv("XDG_RUNTIME_DIR");
        if (!runtime) {
            return NULL;
        }
        snprintf(pidfile, sizeof(pidfile), "%s/pulse/pid", runtime);
        if (stat(pidfile, &st) != 0) {
            return NULL;
        }
    }

    if (!qpa_validate_per_direction_opts(dev, popts->in)) {
        return NULL;
    }
    if (!qpa_validate_per_direction_opts(dev, popts->out)) {
        return NULL;
    }

    g = g_malloc0(sizeof(paaudio));
    server = popts->has_server ? popts->server : NULL;

    g->dev = dev;

    QTAILQ_FOREACH(c, &pa_conns, list) {
        if (server == NULL || c->server == NULL ?
            server == c->server :
            strcmp(server, c->server) == 0) {
            g->conn = c;
            break;
        }
    }
    if (!g->conn) {
        g->conn = qpa_conn_init(server);
    }
    if (!g->conn) {
        g_free(g);
        return NULL;
    }

    ++g->conn->refcount;
    return g;
}

static void qpa_conn_fini(PAConnection *c)
{
    if (c->mainloop) {
        pa_threaded_mainloop_stop(c->mainloop);
    }

    if (c->context) {
        pa_context_disconnect(c->context);
        pa_context_unref(c->context);
    }

    if (c->mainloop) {
        pa_threaded_mainloop_free(c->mainloop);
    }

    QTAILQ_REMOVE(&pa_conns, c, list);
    g_free(c);
}

static void qpa_audio_fini (void *opaque)
{
    paaudio *g = opaque;
    PAConnection *c = g->conn;

    if (--c->refcount == 0) {
        qpa_conn_fini(c);
    }

    g_free(g);
}

static struct audio_pcm_ops qpa_pcm_ops = {
    .init_out = qpa_init_out,
    .fini_out = qpa_fini_out,
    .write    = qpa_write,
    .volume_out = qpa_volume_out,

    .init_in  = qpa_init_in,
    .fini_in  = qpa_fini_in,
    .read     = qpa_read,
    .volume_in = qpa_volume_in
};

static struct audio_driver pa_audio_driver = {
    .name           = "pa",
    .descr          = "http://www.pulseaudio.org/",
    .init           = qpa_audio_init,
    .fini           = qpa_audio_fini,
    .pcm_ops        = &qpa_pcm_ops,
    .can_be_default = 1,
    .max_voices_out = INT_MAX,
    .max_voices_in  = INT_MAX,
    .voice_size_out = sizeof (PAVoiceOut),
    .voice_size_in  = sizeof (PAVoiceIn),
};

static void register_audio_pa(void)
{
    audio_driver_register(&pa_audio_driver);
}
type_init(register_audio_pa);
