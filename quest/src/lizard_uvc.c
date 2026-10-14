/* Fox UVC on the Quest USB bus.
 * Camera A is 0c45:636a (lattice) and camera B is 0c45:636b (colour).
 * Both speak UVC 1.0. MJPEG 1280x720 is format 1, frame 2, 10 fps.
 * The Quest camera service only publishes the headset cameras, so this
 * opens the Fox devices through UsbManager and reads isochronous URBs.
 */
#include "lizard_uvc.h"

#include <android/log.h>
#include <errno.h>
#include <jni.h>
#include <linux/usbdevice_fs.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define TAG "lizard"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, TAG, __VA_ARGS__)

#define NURB 10
#define NPKT 32
#define JPEG_CAP (2 * 1024 * 1024)
#define PROBE_MAX 34

enum { ERR_OK = 0, ERR_ABSENT = 1, ERR_DENIED = 2, ERR_NEG = 3, ERR_BAND = 4 };

static const int k_alt_bytes[] = {0, 128, 256, 800, 1600, 2400, 3072};
static const int k_alt_base[] = {0, 128, 256, 800, 800, 800, 1024};

static JavaVM *g_vm;
static jobject g_usb;
static jobject g_argb;
static jobject g_options;
static jclass g_factory;
static jclass g_bitmap;
static jmethodID m_get_service;
static jmethodID m_dev_list;
static jmethodID m_has_perm;
static jmethodID m_open_dev;
static jmethodID m_values;
static jmethodID m_to_array;
static jmethodID m_vendor;
static jmethodID m_product;
static jmethodID m_serial;
static jmethodID m_devname;
static jmethodID m_get_fd;
static jmethodID m_conn_close;
static jmethodID m_decode;
static jmethodID m_opt_init;
static jmethodID m_get_pixels;
static jmethodID m_recycle;
static jmethodID m_width;
static jmethodID m_height;
static jfieldID f_config;
static jfieldID f_scaled;
static pthread_key_t g_jni_key;
static int g_jni_key_ok;
static __thread int t_err;
static __thread char t_errstr[160];

struct Uvc {
    uint16_t vid, pid;
    int fd;
    int alt;
    int payload;
    int pkt;
    int pkt_base;
    int width, height;
    int claimed;
    int exposure;
    int gain;
    volatile int restream;
    int stamp_seq;
    int stamp_seen;
    int stamp_n;
    volatile int stop;
    volatile int dead;
    volatile int seq;
    volatile int frames;
    int held_len;
    int decoded_seq;
    int bad;
    int saved;
    int fid;
    int have_fid;
    int acc_len;
    pthread_t thread;
    int thread_on;
    pthread_mutex_t mu;
    jobject connection;
    jobject pixels;
    int pixels_n;
    uint8_t *acc;
    uint8_t *held;
    uint8_t *scratch;
    struct usbdevfs_urb *urbs[NURB];
    char serial[48];
    char status[96];
};

static void set_err(int code, const char *msg) {
    t_err = code;
    snprintf(t_errstr, sizeof t_errstr, "%s", msg ? msg : "");
    LOGI("uvc %s", t_errstr);
}

int uvc_last_error(void) { return t_err; }
const char *uvc_last_error_str(void) { return t_errstr[0] ? t_errstr : "ok"; }

static void jni_detach(void *p) {
    (void)p;
    if (g_vm) (*g_vm)->DetachCurrentThread(g_vm);
}

static JNIEnv *jni_env(void) {
    JNIEnv *env = NULL;
    jint st;
    if (!g_vm) return NULL;
    st = (*g_vm)->GetEnv(g_vm, (void **)&env, JNI_VERSION_1_6);
    if (st == JNI_OK) return env;
    if ((*g_vm)->AttachCurrentThread(g_vm, &env, NULL) != JNI_OK) return NULL;
    if (g_jni_key_ok) pthread_setspecific(g_jni_key, (void *)1);
    return env;
}

static int jni_ok(JNIEnv *env, const char *what) {
    if (!(*env)->ExceptionCheck(env)) return 1;
    LOGE("uvc jni exception at %s", what);
    (*env)->ExceptionDescribe(env);
    (*env)->ExceptionClear(env);
    return 0;
}

static void put_u16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}
static void put_u32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}
static uint32_t get_u32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int ctrl(int fd, uint8_t rt, uint8_t req, uint16_t val, uint16_t idx, void *data, uint16_t len) {
    struct usbdevfs_ctrltransfer c;
    int rc;
    memset(&c, 0, sizeof c);
    c.bRequestType = rt;
    c.bRequest = req;
    c.wValue = val;
    c.wIndex = idx;
    c.wLength = len;
    c.timeout = 800;
    c.data = data;
    rc = ioctl(fd, USBDEVFS_CONTROL, &c);
    if (rc < 0 && errno == EPIPE) {
        unsigned int ep = 0;
        ioctl(fd, USBDEVFS_CLEAR_HALT, &ep);
        rc = ioctl(fd, USBDEVFS_CONTROL, &c);
    }
    return rc;
}

static int claim_if(int fd, unsigned int iface) {
    struct usbdevfs_ioctl kick;
    if (ioctl(fd, USBDEVFS_CLAIMINTERFACE, &iface) == 0) return 0;
    if (errno != EBUSY && errno != ENODEV) {
        LOGI("claim if%u: %s", iface, strerror(errno));
    }
    memset(&kick, 0, sizeof kick);
    kick.ifno = (int)iface;
    kick.ioctl_code = USBDEVFS_DISCONNECT;
    kick.data = NULL;
    ioctl(fd, USBDEVFS_IOCTL, &kick);
    if (ioctl(fd, USBDEVFS_CLAIMINTERFACE, &iface) == 0) return 0;
    LOGI("claim if%u after disconnect: %s", iface, strerror(errno));
    return -1;
}

static int ctrl_get(int fd, uint16_t val, uint16_t idx, uint8_t req, void *data, uint16_t len) {
    return ctrl(fd, 0xA1, req, val, idx, data, len);
}

static void log_range(int fd, const char *name, uint16_t val, uint16_t idx, int nbytes) {
    uint8_t cur[4], mn[4], mx[4];
    int rc;
    if (nbytes < 1 || nbytes > 4) nbytes = 2;
    memset(cur, 0, sizeof cur);
    memset(mn, 0, sizeof mn);
    memset(mx, 0, sizeof mx);
    rc = ctrl_get(fd, val, idx, 0x81, cur, (uint16_t)nbytes);
    ctrl_get(fd, val, idx, 0x82, mn, (uint16_t)nbytes);
    ctrl_get(fd, val, idx, 0x83, mx, (uint16_t)nbytes);
    if (nbytes >= 4)
        LOGI("%s cur %u min %u max %u rc %d", name, get_u32(cur), get_u32(mn), get_u32(mx), rc);
    else if (nbytes == 2)
        LOGI("%s cur %u min %u max %u rc %d", name, (unsigned)(cur[0] | (cur[1] << 8)),
             (unsigned)(mn[0] | (mn[1] << 8)), (unsigned)(mx[0] | (mx[1] << 8)), rc);
    else
        LOGI("%s cur %u min %u max %u rc %d", name, cur[0], mn[0], mx[0], rc);
}

/* Desktop defaults: camera A exposure 22 gain 6, camera B exposure 16 gain 4.
 * Applied again after the stream starts because a commit can clear them. */
static void apply_controls(int fd, uint16_t pid, int exposure_100us, int gain) {
    uint8_t mode = 1;
    uint8_t exp[4];
    uint8_t glen[2];
    uint8_t gbuf[4];
    int gl = 2;
    int rc;
    if (exposure_100us < 1) exposure_100us = 1;
    if (exposure_100us > 500) exposure_100us = 500;
    if (gain < 0) gain = 0;
    if (gain > 100) gain = 100;
    rc = ctrl(fd, 0x21, 0x01, (uint16_t)(0x02 << 8), 0x0100, &mode, 1);
    if (rc < 0) LOGI("ae manual not accepted: %s", strerror(errno));
    usleep(20000);
    put_u32(exp, (uint32_t)exposure_100us);
    rc = ctrl(fd, 0x21, 0x01, (uint16_t)(0x04 << 8), 0x0100, exp, 4);
    if (rc < 0) LOGI("exposure %d not accepted: %s", exposure_100us, strerror(errno));
    memset(glen, 0, sizeof glen);
    if (ctrl_get(fd, (uint16_t)(0x04 << 8), 0x0200, 0x85, glen, 2) >= 2) {
        gl = glen[0] | (glen[1] << 8);
        if (gl < 1 || gl > 4) gl = 2;
    }
    memset(gbuf, 0, sizeof gbuf);
    if (gl >= 4) put_u32(gbuf, (uint32_t)gain);
    else if (gl == 2) put_u16(gbuf, (uint16_t)gain);
    else gbuf[0] = (uint8_t)gain;
    rc = ctrl(fd, 0x21, 0x01, (uint16_t)(0x04 << 8), 0x0200, gbuf, (uint16_t)gl);
    if (rc < 0) LOGI("gain %d not accepted: %s", gain, strerror(errno));
    LOGI("pid %04x exposure %d gain %d (gain bytes %d)", pid, exposure_100us, gain, gl);
    log_range(fd, "exposure", (uint16_t)(0x04 << 8), 0x0100, 4);
    log_range(fd, "gain", (uint16_t)(0x04 << 8), 0x0200, gl);
}

static int probe_len(int fd) {
    uint8_t lenb[2];
    int rc = ctrl(fd, 0xA1, 0x85, (uint16_t)(0x01 << 8), 1, lenb, 2);
    int n;
    if (rc < 2) return 26;
    n = lenb[0] | (lenb[1] << 8);
    if (n < 26) return 26;
    if (n > PROBE_MAX) return PROBE_MAX;
    return n;
}

static int xfer_probe(int fd, uint8_t *probe, int n, int commit) {
    uint8_t cs = commit ? 0x02 : 0x01;
    int rc = ctrl(fd, 0x21, 0x01, (uint16_t)(cs << 8), 1, probe, (uint16_t)n);
    if (rc < 0) {
        LOGI("%s probe failed: %s", commit ? "commit" : "set", strerror(errno));
        return -1;
    }
    if (!commit) {
        memset(probe, 0, (size_t)n);
        rc = ctrl(fd, 0xA1, 0x81, (uint16_t)(cs << 8), 1, probe, (uint16_t)n);
        if (rc < 26) {
            LOGI("get probe failed rc=%d %s", rc, rc < 0 ? strerror(errno) : "");
            return -1;
        }
    }
    return 0;
}

static int pick_alt(int payload) {
    int i;
    for (i = 1; i <= 6; i++) {
        if (k_alt_bytes[i] >= payload) return i;
    }
    return 6;
}

static int negotiate(Uvc *dev, int width, int height, int max_payload) {
    uint8_t probe[PROBE_MAX];
    struct usbdevfs_setinterface si;
    int n, frame, want, got, alt;
    frame = 2;
    if (width == 640 && height == 480) frame = 1;
    else if (width == 1024 && height == 576) frame = 3;
    else if (width == 800 && height == 600) frame = 4;
    else if (width == 320 && height == 240) frame = 5;
    else if (width != 1280 || height != 720)
        LOGI("uvc size %dx%d is not a Fox frame; requesting 1280x720", width, height);

    memset(&si, 0, sizeof si);
    si.interface = 1;
    si.altsetting = 0;
    if (ioctl(dev->fd, USBDEVFS_SETINTERFACE, &si) < 0)
        LOGI("alt 0: %s", strerror(errno));

    n = probe_len(dev->fd);
    want = max_payload > 0 ? max_payload : 3072;
    memset(probe, 0, sizeof probe);
    put_u16(probe + 0, 1);
    probe[2] = 1;
    probe[3] = (uint8_t)frame;
    put_u32(probe + 4, 1000000);
    put_u32(probe + 22, (uint32_t)want);
    if (xfer_probe(dev->fd, probe, n, 0) != 0) {
        if (n >= PROBE_MAX) {
            set_err(ERR_NEG, "probe failed");
            return -1;
        }
        n = PROBE_MAX;
        memset(probe, 0, sizeof probe);
        put_u16(probe + 0, 1);
        probe[2] = 1;
        probe[3] = (uint8_t)frame;
        put_u32(probe + 4, 1000000);
        put_u32(probe + 22, (uint32_t)want);
        if (xfer_probe(dev->fd, probe, n, 0) != 0) {
            set_err(ERR_NEG, "probe failed");
            return -1;
        }
    }
    got = (int)get_u32(probe + 22);
    LOGI("uvc probe fmt %u frame %u interval %u payload %d framebytes %u (cap %d, len %d)",
         probe[2], probe[3], get_u32(probe + 4), got, get_u32(probe + 18), want, n);
    if (probe[2] != 1 || probe[3] != (uint8_t)frame) {
        set_err(ERR_NEG, "device refused 1280x720 mjpeg");
        return -1;
    }
    if (got < 128) got = want;
    if (got > want) {
        char msg[80];
        snprintf(msg, sizeof msg, "payload %d above cap %d", got, want);
        set_err(ERR_BAND, msg);
        return -1;
    }
    alt = pick_alt(got);
    if (xfer_probe(dev->fd, probe, n, 1) != 0) {
        set_err(ERR_NEG, "commit failed");
        return -1;
    }
    si.altsetting = (unsigned)alt;
    if (ioctl(dev->fd, USBDEVFS_SETINTERFACE, &si) < 0) {
        char msg[80];
        snprintf(msg, sizeof msg, "alt %d %s", alt, strerror(errno));
        set_err(ERR_BAND, msg);
        return -1;
    }
    dev->alt = alt;
    dev->payload = got;
    dev->pkt = k_alt_bytes[alt];
    dev->pkt_base = k_alt_base[alt];
    LOGI("uvc streaming alt %d pkt %d pid %04x", alt, dev->pkt, dev->pid);
    return 0;
}

static int submit_one(int fd, struct usbdevfs_urb *u, int pkt) {
    int i;
    u->status = 0;
    u->flags = USBDEVFS_URB_ISO_ASAP;
    u->buffer_length = pkt * NPKT;
    u->actual_length = 0;
    u->start_frame = 0;
    u->number_of_packets = NPKT;
    u->error_count = 0;
    u->signr = 0;
    for (i = 0; i < NPKT; i++) {
        u->iso_frame_desc[i].length = (unsigned)pkt;
        u->iso_frame_desc[i].actual_length = 0;
        u->iso_frame_desc[i].status = 0;
    }
    return ioctl(fd, USBDEVFS_SUBMITURB, u);
}

static void drain_urbs(int fd, struct usbdevfs_urb **urbs) {
    int i;
    for (i = 0; i < NURB; i++) {
        if (urbs[i]) ioctl(fd, USBDEVFS_DISCARDURB, urbs[i]);
    }
    for (;;) {
        struct usbdevfs_urb *done = NULL;
        if (ioctl(fd, USBDEVFS_REAPURBNDELAY, &done) < 0) break;
    }
}

static int submit_all(Uvc *dev, int pkt) {
    int i, n = 0, err;
    for (i = 0; i < NURB; i++) {
        if (submit_one(dev->fd, dev->urbs[i], pkt) != 0) {
            err = errno;
            LOGI("submit pkt %d urb %d: %s", pkt, i, strerror(err));
            errno = err;
            break;
        }
        n++;
    }
    if (n == NURB) return 0;
    drain_urbs(dev->fd, dev->urbs);
    return -1;
}

static int jpeg_soi(const uint8_t *p, int n, int *off) {
    int i, lim = n < 32 ? n - 1 : 32;
    if (n < 4) return 0;
    for (i = 0; i < lim; i++) {
        if (p[i] == 0xFF && p[i + 1] == 0xD8) {
            *off = i;
            return 1;
        }
    }
    return 0;
}

static void publish_jpeg(Uvc *dev) {
    int off = 0;
    if (!jpeg_soi(dev->acc, dev->acc_len, &off)) return;
    if (dev->acc_len - off < 1024 || dev->acc_len - off > JPEG_CAP) return;
    {
        int nframe = dev->frames + 1;
        int stamp = dev->stamp_seq != dev->stamp_seen;
        if (nframe == 1 || (nframe % 20) == 0 || stamp) {
            char path[96];
            FILE *f;
            snprintf(path, sizeof path, "/data/data/dev.lab.lizard/files/fox-%04x.jpg", dev->pid);
            f = fopen(path, "wb");
            if (f) {
                fwrite(dev->acc + off, 1, (size_t)(dev->acc_len - off), f);
                fclose(f);
                if (nframe == 1 || stamp)
                    LOGI("saved %s (%d) frame %d", path, dev->acc_len - off, nframe);
            }
            if (stamp && dev->stamp_n < 36) {
                snprintf(path, sizeof path, "/data/data/dev.lab.lizard/files/fox-%04x-e%04d-g%03d.jpg",
                         dev->pid, dev->exposure, dev->gain);
                f = fopen(path, "wb");
                if (f) {
                    fwrite(dev->acc + off, 1, (size_t)(dev->acc_len - off), f);
                    fclose(f);
                    dev->stamp_n++;
                    LOGI("saved %s (%d)", path, dev->acc_len - off);
                }
            }
            if (stamp) dev->stamp_seen = dev->stamp_seq;
        }
    }
    pthread_mutex_lock(&dev->mu);
    memcpy(dev->held, dev->acc + off, (size_t)(dev->acc_len - off));
    dev->held_len = dev->acc_len - off;
    dev->seq++;
    dev->frames++;
    if (dev->frames == 1) LOGI("uvc pid %04x first jpeg %d", dev->pid, dev->held_len);
    pthread_mutex_unlock(&dev->mu);
}

static void consume_packet(Uvc *dev, const uint8_t *data, int actual) {
    int hlen, info, plen, fid;
    const uint8_t *payload;
    if (actual < 2) return;
    hlen = data[0];
    if (hlen < 2 || hlen > actual) return;
    info = data[1];
    if (info & 0x40) {
        dev->acc_len = 0;
        dev->have_fid = 0;
        return;
    }
    fid = info & 1;
    if (dev->have_fid && fid != dev->fid) dev->acc_len = 0;
    dev->fid = fid;
    dev->have_fid = 1;
    payload = data + hlen;
    plen = actual - hlen;
    if (plen > 0) {
        if (dev->acc_len + plen > JPEG_CAP) {
            dev->acc_len = 0;
            return;
        }
        memcpy(dev->acc + dev->acc_len, payload, (size_t)plen);
        dev->acc_len += plen;
    }
    if (info & 0x02) {
        publish_jpeg(dev);
        dev->acc_len = 0;
    }
}

static void consume_urb(Uvc *dev, struct usbdevfs_urb *u) {
    uint8_t *base = (uint8_t *)u->buffer;
    int i;
    for (i = 0; i < u->number_of_packets; i++) {
        int actual = (int)u->iso_frame_desc[i].actual_length;
        int slot = (int)u->iso_frame_desc[i].length;
        if (actual > 0) consume_packet(dev, base, actual);
        if (slot > 0) base += slot;
    }
}

/* Alt 0 stops the sensor. The factory script latches exposure when the
 * stream comes back, which a live SET_CUR does not do on this bridge. */
static int cycle_stream(Uvc *dev, int pkt) {
    struct usbdevfs_setinterface si;
    drain_urbs(dev->fd, dev->urbs);
    memset(&si, 0, sizeof si);
    si.interface = 1;
    si.altsetting = 0;
    if (ioctl(dev->fd, USBDEVFS_SETINTERFACE, &si) < 0)
        LOGI("pid %04x restream alt 0: %s", dev->pid, strerror(errno));
    apply_controls(dev->fd, dev->pid, dev->exposure, dev->gain);
    si.altsetting = (unsigned)dev->alt;
    if (ioctl(dev->fd, USBDEVFS_SETINTERFACE, &si) < 0) {
        LOGE("pid %04x restream alt %d: %s", dev->pid, dev->alt, strerror(errno));
        dev->dead = 1;
        return -1;
    }
    if (submit_all(dev, pkt) != 0) {
        LOGE("pid %04x restream submit failed", dev->pid);
        dev->dead = 1;
        return -1;
    }
    LOGI("pid %04x restreamed exp %d gain %d", dev->pid, dev->exposure, dev->gain);
    return 0;
}

static void *stream_main(void *arg) {
    Uvc *dev = arg;
    int pkt = dev->pkt;
    pthread_setname_np(pthread_self(), "lizard-uvc");
    if (dev->stop) goto out;
    if (submit_all(dev, pkt) != 0) {
        int err = errno;
        if (err == EINVAL && dev->pkt_base > 0 && dev->pkt_base != pkt && submit_all(dev, dev->pkt_base) == 0) {
            pkt = dev->pkt_base;
            dev->pkt = pkt;
            LOGI("uvc pid %04x using base packet %d", dev->pid, pkt);
        } else {
            LOGE("uvc pid %04x isoc submit failed: %s", dev->pid, strerror(err));
            dev->dead = 1;
            goto out;
        }
    }
    LOGI("uvc pid %04x urbs queued pkt %d", dev->pid, pkt);
    while (!dev->stop) {
        struct usbdevfs_urb *done = NULL;
        if (ioctl(dev->fd, USBDEVFS_REAPURBNDELAY, &done) < 0) {
            if (errno == EAGAIN || errno == EINTR) {
                usleep(1000);
                continue;
            }
            LOGE("uvc pid %04x reap: %s", dev->pid, strerror(errno));
            dev->dead = 1;
            break;
        }
        if (!done) continue;
        if (dev->stop) break;
        consume_urb(dev, done);
        if (dev->restream) {
            dev->restream = 0;
            if (cycle_stream(dev, pkt) != 0) break;
            continue;
        }
        if (submit_one(dev->fd, done, pkt) != 0) {
            LOGE("uvc pid %04x resubmit: %s", dev->pid, strerror(errno));
            dev->dead = 1;
            break;
        }
    }
out:
    drain_urbs(dev->fd, dev->urbs);
    pthread_mutex_lock(&dev->mu);
    dev->dead = 1;
    pthread_mutex_unlock(&dev->mu);
    LOGI("uvc pid %04x stream stop frames %d", dev->pid, dev->frames);
    return NULL;
}

static int alloc_urbs(Uvc *dev) {
    int i;
    int pkt = dev->pkt > dev->pkt_base ? dev->pkt : dev->pkt_base;
    size_t usz = sizeof(struct usbdevfs_urb) + (size_t)NPKT * sizeof(struct usbdevfs_iso_packet_desc);
    for (i = 0; i < NURB; i++) {
        struct usbdevfs_urb *u = (struct usbdevfs_urb *)calloc(1, usz);
        if (!u) return -1;
        u->type = USBDEVFS_URB_TYPE_ISO;
        u->endpoint = 0x81;
        u->buffer = malloc((size_t)pkt * NPKT);
        if (!u->buffer) {
            free(u);
            return -1;
        }
        u->usercontext = dev;
        dev->urbs[i] = u;
    }
    return 0;
}

static jobject find_device(JNIEnv *env, uint16_t vid, uint16_t pid, char *serial, size_t serial_n) {
    jobject map, values, found = NULL;
    jobjectArray arr;
    jsize i, n;
    map = (*env)->CallObjectMethod(env, g_usb, m_dev_list);
    if (!jni_ok(env, "getDeviceList") || !map) return NULL;
    values = (*env)->CallObjectMethod(env, map, m_values);
    if (!jni_ok(env, "values") || !values) {
        (*env)->DeleteLocalRef(env, map);
        return NULL;
    }
    arr = (jobjectArray)(*env)->CallObjectMethod(env, values, m_to_array);
    (*env)->DeleteLocalRef(env, values);
    (*env)->DeleteLocalRef(env, map);
    if (!jni_ok(env, "toArray") || !arr) return NULL;
    n = (*env)->GetArrayLength(env, arr);
    for (i = 0; i < n; i++) {
        jobject dev = (*env)->GetObjectArrayElement(env, arr, i);
        int dv, dp, perm;
        jstring name;
        const char *ns;
        if (!dev) continue;
        dv = (*env)->CallIntMethod(env, dev, m_vendor);
        dp = (*env)->CallIntMethod(env, dev, m_product);
        perm = (*env)->CallBooleanMethod(env, g_usb, m_has_perm, dev);
        name = (*env)->CallObjectMethod(env, dev, m_devname);
        ns = name ? (*env)->GetStringUTFChars(env, name, NULL) : NULL;
        LOGI("usb %04x:%04x perm %d %s", dv, dp, perm, ns ? ns : "?");
        if (ns) (*env)->ReleaseStringUTFChars(env, name, ns);
        if (name) (*env)->DeleteLocalRef(env, name);
        if (!jni_ok(env, "device")) {
            (*env)->DeleteLocalRef(env, dev);
            continue;
        }
        if (dv == (int)vid && dp == (int)pid && !found) {
            jstring js;
            const char *ss;
            if (!perm) {
                set_err(ERR_DENIED, "Fox USB permission missing");
                (*env)->DeleteLocalRef(env, dev);
                (*env)->DeleteLocalRef(env, arr);
                return NULL;
            }
            serial[0] = 0;
            js = (*env)->CallObjectMethod(env, dev, m_serial);
            if (!jni_ok(env, "serial")) js = NULL;
            ss = js ? (*env)->GetStringUTFChars(env, js, NULL) : NULL;
            if (ss) {
                snprintf(serial, serial_n, "%s", ss);
                (*env)->ReleaseStringUTFChars(env, js, ss);
            }
            if (js) (*env)->DeleteLocalRef(env, js);
            found = dev;
            continue;
        }
        (*env)->DeleteLocalRef(env, dev);
    }
    (*env)->DeleteLocalRef(env, arr);
    return found;
}

static jclass keep_class(JNIEnv *env, const char *name) {
    jclass local = (*env)->FindClass(env, name);
    jclass global;
    if (!local || !jni_ok(env, name)) {
        LOGE("uvc class %s", name);
        if (local) (*env)->DeleteLocalRef(env, local);
        return NULL;
    }
    global = (jclass)(*env)->NewGlobalRef(env, local);
    (*env)->DeleteLocalRef(env, local);
    return global;
}

int uvc_init(void *vm, void *activity) {
    JNIEnv *env;
    jclass act, um, ud, uc, hm, col, opt, cfg;
    jstring usb;
    jobject mgr, argb, options;
    jfieldID argb_fid;
    if (g_usb) return 0;
    g_vm = (JavaVM *)vm;
    env = jni_env();
    if (!env || !activity) {
        LOGE("uvc init no jni");
        return -1;
    }
    if (!g_jni_key_ok && pthread_key_create(&g_jni_key, jni_detach) == 0) g_jni_key_ok = 1;
    act = (*env)->FindClass(env, "android/content/Context");
    if (!act || !jni_ok(env, "Context")) return -1;
    m_get_service = (*env)->GetMethodID(env, act, "getSystemService", "(Ljava/lang/String;)Ljava/lang/Object;");
    (*env)->DeleteLocalRef(env, act);
    um = keep_class(env, "android/hardware/usb/UsbManager");
    ud = keep_class(env, "android/hardware/usb/UsbDevice");
    uc = keep_class(env, "android/hardware/usb/UsbDeviceConnection");
    hm = keep_class(env, "java/util/HashMap");
    col = keep_class(env, "java/util/Collection");
    g_factory = keep_class(env, "android/graphics/BitmapFactory");
    opt = keep_class(env, "android/graphics/BitmapFactory$Options");
    g_bitmap = keep_class(env, "android/graphics/Bitmap");
    cfg = keep_class(env, "android/graphics/Bitmap$Config");
    if (!m_get_service || !um || !ud || !uc || !hm || !col || !g_factory || !opt || !g_bitmap || !cfg) return -1;
    m_dev_list = (*env)->GetMethodID(env, um, "getDeviceList", "()Ljava/util/HashMap;");
    m_has_perm = (*env)->GetMethodID(env, um, "hasPermission", "(Landroid/hardware/usb/UsbDevice;)Z");
    m_open_dev = (*env)->GetMethodID(env, um, "openDevice",
                                     "(Landroid/hardware/usb/UsbDevice;)Landroid/hardware/usb/UsbDeviceConnection;");
    m_values = (*env)->GetMethodID(env, hm, "values", "()Ljava/util/Collection;");
    m_to_array = (*env)->GetMethodID(env, col, "toArray", "()[Ljava/lang/Object;");
    m_vendor = (*env)->GetMethodID(env, ud, "getVendorId", "()I");
    m_product = (*env)->GetMethodID(env, ud, "getProductId", "()I");
    m_serial = (*env)->GetMethodID(env, ud, "getSerialNumber", "()Ljava/lang/String;");
    m_devname = (*env)->GetMethodID(env, ud, "getDeviceName", "()Ljava/lang/String;");
    m_get_fd = (*env)->GetMethodID(env, uc, "getFileDescriptor", "()I");
    m_conn_close = (*env)->GetMethodID(env, uc, "close", "()V");
    m_decode = (*env)->GetStaticMethodID(env, g_factory, "decodeByteArray",
                                         "([BIILandroid/graphics/BitmapFactory$Options;)Landroid/graphics/Bitmap;");
    m_opt_init = (*env)->GetMethodID(env, opt, "<init>", "()V");
    f_config = (*env)->GetFieldID(env, opt, "inPreferredConfig", "Landroid/graphics/Bitmap$Config;");
    f_scaled = (*env)->GetFieldID(env, opt, "inScaled", "Z");
    m_get_pixels = (*env)->GetMethodID(env, g_bitmap, "getPixels", "([IIIIIII)V");
    m_recycle = (*env)->GetMethodID(env, g_bitmap, "recycle", "()V");
    m_width = (*env)->GetMethodID(env, g_bitmap, "getWidth", "()I");
    m_height = (*env)->GetMethodID(env, g_bitmap, "getHeight", "()I");
    argb_fid = (*env)->GetStaticFieldID(env, cfg, "ARGB_8888", "Landroid/graphics/Bitmap$Config;");
    if (!jni_ok(env, "methods") || !m_dev_list || !m_has_perm || !m_open_dev || !m_values || !m_to_array ||
        !m_vendor || !m_product || !m_serial || !m_devname || !m_get_fd || !m_conn_close || !m_decode ||
        !m_opt_init || !f_config || !f_scaled || !m_get_pixels || !m_recycle || !m_width || !m_height || !argb_fid) {
        LOGE("uvc method lookup failed");
        return -1;
    }
    argb = (*env)->GetStaticObjectField(env, cfg, argb_fid);
    if (!argb || !jni_ok(env, "ARGB_8888")) return -1;
    g_argb = (*env)->NewGlobalRef(env, argb);
    (*env)->DeleteLocalRef(env, argb);
    options = (*env)->NewObject(env, opt, m_opt_init);
    if (!options || !jni_ok(env, "options")) return -1;
    (*env)->SetObjectField(env, options, f_config, g_argb);
    (*env)->SetBooleanField(env, options, f_scaled, JNI_FALSE);
    g_options = (*env)->NewGlobalRef(env, options);
    (*env)->DeleteLocalRef(env, options);
    usb = (*env)->NewStringUTF(env, "usb");
    mgr = (*env)->CallObjectMethod(env, (jobject)activity, m_get_service, usb);
    (*env)->DeleteLocalRef(env, usb);
    if (!mgr || !jni_ok(env, "usb service")) {
        LOGE("uvc no UsbManager");
        return -1;
    }
    g_usb = (*env)->NewGlobalRef(env, mgr);
    (*env)->DeleteLocalRef(env, mgr);
    LOGI("uvc jni ready");
    return 0;
}

static void free_half(Uvc *dev) {
    int i;
    if (!dev) return;
    for (i = 0; i < NURB; i++) {
        if (!dev->urbs[i]) continue;
        free(dev->urbs[i]->buffer);
        free(dev->urbs[i]);
        dev->urbs[i] = NULL;
    }
    free(dev->acc);
    free(dev->held);
    free(dev->scratch);
    dev->acc = dev->held = dev->scratch = NULL;
}

void uvc_close(Uvc *dev) {
    JNIEnv *env;
    if (!dev) return;
    dev->stop = 1;
    if (dev->thread_on) {
        pthread_join(dev->thread, NULL);
        dev->thread_on = 0;
    }
    if (dev->fd >= 0) {
        struct usbdevfs_setinterface si;
        unsigned int iface;
        memset(&si, 0, sizeof si);
        si.interface = 1;
        si.altsetting = 0;
        ioctl(dev->fd, USBDEVFS_SETINTERFACE, &si);
        iface = 1;
        ioctl(dev->fd, USBDEVFS_RELEASEINTERFACE, &iface);
        iface = 0;
        ioctl(dev->fd, USBDEVFS_RELEASEINTERFACE, &iface);
        close(dev->fd);
        dev->fd = -1;
    }
    env = jni_env();
    if (env && dev->connection) {
        (*env)->CallVoidMethod(env, dev->connection, m_conn_close);
        jni_ok(env, "connection.close");
        (*env)->DeleteGlobalRef(env, dev->connection);
        dev->connection = NULL;
    }
    if (env && dev->pixels) {
        (*env)->DeleteGlobalRef(env, dev->pixels);
        dev->pixels = NULL;
    }
    free_half(dev);
    pthread_mutex_destroy(&dev->mu);
    free(dev);
}

Uvc *uvc_open(uint16_t vid, uint16_t pid, int width, int height, int max_payload) {
    JNIEnv *env;
    jobject jdev, conn;
    Uvc *dev;
    int jfd;
    t_err = ERR_OK;
    t_errstr[0] = 0;
    if (!g_usb) {
        set_err(ERR_ABSENT, "uvc not initialised");
        return NULL;
    }
    env = jni_env();
    if (!env) {
        set_err(ERR_ABSENT, "no jni on worker");
        return NULL;
    }
    dev = (Uvc *)calloc(1, sizeof *dev);
    if (!dev) return NULL;
    dev->fd = -1;
    dev->vid = vid;
    dev->pid = pid;
    dev->width = width;
    dev->height = height;
    dev->fid = -1;
    pthread_mutex_init(&dev->mu, NULL);
    jdev = find_device(env, vid, pid, dev->serial, sizeof dev->serial);
    if (!jdev) {
        if (!t_err) {
            char msg[64];
            snprintf(msg, sizeof msg, "Fox %04x:%04x absent", vid, pid);
            set_err(ERR_ABSENT, msg);
        }
        uvc_close(dev);
        return NULL;
    }
    conn = (*env)->CallObjectMethod(env, g_usb, m_open_dev, jdev);
    (*env)->DeleteLocalRef(env, jdev);
    if (!conn || !jni_ok(env, "openDevice")) {
        set_err(ERR_DENIED, "UsbManager openDevice failed");
        if (conn) (*env)->DeleteLocalRef(env, conn);
        uvc_close(dev);
        return NULL;
    }
    dev->connection = (*env)->NewGlobalRef(env, conn);
    (*env)->DeleteLocalRef(env, conn);
    jfd = (*env)->CallIntMethod(env, dev->connection, m_get_fd);
    if (!jni_ok(env, "fd") || jfd < 0) {
        set_err(ERR_DENIED, "no usbfs fd");
        uvc_close(dev);
        return NULL;
    }
    dev->fd = dup(jfd);
    if (dev->fd < 0) {
        set_err(ERR_DENIED, "dup fd failed");
        uvc_close(dev);
        return NULL;
    }
    LOGI("uvc open %04x:%04x serial %s fd %d", vid, pid, dev->serial[0] ? dev->serial : "?", dev->fd);
    if (claim_if(dev->fd, 0) != 0 || claim_if(dev->fd, 1) != 0) {
        set_err(ERR_DENIED, "claim interface failed");
        uvc_close(dev);
        return NULL;
    }
    dev->claimed = 1;
    dev->exposure = 4;
    dev->gain = dev->pid == 0x636b ? 2 : 8;
    apply_controls(dev->fd, dev->pid, dev->exposure, dev->gain);
    if (negotiate(dev, width, height, max_payload) != 0) {
        uvc_close(dev);
        return NULL;
    }
    apply_controls(dev->fd, dev->pid, dev->exposure, dev->gain);
    dev->acc = (uint8_t *)malloc(JPEG_CAP);
    dev->held = (uint8_t *)malloc(JPEG_CAP);
    dev->scratch = (uint8_t *)malloc(JPEG_CAP);
    if (!dev->acc || !dev->held || !dev->scratch || alloc_urbs(dev) != 0) {
        set_err(ERR_NEG, "out of memory");
        uvc_close(dev);
        return NULL;
    }
    if (pthread_create(&dev->thread, NULL, stream_main, dev) != 0) {
        set_err(ERR_NEG, "stream thread failed");
        uvc_close(dev);
        return NULL;
    }
    dev->thread_on = 1;
    snprintf(dev->status, sizeof dev->status, "%04x alt%d", dev->pid, dev->alt);
    return dev;
}

int uvc_set_exposure_gain(Uvc *dev, int exposure_100us, int gain) {
    if (!dev || dev->fd < 0) return -1;
    if (exposure_100us < 1) exposure_100us = 1;
    if (exposure_100us > 500) exposure_100us = 500;
    if (gain < 0) gain = 0;
    if (gain > 100) gain = 100;
    dev->exposure = exposure_100us;
    dev->gain = gain;
    {
        uint8_t exp[4];
        uint8_t gbuf[2];
        put_u32(exp, (uint32_t)exposure_100us);
        ctrl(dev->fd, 0x21, 0x01, (uint16_t)(0x04 << 8), 0x0100, exp, 4);
        put_u16(gbuf, (uint16_t)gain);
        ctrl(dev->fd, 0x21, 0x01, (uint16_t)(0x04 << 8), 0x0200, gbuf, 2);
    }
    dev->stamp_seq++;
    LOGI("pid %04x exposure %d gain %d", dev->pid, exposure_100us, gain);
    return 0;
}

void uvc_restream(Uvc *dev) {
    if (!dev || dev->fd < 0 || dev->dead) return;
    dev->restream = 1;
}

const char *uvc_status(Uvc *dev) {
    if (!dev) return "none";
    snprintf(dev->status, sizeof dev->status, "%04x alt%d pay%d jpeg%d f%d%s", dev->pid, dev->alt, dev->payload,
             dev->held_len, dev->frames, dev->dead ? " dead" : "");
    return dev->status;
}

static uint8_t clamp_u8(int v) {
    if (v < 0) return 0;
    if (v > 255) return 255;
    return (uint8_t)v;
}

static void planes_from_argb(const jint *px, int bw, int bh, uint8_t *y, uint8_t *u, uint8_t *v, int w, int h,
                             int want_uv) {
    int row, col;
    for (row = 0; row < h; row++) {
        int sy = (bh == h) ? row : (int)((long)row * bh / h);
        for (col = 0; col < w; col++) {
            int sx = (bw == w) ? col : (int)((long)col * bw / w);
            jint p = px[(size_t)sy * (size_t)bw + (size_t)sx];
            int R = (p >> 16) & 255;
            int G = (p >> 8) & 255;
            int B = p & 255;
            int Y;
            /* Camera A is a gray lattice. Studio swing (220/256 plus 16) shrinks
             * the peaks until the stripe gate drops the frame. Full range keeps
             * a gray sample equal to itself. Camera B stays studio swing because
             * the colour sampler subtracts 16. */
            if (want_uv)
                Y = ((66 * R + 129 * G + 25 * B + 128) >> 8) + 16;
            else
                Y = (77 * R + 150 * G + 29 * B + 128) >> 8;
            y[(size_t)row * (size_t)w + (size_t)col] = clamp_u8(Y);
            if (want_uv && ((col | row) & 1) == 0) {
                int U = ((-38 * R - 74 * G + 112 * B + 128) >> 8) + 128;
                int V = ((112 * R - 94 * G - 18 * B + 128) >> 8) + 128;
                int uw = w / 2;
                size_t ui = (size_t)(row / 2) * (size_t)uw + (size_t)(col / 2);
                u[ui] = clamp_u8(U);
                v[ui] = clamp_u8(V);
            }
        }
    }
}

int uvc_grab(Uvc *dev, uint8_t *y, uint8_t *u, uint8_t *v, int w, int h, int want_uv) {
    JNIEnv *env;
    int n, seq, bw, bh;
    jbyteArray jbytes = NULL;
    jobject bmp = NULL;
    jintArray pixels = NULL;
    jint *px = NULL;
    int rc = -1;
    if (!dev || !y || w < 2 || h < 2) return -2;
    pthread_mutex_lock(&dev->mu);
    seq = dev->seq;
    if (seq < 1) {
        rc = dev->dead ? -2 : -1;
        pthread_mutex_unlock(&dev->mu);
        return rc;
    }
    if (seq == dev->decoded_seq) {
        rc = dev->bad ? -1 : (dev->dead ? -2 : 1);
        pthread_mutex_unlock(&dev->mu);
        return rc;
    }
    n = dev->held_len;
    if (n < 1024 || n > JPEG_CAP) {
        dev->decoded_seq = seq;
        dev->bad = 1;
        pthread_mutex_unlock(&dev->mu);
        return dev->dead ? -2 : -1;
    }
    memcpy(dev->scratch, dev->held, (size_t)n);
    dev->decoded_seq = seq;
    pthread_mutex_unlock(&dev->mu);

    env = jni_env();
    if (!env) {
        dev->bad = 1;
        return -1;
    }
    jbytes = (*env)->NewByteArray(env, n);
    if (!jbytes) {
        dev->bad = 1;
        return -1;
    }
    (*env)->SetByteArrayRegion(env, jbytes, 0, n, (const jbyte *)dev->scratch);
    bmp = (*env)->CallStaticObjectMethod(env, g_factory, m_decode, jbytes, 0, n, g_options);
    if (!jni_ok(env, "decode") || !bmp) {
        LOGI("uvc pid %04x jpeg %d did not decode", dev->pid, n);
        (*env)->DeleteLocalRef(env, jbytes);
        dev->bad = 1;
        return dev->dead ? -2 : -1;
    }
    bw = (*env)->CallIntMethod(env, bmp, m_width);
    bh = (*env)->CallIntMethod(env, bmp, m_height);
    if (!jni_ok(env, "bounds") || bw < 2 || bh < 2 || bw > 4096 || bh > 4096) {
        (*env)->CallVoidMethod(env, bmp, m_recycle);
        (*env)->DeleteLocalRef(env, bmp);
        (*env)->DeleteLocalRef(env, jbytes);
        dev->bad = 1;
        return -1;
    }
    if (bw != w || bh != h)
        LOGI("uvc pid %04x jpeg is %dx%d, calib is %dx%d", dev->pid, bw, bh, w, h);
    if (!dev->pixels || dev->pixels_n != bw * bh) {
        jintArray local = (*env)->NewIntArray(env, bw * bh);
        if (dev->pixels) (*env)->DeleteGlobalRef(env, dev->pixels);
        dev->pixels = local ? (*env)->NewGlobalRef(env, local) : NULL;
        if (local) (*env)->DeleteLocalRef(env, local);
        dev->pixels_n = dev->pixels ? bw * bh : 0;
    }
    pixels = (jintArray)dev->pixels;
    if (!pixels) {
        (*env)->CallVoidMethod(env, bmp, m_recycle);
        (*env)->DeleteLocalRef(env, bmp);
        (*env)->DeleteLocalRef(env, jbytes);
        dev->bad = 1;
        return -1;
    }
    (*env)->CallVoidMethod(env, bmp, m_get_pixels, pixels, 0, bw, 0, 0, bw, bh);
    if (!jni_ok(env, "getPixels")) {
        (*env)->CallVoidMethod(env, bmp, m_recycle);
        (*env)->DeleteLocalRef(env, bmp);
        (*env)->DeleteLocalRef(env, jbytes);
        dev->bad = 1;
        return -1;
    }
    px = (*env)->GetIntArrayElements(env, pixels, NULL);
    if (!px) {
        dev->bad = 1;
        rc = -1;
    } else {
        planes_from_argb(px, bw, bh, y, u, v, w, h, want_uv);
        (*env)->ReleaseIntArrayElements(env, pixels, px, JNI_ABORT);
        dev->bad = 0;
        rc = 0;
        if (dev->frames < 3) {
            jint p0 = 0;
            /* logged from the converted plane so a black frame is visible */
            LOGI("uvc pid %04x frame %d y0 %u", dev->pid, dev->frames, y[0]);
            (void)p0;
        }
    }
    (*env)->CallVoidMethod(env, bmp, m_recycle);
    jni_ok(env, "recycle");
    (*env)->DeleteLocalRef(env, bmp);
    (*env)->DeleteLocalRef(env, jbytes);
    return rc;
}
