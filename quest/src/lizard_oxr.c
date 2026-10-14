/* LizardTech — Quest 3 passthrough scanner.
 * OpenXR headers on this machine are 1.1.60. The Quest runtime speaks 1.0,
 * which is the version this session requests. The loader is the Meta Android
 * loader shipped beside meta-openxr-sdk v78.
 * The Fox sits on the headset. Head pose places each lattice frame.
 * A world-locked cube is the object. Touch controllers place it from a menu.
 * Hand tracking only masks the scan and reports the hand in the camera frame.
 * Camera B colours the points. */
#define XR_USE_PLATFORM_ANDROID 1
#define XR_USE_GRAPHICS_API_OPENGL_ES 1

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES3/gl3.h>
#include <android/asset_manager.h>
#include <android/log.h>
#include <android_native_app_glue.h>
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>
#include <math.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/system_properties.h>
#include <time.h>
#include <unistd.h>

#include "lizard_cam.h"
#include "lizard_color.h"
#include "lizard_cube.h"
#include "lizard_menu.h"
#include "lizard_mold.h"
#include "lizard_pose.h"
#include "lizard_settings.h"
#include "os_stripe.h"

/* Same rule as src/image/os_pattern.c. Linked in, not copied. */
void openscan_pattern_meter(const uint8_t *y, int w, int h, int *cols, int *clip, int *p50, int *p95);
int openscan_pattern_rescue(int cols, int clip, int p95, int exposure, int *next);

#ifndef EGL_OPENGL_ES3_BIT_KHR
#define EGL_OPENGL_ES3_BIT_KHR 0x0040
#endif

#define TAG "lizard"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, TAG, __VA_ARGS__)

#define MAX_IMG 8
#define LIVE_CAP 40000

typedef struct {
    float *vert;
    int nvert;
    unsigned *idx;
    int nidx;
    float *live;
    int nlive;
    int serial;
} Draw;

static PFN_xrCreatePassthroughFB pfn_create_pt;
static PFN_xrDestroyPassthroughFB pfn_destroy_pt;
static PFN_xrPassthroughStartFB pfn_start_pt;
static PFN_xrCreatePassthroughLayerFB pfn_create_layer;
static PFN_xrDestroyPassthroughLayerFB pfn_destroy_layer;
static PFN_xrPassthroughLayerResumeFB pfn_resume_layer;
static PFN_xrCreateHandTrackerEXT pfn_create_hand;
static PFN_xrDestroyHandTrackerEXT pfn_destroy_hand;
static PFN_xrLocateHandJointsEXT pfn_locate_hands;
static PFN_xrResumeSimultaneousHandsAndControllersTrackingMETA pfn_hands_both;
static int want_hands_both;
static int hands_both_done;

static struct {
    struct android_app *app;
    openscan_calib cal;
    int have_cal;
    LzSettings set;
    LzMount mount;
    char dir[512];
    LzCam *cam;
    LzMold *mold;
    pthread_t worker;
    int worker_on;
    volatile int stop;
    pthread_mutex_t mu;
    LzPose head;
    int head_ok;
    LzJoint hands[52];
    int nhands;
    LzCube cube;
    int cube_reset;
    Draw *draw;
    int pts_last, confirmed, tris, seq_last, cols, stored;
    int export_req;
    int export_ui;
    double exp_hold_until;
    char line[512];
} G;

static uint8_t g_menu_src[LZ_MENU_TEX_W * LZ_MENU_TEX_H * 4];
static uint8_t g_menu_flip[LZ_MENU_TEX_W * LZ_MENU_TEX_H * 4];

static int g_resumed;

static int prop_int(const char *key, int fallback) {
    char b[PROP_VALUE_MAX];
    if (__system_property_get(key, b) <= 0 || !b[0]) return fallback;
    return atoi(b);
}

static int prop_has(const char *key, int *out) {
    char b[PROP_VALUE_MAX];
    if (__system_property_get(key, b) <= 0 || !b[0]) return 0;
    if (out) *out = atoi(b);
    return 1;
}

static void settings_boot(void) {
    int v;
    char path[640];
    lz_settings_default(&G.set);
    if (prop_has("persist.lab.lizard.flip", &v)) G.set.flip = v ? 1 : 0;
    if (prop_has("persist.lab.lizard.stride", &v)) G.set.stride = v;
    if (prop_has("persist.lab.lizard.distance", &v)) G.set.distance = v;
    lz_settings_clamp(&G.set);
    if (G.dir[0]) {
        snprintf(path, sizeof path, "%s/lizard-settings.txt", G.dir);
        if (lz_settings_load(path, &G.set) == 0) LOGI("settings loaded");
    }
    LOGI("settings A %d/%d B %d/%d dist %d near %d far %d stride %d smooth %d sweep %d relief %d solid %d flip %d",
         G.set.ea, G.set.ga, G.set.eb, G.set.gb, G.set.distance, G.set.near_mm, G.set.far_mm, G.set.stride, G.set.smooth,
         G.set.sweep, G.set.relief, G.set.solid, G.set.flip);
    {
        LzMenuView v;
        int i;
        memset(&v, 0, sizeof v);
        lz_menu_view(&v, 0, 0, 0, (int)(G.cube.half * 2000.f + 0.5f), 0, 0, 0, 0, 0, 0.f, 0.f, 0.f);
        for (i = 0; i < LZ_MENU_LINES; i++) LOGI("panel cube %s", v.line[i]);
        lz_menu_settings(&v, 1, &G.set);
        for (i = 0; i < LZ_MENU_LINES; i++) LOGI("panel set %s", v.line[i]);
        lz_menu_settings(&v, 2, &G.set);
        for (i = 0; i < LZ_MENU_LINES; i++) LOGI("panel set %s", v.line[i]);
    }
}

static void settings_store(const LzSettings *s) {
    char path[640];
    if (!s || !G.dir[0]) return;
    snprintf(path, sizeof path, "%s/lizard-settings.txt", G.dir);
    if (lz_settings_save(path, s) != 0) LOGE("settings save");
}

static void mount_from_props(LzMount *m) {
    char b[PROP_VALUE_MAX];
    lz_mount_default(m);
    if (__system_property_get("persist.lab.lizard.ox", b) > 0) m->ox_m = atoi(b) / 1000.f;
    if (__system_property_get("persist.lab.lizard.oy", b) > 0) m->oy_m = atoi(b) / 1000.f;
    if (__system_property_get("persist.lab.lizard.of", b) > 0) m->of_m = atoi(b) / 1000.f;
    /* Tenths of a degree, positive looks down. The crown sheet was sitting
     * about 4 cm above the skull with a level mount. 10° brings that
     * surface down onto the bone. Unset uses that measurement. */
    m->pitch_rad = 10.f * 0.017453292f;
    if (__system_property_get("persist.lab.lizard.pitch", b) > 0)
        m->pitch_rad = atoi(b) * 0.0017453292f;
}

static double now_sec(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

static int egl_init(EGLDisplay *dpy, EGLContext *ctx, EGLSurface *surf, EGLConfig *out_cfg) {
    EGLConfig configs[64];
    EGLint n = 0, i;
    const EGLint ctx_attr[] = {EGL_CONTEXT_CLIENT_VERSION, 3, EGL_NONE};
    const EGLint srf_attr[] = {EGL_WIDTH, 16, EGL_HEIGHT, 16, EGL_NONE};
    *dpy = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    if (*dpy == EGL_NO_DISPLAY || !eglInitialize(*dpy, NULL, NULL)) return -1;
    if (!eglGetConfigs(*dpy, configs, 64, &n) || n < 1) return -1;
    *out_cfg = 0;
    for (i = 0; i < n; i++) {
        EGLint render = 0, surf_type = 0, r = 0, g = 0, b = 0, a = 0, d = 0, s = 0, samples = 0;
        eglGetConfigAttrib(*dpy, configs[i], EGL_RENDERABLE_TYPE, &render);
        eglGetConfigAttrib(*dpy, configs[i], EGL_SURFACE_TYPE, &surf_type);
        eglGetConfigAttrib(*dpy, configs[i], EGL_RED_SIZE, &r);
        eglGetConfigAttrib(*dpy, configs[i], EGL_GREEN_SIZE, &g);
        eglGetConfigAttrib(*dpy, configs[i], EGL_BLUE_SIZE, &b);
        eglGetConfigAttrib(*dpy, configs[i], EGL_ALPHA_SIZE, &a);
        eglGetConfigAttrib(*dpy, configs[i], EGL_DEPTH_SIZE, &d);
        eglGetConfigAttrib(*dpy, configs[i], EGL_STENCIL_SIZE, &s);
        eglGetConfigAttrib(*dpy, configs[i], EGL_SAMPLES, &samples);
        if ((render & EGL_OPENGL_ES3_BIT_KHR) == 0) continue;
        if ((surf_type & (EGL_WINDOW_BIT | EGL_PBUFFER_BIT)) != (EGL_WINDOW_BIT | EGL_PBUFFER_BIT))
            continue;
        if (r != 8 || g != 8 || b != 8 || a != 8 || d != 0 || s != 0 || samples != 0) continue;
        *out_cfg = configs[i];
        break;
    }
    if (!*out_cfg) return -1;
    *ctx = eglCreateContext(*dpy, *out_cfg, EGL_NO_CONTEXT, ctx_attr);
    *surf = eglCreatePbufferSurface(*dpy, *out_cfg, srf_attr);
    if (*ctx == EGL_NO_CONTEXT || *surf == EGL_NO_SURFACE) return -1;
    return eglMakeCurrent(*dpy, *surf, *surf, *ctx) ? 0 : -1;
}

static int ext_ok(XrInstance instance, const char *name) {
    uint32_t n = 0, i;
    XrExtensionProperties *p;
    int ok = 0;
    xrEnumerateInstanceExtensionProperties(NULL, 0, &n, NULL);
    p = (XrExtensionProperties *)calloc(n ? n : 1, sizeof *p);
    if (!p) return 0;
    for (i = 0; i < n; i++) p[i].type = XR_TYPE_EXTENSION_PROPERTIES;
    xrEnumerateInstanceExtensionProperties(NULL, n, &n, p);
    for (i = 0; i < n; i++)
        if (strcmp(p[i].extensionName, name) == 0) ok = 1;
    free(p);
    (void)instance;
    return ok;
}

static int load_calib(struct android_app *app) {
    char path[512];
    const char *dir = app->activity->internalDataPath;
    AAsset *asset;
    FILE *f;
    if (!dir || !dir[0]) dir = "/data/local/tmp";
    snprintf(G.dir, sizeof G.dir, "%s", dir);
    snprintf(path, sizeof path, "%s/lizard-cloud.ply", dir);
    remove(path);
    snprintf(path, sizeof path, "%s/lizard-solid.ply", dir);
    remove(path);
    snprintf(path, sizeof path, "%s/calib.txt", dir);
    asset = AAssetManager_open(app->activity->assetManager, "calib.txt", AASSET_MODE_BUFFER);
    if (asset) {
        const void *buf = AAsset_getBuffer(asset);
        off_t len = AAsset_getLength(asset);
        f = fopen(path, "wb");
        if (f && buf && len > 0) fwrite(buf, 1, (size_t)len, f);
        if (f) fclose(f);
        AAsset_close(asset);
        if (openscan_calib_load(path, &G.cal) == 0) {
            LOGI("calib asset %dx%d serial %s", G.cal.width, G.cal.height, G.cal.serial);
            return 0;
        }
    }
    snprintf(path, sizeof path, "/sdcard/openscan/calib/JMM8006941.txt");
    if (openscan_calib_load(path, &G.cal) == 0) {
        LOGI("calib sdcard %s", path);
        return 0;
    }
    LOGE("no calibration file");
    return -1;
}

static void write_ply(const char *path, const float *vert, int nvert, const unsigned *idx, int nidx) {
    FILE *f = fopen(path, "w");
    int i, ntri, r, g, b;
    if (!f) return;
    ntri = nidx / 3;
    fprintf(f, "ply\nformat ascii 1.0\ncomment LizardTech object cube metres Y-up -Z-forward\n");
    fprintf(f, "element vertex %d\nproperty float x\nproperty float y\nproperty float z\n", nvert);
    fprintf(f, "property uchar red\nproperty uchar green\nproperty uchar blue\n");
    fprintf(f, "element face %d\nproperty list uchar int vertex_indices\nend_header\n", ntri);
    for (i = 0; i < nvert; i++) {
        const float *p = vert + i * 6;
        r = (int)(p[3] * 255.f);
        g = (int)(p[4] * 255.f);
        b = (int)(p[5] * 255.f);
        if (r < 0) r = 0;
        if (g < 0) g = 0;
        if (b < 0) b = 0;
        if (r > 255) r = 255;
        if (g > 255) g = 255;
        if (b > 255) b = 255;
        fprintf(f, "%.5f %.5f %.5f %d %d %d\n", p[0], p[1], p[2], r, g, b);
    }
    for (i = 0; i < ntri; i++)
        fprintf(f, "3 %u %u %u\n", idx[i * 3], idx[i * 3 + 1], idx[i * 3 + 2]);
    fclose(f);
}

static int write_stl(const char *path, const float *vert, int nvert, const unsigned *idx, int nidx) {
    FILE *f;
    char header[80];
    uint32_t ntri;
    int i;
    if (!path || !vert || !idx || nvert < 3 || nidx < 3) return -1;
    ntri = (uint32_t)(nidx / 3);
    f = fopen(path, "wb");
    if (!f) return -1;
    memset(header, 0, sizeof header);
    memcpy(header, "lizardtech cube mm", 18);
    if (fwrite(header, 1, 80, f) != 80) {
        fclose(f);
        return -1;
    }
    if (fwrite(&ntri, 4, 1, f) != 1) {
        fclose(f);
        return -1;
    }
    for (i = 0; i < (int)ntri; i++) {
        unsigned ia = idx[i * 3], ib = idx[i * 3 + 1], ic = idx[i * 3 + 2];
        float ax, ay, az, bx, by, bz, cx, cy, cz, nx, ny, nz, len, rec[12];
        uint16_t attr = 0;
        if (ia >= (unsigned)nvert || ib >= (unsigned)nvert || ic >= (unsigned)nvert) {
            fclose(f);
            return -1;
        }
        ax = vert[ia * 6] * 1000.f;
        ay = vert[ia * 6 + 1] * 1000.f;
        az = vert[ia * 6 + 2] * 1000.f;
        bx = vert[ib * 6] * 1000.f;
        by = vert[ib * 6 + 1] * 1000.f;
        bz = vert[ib * 6 + 2] * 1000.f;
        cx = vert[ic * 6] * 1000.f;
        cy = vert[ic * 6 + 1] * 1000.f;
        cz = vert[ic * 6 + 2] * 1000.f;
        nx = (by - ay) * (cz - az) - (bz - az) * (cy - ay);
        ny = (bz - az) * (cx - ax) - (bx - ax) * (cz - az);
        nz = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
        len = sqrtf(nx * nx + ny * ny + nz * nz);
        if (len < 1e-8f) len = 1.f;
        rec[0] = nx / len;
        rec[1] = ny / len;
        rec[2] = nz / len;
        rec[3] = ax;
        rec[4] = ay;
        rec[5] = az;
        rec[6] = bx;
        rec[7] = by;
        rec[8] = bz;
        rec[9] = cx;
        rec[10] = cy;
        rec[11] = cz;
        if (fwrite(rec, 4, 12, f) != 12 || fwrite(&attr, 2, 1, f) != 1) {
            fclose(f);
            return -1;
        }
    }
    if (fclose(f) != 0) return -1;
    return (int)ntri;
}

static int write_obj(const char *path, const float *vert, int nvert, const unsigned *idx, int nidx) {
    FILE *f;
    int i, ntri;
    if (!path || !vert || !idx || nvert < 3 || nidx < 3) return -1;
    f = fopen(path, "w");
    if (!f) return -1;
    ntri = nidx / 3;
    fprintf(f, "# lizardtech cube metres\n");
    for (i = 0; i < nvert; i++) {
        const float *p = vert + i * 6;
        fprintf(f, "v %.5f %.5f %.5f\n", p[0], p[1], p[2]);
    }
    for (i = 0; i < ntri; i++) {
        unsigned a = idx[i * 3], b = idx[i * 3 + 1], c = idx[i * 3 + 2];
        if (a >= (unsigned)nvert || b >= (unsigned)nvert || c >= (unsigned)nvert) {
            fclose(f);
            return -1;
        }
        fprintf(f, "f %u %u %u\n", a + 1, b + 1, c + 1);
    }
    if (fclose(f) != 0) return -1;
    return ntri;
}

/* Smallest eigenvalue of a 3x3 covariance. RMS is sqrt(lambda / n), metres. */
static float plane_rms(const float *vert, int nvert) {
    double mx = 0, my = 0, mz = 0;
    double A[3][3];
    int i, n = 0, it;
    if (!vert || nvert < 8) return 0.f;
    for (i = 0; i < nvert; i++) {
        mx += vert[i * 6 + 0];
        my += vert[i * 6 + 1];
        mz += vert[i * 6 + 2];
        n++;
    }
    mx /= n;
    my /= n;
    mz /= n;
    memset(A, 0, sizeof A);
    for (i = 0; i < nvert; i++) {
        double x = vert[i * 6 + 0] - mx;
        double y = vert[i * 6 + 1] - my;
        double z = vert[i * 6 + 2] - mz;
        A[0][0] += x * x;
        A[0][1] += x * y;
        A[0][2] += x * z;
        A[1][1] += y * y;
        A[1][2] += y * z;
        A[2][2] += z * z;
    }
    A[1][0] = A[0][1];
    A[2][0] = A[0][2];
    A[2][1] = A[1][2];
    for (it = 0; it < 16; it++) {
        int p = 0, q = 1, r, j;
        double max = 0, app, aqq, apq, tau, t, c, s, arp, arq;
        for (i = 0; i < 3; i++) {
            for (j = i + 1; j < 3; j++) {
                double v = A[i][j] >= 0 ? A[i][j] : -A[i][j];
                if (v > max) {
                    max = v;
                    p = i;
                    q = j;
                }
            }
        }
        if (max < 1e-18) break;
        app = A[p][p];
        aqq = A[q][q];
        apq = A[p][q];
        tau = (aqq - app) / (2.0 * apq);
        t = (tau >= 0 ? 1.0 : -1.0) / (fabs(tau) + sqrt(1.0 + tau * tau));
        c = 1.0 / sqrt(1.0 + t * t);
        s = t * c;
        A[p][p] = app - t * apq;
        A[q][q] = aqq + t * apq;
        A[p][q] = A[q][p] = 0;
        r = 3 - p - q;
        arp = A[r][p];
        arq = A[r][q];
        A[r][p] = A[p][r] = c * arp - s * arq;
        A[r][q] = A[q][r] = s * arp + c * arq;
    }
    {
        double m = A[0][0];
        if (A[1][1] < m) m = A[1][1];
        if (A[2][2] < m) m = A[2][2];
        if (m < 0) m = 0;
        return (float)sqrt(m / (double)n);
    }
}

/* Cube-frame model. STL is millimetres. PLY stays metres, coloured from camera B.
 * A copy under Download is what a file manager can see.
 * A sheet thinner than half a millimetre stays a cloud. */
static void export_model(const Draw *snap) {
    static const char *pubs[] = {"/sdcard/Download", "/storage/emulated/0/Download"};
    char path[640];
    int tris = 0, pts = 0, i, pub = 0;
    float rms = 0.f;
    if (!snap || !G.dir[0]) return;
    if (snap->vert && snap->nvert > 0) rms = plane_rms(snap->vert, snap->nvert);
    /* Half a millimetre of relief is a measured patch. A perfectly flat
     * numeric sheet stays a cloud. */
    if (snap->vert && snap->idx && snap->nvert > 0 && snap->nidx >= 3 && rms >= 0.0005f) {
        snprintf(path, sizeof path, "%s/lizard-model.stl", G.dir);
        tris = write_stl(path, snap->vert, snap->nvert, snap->idx, snap->nidx);
        snprintf(path, sizeof path, "%s/lizard-model.obj", G.dir);
        write_obj(path, snap->vert, snap->nvert, snap->idx, snap->nidx);
        snprintf(path, sizeof path, "%s/lizard-solid.ply", G.dir);
        write_ply(path, snap->vert, snap->nvert, snap->idx, snap->nidx);
        for (i = 0; i < 2; i++) {
            snprintf(path, sizeof path, "%s/lizard-model.stl", pubs[i]);
            if (write_stl(path, snap->vert, snap->nvert, snap->idx, snap->nidx) > 0) {
                pub = 1;
                LOGI("export stl %s", path);
                break;
            }
        }
    }
    if (snap->live && snap->nlive > 0) {
        pts = snap->nlive;
        snprintf(path, sizeof path, "%s/lizard-cloud.ply", G.dir);
        write_ply(path, snap->live, snap->nlive, NULL, 0);
        for (i = 0; i < 2; i++) {
            snprintf(path, sizeof path, "%s/lizard-cloud.ply", pubs[i]);
            write_ply(path, snap->live, snap->nlive, NULL, 0);
        }
    }
    LOGI("export model tris %d pts %d pub %d rms %.2f dir %s", tris < 0 ? 0 : tris, pts, pub, rms * 1000.f, G.dir);
}

static void set_line(int pts, int stripe, const LzCube *cube, const LzPose *head, const LzJoint *hands, int nhands) {
    char cs[96];
    char cam[48];
    const char *pattern;
    int found = -1;
    int i;
    float cx, cy, cz;
    lz_cube_status(cube, hands, nhands, cs, (int)sizeof cs);
    if (hands && nhands > 26 && hands[26].radius > 0.f) found = 26;
    else if (hands && nhands > 0 && hands[0].radius > 0.f) found = 0;
    else if (hands) {
        for (i = 0; i < nhands; i++) {
            if (hands[i].radius > 0.f) {
                found = i;
                break;
            }
        }
    }
    if (found >= 0) {
        lz_world_to_cam(head, &G.mount, hands[found].x, hands[found].y, hands[found].z, &cx, &cy, &cz);
        snprintf(cam, sizeof cam, " cam %.2f,%.2f,%.2f", cx, cy, cz);
    } else {
        snprintf(cam, sizeof cam, " cam -");
    }
    pattern = lz_cam_cols(G.cam) > 0 ? "" : " NO PATTERN";
    pthread_mutex_lock(&G.mu);
    snprintf(G.line, sizeof G.line,
             "%s pts %d stripe %d stored %d solid %d tris %d hands %d %s%s%s mesh %d fl %d d%d n%d f%d st%d sm%d sw%d rl%d",
             cs, pts, stripe, lz_mold_stored(G.mold), G.confirmed, G.tris, nhands, lz_cam_status(G.cam), cam, pattern,
             G.set.solid,
             G.set.flip, G.set.distance, G.set.near_mm, G.set.far_mm, G.set.stride, G.set.smooth, G.set.sweep,
             G.set.relief);
    G.stored = lz_mold_stored(G.mold);
    G.cols = lz_cam_cols(G.cam);
    pthread_mutex_unlock(&G.mu);
}

static void publish(Draw *fresh) {
    Draw *old;
    pthread_mutex_lock(&G.mu);
    old = G.draw;
    G.draw = fresh;
    pthread_mutex_unlock(&G.mu);
    if (!old) return;
    free(old->vert);
    free(old->idx);
    free(old->live);
    free(old);
}

static void publish_clear(int serial) {
    Draw *fresh = (Draw *)calloc(1, sizeof *fresh);
    if (!fresh) return;
    fresh->serial = serial;
    publish(fresh);
}

/* NIR from the pattern lamp lands in camera B's red channel and the old
 * lift turned that cast into a flat red block. Keep the green and blue. */
static void calm_nir(float *r, float *g, float *b) {
    float R, G, B, y, n;
    if (!r || !g || !b) return;
    R = *r;
    G = *g;
    B = *b;
    if (!(R > G + 0.12f && R > B + 0.12f)) return;
    y = 0.25f * R + 0.65f * G + 0.10f * B;
    n = 0.5f * (G + B);
    if (n < 0.04f) n = y;
    *r = n;
}

/* Pull the stored shell 1.5 mm toward the cube centre so the live sheet,
 * which sits on the measured surface, wins the depth test. */
static void shell_inset(LzMesh *m) {
    int i;
    const float back = 0.0015f;
    if (!m || !m->vert) return;
    for (i = 0; i < m->nvert; i++) {
        float *p = m->vert + i * 6;
        float len = sqrtf(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
        float s;
        if (!(len > back + 0.004f)) continue;
        s = (len - back) / len;
        p[0] *= s;
        p[1] *= s;
        p[2] *= s;
    }
}

static void mesh_append(LzMesh *dst, const LzMesh *src) {
    int i;
    float *v;
    unsigned *idx;
    if (!dst || !src || src->nvert < 1 || src->nidx < 3 || !src->vert || !src->idx) return;
    v = (float *)realloc(dst->vert, (size_t)(dst->nvert + src->nvert) * 6 * sizeof(float));
    if (!v) return;
    dst->vert = v;
    memcpy(dst->vert + (size_t)dst->nvert * 6, src->vert, (size_t)src->nvert * 6 * sizeof(float));
    idx = (unsigned *)realloc(dst->idx, (size_t)(dst->nidx + src->nidx) * sizeof(unsigned));
    if (!idx) return;
    dst->idx = idx;
    for (i = 0; i < src->nidx; i++) dst->idx[dst->nidx + i] = src->idx[i] + (unsigned)dst->nvert;
    dst->nvert += src->nvert;
    dst->nidx += src->nidx;
}

static int mesh_copy(LzMesh *dst, const LzMesh *src) {
    float *v;
    unsigned *idx;
    if (!dst) return 0;
    lz_mesh_free(dst);
    if (!src || src->nvert < 1 || src->nidx < 3 || !src->vert || !src->idx) return 0;
    v = (float *)malloc((size_t)src->nvert * 6 * sizeof(float));
    idx = (unsigned *)malloc((size_t)src->nidx * sizeof(unsigned));
    if (!v || !idx) {
        free(v);
        free(idx);
        return 0;
    }
    memcpy(v, src->vert, (size_t)src->nvert * 6 * sizeof(float));
    memcpy(idx, src->idx, (size_t)src->nidx * sizeof(unsigned));
    dst->vert = v;
    dst->idx = idx;
    dst->nvert = src->nvert;
    dst->nidx = src->nidx;
    return 1;
}

/* Best surface seen since the last exposure decision. The lamp is a strobe:
 * a dark phase must not vote, and a frame that returned points must. */
static int win_pts, win_stripe, win_ea, win_p50, win_ok, win_area;
static int good_pts, good_ea, lamp_miss;
static int cover_try, cover_ea, cover_area, cover_wait, cover_stop;

static void clear_surface_window(void) {
    win_pts = win_stripe = win_ea = win_p50 = win_ok = win_area = 0;
}

static void note_surface(int pts, int stripe, int ok, int p50, int ea) {
    if (p50 > win_p50) win_p50 = p50;
    if (stripe > win_stripe) win_stripe = stripe;
    if (ok && pts > win_pts) {
        win_pts = pts;
        win_ea = ea;
    }
    if (ok && (pts >= 30 || stripe >= 30)) {
        win_ok = 1;
        if (win_ea <= 0) win_ea = ea;
    }
}

/* Grid neighbours inside one ribbon. 36 mm closes a hole from a clipped row. */
static int skin_near(const LzSample *s, int a, int b) {
    float dx, dy, dz;
    dx = s[a].x - s[b].x;
    dy = s[a].y - s[b].y;
    dz = s[a].z - s[b].z;
    return dx * dx + dy * dy + dz * dz < 0.036f * 0.036f;
}

/* Facing surface of this frame, in the cube. The mold still waits for a second view. */
static int sheet_mesh(const int *slot, int rows, int cols, const LzSample *samples, int n, LzMesh *out) {
    int r, k, ntri = 0, cap;
    unsigned *idx;
    float *vert;
    if (!out) return 0;
    memset(out, 0, sizeof *out);
    if (!slot || !samples || n < 3 || rows < 2 || cols < 2) return 0;
    vert = (float *)malloc(sizeof(float) * 6 * (size_t)n);
    cap = n * 2;
    if (cap > 80000) cap = 80000;
    idx = (unsigned *)malloc(sizeof(unsigned) * 3 * (size_t)cap);
    if (!vert || !idx) {
        free(vert);
        free(idx);
        return 0;
    }
    for (k = 0; k < n; k++) {
        vert[k * 6 + 0] = samples[k].x;
        vert[k * 6 + 1] = samples[k].y;
        vert[k * 6 + 2] = samples[k].z;
        vert[k * 6 + 3] = samples[k].r;
        vert[k * 6 + 4] = samples[k].g;
        vert[k * 6 + 5] = samples[k].b;
    }
    for (r = 0; r < rows - 1 && ntri < cap; r++) {
        for (k = 0; k < cols - 1 && ntri < cap; k++) {
            int a = slot[r * cols + k];
            int b = slot[r * cols + k + 1];
            int c = slot[(r + 1) * cols + k];
            int d = slot[(r + 1) * cols + k + 1];
            if (a >= 0 && b >= 0 && c >= 0 && skin_near(samples, a, b) && skin_near(samples, b, c) &&
                skin_near(samples, a, c)) {
                idx[ntri * 3 + 0] = (unsigned)a;
                idx[ntri * 3 + 1] = (unsigned)b;
                idx[ntri * 3 + 2] = (unsigned)c;
                ntri++;
            }
            if (ntri >= cap) break;
            if (b >= 0 && d >= 0 && c >= 0 && skin_near(samples, b, d) && skin_near(samples, d, c) &&
                skin_near(samples, b, c)) {
                idx[ntri * 3 + 0] = (unsigned)b;
                idx[ntri * 3 + 1] = (unsigned)d;
                idx[ntri * 3 + 2] = (unsigned)c;
                ntri++;
            }
        }
    }
    if (ntri < 1) {
        free(vert);
        free(idx);
        return 0;
    }
    out->vert = vert;
    out->nvert = n;
    out->idx = idx;
    out->nidx = ntri * 3;
    return ntri;
}

static void *worker(void *arg) {
    uint8_t *ya = NULL, *yb = NULL, *ub = NULL, *vb = NULL;
    LzSample *samples = NULL;
    int last_seq = -1;
    int frame = 0;
    double last_save = 0;
    static double last_rescue;
    static double last_b;
    static double sweep_hold_until;
    static int b_peak95, b_clipmax;
    static int sweep_on, sweep_i, sweep_best_c, sweep_best_e, sweep_score;
    static LzMesh kept_sheet;
    (void)arg;
    if (!G.have_cal) {
        snprintf(G.line, sizeof G.line, "no calibration");
        return NULL;
    }
    ya = (uint8_t *)malloc((size_t)G.cal.width * (size_t)G.cal.height);
    yb = (uint8_t *)malloc((size_t)G.cal.width * (size_t)G.cal.height);
    ub = (uint8_t *)malloc((size_t)(G.cal.width / 2) * (size_t)(G.cal.height / 2));
    vb = (uint8_t *)malloc((size_t)(G.cal.width / 2) * (size_t)(G.cal.height / 2));
    samples = (LzSample *)malloc(sizeof(LzSample) * LIVE_CAP);
    if (!ya || !yb || !ub || !vb || !samples) goto done;
    G.cam = lz_cam_open(G.cal.width, G.cal.height);
    LOGI("camera %s", lz_cam_status(G.cam));
    while (!G.stop) {
        int seq = 0, rc, i, n = 0, grew = 0, stripe_n = 0;
        os_stripe_cloud cloud;
        LzPose head;
        LzJoint hands[52];
        LzCube cube;
        int nhands = 0, head_ok = 0, reset = 0, scan = 0;
        int ea_frame = 0, p50_frame = 0, sheet_tris = 0;
        float ox, oy, oz;
        LzMesh mesh;
        LzSettings kn;
        Draw *fresh;
        pthread_mutex_lock(&G.mu);
        kn = G.set;
        pthread_mutex_unlock(&G.mu);
        {
            /* The bar's downward pitch is a property so a wrong tilt can be
             * corrected without rebuilding. Old cells are in the previous frame. */
            static double last_mount;
            double nowm = now_sec();
            if (nowm - last_mount > 0.5) {
                LzMount m;
                last_mount = nowm;
                mount_from_props(&m);
                if (fabsf(m.pitch_rad - G.mount.pitch_rad) > 0.002f || fabsf(m.oy_m - G.mount.oy_m) > 0.001f ||
                    fabsf(m.of_m - G.mount.of_m) > 0.001f || fabsf(m.ox_m - G.mount.ox_m) > 0.001f) {
                    pthread_mutex_lock(&G.mu);
                    G.mount = m;
                    pthread_mutex_unlock(&G.mu);
                    lz_mold_reset(G.mold);
                    lz_mesh_free(&kept_sheet);
                    LOGI("mount pitch %.1f deg oy %.0f of %.0f", m.pitch_rad * 57.2958f, m.oy_m * 1000.f,
                         m.of_m * 1000.f);
                }
            }
        }
        {
            /* A long shutter washes the strobe into a flat white skull, and the
             * black borders keep clip under the old 18% gate. p95 sees that.
             * Camera B is metered on the brightest frame of the strobe so a dark
             * phase does not yank the colour exposure back and forth. */
            double now = now_sec();
            int cols = 0, clip = 0, p50 = 0, p95 = 0, changed = 0;
            int bclip = 0, bp50 = 0, bp95 = 0;
            ea_frame = kn.ea;
            /* A menu edit owns the shutter for 12 s. Forget the latch so the
             * next second learns the exposure the user just set. */
            if (now < G.exp_hold_until) {
                sweep_on = 0;
                good_pts = 0;
                good_ea = 0;
                lamp_miss = 0;
                cover_try = 0;
                clear_surface_window();
            } else if (lz_cam_meter(G.cam, &cols, &clip, &p50, &p95) && now - last_rescue > 1.0) {
                int next = 0;
                int latched = 0;
                int surf = win_pts >= win_stripe ? win_pts : win_stripe;
                /* cols stays near 3 on this projector even when stripe returns
                 * a thousand points, and the lamp-on frame is often clipped.
                 * Holding the shutter that produced the surface is what keeps
                 * the model growing. A dark strobe phase is not a vote. */
                static const int k_try[] = {16, 12, 10, 8, 6, 20, 4, 5, 3};
                int washed = clip >= 12 || p95 >= 230;
                /* A clipped shutter can return a thousand points that are only
                 * the fringe of the lamp. Try a shorter one when that patch is
                 * still small, and put the old shutter back if the surface dies. */
                if (cover_try && cover_wait > 0) {
                    cover_wait--;
                    latched = 1;
                    LOGI("exp cover wait %d pts %d area %d clip %d", kn.ea, surf, win_area, clip);
                } else if (cover_try && (surf < 80 || win_area < cover_area + cover_area / 4)) {
                    kn.ea = cover_ea > 0 ? cover_ea : kn.ea;
                    cover_stop = kn.ea;
                    cover_try = 0;
                    good_pts = 80;
                    good_ea = kn.ea;
                    lamp_miss = 0;
                    sweep_on = 0;
                    changed = 1;
                    latched = 1;
                    LOGI("exp cover revert %d area %d best %d pts %d", kn.ea, win_area, cover_area, surf);
                } else if (cover_try) {
                    cover_ea = kn.ea;
                    cover_area = win_area;
                    good_pts = surf;
                    good_ea = kn.ea;
                    lamp_miss = 0;
                    sweep_on = 0;
                    LOGI("exp cover keep %d area %d pts %d clip %d", kn.ea, win_area, surf, clip);
                    if (washed && kn.ea > 3 && win_area < 3500) {
                        int shorter = kn.ea >= 10 ? kn.ea - 4 : kn.ea - 2;
                        if (shorter < 3) shorter = 3;
                        kn.ea = shorter;
                        cover_try = kn.ea;
                        cover_wait = 1;
                        changed = 1;
                        LOGI("exp cover try %d", kn.ea);
                    } else {
                        cover_try = 0;
                        cover_stop = kn.ea;
                    }
                    latched = 1;
                } else if (win_ok && surf >= 30) {
                    good_pts = surf;
                    good_ea = win_ea > 0 ? win_ea : kn.ea;
                    lamp_miss = 0;
                    sweep_on = 0;
                    if (kn.ea != good_ea) {
                        kn.ea = good_ea;
                        changed = 1;
                    }
                    LOGI("exp hold A %d/%d pts %d stripe %d area %d cols %d clip %d p95 %d", kn.ea, kn.ga, win_pts,
                         win_stripe, win_area, cols, clip, p95);
                    latched = 1;
                } else if (good_pts >= 30 && lamp_miss < 4) {
                    lamp_miss++;
                    sweep_on = 0;
                    LOGI("exp hold A %d/%d pts %d miss %d cols %d p95 %d", kn.ea, kn.ga, good_pts, lamp_miss, cols,
                         p95);
                    latched = 1;
                } else if (good_pts >= 30) {
                    good_pts = 0;
                    good_ea = 0;
                    lamp_miss = 0;
                }
                if (!latched) {
                    int locked = cols >= 12 && clip < 10 && p95 < 220;
                    if (locked) {
                        sweep_on = 0;
                        if (openscan_pattern_rescue(cols, clip, p95, kn.ea, &next)) {
                            kn.ea = next;
                            changed = 1;
                            LOGI("exp rescue A %d/%d cols %d clip %d p50 %d p95 %d", kn.ea, kn.ga, cols, clip, p50,
                                 p95);
                        }
                    } else if (!sweep_on && now < sweep_hold_until) {
                        if (openscan_pattern_rescue(cols, clip, p95, kn.ea, &next)) {
                            kn.ea = next;
                            changed = 1;
                            LOGI("exp rescue A %d/%d cols %d clip %d p50 %d p95 %d", kn.ea, kn.ga, cols, clip, p50,
                                 p95);
                        }
                    } else if (!sweep_on) {
                        sweep_on = 1;
                        sweep_i = 0;
                        sweep_best_c = -1;
                        sweep_best_e = kn.ea;
                        sweep_score = -1000000000;
                        kn.ea = k_try[0];
                        changed = 1;
                        LOGI("exp sweep start from %d cols %d p95 %d", sweep_best_e, cols, p95);
                    } else {
                        int span = p95 > 170 ? p95 - 170 : 170 - p95;
                        int score = win_pts * 100000 + win_stripe * 1000 + cols * 100 - span * 10 - clip * 100;
                        if (score > sweep_score) {
                            sweep_score = score;
                            sweep_best_c = surf > cols ? surf : cols;
                            sweep_best_e = win_ea > 0 ? win_ea : kn.ea;
                        }
                        sweep_i++;
                        if (sweep_i >= (int)(sizeof k_try / sizeof k_try[0])) {
                            kn.ea = sweep_best_e > 0 ? sweep_best_e : kn.ea;
                            sweep_on = 0;
                            sweep_hold_until = now + (sweep_best_c >= 30 ? 30.0 : 12.0);
                            if (sweep_best_c >= 30) {
                                good_pts = sweep_best_c;
                                good_ea = kn.ea;
                                lamp_miss = 0;
                            }
                            LOGI("exp sweep best %d pts %d", kn.ea, sweep_best_c);
                        } else {
                            LOGI("exp sweep try %d saw %d pts %d cols %d", k_try[sweep_i], kn.ea, win_pts, cols);
                            kn.ea = k_try[sweep_i];
                        }
                        changed = 1;
                    }
                }
                clear_surface_window();
                last_rescue = now;
            }
            if (lz_cam_meter_b(G.cam, &bclip, &bp50, &bp95)) {
                if (bp95 > b_peak95) b_peak95 = bp95;
                if (bclip > b_clipmax) b_clipmax = bclip;
            }
            if (now >= G.exp_hold_until && now - last_b > 2.0 && b_peak95 > 0) {
                int seen = b_peak95, seen_clip = b_clipmax, eb = kn.eb;
                b_peak95 = 0;
                b_clipmax = 0;
                last_b = now;
                if ((seen >= 235 || seen_clip > 12) && eb > 1) eb -= 1;
                else if (seen < 150 && seen_clip < 6 && eb < 24) eb += (seen < 80 ? 2 : 1);
                if (eb != kn.eb) {
                    kn.eb = eb;
                    changed = 1;
                    LOGI("exp color B %d/%d p95 %d clip %d", kn.eb, kn.gb, seen, seen_clip);
                }
            }
            if (changed) {
                pthread_mutex_lock(&G.mu);
                if (G.set.ea != kn.ea || G.set.ga != kn.ga) {
                    /* The old shutter's fringe is the wrong skin. Drop it. */
                    lz_mold_reset(G.mold);
                    lz_mesh_free(&kept_sheet);
                }
                G.set.ea = kn.ea;
                G.set.ga = kn.ga;
                G.set.eb = kn.eb;
                pthread_mutex_unlock(&G.mu);
                settings_store(&kn);
            }
            (void)bp50;
        }
        lz_cam_set_user(G.cam, kn.ea, kn.ga, kn.eb, kn.gb);
        rc = lz_cam_copy(G.cam, ya, yb, ub, vb, &seq);
        if (rc == -2) {
            if (now_sec() - last_save > 2.0) {
                LOGI("camera retry %s", lz_cam_status(G.cam));
                lz_cam_close(G.cam);
                G.cam = lz_cam_open(G.cal.width, G.cal.height);
                lz_cam_set_user(G.cam, kn.ea, kn.ga, kn.eb, kn.gb);
                LOGI("camera %s", lz_cam_status(G.cam));
                last_save = now_sec();
            }
            usleep(200000);
            continue;
        }
        if (rc != 0 || seq == last_seq) {
            if (now_sec() - last_save > 2.0) {
                LOGI("frame wait rc=%d %s", rc, lz_cam_status(G.cam));
                last_save = now_sec();
            }
            usleep(15000);
            continue;
        }
        last_seq = seq;
        /* tune() only publishes a meter every 0.8 s, so it often samples the
         * lamp-off phase and never sees the lattice. Score the frame itself. */
        {
            int fc = 0, fk = 0, fp95 = 0;
            p50_frame = 0;
            openscan_pattern_meter(ya, G.cal.width, G.cal.height, &fc, &fk, &p50_frame, &fp95);
        }
        pthread_mutex_lock(&G.mu);
        head = G.head;
        head_ok = G.head_ok;
        nhands = G.nhands;
        if (nhands > 52) nhands = 52;
        if (nhands < 0) nhands = 0;
        memcpy(hands, G.hands, sizeof(LzJoint) * (size_t)nhands);
        cube = G.cube;
        kn = G.set;
        reset = G.cube_reset;
        G.cube_reset = 0;
        G.cols = lz_cam_cols(G.cam);
        G.stored = lz_mold_stored(G.mold);
        pthread_mutex_unlock(&G.mu);
        if (reset) {
            lz_mold_reset(G.mold);
            lz_mesh_free(&kept_sheet);
            G.confirmed = 0;
            G.tris = 0;
            G.pts_last = 0;
            G.stored = 0;
            publish_clear(++frame);
        }
        if (!head_ok) {
            if (now_sec() - last_save > 2.0) {
                LOGI("tracking wait %s", lz_cam_status(G.cam));
                last_save = now_sec();
            }
            continue;
        }
        memset(&cloud, 0, sizeof cloud);
        if (os_stripe_make(&G.cal, ya, yb, G.cal.width, G.cal.height, kn.flip, (float)kn.distance,
                           (float)kn.near_mm, (float)kn.far_mm, kn.stride, &cloud) != 0) {
            os_stripe_free(&cloud);
            G.pts_last = 0;
            note_surface(0, 0, 0, p50_frame, ea_frame);
            if (now_sec() - last_save > 2.0) {
                char spath[640];
                set_line(0, 0, &cube, &head, hands, nhands);
                LOGI("%s", G.line);
                snprintf(spath, sizeof spath, "%s/lizard-status.txt", G.dir);
                {
                    FILE *sf = fopen(spath, "w");
                    if (sf) {
                        fprintf(sf, "%s\n", G.line);
                        fclose(sf);
                    }
                }
                last_save = now_sec();
            }
            continue;
        }
        scan = lz_cube_scanning(&cube);
        lz_cam_origin_world(&head, &G.mount, &ox, &oy, &oz);
        n = 0;
        stripe_n = 0;
        {
            int total = cloud.pt ? cloud.rows * cloud.cols : 0;
            int *slot = NULL;
            if (cloud.pt) {
                for (i = 0; i < total; i++)
                    if (cloud.pt[i].ok) stripe_n++;
            }
            /* Show every in-cube sample immediately. The mold grows only after the cube is locked. */
            if (cloud.pt && cube.ready) {
                LzJoint local_hands[52];
                slot = (int *)malloc(sizeof(int) * (size_t)total);
                if (slot)
                    for (i = 0; i < total; i++) slot[i] = -1;
                lz_joints_into_cube(&cube, hands, nhands, local_hands);
                if (scan) lz_cube_to_local(&cube, ox, oy, oz, &ox, &oy, &oz);
                for (i = 0; i < total && n < LIVE_CAP; i++) {
                    float u, v, r, g, b, wx, wy, wz, sx, sy, sz;
                    if (!cloud.pt[i].ok) continue;
                    r = g = b = 0.72f;
                    if (lz_project_b(&G.cal.cam[1], cloud.pt[i].x, cloud.pt[i].y, cloud.pt[i].z, &u, &v)) {
                        /* Projection is the calibration pose. A flipped buffer is 180° from that. */
                        if (kn.flip) {
                            u = (float)(G.cal.width - 1) - u;
                            v = (float)(G.cal.height - 1) - v;
                        }
                        if (u >= 0.f && v >= 0.f && u < G.cal.width && v < G.cal.height)
                            lz_sample_yuv(yb, G.cal.width, G.cal.height, G.cal.width, ub, vb, G.cal.width / 2,
                                          G.cal.height / 2, G.cal.width / 2, G.cal.height / 2, 1, u, v, &r, &g, &b);
                    }
                    calm_nir(&r, &g, &b);
                    lz_cam_to_world(&head, &G.mount, cloud.pt[i].x, cloud.pt[i].y, cloud.pt[i].z, &wx, &wy,
                                    &wz);
                    if (!lz_cube_contains(&cube, wx, wy, wz, 0.f)) continue;
                    lz_cube_to_local(&cube, wx, wy, wz, &sx, &sy, &sz);
                    if (lz_hand_hits(local_hands, nhands, sx, sy, sz)) continue;
                    if (slot) slot[i] = n;
                    samples[n].x = sx;
                    samples[n].y = sy;
                    samples[n].z = sz;
                    samples[n].r = r;
                    samples[n].g = g;
                    samples[n].b = b;
                    n++;
                }
                if (scan) grew = lz_mold_integrate(G.mold, samples, n, ox, oy, oz, local_hands, nhands);
            }
            (void)grew;
            frame++;
            memset(&mesh, 0, sizeof mesh);
            /* Stored cells are the model inside the cube, from the first frame they
             * touch. Confirmation still waits for a moved scanner and is not faked.
             * The one-frame sheet only covers the gap before two cells share an edge.
             * Live dots are not drawn over that surface: they were winning the depth
             * test and leaving a flash where the solid should be. */
            if (kn.solid) {
                LzMesh sheet;
                memset(&sheet, 0, sizeof sheet);
                /* The shell is every stored cell, pulled slightly inward so it
                 * stays visible behind the live sheet. The sheet is this
                 * frame and is what covers the object in view. */
                lz_mold_shell(G.mold, &mesh);
                shell_inset(&mesh);
                sheet_mesh(slot, cloud.rows, cloud.cols, samples, n, &sheet);
                /* A dark strobe frame returns no sheet and used to wipe the
                 * skin, leaving the block shell alone in the cube. Keep the
                 * last surface until the next one is measured. A moving cube
                 * is a different local frame, so that skin is dropped. */
                if (!cube.placed || cube.grabbed)
                    lz_mesh_free(&kept_sheet);
                else if (sheet.nidx >= 3)
                    mesh_copy(&kept_sheet, &sheet);
                if (sheet.nidx >= 3)
                    mesh_append(&mesh, &sheet);
                else
                    mesh_append(&mesh, &kept_sheet);
                sheet_tris = sheet.nidx >= 3 ? sheet.nidx / 3 : kept_sheet.nidx / 3;
                lz_mesh_free(&sheet);
            } else {
                lz_mesh_free(&kept_sheet);
            }
            free(slot);
        }
        if (mesh.nvert > 0 && mesh.vert) {
            static double last_model;
            double nowm = now_sec();
            if (nowm - last_model > 2.0) {
                int vi;
                float mn0 = mesh.vert[0], mn1 = mesh.vert[1], mn2 = mesh.vert[2];
                float mx0 = mn0, mx1 = mn1, mx2 = mn2, ar = 0, ag = 0, ab = 0;
                for (vi = 0; vi < mesh.nvert; vi++) {
                    float *p = mesh.vert + vi * 6;
                    if (p[0] < mn0) mn0 = p[0];
                    if (p[1] < mn1) mn1 = p[1];
                    if (p[2] < mn2) mn2 = p[2];
                    if (p[0] > mx0) mx0 = p[0];
                    if (p[1] > mx1) mx1 = p[1];
                    if (p[2] > mx2) mx2 = p[2];
                    ar += p[3];
                    ag += p[4];
                    ab += p[5];
                }
                ar /= (float)mesh.nvert;
                ag /= (float)mesh.nvert;
                ab /= (float)mesh.nvert;
                LOGI("model tris %d sheet %d rgb %.2f %.2f %.2f mm %.0f %.0f %.0f .. %.0f %.0f %.0f", mesh.nidx / 3,
                     sheet_tris, ar, ag, ab, mn0 * 1000.f, mn1 * 1000.f, mn2 * 1000.f, mx0 * 1000.f, mx1 * 1000.f,
                     mx2 * 1000.f);
                last_model = nowm;
            }
        }
        fresh = (Draw *)calloc(1, sizeof *fresh);
        if (fresh) {
            fresh->vert = mesh.vert;
            fresh->nvert = mesh.nvert;
            fresh->idx = mesh.idx;
            fresh->nidx = mesh.nidx;
            fresh->serial = frame;
            mesh.vert = NULL;
            mesh.idx = NULL;
            if (fresh->nidx < 3) {
                int k, got = 0;
                fresh->live = (float *)malloc(sizeof(float) * 6 * (size_t)LIVE_CAP);
                if (fresh->live && !kn.solid)
                    got = lz_mold_cloud(G.mold, fresh->live, LIVE_CAP);
                if (got < 1 && n > 0 && fresh->live) {
                    int copy_n = n > LIVE_CAP ? LIVE_CAP : n;
                    got = copy_n;
                    for (k = 0; k < copy_n; k++) {
                        fresh->live[k * 6 + 0] = samples[k].x;
                        fresh->live[k * 6 + 1] = samples[k].y;
                        fresh->live[k * 6 + 2] = samples[k].z;
                        fresh->live[k * 6 + 3] = samples[k].r;
                        fresh->live[k * 6 + 4] = samples[k].g;
                        fresh->live[k * 6 + 5] = samples[k].b;
                    }
                }
                if (got > 0) fresh->nlive = got;
                else {
                    free(fresh->live);
                    fresh->live = NULL;
                }
            }
            publish(fresh);
        }
        lz_mesh_free(&mesh);
        note_surface(n, stripe_n, 1, p50_frame, ea_frame);
        if (n >= 8) {
            float x0 = samples[0].x, y0 = samples[0].y, z0 = samples[0].z;
            float x1 = x0, y1 = y0, z1 = z0;
            float ex, ey, ez, big, mid;
            int si, area;
            for (si = 1; si < n; si++) {
                if (samples[si].x < x0) x0 = samples[si].x;
                if (samples[si].y < y0) y0 = samples[si].y;
                if (samples[si].z < z0) z0 = samples[si].z;
                if (samples[si].x > x1) x1 = samples[si].x;
                if (samples[si].y > y1) y1 = samples[si].y;
                if (samples[si].z > z1) z1 = samples[si].z;
            }
            ex = (x1 - x0) * 1000.f;
            ey = (y1 - y0) * 1000.f;
            ez = (z1 - z0) * 1000.f;
            big = ex;
            mid = ey;
            if (big < mid) {
                float t = big;
                big = mid;
                mid = t;
            }
            if (mid < ez) {
                mid = ez;
                if (big < mid) {
                    float t = big;
                    big = mid;
                    mid = t;
                }
            }
            area = (int)(big * mid);
            if (area > win_area) win_area = area;
        }
        G.pts_last = n;
        G.confirmed = lz_mold_confirmed(G.mold);
        G.tris = G.draw ? G.draw->nidx / 3 : 0;
        G.seq_last = seq;
        G.stored = lz_mold_stored(G.mold);
        G.cols = lz_cam_cols(G.cam);
        os_stripe_free(&cloud);
        {
            int want = 0;
            char path[640];
            Draw *snap;
            pthread_mutex_lock(&G.mu);
            want = G.export_req;
            if (want) G.export_req = 0;
            snap = G.draw;
            pthread_mutex_unlock(&G.mu);
            if (want || now_sec() - last_save > 2.0) {
                set_line(G.pts_last, stripe_n, &cube, &head, hands, nhands);
                LOGI("%s", G.line);
                snprintf(path, sizeof path, "%s/lizard-status.txt", G.dir);
                {
                    FILE *sf = fopen(path, "w");
                    if (sf) {
                        fprintf(sf, "%s\n", G.line);
                        fclose(sf);
                    }
                }
                if (want || (snap && (snap->nlive > 0 || snap->nidx >= 3))) export_model(snap);
                if (want) {
                    pthread_mutex_lock(&G.mu);
                    G.export_ui = 1;
                    pthread_mutex_unlock(&G.mu);
                }
                last_save = now_sec();
            }
        }
    }
done:
    lz_mesh_free(&kept_sheet);
    free(ya);
    free(yb);
    free(ub);
    free(vb);
    free(samples);
    return NULL;
}

static GLuint compile_shader(GLenum type, const char *src) {
    GLuint s = glCreateShader(type);
    GLint ok = 0;
    glShaderSource(s, 1, &src, NULL);
    glCompileShader(s);
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetShaderInfoLog(s, sizeof log, NULL, log);
        LOGE("shader %s", log);
    }
    return s;
}

static GLuint make_program(void) {
    const char *vs =
        "#version 300 es\n"
        "layout(location=0) in vec3 aPos;\n"
        "layout(location=1) in vec3 aCol;\n"
        "uniform mat4 uMVP;\n"
        "uniform float uPointSize;\n"
        "out vec3 vCol;\n"
        "void main(){ gl_Position=uMVP*vec4(aPos,1.0); gl_PointSize=uPointSize; vCol=aCol; }\n";
    const char *fs =
        "#version 300 es\n"
        "precision mediump float;\n"
        "in vec3 vCol;\n"
        "uniform float uAlpha;\n"
        "uniform float uLift;\n"
        "out vec4 oCol;\n"
        "void main(){\n"
        "  vec3 c=vCol;\n"
        "  if(uLift>0.5){\n"
        "    float y=dot(c,vec3(0.25,0.65,0.10));\n"
        "    float shown=y<0.50?mix(0.50,0.82,y/0.50):y;\n"
        "    float g=min(shown/max(y,0.06),3.5);\n"
        "    c=min(c*g,vec3(1.0));\n"
        "  }\n"
        "  oCol=vec4(c,uAlpha);\n"
        "}\n";
    GLuint v = compile_shader(GL_VERTEX_SHADER, vs);
    GLuint f = compile_shader(GL_FRAGMENT_SHADER, fs);
    GLuint p = glCreateProgram();
    glAttachShader(p, v);
    glAttachShader(p, f);
    glLinkProgram(p);
    glDeleteShader(v);
    glDeleteShader(f);
    return p;
}

static GLuint make_menu_program(void) {
    const char *vs =
        "#version 300 es\n"
        "layout(location=0) in vec3 aPos;\n"
        "layout(location=1) in vec2 aUV;\n"
        "uniform mat4 uMVP;\n"
        "out vec2 vUV;\n"
        "void main(){ gl_Position=uMVP*vec4(aPos,1.0); vUV=aUV; }\n";
    const char *fs =
        "#version 300 es\n"
        "precision mediump float;\n"
        "in vec2 vUV;\n"
        "uniform sampler2D uTex;\n"
        "out vec4 oCol;\n"
        "void main(){ oCol=texture(uTex, vUV); }\n";
    GLuint v = compile_shader(GL_VERTEX_SHADER, vs);
    GLuint f = compile_shader(GL_FRAGMENT_SHADER, fs);
    GLuint p = glCreateProgram();
    glAttachShader(p, v);
    glAttachShader(p, f);
    glLinkProgram(p);
    glDeleteShader(v);
    glDeleteShader(f);
    return p;
}

static void mul_mm(const float a[16], const float b[16], float o[16]) {
    int c, r;
    for (c = 0; c < 4; c++)
        for (r = 0; r < 4; r++)
            o[c * 4 + r] = a[0 * 4 + r] * b[c * 4 + 0] + a[1 * 4 + r] * b[c * 4 + 1] +
                           a[2 * 4 + r] * b[c * 4 + 2] + a[3 * 4 + r] * b[c * 4 + 3];
}

static void view_matrix(const XrPosef *pose, float m[16]) {
    float qx = pose->orientation.x, qy = pose->orientation.y, qz = pose->orientation.z,
          qw = pose->orientation.w;
    float tx = pose->position.x, ty = pose->position.y, tz = pose->position.z;
    float c0x = 1 - 2 * (qy * qy + qz * qz);
    float c0y = 2 * (qx * qy + qw * qz);
    float c0z = 2 * (qx * qz - qw * qy);
    float c1x = 2 * (qx * qy - qw * qz);
    float c1y = 1 - 2 * (qx * qx + qz * qz);
    float c1z = 2 * (qy * qz + qw * qx);
    float c2x = 2 * (qx * qz + qw * qy);
    float c2y = 2 * (qy * qz - qw * qx);
    float c2z = 1 - 2 * (qx * qx + qy * qy);
    memset(m, 0, sizeof(float) * 16);
    m[0] = c0x;
    m[1] = c1x;
    m[2] = c2x;
    m[4] = c0y;
    m[5] = c1y;
    m[6] = c2y;
    m[8] = c0z;
    m[9] = c1z;
    m[10] = c2z;
    m[12] = -(c0x * tx + c0y * ty + c0z * tz);
    m[13] = -(c1x * tx + c1y * ty + c1z * tz);
    m[14] = -(c2x * tx + c2y * ty + c2z * tz);
    m[15] = 1.f;
}

static void proj_matrix(const XrFovf *fov, float m[16]) {
    float n = 0.04f, f = 8.f;
    float l = tanf(fov->angleLeft), r = tanf(fov->angleRight);
    float b = tanf(fov->angleDown), t = tanf(fov->angleUp);
    float w = r - l, h = t - b;
    memset(m, 0, sizeof(float) * 16);
    m[0] = 2.f / w;
    m[8] = (r + l) / w;
    m[5] = 2.f / h;
    m[9] = (t + b) / h;
    m[10] = -(f + n) / (f - n);
    m[11] = -1.f;
    m[14] = -(2.f * f * n) / (f - n);
}

static void read_hand(XrHandTrackerEXT tr, XrSpace space, XrTime time, LzJoint *dst) {
    XrHandJointLocationEXT jl[XR_HAND_JOINT_COUNT_EXT];
    XrHandJointLocationsEXT loc;
    XrHandJointsLocateInfoEXT info;
    int i;
    memset(&info, 0, sizeof info);
    info.type = XR_TYPE_HAND_JOINTS_LOCATE_INFO_EXT;
    info.baseSpace = space;
    info.time = time;
    memset(&loc, 0, sizeof loc);
    loc.type = XR_TYPE_HAND_JOINT_LOCATIONS_EXT;
    loc.jointCount = XR_HAND_JOINT_COUNT_EXT;
    loc.jointLocations = jl;
    for (i = 0; i < XR_HAND_JOINT_COUNT_EXT; i++) dst[i].radius = 0.f;
    if (!tr || !pfn_locate_hands) return;
    if (XR_FAILED(pfn_locate_hands(tr, &info, &loc)) || !loc.isActive) {
        static int idle_logged;
        if (!idle_logged) {
            idle_logged = 1;
            LOGI("hands locate idle");
        }
        return;
    }
    for (i = 0; i < XR_HAND_JOINT_COUNT_EXT; i++) {
        float rad;
        if ((jl[i].locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) == 0) continue;
        rad = jl[i].radius;
        if (rad < 0.006f) rad = 0.012f;
        dst[i].x = jl[i].pose.position.x;
        dst[i].y = jl[i].pose.position.y;
        dst[i].z = jl[i].pose.position.z;
        dst[i].radius = rad;
        {
            static int pose_logged;
            if (!pose_logged) {
                pose_logged = 1;
                LOGI("hands pose");
            }
        }
    }
}

typedef struct {
    int ok;
    int tracked; /* 1 when the runtime reported a position. 0 is orientation only. */
    float ax, ay, az;
    float dx, dy, dz;
    int trigger_edge;
    int trigger_down;
    int grip;
} Stick;

static struct {
    int ready;
    int attached;
    XrActionSet set;
    XrAction aim;
    XrAction trigger;
    XrAction grip;
    XrAction haptic;
    XrSpace space[2];
    XrPath hand[2];
    XrInstance instance;
    int held_trig[2];
    int held_grip[2];
    int announced;
    int noted;
    int sync_err;
    int syncs;
} Ctl;

static int latch_axis(int *held, float v, int active) {
    int edge = 0;
    if (!held) return 0;
    if (!active) {
        *held = 0;
        return 0;
    }
    if (!*held && v >= 0.60f) {
        *held = 1;
        edge = 1;
    } else if (*held && v <= 0.40f) {
        *held = 0;
    }
    return edge;
}

static XrAction make_action(XrActionSet set, const char *name, const char *local_name, XrActionType type,
                            const XrPath *hands) {
    XrActionCreateInfo ci;
    XrAction action = XR_NULL_HANDLE;
    memset(&ci, 0, sizeof ci);
    ci.type = XR_TYPE_ACTION_CREATE_INFO;
    ci.actionType = type;
    ci.countSubactionPaths = 2;
    ci.subactionPaths = hands;
    strncpy(ci.actionName, name, sizeof ci.actionName - 1);
    strncpy(ci.localizedActionName, local_name, sizeof ci.localizedActionName - 1);
    if (XR_FAILED(xrCreateAction(set, &ci, &action))) return XR_NULL_HANDLE;
    return action;
}

static int suggest_profile(XrInstance instance, const char *profile, const char *pose_tail) {
    XrActionSuggestedBinding b[8];
    XrInteractionProfileSuggestedBinding info;
    XrAction act[4];
    const char *side[2] = {"/user/hand/left", "/user/hand/right"};
    const char *tail[4] = {pose_tail, "/input/trigger/value", "/input/squeeze/value", "/output/haptic"};
    int i, k, n = 0;
    XrResult r;
    memset(&info, 0, sizeof info);
    memset(b, 0, sizeof b);
    info.type = XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING;
    if (XR_FAILED(xrStringToPath(instance, profile, &info.interactionProfile))) return 0;
    act[0] = Ctl.aim;
    act[1] = Ctl.trigger;
    act[2] = Ctl.grip;
    act[3] = Ctl.haptic;
    for (i = 0; i < 2; i++) {
        for (k = 0; k < 4; k++) {
            char path[160];
            snprintf(path, sizeof path, "%s%s", side[i], tail[k]);
            if (XR_FAILED(xrStringToPath(instance, path, &b[n].binding))) return 0;
            b[n].action = act[k];
            n++;
        }
    }
    info.countSuggestedBindings = (uint32_t)n;
    info.suggestedBindings = b;
    r = xrSuggestInteractionProfileBindings(instance, &info);
    if (XR_FAILED(r)) {
        LOGE("controller profile %s %d", profile, (int)r);
        return 0;
    }
    return 1;
}

static void ctrl_create(XrInstance instance) {
    XrActionSetCreateInfo asci;
    XrResult r;
    int nbind = 0;
    memset(&Ctl, 0, sizeof Ctl);
    Ctl.instance = instance;
    if (!instance) return;
    if (XR_FAILED(xrStringToPath(instance, "/user/hand/left", &Ctl.hand[0]))) return;
    if (XR_FAILED(xrStringToPath(instance, "/user/hand/right", &Ctl.hand[1]))) return;
    memset(&asci, 0, sizeof asci);
    asci.type = XR_TYPE_ACTION_SET_CREATE_INFO;
    strncpy(asci.actionSetName, "lizard", sizeof asci.actionSetName - 1);
    strncpy(asci.localizedActionSetName, "Lizard", sizeof asci.localizedActionSetName - 1);
    r = xrCreateActionSet(instance, &asci, &Ctl.set);
    if (XR_FAILED(r)) {
        LOGE("controller action set %d", (int)r);
        return;
    }
    Ctl.aim = make_action(Ctl.set, "aim", "Aim", XR_ACTION_TYPE_POSE_INPUT, Ctl.hand);
    Ctl.trigger = make_action(Ctl.set, "trigger", "Trigger", XR_ACTION_TYPE_FLOAT_INPUT, Ctl.hand);
    Ctl.grip = make_action(Ctl.set, "squeeze", "Grip", XR_ACTION_TYPE_FLOAT_INPUT, Ctl.hand);
    Ctl.haptic = make_action(Ctl.set, "haptic", "Haptic", XR_ACTION_TYPE_VIBRATION_OUTPUT, Ctl.hand);
    if (!Ctl.aim || !Ctl.trigger || !Ctl.grip || !Ctl.haptic) {
        LOGE("controller actions missing");
        return;
    }
    if (suggest_profile(instance, "/interaction_profiles/oculus/touch_controller", "/input/aim/pose")) nbind++;
    if (suggest_profile(instance, "/interaction_profiles/meta/touch_controller_plus", "/input/aim/pose")) nbind++;
    if (suggest_profile(instance, "/interaction_profiles/meta/touch_plus_controller", "/input/aim/pose"))
        nbind++;
    else if (suggest_profile(instance, "/interaction_profiles/meta/touch_plus_controller", "/input/grip/pose")) {
        nbind++;
        LOGI("controller plus grip pose");
    }
    LOGI("controller profiles %d", nbind);
    Ctl.ready = 1;
}

static void ctrl_attach(XrSession session) {
    XrSessionActionSetsAttachInfo ai;
    XrActionSpaceCreateInfo sp;
    XrActionSet sets[1];
    XrResult r;
    int i;
    if (!Ctl.ready || Ctl.attached || !session) return;
    memset(&ai, 0, sizeof ai);
    ai.type = XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO;
    sets[0] = Ctl.set;
    ai.countActionSets = 1;
    ai.actionSets = sets;
    r = xrAttachSessionActionSets(session, &ai);
    if (XR_FAILED(r)) {
        LOGE("controller attach %d", (int)r);
        return;
    }
    memset(&sp, 0, sizeof sp);
    sp.type = XR_TYPE_ACTION_SPACE_CREATE_INFO;
    sp.action = Ctl.aim;
    sp.poseInActionSpace.orientation.w = 1.f;
    for (i = 0; i < 2; i++) {
        sp.subactionPath = Ctl.hand[i];
        r = xrCreateActionSpace(session, &sp, &Ctl.space[i]);
        if (XR_FAILED(r)) LOGE("controller space %d %d", i, (int)r);
    }
    Ctl.attached = 1;
    LOGI("controllers attached");
}

static void ctrl_note(XrSession session, int i, int active) {
    XrInteractionProfileState st;
    char name[160];
    uint32_t len = 0;
    if (!session || (Ctl.noted & (1 << i))) return;
    memset(&st, 0, sizeof st);
    st.type = XR_TYPE_INTERACTION_PROFILE_STATE;
    name[0] = 0;
    if (!Ctl.instance || XR_FAILED(xrGetCurrentInteractionProfile(session, Ctl.hand[i], &st)) ||
        !st.interactionProfile) {
        if (Ctl.syncs < 300) return;
        Ctl.noted |= 1 << i;
        LOGI("controller %s profile none active %d", i ? "right" : "left", active);
        return;
    }
    if (XR_FAILED(xrPathToString(Ctl.instance, st.interactionProfile, (uint32_t)sizeof name, &len, name)))
        snprintf(name, sizeof name, "?");
    Ctl.noted |= 1 << i;
    LOGI("controller %s profile %s active %d", i ? "right" : "left", name, active);
}

static void ctrl_sync(XrSession session, XrSpace local, XrTime time, Stick *out) {
    XrActiveActionSet active;
    XrActionsSyncInfo si;
    XrResult sr;
    int i;
    if (!out) return;
    memset(out, 0, sizeof(Stick) * 2);
    if (!Ctl.attached || !session || !local) return;
    memset(&si, 0, sizeof si);
    si.type = XR_TYPE_ACTIONS_SYNC_INFO;
    memset(&active, 0, sizeof active);
    active.actionSet = Ctl.set;
    active.subactionPath = XR_NULL_PATH;
    si.countActiveActionSets = 1;
    si.activeActionSets = &active;
    sr = xrSyncActions(session, &si);
    if (XR_FAILED(sr)) {
        if (!Ctl.sync_err) {
            Ctl.sync_err = 1;
            LOGE("controller sync %d", (int)sr);
        }
        return;
    }
    Ctl.syncs++;
    for (i = 0; i < 2; i++) {
        XrActionStateGetInfo gi;
        XrActionStateFloat fl;
        XrActionStatePose pose;
        XrSpaceLocation loc;
        int active_pose = 0;
        memset(&gi, 0, sizeof gi);
        gi.type = XR_TYPE_ACTION_STATE_GET_INFO;
        gi.subactionPath = Ctl.hand[i];
        gi.action = Ctl.trigger;
        memset(&fl, 0, sizeof fl);
        fl.type = XR_TYPE_ACTION_STATE_FLOAT;
        if (XR_SUCCEEDED(xrGetActionStateFloat(session, &gi, &fl))) {
            out[i].trigger_edge = latch_axis(&Ctl.held_trig[i], fl.currentState, fl.isActive);
            out[i].trigger_down = Ctl.held_trig[i];
        }
        gi.action = Ctl.grip;
        memset(&fl, 0, sizeof fl);
        fl.type = XR_TYPE_ACTION_STATE_FLOAT;
        if (XR_SUCCEEDED(xrGetActionStateFloat(session, &gi, &fl)))
            latch_axis(&Ctl.held_grip[i], fl.currentState, fl.isActive);
        out[i].grip = Ctl.held_grip[i];
        gi.action = Ctl.aim;
        memset(&pose, 0, sizeof pose);
        pose.type = XR_TYPE_ACTION_STATE_POSE;
        if (XR_SUCCEEDED(xrGetActionStatePose(session, &gi, &pose)) && pose.isActive && Ctl.space[i]) {
            memset(&loc, 0, sizeof loc);
            loc.type = XR_TYPE_SPACE_LOCATION;
            if (XR_SUCCEEDED(xrLocateSpace(Ctl.space[i], local, time, &loc)) &&
                (loc.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT)) {
                int pos = (loc.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) != 0;
                active_pose = pos;
                out[i].ok = 1;
                out[i].tracked = pos;
                if (pos) {
                    out[i].ax = loc.pose.position.x;
                    out[i].ay = loc.pose.position.y;
                    out[i].az = loc.pose.position.z;
                }
                lz_quat_rotate(loc.pose.orientation.x, loc.pose.orientation.y, loc.pose.orientation.z,
                               loc.pose.orientation.w, 0.f, 0.f, -1.f, &out[i].dx, &out[i].dy, &out[i].dz);
                if ((Ctl.announced & (1 << i)) == 0) {
                    Ctl.announced |= 1 << i;
                    LOGI("controller %s%s", i ? "right" : "left", pos ? "" : " orientation");
                }
            }
        }
        ctrl_note(session, i, active_pose);
    }
}

static void ctrl_buzz(XrSession session, int hand) {
    XrHapticActionInfo hai;
    XrHapticVibration vib;
    if (!Ctl.attached || !session || hand < 0 || hand > 1) return;
    memset(&hai, 0, sizeof hai);
    hai.type = XR_TYPE_HAPTIC_ACTION_INFO;
    hai.action = Ctl.haptic;
    hai.subactionPath = Ctl.hand[hand];
    memset(&vib, 0, sizeof vib);
    vib.type = XR_TYPE_HAPTIC_VIBRATION;
    vib.duration = 20000000;
    vib.frequency = XR_FREQUENCY_UNSPECIFIED;
    vib.amplitude = 0.55f;
    xrApplyHapticFeedback(session, &hai, (const XrHapticBaseHeader *)&vib);
}

/* Framework classes only. ExportActivity lives in this apk and opens DocumentsUI. */
static void start_export_ui(struct android_app *app) {
    JNIEnv *env = NULL;
    JavaVM *vm;
    jobject act, intent;
    jclass act_cls, intent_cls, exp_cls;
    jmethodID start, ctor;
    if (!app || !app->activity || !app->activity->vm || !app->activity->clazz) return;
    vm = app->activity->vm;
    if ((*vm)->AttachCurrentThread(vm, &env, NULL) != JNI_OK || !env) return;
    act = app->activity->clazz;
    exp_cls = (*env)->FindClass(env, "dev/lab/lizard/ExportActivity");
    intent_cls = (*env)->FindClass(env, "android/content/Intent");
    if ((*env)->ExceptionCheck(env) || !exp_cls || !intent_cls) {
        if ((*env)->ExceptionCheck(env)) (*env)->ExceptionClear(env);
        LOGE("export activity missing");
        return;
    }
    ctor = (*env)->GetMethodID(env, intent_cls, "<init>", "(Landroid/content/Context;Ljava/lang/Class;)V");
    intent = ctor ? (*env)->NewObject(env, intent_cls, ctor, act, exp_cls) : NULL;
    if ((*env)->ExceptionCheck(env) || !intent) {
        if ((*env)->ExceptionCheck(env)) (*env)->ExceptionClear(env);
        LOGE("export intent failed");
        return;
    }
    act_cls = (*env)->GetObjectClass(env, act);
    start = (*env)->GetMethodID(env, act_cls, "startActivity", "(Landroid/content/Intent;)V");
    if (start) (*env)->CallVoidMethod(env, act, start, intent);
    if ((*env)->ExceptionCheck(env)) {
        (*env)->ExceptionClear(env);
        LOGE("export start failed");
    } else {
        LOGI("export document");
    }
}

static void android_handle(struct android_app *app, int32_t cmd) {
    (void)app;
    if (cmd == APP_CMD_RESUME) g_resumed = 1;
    else if (cmd == APP_CMD_PAUSE) g_resumed = 0;
}

static int make_eye(XrSession session, uint32_t w, uint32_t h, XrSwapchain *sc, GLuint *tex,
                    GLuint *fbo, uint32_t *nimg, GLuint *depth) {
    XrSwapchainCreateInfo ci = {.type = XR_TYPE_SWAPCHAIN_CREATE_INFO};
    XrSwapchainImageOpenGLESKHR imgs[MAX_IMG];
    uint32_t i, n = 0;
    ci.usageFlags = XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT | XR_SWAPCHAIN_USAGE_SAMPLED_BIT;
    ci.format = (int64_t)GL_RGBA8;
    ci.sampleCount = 1;
    ci.width = w;
    ci.height = h;
    ci.faceCount = 1;
    ci.arraySize = 1;
    ci.mipCount = 1;
    if (XR_FAILED(xrCreateSwapchain(session, &ci, sc))) return -1;
    xrEnumerateSwapchainImages(*sc, 0, &n, NULL);
    if (n > MAX_IMG) n = MAX_IMG;
    for (i = 0; i < n; i++) imgs[i].type = XR_TYPE_SWAPCHAIN_IMAGE_OPENGL_ES_KHR;
    if (XR_FAILED(xrEnumerateSwapchainImages(*sc, n, &n, (XrSwapchainImageBaseHeader *)imgs)))
        return -1;
    glGenRenderbuffers(1, depth);
    glBindRenderbuffer(GL_RENDERBUFFER, *depth);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, (GLsizei)w, (GLsizei)h);
    for (i = 0; i < n; i++) {
        tex[i] = (GLuint)imgs[i].image;
        glGenFramebuffers(1, &fbo[i]);
        glBindFramebuffer(GL_FRAMEBUFFER, fbo[i]);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex[i], 0);
        glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, *depth);
        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
            LOGE("fbo incomplete");
    }
    *nimg = n;
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    return 0;
}

void android_main(struct android_app *app) {
    EGLDisplay dpy = EGL_NO_DISPLAY;
    EGLContext egl_ctx = EGL_NO_CONTEXT;
    EGLSurface egl_surf = EGL_NO_SURFACE;
    EGLConfig egl_cfg = 0;
    XrInstance instance = XR_NULL_HANDLE;
    XrSession session = XR_NULL_HANDLE;
    XrSpace local_space = XR_NULL_HANDLE;
    XrSpace view_space = XR_NULL_HANDLE;
    XrSystemId system = XR_NULL_SYSTEM_ID;
    XrSwapchain sc[2] = {0, 0};
    GLuint tex[2][MAX_IMG], fbo[2][MAX_IMG], depth[2] = {0, 0};
    uint32_t nimg[2] = {0, 0};
    int eye_w = 1440, eye_h = 1440;
    XrSessionState st = XR_SESSION_STATE_UNKNOWN;
    int running = 0;
    int have_pt = 0, have_hands = 0;
    XrPassthroughFB passthrough = XR_NULL_HANDLE;
    XrPassthroughLayerFB pt_layer = XR_NULL_HANDLE;
    XrHandTrackerEXT hand_l = XR_NULL_HANDLE, hand_r = XR_NULL_HANDLE;
    GLuint prog = 0, menu_prog = 0, vao = 0, vbo = 0, ibo = 0, live_vbo = 0, cube_vbo = 0, menu_tex = 0;
    int uploaded = -1;
    const char *exts[8];
    uint32_t nexts = 0;

    memset(&G, 0, sizeof G);
    memset(&Ctl, 0, sizeof Ctl);
    pthread_mutex_init(&G.mu, NULL);
    G.app = app;
    app->onAppCmd = android_handle;
    app->userData = &G;
    mount_from_props(&G.mount);
    {
        int edge = prop_int("persist.lab.lizard.cube", 240);
        if (edge < 80) edge = 80;
        if (edge > 500) edge = 500;
        lz_cube_init(&G.cube, edge / 2000.f);
    }
    G.mold = lz_mold_create();
    LOGI("LizardTech OpenXR 1.1.60 headers, runtime request 1.0, mount %.0f %.0f %.0f mm pitch %.1f cube %.0f mm",
         G.mount.ox_m * 1000.f, G.mount.oy_m * 1000.f, G.mount.of_m * 1000.f,
         G.mount.pitch_rad * 57.2958f, G.cube.half * 2000.f);

    {
        PFN_xrInitializeLoaderKHR init_loader = NULL;
        XrLoaderInitInfoAndroidKHR li = {.type = XR_TYPE_LOADER_INIT_INFO_ANDROID_KHR};
        JNIEnv *jni = NULL;
        (*app->activity->vm)->AttachCurrentThread(app->activity->vm, &jni, NULL);
        li.applicationVM = app->activity->vm;
        li.applicationContext = app->activity->clazz;
        xrGetInstanceProcAddr(XR_NULL_HANDLE, "xrInitializeLoaderKHR", (PFN_xrVoidFunction *)&init_loader);
        if (!init_loader || XR_FAILED(init_loader((XrLoaderInitInfoBaseHeaderKHR *)&li))) {
            LOGE("xrInitializeLoaderKHR");
            goto pump;
        }
    }
    while (!app->destroyRequested && !g_resumed) {
        int events;
        struct android_poll_source *src;
        if (ALooper_pollOnce(-1, NULL, &events, (void **)&src) >= 0 && src) src->process(app, src);
    }
    if (app->destroyRequested) goto pump;
    if (egl_init(&dpy, &egl_ctx, &egl_surf, &egl_cfg) != 0) {
        LOGE("egl");
        goto pump;
    }
    G.have_cal = load_calib(app) == 0;
    settings_boot();

    exts[nexts++] = XR_KHR_OPENGL_ES_ENABLE_EXTENSION_NAME;
    exts[nexts++] = XR_KHR_ANDROID_CREATE_INSTANCE_EXTENSION_NAME;
    if (ext_ok(XR_NULL_HANDLE, XR_FB_PASSTHROUGH_EXTENSION_NAME)) exts[nexts++] = XR_FB_PASSTHROUGH_EXTENSION_NAME;
    if (ext_ok(XR_NULL_HANDLE, XR_EXT_HAND_TRACKING_EXTENSION_NAME))
        exts[nexts++] = XR_EXT_HAND_TRACKING_EXTENSION_NAME;
    if (ext_ok(XR_NULL_HANDLE, XR_META_TOUCH_CONTROLLER_PLUS_EXTENSION_NAME))
        exts[nexts++] = XR_META_TOUCH_CONTROLLER_PLUS_EXTENSION_NAME;
    if (ext_ok(XR_NULL_HANDLE, XR_META_SIMULTANEOUS_HANDS_AND_CONTROLLERS_EXTENSION_NAME)) {
        exts[nexts++] = XR_META_SIMULTANEOUS_HANDS_AND_CONTROLLERS_EXTENSION_NAME;
        want_hands_both = 1;
    }
    {
        XrInstanceCreateInfoAndroidKHR and = {.type = XR_TYPE_INSTANCE_CREATE_INFO_ANDROID_KHR};
        XrInstanceCreateInfo ici = {.type = XR_TYPE_INSTANCE_CREATE_INFO};
        and.applicationVM = app->activity->vm;
        and.applicationActivity = app->activity->clazz;
        ici.next = &and;
        strncpy(ici.applicationInfo.applicationName, "LizardTech", XR_MAX_APPLICATION_NAME_SIZE - 1);
        strncpy(ici.applicationInfo.engineName, "OpenScan", XR_MAX_ENGINE_NAME_SIZE - 1);
        ici.applicationInfo.apiVersion = XR_MAKE_VERSION(1, 0, 0);
        ici.enabledExtensionCount = nexts;
        ici.enabledExtensionNames = exts;
        if (XR_FAILED(xrCreateInstance(&ici, &instance))) {
            LOGE("xrCreateInstance");
            goto pump;
        }
        ctrl_create(instance);
    }
    {
        XrSystemGetInfo gi = {.type = XR_TYPE_SYSTEM_GET_INFO, .formFactor = XR_FORM_FACTOR_HEAD_MOUNTED_DISPLAY};
        if (XR_FAILED(xrGetSystem(instance, &gi, &system))) {
            LOGE("xrGetSystem");
            goto pump;
        }
    }
#define LOAD(name, dst) xrGetInstanceProcAddr(instance, name, (PFN_xrVoidFunction *)(dst))
    LOAD("xrCreatePassthroughFB", &pfn_create_pt);
    LOAD("xrDestroyPassthroughFB", &pfn_destroy_pt);
    LOAD("xrPassthroughStartFB", &pfn_start_pt);
    LOAD("xrCreatePassthroughLayerFB", &pfn_create_layer);
    LOAD("xrDestroyPassthroughLayerFB", &pfn_destroy_layer);
    LOAD("xrPassthroughLayerResumeFB", &pfn_resume_layer);
    LOAD("xrCreateHandTrackerEXT", &pfn_create_hand);
    LOAD("xrDestroyHandTrackerEXT", &pfn_destroy_hand);
    LOAD("xrLocateHandJointsEXT", &pfn_locate_hands);
    if (want_hands_both)
        LOAD("xrResumeSimultaneousHandsAndControllersTrackingMETA", &pfn_hands_both);
#undef LOAD
    {
        PFN_xrGetOpenGLESGraphicsRequirementsKHR pfn = NULL;
        XrGraphicsRequirementsOpenGLESKHR req = {.type = XR_TYPE_GRAPHICS_REQUIREMENTS_OPENGL_ES_KHR};
        xrGetInstanceProcAddr(instance, "xrGetOpenGLESGraphicsRequirementsKHR", (PFN_xrVoidFunction *)&pfn);
        if (pfn) pfn(instance, system, &req);
    }
    {
        XrGraphicsBindingOpenGLESAndroidKHR bind = {.type = XR_TYPE_GRAPHICS_BINDING_OPENGL_ES_ANDROID_KHR};
        XrSessionCreateInfo sci = {.type = XR_TYPE_SESSION_CREATE_INFO};
        bind.display = dpy;
        bind.config = egl_cfg;
        bind.context = egl_ctx;
        sci.next = &bind;
        sci.systemId = system;
        if (XR_FAILED(xrCreateSession(instance, &sci, &session))) {
            LOGE("xrCreateSession");
            goto pump;
        }
        ctrl_attach(session);
    }
    {
        XrReferenceSpaceCreateInfo rsi = {.type = XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
        rsi.poseInReferenceSpace.orientation.w = 1.f;
        rsi.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL;
        xrCreateReferenceSpace(session, &rsi, &local_space);
        rsi.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_VIEW;
        xrCreateReferenceSpace(session, &rsi, &view_space);
    }
    {
        XrViewConfigurationView vc[2] = {{.type = XR_TYPE_VIEW_CONFIGURATION_VIEW},
                                         {.type = XR_TYPE_VIEW_CONFIGURATION_VIEW}};
        uint32_t nv = 0;
        xrEnumerateViewConfigurationViews(instance, system, XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, 2, &nv, vc);
        if (nv) {
            eye_w = (int)vc[0].recommendedImageRectWidth;
            eye_h = (int)vc[0].recommendedImageRectHeight;
        }
    }
    if (make_eye(session, (uint32_t)eye_w, (uint32_t)eye_h, &sc[0], tex[0], fbo[0], &nimg[0], &depth[0]) ||
        make_eye(session, (uint32_t)eye_w, (uint32_t)eye_h, &sc[1], tex[1], fbo[1], &nimg[1], &depth[1])) {
        LOGE("swapchain");
        goto pump;
    }
    prog = make_program();
    menu_prog = make_menu_program();
    glGenTextures(1, &menu_tex);
    glBindTexture(GL_TEXTURE_2D, menu_tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, LZ_MENU_TEX_W, LZ_MENU_TEX_H, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glGenBuffers(1, &ibo);
    glGenBuffers(1, &live_vbo);
    glGenBuffers(1, &cube_vbo);
    lz_cam_set_android(app->activity->vm, app->activity->clazz);
    if (G.have_cal && G.mold) {
        pthread_create(&G.worker, NULL, worker, NULL);
        G.worker_on = 1;
    }

pump:
    while (!app->destroyRequested) {
        int events;
        struct android_poll_source *src;
        XrEventDataBuffer ev = {.type = XR_TYPE_EVENT_DATA_BUFFER};
        int timeout = running ? 0 : 250;
        while (ALooper_pollOnce(timeout, NULL, &events, (void **)&src) >= 0) {
            if (src) src->process(app, src);
            if (app->destroyRequested) break;
        }
        while (instance && xrPollEvent(instance, &ev) == XR_SUCCESS) {
            if (ev.type == XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED) {
                XrEventDataSessionStateChanged *scv = (XrEventDataSessionStateChanged *)&ev;
                st = scv->state;
                if (st == XR_SESSION_STATE_READY) {
                    XrSessionBeginInfo bi = {.type = XR_TYPE_SESSION_BEGIN_INFO,
                                             .primaryViewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO};
                    xrBeginSession(session, &bi);
                    running = 1;
                    LOGI("session begun");
                    if (want_hands_both && pfn_hands_both && !hands_both_done) {
                        XrSimultaneousHandsAndControllersTrackingResumeInfoMETA both = {
                            .type = XR_TYPE_SIMULTANEOUS_HANDS_AND_CONTROLLERS_TRACKING_RESUME_INFO_META};
                        XrResult br = pfn_hands_both(session, &both);
                        hands_both_done = 1;
                        if (XR_SUCCEEDED(br)) LOGI("hands and controllers");
                        else LOGE("hands and controllers %d", (int)br);
                    }
                } else if (st == XR_SESSION_STATE_STOPPING) {
                    xrEndSession(session);
                    running = 0;
                }
            }
            ev.type = XR_TYPE_EVENT_DATA_BUFFER;
        }
        {
            int show = 0;
            pthread_mutex_lock(&G.mu);
            if (G.export_ui) {
                G.export_ui = 0;
                show = 1;
            }
            pthread_mutex_unlock(&G.mu);
            if (show) start_export_ui(app);
        }
        if (!running || !session) continue;

        if (!have_pt && pfn_create_pt && pfn_create_layer && pfn_start_pt) {
            XrPassthroughCreateInfoFB ptci = {.type = XR_TYPE_PASSTHROUGH_CREATE_INFO_FB};
            XrPassthroughLayerCreateInfoFB plci = {.type = XR_TYPE_PASSTHROUGH_LAYER_CREATE_INFO_FB};
            if (XR_SUCCEEDED(pfn_create_pt(session, &ptci, &passthrough))) {
                pfn_start_pt(passthrough);
                plci.passthrough = passthrough;
                plci.purpose = XR_PASSTHROUGH_LAYER_PURPOSE_RECONSTRUCTION_FB;
                if (XR_SUCCEEDED(pfn_create_layer(session, &plci, &pt_layer))) {
                    if (pfn_resume_layer) pfn_resume_layer(pt_layer);
                    have_pt = 1;
                    LOGI("passthrough on");
                }
            } else {
                LOGE("passthrough create failed");
                pfn_create_pt = NULL;
            }
        }
        if (!have_hands && pfn_create_hand) {
            XrHandTrackerCreateInfoEXT ci = {.type = XR_TYPE_HAND_TRACKER_CREATE_INFO_EXT};
            ci.handJointSet = XR_HAND_JOINT_SET_DEFAULT_EXT;
            ci.hand = XR_HAND_LEFT_EXT;
            if (XR_SUCCEEDED(pfn_create_hand(session, &ci, &hand_l))) {
                ci.hand = XR_HAND_RIGHT_EXT;
                pfn_create_hand(session, &ci, &hand_r);
                have_hands = 1;
                LOGI("hands on");
            } else {
                pfn_create_hand = NULL;
            }
        }

        {
            XrFrameWaitInfo fwi = {.type = XR_TYPE_FRAME_WAIT_INFO};
            XrFrameState fs = {.type = XR_TYPE_FRAME_STATE};
            XrFrameBeginInfo fbi = {.type = XR_TYPE_FRAME_BEGIN_INFO};
            XrFrameEndInfo fei = {.type = XR_TYPE_FRAME_END_INFO};
            XrCompositionLayerPassthroughFB pt;
            XrCompositionLayerProjection proj;
            XrCompositionLayerProjectionView pv[2];
            const XrCompositionLayerBaseHeader *layers[2];
            uint32_t nlayers = 0;
            int eye;
            int menu_on = 0;
            int menu_nray = 0;
            float menu_quad[6 * 5];
            float menu_rayv[LZ_MENU_RAY_VERTS * 6];
            Stick stick[2];
            if (XR_FAILED(xrWaitFrame(session, &fwi, &fs))) continue;
            xrBeginFrame(session, &fbi);
            memset(stick, 0, sizeof stick);
            if (local_space) ctrl_sync(session, local_space, fs.predictedDisplayTime, stick);
            if (view_space && local_space) {
                XrSpaceLocation loc = {.type = XR_TYPE_SPACE_LOCATION};
                if (XR_SUCCEEDED(xrLocateSpace(view_space, local_space, fs.predictedDisplayTime, &loc)) &&
                    (loc.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) &&
                    (loc.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT)) {
                    {
                        LzCube before, after;
                        LzCubeDrive drive;
                        LzMenuView mview;
                        LzPanel panel;
                        int changed = 0;
                        int ray = stick[1].ok ? 1 : stick[0].ok ? 0 : -1;
                        int anchor = -1;
                        int hit = 0;
                        int clicked = 0;
                        int buzz = -1;
                        int found = -1;
                        int hand_side = 0;
                        int hi;
                        int side = 0;
                        int hit_line = -1;
                        int do_save = 0;
                        int click_rc = 0;
                        static int menu_page;
                        static int repeat_id, repeat_side, repeat_n;
                        LzSettings to_save;
                        float t_hit = 0.f;
                        float hx = 0.f, hy = 0.f, hz = 0.f;
                        float ax, ay, az;
                        if (ray == 1 && stick[0].ok) anchor = 0;
                        else if (ray == 0 && stick[1].ok) anchor = 1;
                        pthread_mutex_lock(&G.mu);
                        G.head.px = loc.pose.position.x;
                        G.head.py = loc.pose.position.y;
                        G.head.pz = loc.pose.position.z;
                        G.head.qx = loc.pose.orientation.x;
                        G.head.qy = loc.pose.orientation.y;
                        G.head.qz = loc.pose.orientation.z;
                        G.head.qw = loc.pose.orientation.w;
                        G.head_ok = 1;
                        {
                            int si;
                            for (si = 0; si < 2; si++) {
                                float ox, wx, wy, wz;
                                if (!stick[si].ok || stick[si].tracked) continue;
                                ox = si ? 0.16f : -0.16f;
                                lz_quat_rotate(G.head.qx, G.head.qy, G.head.qz, G.head.qw, ox, -0.18f, -0.28f, &wx,
                                               &wy, &wz);
                                stick[si].ax = G.head.px + wx;
                                stick[si].ay = G.head.py + wy;
                                stick[si].az = G.head.pz + wz;
                            }
                        }
                        if (have_hands) {
                            read_hand(hand_l, local_space, fs.predictedDisplayTime, G.hands);
                            read_hand(hand_r, local_space, fs.predictedDisplayTime, G.hands + 26);
                            G.nhands = 52;
                        }
                        if (G.cube.ready && G.cube.placed && !G.cube.grabbed) {
                            /* The off-hand panel was sitting on the object, and
                             * the depth-off menu erased the surface. Keep it
                             * up and to the left of the cube, clear of the volume. */
                            float sx, sy, sz, sl;
                            lz_quat_rotate(G.head.qx, G.head.qy, G.head.qz, G.head.qw, -1.f, 0.45f, 0.f, &sx, &sy,
                                           &sz);
                            sl = sqrtf(sx * sx + sy * sy + sz * sz);
                            if (sl > 1e-4f) {
                                sx /= sl;
                                sy /= sl;
                                sz /= sl;
                            }
                            sl = G.cube.half + LZ_MENU_W * 0.5f + 0.04f;
                            ax = G.cube.cx + sx * sl;
                            ay = G.cube.cy + sy * sl;
                            az = G.cube.cz + sz * sl;
                        } else if (anchor >= 0) {
                            ax = stick[anchor].ax;
                            ay = stick[anchor].ay;
                            az = stick[anchor].az;
                        } else {
                            ax = G.head.px;
                            ay = G.head.py;
                            az = G.head.pz;
                        }
                        lz_menu_panel_at(&panel, ax, ay, az, &G.head);
                        memset(&drive, 0, sizeof drive);
                        memset(&to_save, 0, sizeof to_save);
                        if (ray >= 0) {
                            int fire = 0;
                            drive.aim_ok = stick[ray].tracked;
                            drive.ax = stick[ray].ax;
                            drive.ay = stick[ray].ay;
                            drive.az = stick[ray].az;
                            drive.dx = stick[ray].dx;
                            drive.dy = stick[ray].dy;
                            drive.dz = stick[ray].dz;
                            drive.grip = stick[ray].grip;
                            hit = lz_menu_pick(&panel, menu_page, stick[ray].ax, stick[ray].ay, stick[ray].az,
                                               stick[ray].dx, stick[ray].dy, stick[ray].dz, &side, &hit_line, &t_hit);
                            if (hit && !G.cube.grabbed) drive.grip = 0;
                            if (hit && !(hit >= LZ_BTN_FIELD && side == 0)) {
                                if (stick[ray].trigger_edge) {
                                    fire = 1;
                                    repeat_id = hit;
                                    repeat_side = side;
                                    repeat_n = 0;
                                } else if (stick[ray].trigger_down && hit == repeat_id && side == repeat_side &&
                                           (hit == LZ_BTN_BIGGER || hit == LZ_BTN_SMALLER || hit >= LZ_BTN_FIELD)) {
                                    int gap = 4;
                                    int field = hit - LZ_BTN_FIELD;
                                    repeat_n++;
                                    if (hit == LZ_BTN_BIGGER || hit == LZ_BTN_SMALLER) gap = 8;
                                    else if (field == LZ_SET_STRIDE || field == LZ_SET_SMOOTH)
                                        gap = 10;
                                    else if (field == LZ_SET_EA || field == LZ_SET_EB || field == LZ_SET_DIST ||
                                             field == LZ_SET_NEAR || field == LZ_SET_FAR || field == LZ_SET_SWEEP)
                                        gap = 2;
                                    if (repeat_n >= 14 && gap > 0 && (repeat_n % gap) == 0) fire = 1;
                                }
                            }
                            if (!stick[ray].trigger_down) {
                                repeat_n = 0;
                                repeat_id = 0;
                            }
                            if (fire && menu_page == 0) {
                                if (hit == LZ_BTN_LOCK) drive.lock_toggle = 1;
                                else if (hit == LZ_BTN_BIGGER) drive.bigger = 1;
                                else if (hit == LZ_BTN_SMALLER) drive.smaller = 1;
                                else if (hit == LZ_BTN_RESET) drive.reset_scan = 1;
                                else if (hit == LZ_BTN_EXPORT) G.export_req = 1;
                            }
                            if (fire) {
                                LzSettings before = G.set;
                                click_rc = lz_menu_click(&menu_page, hit, side, &G.set);
                                if ((click_rc & 1) && memcmp(&before, &G.set, sizeof before) != 0) {
                                    to_save = G.set;
                                    do_save = 1;
                                    clicked = hit;
                                    buzz = ray;
                                    if (hit == LZ_BTN_FIELD + LZ_SET_EA || hit == LZ_BTN_FIELD + LZ_SET_EB)
                                        G.exp_hold_until = now_sec() + 12.0;
                                } else if (click_rc || drive.lock_toggle || drive.bigger || drive.smaller ||
                                           drive.reset_scan || hit == LZ_BTN_EXPORT) {
                                    clicked = hit;
                                    buzz = ray;
                                }
                            }
                        }
                        before = G.cube;
                        if (lz_cube_drive(&G.cube, &G.head, 1, &drive)) G.cube_reset = 1;
                        after = G.cube;
                        changed = before.ready != after.ready || before.placed != after.placed ||
                                  before.grabbed != after.grabbed || fabsf(before.half - after.half) > 1e-6f;
                        if (G.nhands > 26 && G.hands[26].radius > 0.f) found = 26;
                        else if (G.nhands > 0 && G.hands[0].radius > 0.f) found = 0;
                        else {
                            for (hi = 0; hi < G.nhands; hi++) {
                                if (G.hands[hi].radius > 0.f) {
                                    found = hi;
                                    break;
                                }
                            }
                        }
                        if (found >= 0) {
                            hand_side = found >= 26 ? 2 : 1;
                            lz_world_to_cam(&G.head, &G.mount, G.hands[found].x, G.hands[found].y,
                                            G.hands[found].z, &hx, &hy, &hz);
                        }
                        memset(&mview, 0, sizeof mview);
                        if (menu_page == 0)
                            lz_menu_view(&mview, ray >= 0, after.placed, after.grabbed,
                                         (int)(after.half * 2000.f + 0.5f), G.pts_last, G.stored, G.cols, G.tris,
                                         hand_side, hx, hy, hz);
                        else
                            lz_menu_settings(&mview, menu_page, &G.set);
                        mview.hot = hit;
                        mview.hot_line = hit_line;
                        lz_menu_quad(&panel, menu_quad);
                        menu_nray = 0;
                        if (ray >= 0) {
                            float len = (hit && t_hit > 0.02f) ? t_hit : 0.75f;
                            menu_nray = lz_menu_ray(stick[ray].ax, stick[ray].ay, stick[ray].az, stick[ray].dx,
                                                    stick[ray].dy, stick[ray].dz, len, hit, menu_rayv);
                        }
                        pthread_mutex_unlock(&G.mu);
                        if (do_save) {
                            settings_store(&to_save);
                            LOGI("settings A %d/%d B %d/%d dist %d near %d far %d stride %d smooth %d sweep %d relief %d "
                                 "solid %d flip %d",
                                 to_save.ea, to_save.ga, to_save.eb, to_save.gb, to_save.distance, to_save.near_mm,
                                 to_save.far_mm, to_save.stride, to_save.smooth, to_save.sweep, to_save.relief,
                                 to_save.solid, to_save.flip);
                        }
                        lz_menu_raster(&mview, g_menu_src);
                        menu_on = 1;
                        if (changed) {
                            const char *cst = !after.ready ? "wait" : after.grabbed ? "grab"
                                                                     : after.placed ? "lock"
                                                                                    : "follow";
                            LOGI("cube %s %.3f %.3f %.3f edge %.0f", cst, after.cx, after.cy, after.cz,
                                 after.half * 2000.f);
                        }
                        if (clicked) LOGI("menu %s", lz_menu_button_name(clicked));
                        if (buzz >= 0) ctrl_buzz(session, buzz);
                    }
                }
            }
            memset(pv, 0, sizeof pv);
            if (fs.shouldRender && sc[0] && sc[1]) {
                XrViewState vs = {.type = XR_TYPE_VIEW_STATE};
                XrView views[2] = {{.type = XR_TYPE_VIEW}, {.type = XR_TYPE_VIEW}};
                XrViewLocateInfo vli = {.type = XR_TYPE_VIEW_LOCATE_INFO};
                uint32_t nv = 0;
                vli.viewConfigurationType = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
                vli.displayTime = fs.predictedDisplayTime;
                vli.space = local_space;
                xrLocateViews(session, &vli, &vs, 2, &nv, views);
                if (menu_on && menu_tex) {
                    int row;
                    for (row = 0; row < LZ_MENU_TEX_H; row++)
                        memcpy(g_menu_flip + (size_t)(LZ_MENU_TEX_H - 1 - row) * LZ_MENU_TEX_W * 4,
                               g_menu_src + (size_t)row * LZ_MENU_TEX_W * 4, LZ_MENU_TEX_W * 4);
                    glBindTexture(GL_TEXTURE_2D, menu_tex);
                    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, LZ_MENU_TEX_W, LZ_MENU_TEX_H, GL_RGBA,
                                    GL_UNSIGNED_BYTE, g_menu_flip);
                }
                for (eye = 0; eye < 2 && eye < (int)nv; eye++) {
                    XrSwapchainImageAcquireInfo acq = {.type = XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
                    XrSwapchainImageWaitInfo wait = {.type = XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO,
                                                     .timeout = XR_INFINITE_DURATION};
                    XrSwapchainImageReleaseInfo rel = {.type = XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
                    uint32_t idx = 0;
                    float V[16], P[16], MVP[16], object_mvp[16];
                    int surface_idx = 0;
                    xrAcquireSwapchainImage(sc[eye], &acq, &idx);
                    xrWaitSwapchainImage(sc[eye], &wait);
                    if (idx < nimg[eye]) {
                        glBindFramebuffer(GL_FRAMEBUFFER, fbo[eye][idx]);
                        glViewport(0, 0, eye_w, eye_h);
                        glClearColor(0.f, 0.f, 0.f, 0.f);
                        glClearDepthf(1.f);
                        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                        glEnable(GL_DEPTH_TEST);
                        glDisable(GL_CULL_FACE);
                        glEnable(GL_BLEND);
                        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
                        view_matrix(&views[eye].pose, V);
                        proj_matrix(&views[eye].fov, P);
                        mul_mm(P, V, MVP);
                        glUseProgram(prog);
                        glBindVertexArray(vao);
                        pthread_mutex_lock(&G.mu);
                        {
                            float model[16];
                            LzCube cube = G.cube;
                            /* Points and the solid are cube-local. The same pose draws the frame. */
                            lz_cube_model(&cube, model);
                            mul_mm(MVP, model, object_mvp);
                            glUniformMatrix4fv(glGetUniformLocation(prog, "uMVP"), 1, GL_FALSE, object_mvp);
                            glUniform1f(glGetUniformLocation(prog, "uLift"), 1.f);
                            if (G.draw && G.draw->serial != uploaded) {
                                glBindBuffer(GL_ARRAY_BUFFER, vbo);
                                glBufferData(GL_ARRAY_BUFFER,
                                             (GLsizeiptr)G.draw->nvert * 6 * (GLsizeiptr)sizeof(float), G.draw->vert,
                                             GL_DYNAMIC_DRAW);
                                glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
                                glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                                             (GLsizeiptr)G.draw->nidx * (GLsizeiptr)sizeof(unsigned), G.draw->idx,
                                             GL_DYNAMIC_DRAW);
                                glBindBuffer(GL_ARRAY_BUFFER, live_vbo);
                                glBufferData(GL_ARRAY_BUFFER,
                                             (GLsizeiptr)G.draw->nlive * 6 * (GLsizeiptr)sizeof(float), G.draw->live,
                                             GL_DYNAMIC_DRAW);
                                uploaded = G.draw->serial;
                            }
                            /* Upload only. The menu is drawn with depth off and used
                             * to erase this surface. The draw is after the panel,
                             * from this VBO, so a freed Draw cannot race it. */
                            if (G.draw && G.draw->nidx >= 3 && G.draw->vert && G.draw->idx) {
                                surface_idx = G.draw->nidx;
                            } else if (G.draw && G.draw->nlive > 0 && G.draw->live) {
                                glUniform1f(glGetUniformLocation(prog, "uAlpha"), 1.f);
                                glUniform1f(glGetUniformLocation(prog, "uPointSize"), 10.f);
                                glBindBuffer(GL_ARRAY_BUFFER, live_vbo);
                                glEnableVertexAttribArray(0);
                                glEnableVertexAttribArray(1);
                                glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)0);
                                glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                                                      (void *)(3 * sizeof(float)));
                                glDrawArrays(GL_POINTS, 0, G.draw->nlive);
                            }
                            glUniform1f(glGetUniformLocation(prog, "uLift"), 0.f);
                            if (cube.ready) {
                                float beams[LZ_CUBE_BEAM_VERTS * 6];
                                float corners[LZ_CUBE_CORNER_VERTS * 6];
                                int nb = lz_cube_beams(&cube, beams);
                                int nc = lz_cube_corners(&cube, corners);
                                glDepthMask(GL_FALSE);
                                if (nb > 0) {
                                    glUniform1f(glGetUniformLocation(prog, "uAlpha"), 0.9f);
                                    glUniform1f(glGetUniformLocation(prog, "uPointSize"), 4.f);
                                    glBindBuffer(GL_ARRAY_BUFFER, cube_vbo);
                                    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)nb * 6 * (GLsizeiptr)sizeof(float),
                                                 beams, GL_DYNAMIC_DRAW);
                                    glEnableVertexAttribArray(0);
                                    glEnableVertexAttribArray(1);
                                    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)0);
                                    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                                                          (void *)(3 * sizeof(float)));
                                    glDrawArrays(GL_TRIANGLES, 0, nb);
                                }
                                if (nc > 0) {
                                    glUniform1f(glGetUniformLocation(prog, "uAlpha"), 1.f);
                                    glUniform1f(glGetUniformLocation(prog, "uPointSize"), 18.f);
                                    glBindBuffer(GL_ARRAY_BUFFER, cube_vbo);
                                    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)nc * 6 * (GLsizeiptr)sizeof(float),
                                                 corners, GL_DYNAMIC_DRAW);
                                    glEnableVertexAttribArray(0);
                                    glEnableVertexAttribArray(1);
                                    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)0);
                                    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                                                          (void *)(3 * sizeof(float)));
                                    glDrawArrays(GL_POINTS, 0, nc);
                                }
                                glDepthMask(GL_TRUE);
                            }
                        }
                        pthread_mutex_unlock(&G.mu);
                        if (menu_on && menu_prog && menu_tex) {
                            glUseProgram(menu_prog);
                            glUniformMatrix4fv(glGetUniformLocation(menu_prog, "uMVP"), 1, GL_FALSE, MVP);
                            glActiveTexture(GL_TEXTURE0);
                            glBindTexture(GL_TEXTURE_2D, menu_tex);
                            glUniform1i(glGetUniformLocation(menu_prog, "uTex"), 0);
                            glDisable(GL_DEPTH_TEST);
                            glDepthMask(GL_FALSE);
                            glBindBuffer(GL_ARRAY_BUFFER, cube_vbo);
                            glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)(6 * 5 * (int)sizeof(float)), menu_quad,
                                         GL_DYNAMIC_DRAW);
                            glEnableVertexAttribArray(0);
                            glEnableVertexAttribArray(1);
                            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void *)0);
                            glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float),
                                                  (void *)(3 * sizeof(float)));
                            glDrawArrays(GL_TRIANGLES, 0, 6);
                            glUseProgram(prog);
                            glUniformMatrix4fv(glGetUniformLocation(prog, "uMVP"), 1, GL_FALSE, MVP);
                            if (menu_nray > 0) {
                                glUniform1f(glGetUniformLocation(prog, "uAlpha"), 0.95f);
                                glUniform1f(glGetUniformLocation(prog, "uPointSize"), 4.f);
                                glBindBuffer(GL_ARRAY_BUFFER, cube_vbo);
                                glBufferData(GL_ARRAY_BUFFER,
                                             (GLsizeiptr)menu_nray * 6 * (GLsizeiptr)sizeof(float), menu_rayv,
                                             GL_DYNAMIC_DRAW);
                                glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)0);
                                glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                                                      (void *)(3 * sizeof(float)));
                                glDrawArrays(GL_TRIANGLES, 0, menu_nray);
                            }
                            glDepthMask(GL_TRUE);
                            glEnable(GL_DEPTH_TEST);
                        }
                        if (surface_idx >= 3) {
                            /* Depth was cleared and the panel does not write it,
                             * so this skin lands in front of the menu and replaces
                             * passthrough on the measured object. */
                            glUseProgram(prog);
                            glUniformMatrix4fv(glGetUniformLocation(prog, "uMVP"), 1, GL_FALSE, object_mvp);
                            glUniform1f(glGetUniformLocation(prog, "uLift"), 1.f);
                            glUniform1f(glGetUniformLocation(prog, "uAlpha"), 1.f);
                            glDisable(GL_BLEND);
                            glEnable(GL_DEPTH_TEST);
                            glDepthMask(GL_TRUE);
                            glDepthFunc(GL_LEQUAL);
                            glEnable(GL_POLYGON_OFFSET_FILL);
                            glPolygonOffset(-1.5f, -2.f);
                            glBindBuffer(GL_ARRAY_BUFFER, vbo);
                            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
                            glEnableVertexAttribArray(0);
                            glEnableVertexAttribArray(1);
                            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)0);
                            glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                                                  (void *)(3 * sizeof(float)));
                            glDrawElements(GL_TRIANGLES, surface_idx, GL_UNSIGNED_INT, (void *)0);
                            glDisable(GL_POLYGON_OFFSET_FILL);
                            glDepthFunc(GL_LESS);
                            glEnable(GL_BLEND);
                            glUniform1f(glGetUniformLocation(prog, "uLift"), 0.f);
                        }
                        glBindVertexArray(0);
                        glBindFramebuffer(GL_FRAMEBUFFER, 0);
                    }
                    xrReleaseSwapchainImage(sc[eye], &rel);
                    pv[eye].type = XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW;
                    pv[eye].pose = views[eye].pose;
                    pv[eye].fov = views[eye].fov;
                    pv[eye].subImage.swapchain = sc[eye];
                    pv[eye].subImage.imageRect.offset.x = 0;
                    pv[eye].subImage.imageRect.offset.y = 0;
                    pv[eye].subImage.imageRect.extent.width = eye_w;
                    pv[eye].subImage.imageRect.extent.height = eye_h;
                }
                proj.type = XR_TYPE_COMPOSITION_LAYER_PROJECTION;
                proj.next = NULL;
                proj.layerFlags = XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT;
                proj.space = local_space;
                proj.viewCount = nv >= 2 ? 2 : nv;
                proj.views = pv;
            }
            if (have_pt && pt_layer) {
                memset(&pt, 0, sizeof pt);
                pt.type = XR_TYPE_COMPOSITION_LAYER_PASSTHROUGH_FB;
                /* Quest rejects a passthrough layer that names a space. */
                pt.space = XR_NULL_HANDLE;
                pt.layerHandle = pt_layer;
                layers[nlayers++] = (const XrCompositionLayerBaseHeader *)&pt;
            }
            if (sc[0] && fs.shouldRender) layers[nlayers++] = (const XrCompositionLayerBaseHeader *)&proj;
            fei.displayTime = fs.predictedDisplayTime;
            fei.environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
            fei.layerCount = nlayers;
            fei.layers = nlayers ? layers : NULL;
            xrEndFrame(session, &fei);
        }
    }

    G.stop = 1;
    if (G.worker_on) pthread_join(G.worker, NULL);
    lz_cam_close(G.cam);
    lz_mold_free(G.mold);
    if (G.draw) {
        free(G.draw->vert);
        free(G.draw->idx);
        free(G.draw->live);
        free(G.draw);
    }
    if (hand_l && pfn_destroy_hand) pfn_destroy_hand(hand_l);
    if (hand_r && pfn_destroy_hand) pfn_destroy_hand(hand_r);
    if (pt_layer && pfn_destroy_layer) pfn_destroy_layer(pt_layer);
    if (passthrough && pfn_destroy_pt) pfn_destroy_pt(passthrough);
    if (sc[0]) xrDestroySwapchain(sc[0]);
    if (sc[1]) xrDestroySwapchain(sc[1]);
    if (view_space) xrDestroySpace(view_space);
    if (local_space) xrDestroySpace(local_space);
    if (Ctl.space[0]) xrDestroySpace(Ctl.space[0]);
    if (Ctl.space[1]) xrDestroySpace(Ctl.space[1]);
    if (session) xrDestroySession(session);
    if (instance) xrDestroyInstance(instance);
    pthread_mutex_destroy(&G.mu);
}
