#include "lizard_cam.h"

#if !defined(__ANDROID__)

struct LzCam {
    int unused;
};

void lz_cam_set_android(void *vm, void *activity) {
    (void)vm;
    (void)activity;
}
LzCam *lz_cam_open(int width, int height) {
    (void)width;
    (void)height;
    return 0;
}
void lz_cam_close(LzCam *cam) { (void)cam; }
int lz_cam_copy(LzCam *cam, uint8_t *ya, uint8_t *yb, uint8_t *ub, uint8_t *vb, int *seq) {
    (void)cam;
    (void)ya;
    (void)yb;
    (void)ub;
    (void)vb;
    (void)seq;
    return -1;
}
int lz_cam_locked(const LzCam *cam) {
    (void)cam;
    return 0;
}
int lz_cam_cols(const LzCam *cam) {
    (void)cam;
    return 0;
}
int lz_cam_meter(const LzCam *cam, int *cols, int *clip, int *p50, int *p95) {
    (void)cam;
    if (cols) *cols = 0;
    if (clip) *clip = 0;
    if (p50) *p50 = 0;
    if (p95) *p95 = 0;
    return 0;
}
int lz_cam_meter_b(const LzCam *cam, int *clip, int *p50, int *p95) {
    (void)cam;
    if (clip) *clip = 0;
    if (p50) *p50 = 0;
    if (p95) *p95 = 0;
    return 0;
}
const char *lz_cam_status(const LzCam *cam) {
    (void)cam;
    return "no camera";
}
void lz_cam_set_user(LzCam *cam, int exp_a, int gain_a, int exp_b, int gain_b) {
    (void)cam;
    (void)exp_a;
    (void)gain_a;
    (void)exp_b;
    (void)gain_b;
}

#else

#include "lizard_uvc.h"

#include <android/log.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define TAG "lizard"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, TAG, __VA_ARGS__)

/* Smallest useful isoc reservation first would hide a device that only
 * accepts a large payload. Try the device's largest alt, then step down
 * so the second camera still fits on the USB2 hub. */
static const int k_caps[] = {3072, 2400, 1600, 800, 256, 128};

struct LzCam {
    Uvc *a;
    Uvc *b;
    int locked;
    int seq;
    int w, h;
    int scored;
    int exp_a, gain_a, exp_b, gain_b;
    int user;
    int want_ea, want_ga, want_eb, want_gb;
    double tune_a, tune_b;
    int sweep_a, sweep_b;
    int best_a, best_ae, best_ag;
    int best_b, best_be, best_bg;
    int cols, sharp, clip_a, p50_a, p95_a, metered;
    int clip_b, p50_b, p95_b, metered_b;
    char status[192];
};

static double mono_sec(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

static void level(const uint8_t *y, int w, int h, int *p50, int *p95, int *peak, int *clip) {
    int hist[256];
    int row, col, n = 0, i, acc = 0, got50 = 0, hi = 0;
    memset(hist, 0, sizeof hist);
    *p50 = *p95 = *peak = *clip = 0;
    if (!y || w < 2 || h < 2) return;
    for (row = 0; row < h; row += 4) {
        const uint8_t *p = y + (size_t)row * (size_t)w;
        for (col = 0; col < w; col += 4) {
            hist[p[col]]++;
            n++;
        }
    }
    if (!n) return;
    for (i = 255; i >= 0; i--) {
        if (hist[i]) {
            *peak = i;
            break;
        }
    }
    for (i = 230; i < 256; i++) hi += hist[i];
    *clip = hi * 100 / n;
    for (i = 0; i < 256; i++) {
        acc += hist[i];
        if (!got50 && acc * 100 >= n * 50) {
            *p50 = i;
            got50 = 1;
        }
        if (acc * 100 >= n * 95) {
            *p95 = i;
            break;
        }
    }
}

/* Mean horizontal step, in tenths of a count. A focused lattice is tens. */
static int sharp_of(const uint8_t *y, int w, int h) {
    int row, col, acc = 0, n = 0;
    if (!y || w < 2 || h < 2) return 0;
    for (row = h / 5; row < (h * 4) / 5; row += 4) {
        const uint8_t *p = y + (size_t)row * (size_t)w;
        for (col = 1; col < w; col += 2) {
            int d = (int)p[col] - (int)p[col - 1];
            if (d < 0) d = -d;
            acc += d;
            n++;
        }
    }
    return n ? acc * 10 / n : 0;
}

/* Median count of 13 px lattice peaks on a row. 12 or more is a real pattern. */
static int column_score(const uint8_t *y, int w, int h) {
    int counts[48];
    int nrows = 0, r, i, acc;
    if (!y || w < 32 || h < 32) return 0;
    for (r = h / 8; r < (h * 7) / 8 && nrows < 48; r += h / 40 > 0 ? h / 40 : 1) {
        const uint8_t *row = y + (size_t)r * (size_t)w;
        int hp_at[1280];
        int np = 0, last = -100, x, lim = w < 1280 ? w : 1280;
        if (w > 1280) continue;
        for (x = 0; x < lim; x++) hp_at[x] = 0;
        for (x = 8; x < lim - 8; x++) {
            int t, s1 = 0, s6 = 0;
            for (t = x - 1; t <= x + 1; t++) s1 += row[t];
            for (t = x - 6; t <= x + 6; t++) s6 += row[t];
            hp_at[x] = s1 / 3 - s6 / 13;
        }
        for (x = 8; x < lim - 8; x++) {
            int hp = hp_at[x];
            if (hp < 6) continue;
            if (!(hp >= hp_at[x - 1] && hp > hp_at[x + 1] && hp >= hp_at[x - 2] && hp > hp_at[x + 2]))
                continue;
            if (x - last < 8) continue;
            np++;
            last = x;
        }
        counts[nrows++] = np > 80 ? 80 : np;
    }
    if (!nrows) return 0;
    for (i = 1; i < nrows; i++) {
        int v = counts[i], j = i;
        while (j > 0 && counts[j - 1] > v) {
            counts[j] = counts[j - 1];
            j--;
        }
        counts[j] = v;
    }
    acc = counts[nrows / 2];
    return acc;
}

static void save_pgm(const char *path, const uint8_t *y, int w, int h) {
    FILE *f = fopen(path, "wb");
    if (!f) return;
    fprintf(f, "P5\n%d %d\n255\n", w, h);
    fwrite(y, 1, (size_t)w * (size_t)h, f);
    fclose(f);
}

/* Short exposures first. The NIR lamp is a frame strobe, so a long shutter
 * only adds room light and hides the columns. Camera B wants a mid grey. */
static const int k_a_exp[] = {4, 2, 1, 6, 8, 4, 2, 3, 8, 12};
static const int k_a_gain[] = {8, 8, 12, 8, 4, 20, 30, 16, 16, 4};
static const int k_b_exp[] = {4, 2, 1, 6, 8, 3, 4};
static const int k_b_gain[] = {2, 2, 0, 2, 2, 4, 6};

static void apply_exp(Uvc *dev, int *exp, int *gain, int e, int g, int restream) {
    if (e < 1) e = 1;
    if (e > 80) e = 80;
    if (g < 0) g = 0;
    if (g > 80) g = 80;
    if (e == *exp && g == *gain) return;
    *exp = e;
    *gain = g;
    uvc_set_exposure_gain(dev, e, g);
    if (restream) uvc_restream(dev);
}

/* Slider range. The automatic sweep stays on the shorter tuner cap above. */
static void apply_user(Uvc *dev, int *exp, int *gain, int e, int g) {
    if (!dev) return;
    if (e < 1) e = 1;
    if (e > 200) e = 200;
    if (g < 0) g = 0;
    if (g > 100) g = 100;
    if (e == *exp && g == *gain) return;
    *exp = e;
    *gain = g;
    uvc_set_exposure_gain(dev, e, g);
}

static void tune(LzCam *cam, Uvc *dev, const uint8_t *y, int color) {
    int p50 = 0, p95 = 0, peak = 0, clip = 0, cols = 0, sharp = 0;
    int *exp = color ? &cam->exp_b : &cam->exp_a;
    int *gain = color ? &cam->gain_b : &cam->gain_a;
    double *when = color ? &cam->tune_b : &cam->tune_a;
    double now;
    if (!dev || !y) return;
    now = mono_sec();
    if (now - *when < 0.8) return;
    *when = now;
    level(y, cam->w, cam->h, &p50, &p95, &peak, &clip);
    sharp = sharp_of(y, cam->w, cam->h);
    if (!color) {
        char path[96];
        int npre = (int)(sizeof k_a_exp / sizeof k_a_exp[0]);
        int key;
        cols = column_score(y, cam->w, cam->h);
        cam->cols = cols;
        cam->sharp = sharp;
        cam->clip_a = clip;
        cam->p50_a = p50;
        cam->p95_a = p95;
        cam->metered = 1;
        LOGI("see A exp %d gain %d cols %d sharp %d clip %d p50 %d p95 %d peak %d", *exp, *gain, cols,
             sharp, clip, p50, p95, peak);
        if (cam->user) return;
        if (cam->sweep_a >= 0 && cam->sweep_a < npre) {
            key = cols * 100000 + (clip > 30 ? 0 : sharp * 100) - clip * 10;
            if (key > cam->best_a) {
                cam->best_a = key;
                cam->best_ae = *exp;
                cam->best_ag = *gain;
            }
            snprintf(path, sizeof path, "/data/data/dev.lab.lizard/files/a-e%04d-g%03d.pgm", *exp, *gain);
            save_pgm(path, y, cam->w, cam->h);
            cam->sweep_a++;
            if (cam->sweep_a < npre) {
                apply_exp(dev, exp, gain, k_a_exp[cam->sweep_a], k_a_gain[cam->sweep_a], 0);
                return;
            }
            cam->sweep_a = -1;
            LOGI("sweep A best exp %d gain %d key %d", cam->best_ae, cam->best_ag, cam->best_a);
            apply_exp(dev, exp, gain, cam->best_ae, cam->best_ag, 0);
            return;
        }
        if (cols >= 8) LOGI("lattice lock cols %d exp %d gain %d", cols, *exp, *gain);
        if (clip > 12 && *exp > 1) apply_exp(dev, exp, gain, *exp > 3 ? *exp * 2 / 3 : *exp - 1, *gain, 0);
        else if (cols < 8 && clip < 5 && p50 < 40 && *gain < 48) apply_exp(dev, exp, gain, *exp, *gain + 4, 0);
        else if (cols < 8 && clip < 3 && p50 < 28 && *exp < 24) apply_exp(dev, exp, gain, *exp + 2, *gain, 0);
        else if (cols >= 8 && clip > 8 && *gain > 0) apply_exp(dev, exp, gain, *exp, *gain > 4 ? *gain - 4 : 0, 0);
        return;
    }
    LOGI("see B exp %d gain %d sharp %d clip %d p50 %d p95 %d peak %d", *exp, *gain, sharp, clip, p50, p95,
         peak);
    cam->clip_b = clip;
    cam->p50_b = p50;
    cam->p95_b = p95;
    cam->metered_b = 1;
    if (cam->user) return;
    {
        int npre = (int)(sizeof k_b_exp / sizeof k_b_exp[0]);
        int err = (p50 > 55 ? p50 - 55 : 55 - p50) + clip * 3;
        if (cam->sweep_b >= 0 && cam->sweep_b < npre) {
            if (cam->best_b == 0 || err < cam->best_b) {
                cam->best_b = err > 0 ? err : 1;
                cam->best_be = *exp;
                cam->best_bg = *gain;
            }
            cam->sweep_b++;
            if (cam->sweep_b < npre) {
                apply_exp(dev, exp, gain, k_b_exp[cam->sweep_b], k_b_gain[cam->sweep_b], 0);
                return;
            }
            cam->sweep_b = -1;
            LOGI("sweep B best exp %d gain %d err %d", cam->best_be, cam->best_bg, cam->best_b);
            apply_exp(dev, exp, gain, cam->best_be, cam->best_bg, 0);
            return;
        }
    }
    if ((clip > 8 || p50 > 110) && *exp > 1) apply_exp(dev, exp, gain, *exp > 2 ? *exp / 2 : *exp - 1, *gain, 0);
    else if ((clip > 8 || p50 > 110) && *gain > 0) apply_exp(dev, exp, gain, *exp, *gain - 1, 0);
    else if (p50 < 35 && clip < 3 && *exp < 40) apply_exp(dev, exp, gain, *exp + 2, *gain, 0);
    else if (p50 < 35 && clip < 3 && *gain < 40) apply_exp(dev, exp, gain, *exp, *gain + 4, 0);
}

static int edge_score(const uint8_t *y, int w, int h) {
    int row, col, acc = 0, n = 0;
    if (!y || w < 2 || h < 2) return 0;
    for (row = h / 4; row < (h * 3) / 4; row += 8) {
        const uint8_t *p = y + (size_t)row * (size_t)w;
        for (col = 1; col < w; col += 4) {
            int d = (int)p[col] - (int)p[col - 1];
            if (d < 0) d = -d;
            acc += d;
            n++;
        }
    }
    return n ? acc / n : 0;
}

static void refresh(LzCam *cam) {
    if (cam->a && cam->b) {
        snprintf(cam->status, sizeof cam->status, "A %d/%d c%d s%d k%d | B %d/%d | %s | %s", cam->exp_a,
                 cam->gain_a, cam->cols, cam->sharp, cam->clip_a, cam->exp_b, cam->gain_b, uvc_status(cam->a),
                 uvc_status(cam->b));
    }
}

void lz_cam_set_android(void *vm, void *activity) {
    if (uvc_init(vm, activity) != 0) LOGI("uvc init failed");
}

LzCam *lz_cam_open(int width, int height) {
    LzCam *cam = (LzCam *)calloc(1, sizeof *cam);
    int i;
    if (!cam) return NULL;
    cam->w = width;
    cam->h = height;
    cam->exp_a = 4;
    cam->gain_a = 8;
    cam->exp_b = 4;
    cam->gain_b = 2;
    cam->best_a = -2000000000;
    cam->best_ae = 4;
    cam->best_ag = 8;
    snprintf(cam->status, sizeof cam->status, "waiting for Fox USB");
    for (i = 0; i < (int)(sizeof k_caps / sizeof k_caps[0]); i++) {
        int err;
        cam->a = uvc_open(0x0c45, 0x636a, width, height, k_caps[i]);
        if (!cam->a) {
            err = uvc_last_error();
            snprintf(cam->status, sizeof cam->status, "waiting for Fox USB (%s)", uvc_last_error_str());
            if (err != 4) break;
            continue;
        }
        cam->b = uvc_open(0x0c45, 0x636b, width, height, k_caps[i]);
        if (cam->b) {
            cam->locked = 1;
            refresh(cam);
            LOGI("camera %s", cam->status);
            return cam;
        }
        err = uvc_last_error();
        snprintf(cam->status, sizeof cam->status, "waiting for Fox USB (%s)", uvc_last_error_str());
        uvc_close(cam->a);
        cam->a = NULL;
        if (err != 4) break;
        usleep(100000);
    }
    LOGI("camera %s", cam->status);
    return cam;
}

void lz_cam_close(LzCam *cam) {
    if (!cam) return;
    uvc_close(cam->a);
    uvc_close(cam->b);
    free(cam);
}

int lz_cam_copy(LzCam *cam, uint8_t *ya, uint8_t *yb, uint8_t *ub, uint8_t *vb, int *seq) {
    int ga, gb;
    if (seq) *seq = cam ? cam->seq : 0;
    if (!cam || !cam->a || !cam->b) return -2;
    ga = uvc_grab(cam->a, ya, NULL, NULL, cam->w, cam->h, 0);
    gb = uvc_grab(cam->b, yb, ub, vb, cam->w, cam->h, 1);
    refresh(cam);
    if (ga == -2 || gb == -2) return -2;
    if (ga == -1 || gb == -1) return -1;
    if (!cam->scored && ya && yb && (ga == 0 || gb == 0)) {
        cam->scored = 1;
        LOGI("contrast A %d B %d", edge_score(ya, cam->w, cam->h), edge_score(yb, cam->w, cam->h));
    }
    if (ga == 0) tune(cam, cam->a, ya, 0);
    if (gb == 0) tune(cam, cam->b, yb, 1);
    if (ga == 0 || gb == 0) {
        cam->seq++;
        if (seq) *seq = cam->seq;
        return 0;
    }
    if (seq) *seq = cam->seq;
    return 1;
}

int lz_cam_locked(const LzCam *cam) { return cam && cam->locked; }

int lz_cam_cols(const LzCam *cam) { return cam ? cam->cols : 0; }

int lz_cam_meter(const LzCam *cam, int *cols, int *clip, int *p50, int *p95) {
    if (!cam || !cam->metered) {
        if (cols) *cols = 0;
        if (clip) *clip = 0;
        if (p50) *p50 = 0;
        if (p95) *p95 = 0;
        return 0;
    }
    if (cols) *cols = cam->cols;
    if (clip) *clip = cam->clip_a;
    if (p50) *p50 = cam->p50_a;
    if (p95) *p95 = cam->p95_a;
    return 1;
}

int lz_cam_meter_b(const LzCam *cam, int *clip, int *p50, int *p95) {
    if (!cam || !cam->metered_b) {
        if (clip) *clip = 0;
        if (p50) *p50 = 0;
        if (p95) *p95 = 0;
        return 0;
    }
    if (clip) *clip = cam->clip_b;
    if (p50) *p50 = cam->p50_b;
    if (p95) *p95 = cam->p95_b;
    return 1;
}

void lz_cam_set_user(LzCam *cam, int exp_a, int gain_a, int exp_b, int gain_b) {
    if (!cam) return;
    if (exp_a < 1) exp_a = 1;
    if (exp_a > 200) exp_a = 200;
    if (exp_b < 1) exp_b = 1;
    if (exp_b > 200) exp_b = 200;
    if (gain_a < 0) gain_a = 0;
    if (gain_a > 100) gain_a = 100;
    if (gain_b < 0) gain_b = 0;
    if (gain_b > 100) gain_b = 100;
    cam->user = 1;
    cam->want_ea = exp_a;
    cam->want_ga = gain_a;
    cam->want_eb = exp_b;
    cam->want_gb = gain_b;
    apply_user(cam->a, &cam->exp_a, &cam->gain_a, exp_a, gain_a);
    apply_user(cam->b, &cam->exp_b, &cam->gain_b, exp_b, gain_b);
}

const char *lz_cam_status(const LzCam *cam) { return cam ? cam->status : "no camera"; }

#endif
