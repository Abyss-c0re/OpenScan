/* Live model. Mold and cloud keep confirmed projector points. Shapes in a
 * 3D matrix locate the turn, so a new side can join without the silhouette.
 * Measured is the projector sheet in front of the scanner. A photograph is
 * not wrapped around the turn axis. No image library. */
#include "openscan/openscan_live.h"
#include "os_logic.h"
#include "os_stripe.h"
#include "os_cloud.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
    float x, y, z, tone;
    int ok;
} Vert;

typedef struct {
    float a[3], b[3], c[3];
    float tone;
} Tri;

struct openscan_live {
    openscan_calib cal;
    int has_cal;
    openscan_build build;
    float distance;
    int mode;
    int shape;
    Vert *grid;
    int gw, gh;
    Tri *model;
    int nmodel, cap;
    unsigned *cell;
    int ncell;
    unsigned shell_gen;
    int scanned_bins;
    uint8_t seen[72];
    uint8_t seen_hold[72];
    uint8_t *preview;
    uint8_t *panel[5];
    int pw, ph;
    int quit_flag;
    int save_flag;
    os_turn turn;
    float prev_cx, prev_top;
    int have_prev;
    float axis_x, axis_y, axis_z;
    int have_axis;
    float orbit_yaw, orbit_pitch;
    uint8_t *view_front;
    uint8_t *view_side;
    int view_fw, view_fh, view_sw, view_sh;
    float *zbuf;
    int zcap;
    int cache_n, cache_yq, cache_pq, cache_ok;
    Tri *held_preview;
    int nheld, hold_tick;
    int *row_mid;
    int row_n;
    int have_rows;
    int fused, lost;
    uint8_t *prev_gray;
    int prev_gw, prev_gh, prev_ready;
    os_cloud *cloud;
};

static int clampi(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void default_build(openscan_build *b) {
    memset(b, 0, sizeof *b);
    b->near_mm = 80.f;
    b->far_mm = 500.f;
    b->stride = 1;
    b->smooth = 2;
    b->sweep_deg = 80;
    b->relief = 40;
    b->solid = 1;
    b->flip = 1;
}

openscan_live *openscan_live_create(const openscan_calib *calib, const openscan_scan_opts *opt) {
    openscan_live *L = calloc(1, sizeof *L);
    if (!L) return NULL;
    (void)opt;
    if (calib) {
        L->cal = *calib;
        L->has_cal = 1;
    }
    default_build(&L->build);
    L->distance = (float)openscan_default_distance_mm();
    L->shape = OPENSCAN_SHAPE_MOLD;
    L->mode = OPENSCAN_MODE_STOP;
    L->cap = 60000;
    L->model = calloc((size_t)L->cap, sizeof(Tri));
    L->ncell = 1 << 18;
    L->cell = calloc((size_t)L->ncell, sizeof(unsigned));
    L->pw = 1180;
    L->ph = 520;
    L->preview = calloc((size_t)L->pw * L->ph * 3, 1);
    for (int i = 0; i < 5; i++) L->panel[i] = calloc((size_t)L->pw * L->ph * 3, 1);
    L->cloud = os_cloud_create();
    if (!L->model || !L->cell || !L->preview || !L->cloud) {
        openscan_live_destroy(L);
        return NULL;
    }
    return L;
}

void openscan_live_destroy(openscan_live *live) {
    if (!live) return;
    free(live->grid);
    free(live->model);
    free(live->cell);
    free(live->preview);
    free(live->view_front);
    free(live->view_side);
    free(live->zbuf);
    free(live->held_preview);
    free(live->row_mid);
    free(live->prev_gray);
    os_cloud_free(live->cloud);
    for (int i = 0; i < 5; i++) free(live->panel[i]);
    free(live);
}

void openscan_live_reset(openscan_live *live) {
    if (!live) return;
    live->nmodel = 0;
    memset(&live->turn, 0, sizeof live->turn);
    live->have_prev = 0;
    live->have_axis = 0;
    live->scanned_bins = 0;
    live->fused = 0;
    live->lost = 0;
    live->nheld = 0;
    live->cache_ok = 0;
    live->have_rows = 0;
    memset(live->seen, 0, sizeof live->seen);
    memset(live->seen_hold, 0, sizeof live->seen_hold);
    memset(live->cell, 0, (size_t)live->ncell * sizeof(unsigned));
    os_cloud_reset(live->cloud);
}

void openscan_live_set_mode(openscan_live *live, int mode) {
    if (live) live->mode = mode;
}
int openscan_live_mode(const openscan_live *live) { return live ? live->mode : OPENSCAN_MODE_STOP; }

void openscan_live_set_shape(openscan_live *live, int shape) {
    int next = OPENSCAN_SHAPE_MOLD;
    if (!live) return;
    if (shape == OPENSCAN_SHAPE_MEASURED) next = OPENSCAN_SHAPE_MEASURED;
    else if (shape == OPENSCAN_SHAPE_CLOUD) next = OPENSCAN_SHAPE_CLOUD;
    if (live->shape != next) openscan_live_reset(live);
    live->shape = next;
}
int openscan_live_shape(const openscan_live *live) {
    return live ? live->shape : OPENSCAN_SHAPE_MOLD;
}

void openscan_live_set_distance_mm(openscan_live *live, float mm) {
    if (!live) return;
    live->distance = openscan_outline_distance_mm(mm);
}
float openscan_live_distance_mm(const openscan_live *live) {
    return live ? live->distance : (float)openscan_default_distance_mm();
}
void openscan_live_set_build(openscan_live *live, const openscan_build *build) {
    if (live && build) live->build = *build;
}
void openscan_live_get_build(const openscan_live *live, openscan_build *build) {
    if (live && build) *build = live->build;
}
int openscan_live_points(const openscan_live *live) { return live ? live->nmodel : 0; }
void openscan_live_mouse(openscan_live *live, int event, int x, int y, int flags) {
    (void)live; (void)event; (void)x; (void)y; (void)flags;
}
void openscan_live_orbit(openscan_live *live, float dyaw, float dpitch) {
    if (!live) return;
    live->orbit_yaw += dyaw;
    live->orbit_pitch += dpitch;
    if (live->orbit_pitch > 1.2f) live->orbit_pitch = 1.2f;
    if (live->orbit_pitch < -1.2f) live->orbit_pitch = -1.2f;
}
void openscan_live_key(openscan_live *live, int key) {
    if (!live) return;
    if (key == 'q' || key == 'Q' || key == 27) live->quit_flag = 1;
    if (key == 's' || key == 'S') live->save_flag = 1;
    if (key == ' ') {
        live->mode = live->mode == OPENSCAN_MODE_SCAN ? OPENSCAN_MODE_PAUSE : OPENSCAN_MODE_SCAN;
    }
}
int openscan_live_take_save(openscan_live *live) {
    if (!live || !live->save_flag) return 0;
    live->save_flag = 0;
    return 1;
}
int openscan_live_quit(const openscan_live *live) { return live && live->quit_flag; }

static void flip180(uint8_t *p, int n) {
    for (int i = 0; i < n / 2; i++) {
        uint8_t t = p[i];
        p[i] = p[n - 1 - i];
        p[n - 1 - i] = t;
    }
}

static void box_blur(const uint8_t *src, uint8_t *dst, int w, int h, int r) {
    /* Two-pass box. r is the radius in pixels. */
    int *tmp = malloc((size_t)w * h * sizeof(int));
    if (!tmp || r < 1) {
        if (dst != src) memcpy(dst, src, (size_t)w * h);
        free(tmp);
        return;
    }
    for (int y = 0; y < h; y++) {
        int acc = 0;
        for (int x = 0; x < w; x++) {
            acc += src[y * w + x];
            if (x >= 2 * r + 1) acc -= src[y * w + x - (2 * r + 1)];
            int n = x < 2 * r ? x + r + 1 : (x >= w - r ? w - (x - r) : 2 * r + 1);
            if (n < 1) n = 1;
            int xx = x - r;
            if (xx < 0) xx = 0;
            tmp[y * w + xx] = acc / n;
        }
    }
    for (int x = 0; x < w; x++) {
        int acc = 0;
        for (int y = 0; y < h; y++) {
            acc += tmp[y * w + x];
            if (y >= 2 * r + 1) acc -= tmp[(y - (2 * r + 1)) * w + x];
            int n = y < 2 * r ? y + r + 1 : (y >= h - r ? h - (y - r) : 2 * r + 1);
            if (n < 1) n = 1;
            int yy = y - r;
            if (yy < 0) yy = 0;
            dst[yy * w + x] = (uint8_t)(acc / n);
        }
    }
    free(tmp);
}

static int largest_mask(uint8_t *mask, int w, int h) {
    int *lab = calloc((size_t)w * h, sizeof(int));
    int *st = malloc((size_t)w * h * sizeof(int));
    if (!lab || !st) {
        free(lab);
        free(st);
        return 0;
    }
    int id = 0, best = 0, bestn = 0;
    for (int i = 0; i < w * h; i++) {
        if (!mask[i] || lab[i]) continue;
        int sp = 0, n = 0;
        id++;
        st[sp++] = i;
        lab[i] = id;
        while (sp) {
            int p = st[--sp];
            int x = p % w, y = p / w;
            n++;
            const int nx[4] = {1, -1, 0, 0};
            const int ny[4] = {0, 0, 1, -1};
            for (int k = 0; k < 4; k++) {
                int xx = x + nx[k], yy = y + ny[k];
                if (xx < 0 || yy < 0 || xx >= w || yy >= h) continue;
                int q = yy * w + xx;
                if (!mask[q] || lab[q]) continue;
                lab[q] = id;
                st[sp++] = q;
            }
        }
        if (n > bestn) {
            bestn = n;
            best = id;
        }
    }
    if (bestn < 200) {
        memset(mask, 0, (size_t)w * h);
    } else {
        for (int i = 0; i < w * h; i++) mask[i] = lab[i] == best ? 255 : 0;
    }
    free(lab);
    free(st);
    return bestn;
}

static void drop_platter(uint8_t *mask, int w, int h) {
    int *width = calloc((size_t)h, sizeof(int));
    if (!width) return;
    int top = h, bot = -1;
    for (int y = 0; y < h; y++) {
        int l = -1, r = -1;
        for (int x = 0; x < w; x++)
            if (mask[y * w + x]) { l = x; break; }
        for (int x = w - 1; x >= 0; x--)
            if (mask[y * w + x]) { r = x; break; }
        if (l < 0) continue;
        width[y] = r - l;
        if (y < top) top = y;
        bot = y;
    }
    int span = bot - top;
    if (span > 40) {
        int mid[512], nm = 0;
        for (int y = top + span / 5; y < top + span * 3 / 5 && nm < 512; y++)
            if (width[y] > 10) mid[nm++] = width[y];
        if (nm >= 8) {
            for (int i = 0; i < nm; i++)
                for (int j = i + 1; j < nm; j++)
                    if (mid[j] < mid[i]) {
                        int t = mid[i];
                        mid[i] = mid[j];
                        mid[j] = t;
                    }
            int body = mid[nm / 2];
            for (int y = top; y <= bot; y++) {
                int lower = y >= top + span * 3 / 5;
                int upper = y < top + span / 6;
                if ((lower || upper) && width[y] > body * 3 / 2)
                    memset(mask + y * w, 0, (size_t)w);
            }
        }
    }
    free(width);
}

static void row_edges(const uint8_t *mask, int w, int h, int *left, int *right) {
    for (int y = 0; y < h; y++) {
        left[y] = -1;
        right[y] = -1;
        for (int x = 0; x < w; x++)
            if (mask[y * w + x]) { left[y] = x; break; }
        for (int x = w - 1; x >= 0; x--)
            if (mask[y * w + x]) { right[y] = x; break; }
    }
}

static void rot_y(float yaw, float ax, float az, float *x, float *z) {
    float dx = *x - ax, dz = *z - az;
    float c = os_cos(yaw), s = os_sin(yaw);
    *x = ax + c * dx + s * dz;
    *z = az - s * dx + c * dz;
}

static void rot_x(float pitch, float *y, float *z) {
    float c = os_cos(pitch), s = os_sin(pitch);
    float y0 = *y, z0 = *z;
    *y = y0 * c - z0 * s;
    *z = y0 * s + z0 * c;
}

static int cell_new(openscan_live *L, float x, float y, float z) {
    int ix = os_lround(x / 0.75f);
    int iy = os_lround(y / 0.75f);
    int iz = os_lround(z / 0.75f);
    unsigned h = (unsigned)(ix * 73856093 ^ iy * 19349663 ^ iz * 83492791);
    unsigned key = (L->shell_gen << 16) | (h & 0xFFFFu);
    unsigned slot = h & (unsigned)(L->ncell - 1);
    for (int n = 0; n < 32; n++) {
        unsigned i = (slot + (unsigned)n) & (unsigned)(L->ncell - 1);
        unsigned got = L->cell[i];
        if (got == 0) {
            L->cell[i] = key;
            return 1;
        }
        if ((got & 0xFFFFu) != (h & 0xFFFFu)) continue;
        /* Same spot in this shell still gets its triangle. An older shell blocks it. */
        return (got >> 16) == L->shell_gen;
    }
    return 0;
}

static int bin_of(const openscan_live *L, float x, float z) {
    float ang = os_atan2(x - L->axis_x, -(z - L->axis_z));
    int b = (int)((ang + 3.14159265f) / (2.f * 3.14159265f) * 72.f);
    if (b < 0) b = 0;
    if (b > 71) b = 71;
    return b;
}

static void mark_bin(openscan_live *L, float x, float z) {
    L->seen[bin_of(L, x, z)] = 1;
}

static int count_bins(const openscan_live *L) {
    int n = 0;
    for (int i = 0; i < 72; i++) n += L->seen[i] ? 1 : 0;
    return n;
}

static void add_tri(openscan_live *L, const Vert *a, const Vert *b, const Vert *c, float yaw) {
    if (L->nmodel >= L->cap) return;
    Tri t;
    t.a[0] = a->x; t.a[1] = a->y; t.a[2] = a->z;
    t.b[0] = b->x; t.b[1] = b->y; t.b[2] = b->z;
    t.c[0] = c->x; t.c[1] = c->y; t.c[2] = c->z;
    rot_y(-yaw, L->axis_x, L->axis_z, &t.a[0], &t.a[2]);
    rot_y(-yaw, L->axis_x, L->axis_z, &t.b[0], &t.b[2]);
    rot_y(-yaw, L->axis_x, L->axis_z, &t.c[0], &t.c[2]);
    float mx = (t.a[0] + t.b[0] + t.c[0]) / 3.f;
    float my = (t.a[1] + t.b[1] + t.c[1]) / 3.f;
    float mz = (t.a[2] + t.b[2] + t.c[2]) / 3.f;
    /* A side already kept is not drawn again as a second sheet. */
    if (L->shape == OPENSCAN_SHAPE_MEASURED && L->seen_hold[bin_of(L, mx, mz)]) return;
    if (!cell_new(L, mx, my, mz)) return;
    t.tone = (a->tone + b->tone + c->tone) / 3.f;
    L->model[L->nmodel++] = t;
    mark_bin(L, mx, mz);
}

static void gray_bgr(uint8_t *dst, int dw, int dh, const uint8_t *src, int sw, int sh) {
    for (int y = 0; y < dh; y++) {
        int sy = y * sh / dh;
        for (int x = 0; x < dw; x++) {
            int sx = x * sw / dw;
            uint8_t g = src[sy * sw + sx];
            int i = (y * dw + x) * 3;
            dst[i] = dst[i + 1] = dst[i + 2] = g;
        }
    }
}

static void paint_model(uint8_t *dst, float *zbuf, int dw, int dh, const Tri *tris, int n,
                        float yaw, float pitch, float ax, float ay, float az) {
    memset(dst, 12, (size_t)dw * dh * 3);
    if (!zbuf) return;
    for (int i = 0; i < dw * dh; i++) zbuf[i] = 1e9f;
    float minx = 1e9f, maxx = -1e9f, miny = 1e9f, maxy = -1e9f;
    for (int i = 0; i < n; i++) {
        const float *p[3] = {tris[i].a, tris[i].b, tris[i].c};
        for (int k = 0; k < 3; k++) {
            float x = p[k][0], y = p[k][1], z = p[k][2];
            rot_y(yaw, ax, az, &x, &z);
            rot_x(pitch, &y, &z);
            if (x < minx) minx = x;
            if (x > maxx) maxx = x;
            if (y < miny) miny = y;
            if (y > maxy) maxy = y;
        }
    }
    float ext = maxx - minx;
    if (maxy - miny > ext) ext = maxy - miny;
    if (ext < 1.f) ext = 1.f;
    float scale = 0.86f * (float)dh / ext;
    float cx = 0.5f * (minx + maxx);
    float cy = 0.5f * (miny + maxy);
    for (int i = 0; i < n; i++) {
        float xs[3], ys[3], zs[3], xm[3], ym[3], zm[3];
        const float *p[3] = {tris[i].a, tris[i].b, tris[i].c};
        for (int k = 0; k < 3; k++) {
            float x = p[k][0], y = p[k][1], z = p[k][2];
            rot_y(yaw, ax, az, &x, &z);
            rot_x(pitch, &y, &z);
            xm[k] = x;
            ym[k] = y;
            zm[k] = z;
            xs[k] = dw * 0.5f + (x - cx) * scale;
            ys[k] = dh * 0.5f - (y - cy) * scale;
            zs[k] = z;
        }
        float area = (xs[1] - xs[0]) * (ys[2] - ys[0]) - (ys[1] - ys[0]) * (xs[2] - xs[0]);
        if (os_fabs(area) < 0.2f) continue;
        int minpx = (int)os_min(xs[0], os_min(xs[1], xs[2]));
        int maxpx = (int)os_max(xs[0], os_max(xs[1], xs[2]));
        int minpy = (int)os_min(ys[0], os_min(ys[1], ys[2]));
        int maxpy = (int)os_max(ys[0], os_max(ys[1], ys[2]));
        if (minpx < 0) minpx = 0;
        if (minpy < 0) minpy = 0;
        if (maxpx >= dw) maxpx = dw - 1;
        if (maxpy >= dh) maxpy = dh - 1;
        if (minpx > maxpx || minpy > maxpy) {
            int cxp = clampi((int)((xs[0] + xs[1] + xs[2]) / 3.f), 0, dw - 1);
            int cyp = clampi((int)((ys[0] + ys[1] + ys[2]) / 3.f), 0, dh - 1);
            int id = cyp * dw + cxp;
            dst[id * 3] = dst[id * 3 + 1] = dst[id * 3 + 2] = 180;
            continue;
        }
        float e1x, e1y, e1z, e2x, e2y, e2z, nx, ny, nz, ln, lam;
        if (tris[i].tone > 1.5f) {
            e1x = xm[1] - xm[0]; e1y = ym[1] - ym[0]; e1z = zm[1] - zm[0];
            e2x = xm[2] - xm[0]; e2y = ym[2] - ym[0]; e2z = zm[2] - zm[0];
        } else {
            e1x = xs[1] - xs[0]; e1y = ys[1] - ys[0]; e1z = zs[1] - zs[0];
            e2x = xs[2] - xs[0]; e2y = ys[2] - ys[0]; e2z = zs[2] - zs[0];
        }
        nx = e1y * e2z - e1z * e2y;
        ny = e1z * e2x - e1x * e2z;
        nz = e1x * e2y - e1y * e2x;
        if (tris[i].tone > 1.5f && nz > 0.f) {
            nx = -nx;
            ny = -ny;
            nz = -nz;
        }
        ln = os_sqrt(nx * nx + ny * ny + nz * nz) + 1e-6f;
        lam = ((-0.2f * nx + 0.35f * ny - 0.9f * nz) / ln);
        if (lam < 0.f) lam = 0.f;
        if (lam > 1.f) lam = 1.f;
        /* tone above 1 is a measured skin: light the shape. A photograph is not the model. */
        int shade = tris[i].tone > 1.5f
                        ? (int)(255.f * (0.15f + 0.85f * lam))
                        : (int)(255.f * (tris[i].tone * 0.82f + lam * 0.18f));
        if (shade < 0) shade = 0;
        if (shade > 255) shade = 255;
        float x0 = xs[0], y0 = ys[0], x1 = xs[1], y1 = ys[1], x2 = xs[2], y2 = ys[2];
        float dw0 = y1 - y2, dw1 = y2 - y0, dw2 = y0 - y1;
        float dh0 = x2 - x1, dh1 = x0 - x2, dh2 = x1 - x0;
        float py0 = (float)minpy + 0.5f, px0 = (float)minpx + 0.5f;
        float w0r = (x1 - px0) * (y2 - py0) - (y1 - py0) * (x2 - px0);
        float w1r = (x2 - px0) * (y0 - py0) - (y2 - py0) * (x0 - px0);
        float w2r = (x0 - px0) * (y1 - py0) - (y0 - py0) * (x1 - px0);
        float inv = 1.f / area;
        float z0 = zs[0], z1 = zs[1], z2 = zs[2];
        float eps = 0.6f * (os_fabs(dw0) + os_fabs(dw1) + os_fabs(dw2) + os_fabs(dh0) + os_fabs(dh1) + os_fabs(dh2)) / 6.f;
        for (int py = minpy; py <= maxpy; py++) {
            float w0 = w0r, w1 = w1r, w2 = w2r;
            int row = py * dw;
            for (int px = minpx; px <= maxpx; px++) {
                int pos = w0 >= -eps && w1 >= -eps && w2 >= -eps;
                int neg = w0 <= eps && w1 <= eps && w2 <= eps;
                if (pos || neg) {
                    float zc = (w0 * z0 + w1 * z1 + w2 * z2) * inv;
                    int id = row + px;
                    if (zc < zbuf[id]) {
                        zbuf[id] = zc;
                        dst[id * 3] = dst[id * 3 + 1] = dst[id * 3 + 2] = (uint8_t)shade;
                    }
                }
                w0 += dw0;
                w1 += dw1;
                w2 += dw2;
            }
            w0r += dh0;
            w1r += dh1;
            w2r += dh2;
        }
    }
    /* Shade the measured skin from the depth of the finished picture.
     * A 3 px triangle's own normal is the crease, not the head.
     * The sample is wide enough that a one-millimetre stripe step is not a ditch. */
    if (n > 0 && tris[0].tone > 1.5f) {
        int k = 24;
        for (int py = 0; py < dh; py++) {
            for (int px = 0; px < dw; px++) {
                float zc, zl, zr, zu, zd, gx, gy, nx, ny, nz, ln, lam;
                int id = py * dw + px;
                int xl, xr, yu, yd;
                if (zbuf[id] > 1e8f) continue;
                zc = zbuf[id];
                xl = px - k;
                xr = px + k;
                yu = py - k;
                yd = py + k;
                zl = zr = zu = zd = zc;
                if (xl >= 0 && zbuf[py * dw + xl] < 1e8f && os_fabs(zbuf[py * dw + xl] - zc) < 12.f)
                    zl = zbuf[py * dw + xl];
                else
                    xl = px;
                if (xr < dw && zbuf[py * dw + xr] < 1e8f && os_fabs(zbuf[py * dw + xr] - zc) < 12.f)
                    zr = zbuf[py * dw + xr];
                else
                    xr = px;
                if (yu >= 0 && zbuf[yu * dw + px] < 1e8f && os_fabs(zbuf[yu * dw + px] - zc) < 12.f)
                    zu = zbuf[yu * dw + px];
                else
                    yu = py;
                if (yd < dh && zbuf[yd * dw + px] < 1e8f && os_fabs(zbuf[yd * dw + px] - zc) < 12.f)
                    zd = zbuf[yd * dw + px];
                else
                    yd = py;
                gx = (xr != xl) ? (zr - zl) * scale / (float)(xr - xl) : 0.f;
                gy = (yd != yu) ? (zd - zu) * scale / (float)(yu - yd) : 0.f;
                nx = -gx;
                ny = -gy;
                nz = 1.f;
                if (nz > 0.f) {
                    nx = -nx;
                    ny = -ny;
                    nz = -nz;
                }
                ln = os_sqrt(nx * nx + ny * ny + nz * nz) + 1e-6f;
                lam = ((-0.2f * nx + 0.35f * ny - 0.9f * nz) / ln);
                if (lam < 0.f) lam = 0.f;
                if (lam > 1.f) lam = 1.f;
                {
                    int shade = (int)(255.f * (0.22f + 0.78f * lam));
                    if (shade < 0) shade = 0;
                    if (shade > 255) shade = 255;
                    dst[id * 3] = dst[id * 3 + 1] = dst[id * 3 + 2] = (uint8_t)shade;
                }
            }
        }
    }
    (void)ay;
}

static void copy_tri(Tri *t, const Vert *a, const Vert *b, const Vert *c) {
    t->a[0] = a->x; t->a[1] = a->y; t->a[2] = a->z;
    t->b[0] = b->x; t->b[1] = b->y; t->b[2] = b->z;
    t->c[0] = c->x; t->c[1] = c->y; t->c[2] = c->z;
    t->tone = (a->tone + b->tone + c->tone) / 3.f;
}

__attribute__((unused)) static int build_shell(openscan_live *L, const uint8_t *gray, int w, int h,
                                                      int *ntri_out,
                       Tri *preview, int preview_cap, int *preview_n, int commit) {
    uint8_t *mask = malloc((size_t)w * h);
    uint8_t *blur = malloc((size_t)w * h);
    int *left = malloc((size_t)h * sizeof(int));
    int *right = malloc((size_t)h * sizeof(int));
    if (!mask || !blur || !left || !right) {
        free(mask); free(blur); free(left); free(right);
        return 0;
    }
    for (int i = 0; i < w * h; i++) mask[i] = gray[i] > 28 ? 255 : 0;
    int fg = largest_mask(mask, w, h);
    drop_platter(mask, w, h);
    largest_mask(mask, w, h);
    row_edges(mask, w, h, left, right);
    box_blur(gray, blur, w, h, 4);
    int stride = L->build.stride < 1 ? 1 : L->build.stride;
    if (stride > 6) stride = 6;
    int ystep = 3 * stride;
    int gh = (h + ystep - 1) / ystep;
    int gw = 112 / stride;
    if (gw < 24) gw = 24;
    Vert *grid = calloc((size_t)gw * gh, sizeof(Vert));
    if (!grid) {
        free(mask); free(blur); free(left); free(right);
        return 0;
    }
    float z0 = openscan_outline_distance_mm(L->distance);
    double fx = L->has_cal && L->cal.cam[1].fx > 100.0 ? L->cal.cam[1].fx : (w * 0.9);
    double fy = L->has_cal && L->cal.cam[1].fy > 100.0 ? L->cal.cam[1].fy : fx;
    double cx = L->has_cal ? L->cal.cam[1].cx : w * 0.5;
    double cy = L->has_cal ? L->cal.cam[1].cy : h * 0.5;
    if (L->build.flip) {
        cx = (w - 1) - cx;
        cy = (h - 1) - cy;
    }
    float sweep = (float)L->build.sweep_deg;
    if (sweep < 60.f) sweep = 60.f;
    float th_max = sweep * 3.14159265f / 180.f;
    float sin_max = os_sin(th_max);
    if (sin_max < 0.05f) sin_max = 0.05f;
    float sx = 0, sy = 0;
    int nok = 0;
    for (int iy = 0; iy < gh; iy++) {
        int y = iy * ystep;
        if (y >= h) y = h - 1;
        float half = 0.5f * (float)(right[y] - left[y]);
        float mid_x = 0.5f * (float)(right[y] + left[y]);
        int row = left[y] >= 0 && right[y] > left[y] && half > 8.f;
        for (int ix = 0; ix < gw; ix++) {
            Vert *v = &grid[iy * gw + ix];
            if (!row) continue;
            float u = -1.f + 2.f * (float)ix / (float)(gw - 1);
            float th = u * th_max;
            float x_geom = mid_x + half * (os_sin(th) / sin_max);
            int x = clampi(os_lround(x_geom), 0, w - 1);
            float radius = half * z0 / (float)fx;
            v->x = (x_geom - (float)cx) * z0 / (float)fx;
            v->y = -((float)y - (float)cy) * z0 / (float)fy;
            v->z = z0 - radius * os_cos(th);
            if (v->z < L->build.near_mm || v->z > L->build.far_mm) continue;
            float bump = ((float)gray[y * w + x] - (float)blur[y * w + x]) / 40.f;
            bump *= L->build.relief / 40.f;
            if (bump > 1.f) bump = 1.f;
            if (bump < -1.f) bump = -1.f;
            v->z += bump * 2.f;
            v->tone = gray[y * w + x] / 255.f;
            v->ok = 1;
            sx += v->x; sy += v->y;
            nok++;
        }
    }
    int ntri = 0;
    if (commit) {
        if (++L->shell_gen >= 65535u) {
            memset(L->cell, 0, (size_t)L->ncell * sizeof(unsigned));
            L->shell_gen = 1;
        }
    }
    if (commit && nok > 80 && L->build.solid && !L->have_axis) {
        L->axis_x = sx / (float)nok;
        L->axis_y = sy / (float)nok;
        L->axis_z = z0;
        L->have_axis = 1;
    }
    if (commit && L->shape == OPENSCAN_SHAPE_MEASURED)
        memcpy(L->seen_hold, L->seen, sizeof L->seen);
    if ((commit && nok > 80 && L->build.solid) || preview) {
        for (int iy = 0; iy < gh - 1; iy++) {
            for (int ix = 0; ix < gw - 1; ix++) {
                Vert *v00 = &grid[iy * gw + ix];
                Vert *v10 = &grid[iy * gw + ix + 1];
                Vert *v01 = &grid[(iy + 1) * gw + ix];
                Vert *v11 = &grid[(iy + 1) * gw + ix + 1];
                if (v00->ok && v10->ok && v01->ok) {
                    if (commit && L->build.solid) {
                        add_tri(L, v00, v10, v01, L->turn.yaw);
                        ntri++;
                    }
                    if (preview && preview_n && *preview_n < preview_cap)
                        copy_tri(&preview[(*preview_n)++], v00, v10, v01);
                }
                if (v10->ok && v11->ok && v01->ok) {
                    if (commit && L->build.solid) {
                        add_tri(L, v10, v11, v01, L->turn.yaw);
                        ntri++;
                    }
                    if (preview && preview_n && *preview_n < preview_cap)
                        copy_tri(&preview[(*preview_n)++], v10, v11, v01);
                }
            }
        }
    }
    free(L->grid);
    L->grid = grid;
    L->gw = gw;
    L->gh = gh;
    free(mask);
    free(blur);
    free(left);
    free(right);
    L->scanned_bins = count_bins(L);
    (void)fg;
    if (ntri_out) *ntri_out = ntri;
    return nok;
}

static void paste(uint8_t *dst, int dw, int dh, const uint8_t *src, int sw, int sh, int x0, int y0) {
    for (int y = 0; y < sh && y0 + y < dh; y++) {
        if (y0 + y < 0) continue;
        int n = sw;
        if (x0 + n > dw) n = dw - x0;
        if (n <= 0 || x0 < 0) continue;
        memcpy(dst + ((y0 + y) * dw + x0) * 3, src + y * sw * 3, (size_t)n * 3);
    }
}

static void scale_bgr(uint8_t *dst, int dw, int dh, const uint8_t *src, int sw, int sh) {
    if (sw < 1 || sh < 1) return;
    for (int y = 0; y < dh; y++) {
        int sy = y * sh / dh;
        if (sy >= sh) sy = sh - 1;
        const uint8_t *row = src + sy * sw * 3;
        uint8_t *out = dst + y * dw * 3;
        for (int x = 0; x < dw; x++) {
            int sx = x * sw / dw;
            if (sx >= sw) sx = sw - 1;
            out[x * 3] = row[sx * 3];
            out[x * 3 + 1] = row[sx * 3 + 1];
            out[x * 3 + 2] = row[sx * 3 + 2];
        }
    }
}

static int ensure_view(openscan_live *L, int fw, int fh, int sw, int sh) {
    int zneed = fw * fh;
    if (sw * sh > zneed) zneed = sw * sh;
    if (L->view_fw != fw || L->view_fh != fh || !L->view_front) {
        free(L->view_front);
        L->view_front = malloc((size_t)fw * fh * 3);
        L->view_fw = fw;
        L->view_fh = fh;
        L->cache_ok = 0;
    }
    if (L->view_sw != sw || L->view_sh != sh || !L->view_side) {
        free(L->view_side);
        L->view_side = malloc((size_t)sw * sh * 3);
        L->view_sw = sw;
        L->view_sh = sh;
        L->cache_ok = 0;
    }
    if (L->zcap < zneed || !L->zbuf) {
        free(L->zbuf);
        L->zbuf = malloc((size_t)zneed * sizeof(float));
        L->zcap = zneed;
    }
    return L->view_front && L->view_side && L->zbuf;
}

static void draw_panels(openscan_live *L, const uint8_t *clean, const uint8_t *pattern, int w, int h,
                        const Tri *show, int nshow) {
    int dw = L->pw / 2, dh = L->ph;
    gray_bgr(L->panel[0], dw, dh, clean, w, h);
    gray_bgr(L->panel[2], dw, dh, pattern, w, h);
    gray_bgr(L->panel[3], dw, dh, clean, w, h);
    const Tri *tris = nshow > 0 ? show : L->model;
    int ntris = nshow > 0 ? nshow : L->nmodel;
    float yaw = nshow > 0 ? L->orbit_yaw : L->turn.yaw + L->orbit_yaw;
    float ax = nshow > 0 ? 0.f : L->axis_x;
    float az = nshow > 0 ? 0.f : L->axis_z;
    int cam_w = L->pw * 42 / 100;
    int mid_w = L->pw * 42 / 100;
    int side_w = L->pw - cam_w - mid_w;
    int side_h = dh / 3;
    if (side_h < 8) side_h = 8;
    /* Half size turns each triangle into one speck when the picture is scaled up. */
    int fw = mid_w;
    int fh = dh;
    if (fw < 32) fw = 32;
    if (fh < 32) fh = 32;
    int yq = (int)(yaw * 500.f);
    int pq = (int)(L->orbit_pitch * 500.f);
    int same = L->cache_ok && L->cache_n == ntris && L->cache_yq == yq && L->cache_pq == pq;
    if (!same && ensure_view(L, fw, fh, fw, fh)) {
        paint_model(L->view_front, L->zbuf, fw, fh, tris, ntris, yaw, L->orbit_pitch, ax, L->axis_y, az);
        paint_model(L->view_side, L->zbuf, fw, fh, tris, ntris, yaw + 1.15f, L->orbit_pitch,
                    ax, L->axis_y, az);
        scale_bgr(L->panel[1], dw, dh, L->view_front, fw, fh);
        scale_bgr(L->panel[4], dw, dh, L->view_side, fw, fh);
        L->cache_n = ntris;
        L->cache_yq = yq;
        L->cache_pq = pq;
        L->cache_ok = 1;
    }
    uint8_t *cam = malloc((size_t)cam_w * dh * 3);
    uint8_t *mid = malloc((size_t)mid_w * dh * 3);
    uint8_t *sa = malloc((size_t)side_w * side_h * 3);
    uint8_t *sb = malloc((size_t)side_w * side_h * 3);
    uint8_t *sc = malloc((size_t)side_w * side_h * 3);
    memset(L->preview, 18, (size_t)L->pw * L->ph * 3);
    if (cam && mid && sa && sb && sc && L->view_front && L->view_side) {
        gray_bgr(cam, cam_w, dh, clean, w, h);
        scale_bgr(mid, mid_w, dh, L->view_front, L->view_fw, L->view_fh);
        gray_bgr(sa, side_w, side_h, pattern, w, h);
        gray_bgr(sb, side_w, side_h, clean, w, h);
        scale_bgr(sc, side_w, side_h, L->view_side, L->view_sw, L->view_sh);
        paste(L->preview, L->pw, L->ph, cam, cam_w, dh, 0, 0);
        paste(L->preview, L->pw, L->ph, mid, mid_w, dh, cam_w + 4, 0);
        paste(L->preview, L->pw, L->ph, sa, side_w, side_h, cam_w + mid_w + 8, 0);
        paste(L->preview, L->pw, L->ph, sb, side_w, side_h, cam_w + mid_w + 8, side_h + 4);
        paste(L->preview, L->pw, L->ph, sc, side_w, side_h, cam_w + mid_w + 8, side_h * 2 + 8);
    }
    free(sc);
    free(cam);
    free(mid);
    free(sa);
    free(sb);
}


/* Shift of the thing that moved, in pixels. A bright static background keeps
 * the silhouette still while the object in front of it turns. The two changed
 * clusters are where that object was and where it is now. */
static float motion_shift(const uint8_t *prev, const uint8_t *gray, int w, int h,
                          int *usable, int *wide) {
    int step = 4, samples = 0, sn = 0, nl = 0, nr = 0;
    int l0 = w, l1 = 0, r0 = w, r1 = 0;
    double sx = 0, sln = 0, slo = 0, srn = 0, sro = 0;
    float mid, lc, rc, travel, left_delta, right_delta, dx;
    int span, spanr;
    if (usable) *usable = 0;
    if (wide) *wide = 0;
    if (!prev || !gray || w < 16 || h < 16) return 0.f;
    for (int y = 0; y < h; y += step) {
        for (int x = 0; x < w; x += step) {
            int d = (int)gray[y * w + x] - (int)prev[y * w + x];
            samples++;
            if (d < 0) d = -d;
            if (d < 28) continue;
            sn++;
            sx += x;
        }
    }
    if (sn < 40 || samples < 1 || sn * 3 > samples) return 0.f;
    mid = (float)(sx / sn);
    for (int y = 0; y < h; y += step) {
        for (int x = 0; x < w; x += step) {
            int i = y * w + x;
            int d = (int)gray[i] - (int)prev[i];
            if (d < 0) d = -d;
            if (d < 28) continue;
            if ((float)x < mid) {
                sln += gray[i];
                slo += prev[i];
                nl++;
                if (x < l0) l0 = x;
                if (x > l1) l1 = x;
            } else {
                srn += gray[i];
                sro += prev[i];
                nr++;
                if (x < r0) r0 = x;
                if (x > r1) r1 = x;
            }
        }
    }
    if (nl < 12 || nr < 12) return 0.f;
    left_delta = (float)((sln - slo) / nl);
    right_delta = (float)((srn - sro) / nr);
    lc = 0.5f * (float)(l0 + l1);
    rc = 0.5f * (float)(r0 + r1);
    travel = rc - lc;
    if (travel < 4.f) return 0.f;
    if (left_delta > 8.f && right_delta < -8.f) dx = travel;
    else if (left_delta < -8.f && right_delta > 8.f) dx = -travel;
    else return 0.f;
    span = l1 - l0;
    spanr = r1 - r0;
    if (spanr > span) span = spanr;
    if (wide) *wide = span;
    if (usable) *usable = 1;
    return dx;
}

static int cell_old(const openscan_live *L, int ix, int iy, int iz) {
    unsigned h = (unsigned)(ix * 73856093 ^ iy * 19349663 ^ iz * 83492791);
    unsigned slot = h & (unsigned)(L->ncell - 1);
    for (int n = 0; n < 16; n++) {
        unsigned got = L->cell[(slot + (unsigned)n) & (unsigned)(L->ncell - 1)];
        if (got == 0) return 0;
        if ((got & 0xFFFFu) != (h & 0xFFFFu)) continue;
        return (got >> 16) != 0 && (got >> 16) != L->shell_gen;
    }
    return 0;
}

/* cells is the search radius in 0.75 mm steps. Earlier shells count. This one does not. */
static int depth_near(const openscan_live *L, float x, float y, float z, int cells, int stride) {
    int ix = os_lround(x / 0.75f);
    int iy = os_lround(y / 0.75f);
    int iz = os_lround(z / 0.75f);
    if (stride < 1) stride = 1;
    for (int dz = -cells; dz <= cells; dz += stride) {
        for (int dy = -cells; dy <= cells; dy += stride) {
            for (int dx = -cells; dx <= cells; dx += stride) {
                if (cell_old(L, ix + dx, iy + dy, iz + dz)) return 1;
            }
        }
    }
    return 0;
}

static int depth_link(const os_stripe_pt *a, const os_stripe_pt *b) {
    float dx, dy, dz;
    if (!a || !b || !a->ok || !b->ok) return 0;
    dx = a->x - b->x;
    dy = a->y - b->y;
    dz = a->z - b->z;
    /* Nearby samples are one skin even when the stripe index drifted. */
    return dx * dx + dy * dy + dz * dz < 22.f * 22.f;
}

static void yaw_into(const openscan_live *L, float yaw, float *x, float *z) {
    if (L->have_axis) rot_y(-yaw, L->axis_x, L->axis_z, x, z);
}

static int emit_depth(openscan_live *L, const os_stripe_pt *a, const os_stripe_pt *b,
                      const os_stripe_pt *c, float yaw, int commit, int filter, Tri *preview,
                      int preview_cap, int *preview_n) {
    float ax = a->x, ay = a->y, az = a->z;
    float bx = b->x, by = b->y, bz = b->z;
    float cx = c->x, cy = c->y, cz = c->z;
    float mx, my, mz;
    Tri t;
    if (!depth_link(a, b) || !depth_link(b, c) || !depth_link(a, c)) return 0;
    t.tone = 2.f;
    if (!commit) {
        if (!preview || !preview_n || *preview_n >= preview_cap) return 0;
        t.a[0] = ax; t.a[1] = ay; t.a[2] = az;
        t.b[0] = bx; t.b[1] = by; t.b[2] = bz;
        t.c[0] = cx; t.c[1] = cy; t.c[2] = cz;
        preview[(*preview_n)++] = t;
        return 1;
    }
    yaw_into(L, yaw, &ax, &az);
    yaw_into(L, yaw, &bx, &bz);
    yaw_into(L, yaw, &cx, &cz);
    mx = (ax + bx + cx) / 3.f;
    my = (ay + by + cy) / 3.f;
    mz = (az + bz + cz) / 3.f;
    if (filter) {
        int on = depth_near(L, mx, my, mz, 6, 2);
        int touch = on || depth_near(L, mx, my, mz, 24, 4);
        if (on || !touch) return 0;
    }
    if (L->nmodel >= L->cap || !cell_new(L, mx, my, mz)) return 0;
    t.a[0] = ax; t.a[1] = ay; t.a[2] = az;
    t.b[0] = bx; t.b[1] = by; t.b[2] = bz;
    t.c[0] = cx; t.c[1] = cy; t.c[2] = cz;
    L->model[L->nmodel++] = t;
    mark_bin(L, mx, mz);
    return 1;
}

static const os_stripe_pt *depth_ahead(const os_stripe_pt *p, const os_stripe_cloud *c, int r) {
    const os_stripe_pt *best = NULL;
    float best_d = 22.f * 22.f;
    int last = r + 8;
    if (last > c->rows) last = c->rows;
    for (int rr = r + 1; rr < last; rr++) {
        const os_stripe_pt *nrow = c->pt + rr * c->cols;
        for (int t = 0; t < c->cols; t++) {
            float dx, dy, dz, d;
            if (!nrow[t].ok) continue;
            dx = p->x - nrow[t].x;
            dy = p->y - nrow[t].y;
            dz = p->z - nrow[t].z;
            d = dx * dx + dy * dy + dz * dz;
            if (d < best_d) {
                best_d = d;
                best = &nrow[t];
            }
        }
        if (best) break;
    }
    return best;
}

static int mesh_cloud(openscan_live *L, const os_stripe_cloud *c, float yaw, int commit,
                      int filter, Tri *preview, int preview_cap, int *preview_n);

/* Close a missed stripe when both sides are the same surface.
 * One ridge on this frame is about 13 px, and a single miss is about 24.
 * 32 px joins that miss. A wider hole, such as an eye, stays open. 14 mm. */
static int mesh_sheet(openscan_live *L, const os_stripe_cloud *c, float yaw, int commit,
                      int filter, Tri *preview, int preview_cap, int *preview_n) {
    enum { STEP = 3, GAP = 32 };
    int gw, gh, r, col, i, made = 0;
    typedef struct {
        float x, y, z, tone;
        int ok;
    } Cell;
    Cell *g;
    if (!c || !c->pt || c->n < 80) return 0;
    gw = 0;
    gh = 0;
    for (i = 0; i < c->rows * c->cols; i++) {
        if (!c->pt[i].ok) continue;
        if (c->pt[i].u / STEP + 1 > gw) gw = c->pt[i].u / STEP + 1;
        if (c->pt[i].v / STEP + 1 > gh) gh = c->pt[i].v / STEP + 1;
    }
    if (gw < 2 || gh < 2) return mesh_cloud(L, c, yaw, commit, filter, preview, preview_cap, preview_n);
    g = calloc((size_t)gw * (size_t)gh, sizeof(Cell));
    if (!g) return mesh_cloud(L, c, yaw, commit, filter, preview, preview_cap, preview_n);
    for (i = 0; i < c->rows * c->cols; i++) {
        const os_stripe_pt *p = &c->pt[i];
        int col, row;
        Cell *cell;
        if (!p->ok) continue;
        col = p->u / STEP;
        row = p->v / STEP;
        if (col < 0 || row < 0 || col >= gw || row >= gh) continue;
        cell = &g[row * gw + col];
        if (!cell->ok) {
            cell->x = p->x;
            cell->y = p->y;
            cell->z = p->z;
            cell->tone = p->tone;
            cell->ok = 1;
        } else if (os_fabs(p->z - cell->z) <= 14.f) {
            cell->x = (cell->x + p->x) * 0.5f;
            cell->y = (cell->y + p->y) * 0.5f;
            cell->z = (cell->z + p->z) * 0.5f;
            cell->tone = (cell->tone + p->tone) * 0.5f;
        }
    }
    for (r = 0; r < gh; r++) {
        int prev = -1;
        for (col = 0; col < gw; col++) {
            Cell *cell = &g[r * gw + col];
            if (!cell->ok) continue;
            if (prev >= 0 && (col - prev) * STEP <= GAP) {
                Cell *a = &g[r * gw + prev];
                if (os_fabs(cell->z - a->z) <= 14.f) {
                    for (int k = prev + 1; k < col; k++) {
                        float t = (float)(k - prev) / (float)(col - prev);
                        Cell *d = &g[r * gw + k];
                        if (d->ok) continue;
                        d->x = a->x + (cell->x - a->x) * t;
                        d->y = a->y + (cell->y - a->y) * t;
                        d->z = a->z + (cell->z - a->z) * t;
                        d->tone = a->tone + (cell->tone - a->tone) * t;
                        d->ok = 1;
                    }
                }
            }
            prev = col;
        }
    }
    for (col = 0; col < gw; col++) {
        int prev = -1;
        for (r = 0; r < gh; r++) {
            Cell *cell = &g[r * gw + col];
            if (!cell->ok) continue;
            if (prev >= 0 && (r - prev) * STEP <= GAP) {
                Cell *a = &g[prev * gw + col];
                if (os_fabs(cell->z - a->z) <= 14.f) {
                    for (int k = prev + 1; k < r; k++) {
                        float t = (float)(k - prev) / (float)(r - prev);
                        Cell *d = &g[k * gw + col];
                        if (d->ok) continue;
                        d->x = a->x + (cell->x - a->x) * t;
                        d->y = a->y + (cell->y - a->y) * t;
                        d->z = a->z + (cell->z - a->z) * t;
                        d->tone = a->tone + (cell->tone - a->tone) * t;
                        d->ok = 1;
                    }
                }
            }
            prev = r;
        }
    }
    /* A stripe a millimetre off turns a 3 px triangle into a black crease.
     * The face itself is many times wider than that step, so the blur
     * has to outlast the step and leave the head. */
    {
        float *zz = malloc((size_t)gw * (size_t)gh * sizeof(float));
        float *tmp = malloc((size_t)gw * (size_t)gh * sizeof(float));
        if (zz && tmp) {
            int pass;
            for (r = 0; r < gh; r++)
                for (col = 0; col < gw; col++)
                    zz[r * gw + col] = g[r * gw + col].ok ? g[r * gw + col].z : 0.f;
            for (pass = 0; pass < 36; pass++) {
                for (r = 0; r < gh; r++) {
                    for (col = 0; col < gw; col++) {
                        float s = 0.f;
                        int m = 0, rr, cc;
                        if (!g[r * gw + col].ok) {
                            tmp[r * gw + col] = 0.f;
                            continue;
                        }
                        for (rr = r - 1; rr <= r + 1; rr++) {
                            if (rr < 0 || rr >= gh) continue;
                            for (cc = col - 1; cc <= col + 1; cc++) {
                                if (cc < 0 || cc >= gw || !g[rr * gw + cc].ok) continue;
                                s += zz[rr * gw + cc];
                                m++;
                            }
                        }
                        tmp[r * gw + col] = m ? s / (float)m : zz[r * gw + col];
                    }
                }
                for (r = 0; r < gh; r++)
                    for (col = 0; col < gw; col++)
                        if (g[r * gw + col].ok) zz[r * gw + col] = tmp[r * gw + col];
            }
            for (r = 0; r < gh; r++)
                for (col = 0; col < gw; col++)
                    if (g[r * gw + col].ok) g[r * gw + col].z = zz[r * gw + col];
        }
        free(zz);
        free(tmp);
    }
    for (r = 0; r < gh - 1; r++) {
        for (col = 0; col < gw - 1; col++) {
            Cell *a = &g[r * gw + col];
            Cell *b = &g[r * gw + col + 1];
            Cell *d = &g[(r + 1) * gw + col];
            Cell *e = &g[(r + 1) * gw + col + 1];
            os_stripe_pt pa, pb, pd, pe;
            if (!a->ok || !b->ok || !d->ok) continue;
            memset(&pa, 0, sizeof pa);
            memset(&pb, 0, sizeof pb);
            memset(&pd, 0, sizeof pd);
            memset(&pe, 0, sizeof pe);
            pa.ok = pb.ok = pd.ok = pe.ok = 1;
            pa.x = a->x; pa.y = a->y; pa.z = a->z; pa.tone = a->tone;
            pb.x = b->x; pb.y = b->y; pb.z = b->z; pb.tone = b->tone;
            pd.x = d->x; pd.y = d->y; pd.z = d->z; pd.tone = d->tone;
            pe.x = e->x; pe.y = e->y; pe.z = e->z; pe.tone = e->tone;
            made += emit_depth(L, &pa, &pb, &pd, yaw, commit, filter, preview, preview_cap, preview_n);
            if (e->ok)
                made += emit_depth(L, &pb, &pe, &pd, yaw, commit, filter, preview, preview_cap,
                                   preview_n);
        }
    }
    free(g);
    return made;
}

static int mesh_cloud(openscan_live *L, const os_stripe_cloud *c, float yaw, int commit,
                      int filter, Tri *preview, int preview_cap, int *preview_n) {
    int made = 0;
    if (!c || !c->pt || c->rows < 2 || c->cols < 2) return 0;
    for (int r = 0; r < c->rows - 1; r++) {
        const os_stripe_pt *row = c->pt + r * c->cols;
        for (int k = 0; k < c->cols - 1; k++) {
            const os_stripe_pt *p = &row[k];
            const os_stripe_pt *right = &row[k + 1];
            const os_stripe_pt *down;
            const os_stripe_pt *downr;
            if (!depth_link(p, right)) continue;
            down = depth_ahead(p, c, r);
            downr = depth_ahead(right, c, r);
            if (down && depth_link(right, down))
                made += emit_depth(L, p, right, down, yaw, commit, filter, preview, preview_cap,
                                   preview_n);
            if (down && downr && down != downr && depth_link(right, downr) && depth_link(down, downr))
                made += emit_depth(L, right, downr, down, yaw, commit, filter, preview, preview_cap,
                                   preview_n);
        }
    }
    return made;
}

static int cloud_touches(const openscan_live *L, const os_stripe_cloud *c, float yaw) {
    int samples = 0, hit = 0;
    if (L->nmodel < 80) return 1;
    for (int i = 0; i < c->rows * c->cols && samples < 240; i += 2) {
        float x, y, z;
        if (!c->pt[i].ok) continue;
        x = c->pt[i].x;
        y = c->pt[i].y;
        z = c->pt[i].z;
        yaw_into(L, yaw, &x, &z);
        samples++;
        if (depth_near(L, x, y, z, 24, 4)) hit++;
    }
    return samples >= 30 && hit * 4 >= samples;
}

static int rebuild_cloud(openscan_live *L) {
    os_cloud_tri *tris;
    int n, i, cap;
    if (!L->cloud) return 0;
    cap = L->cap;
    tris = malloc(sizeof(*tris) * (size_t)cap);
    if (!tris) return 0;
    n = L->build.solid ? os_cloud_body(L->cloud, tris, cap) : os_cloud_specks(L->cloud, tris, cap);
    L->nmodel = 0;
    memset(L->seen, 0, sizeof L->seen);
    for (i = 0; i < n; i++) {
        Tri t;
        float mx, my, mz;
        t.a[0] = tris[i].a[0];
        t.a[1] = tris[i].a[1];
        t.a[2] = tris[i].a[2];
        t.b[0] = tris[i].b[0];
        t.b[1] = tris[i].b[1];
        t.b[2] = tris[i].b[2];
        t.c[0] = tris[i].c[0];
        t.c[1] = tris[i].c[1];
        t.c[2] = tris[i].c[2];
        t.tone = 2.f;
        L->model[L->nmodel++] = t;
        mx = (t.a[0] + t.b[0] + t.c[0]) / 3.f;
        my = (t.a[1] + t.b[1] + t.c[1]) / 3.f;
        mz = (t.a[2] + t.b[2] + t.c[2]) / 3.f;
        (void)my;
        mark_bin(L, mx, mz);
    }
    L->scanned_bins = count_bins(L);
    L->cache_ok = 0;
    free(tris);
    return n;
}

/* Returns triangles written, or -1 when the view does not meet the body. */
static int commit_depth(openscan_live *L, const os_stripe_cloud *c, float yaw, int commit,
                        Tri *preview, int preview_cap, int *preview_n) {
    if (!c || c->n < 80) return 0;
    if (commit && !L->have_axis) {
        double sx = 0, sy = 0, sz = 0;
        int n = 0;
        for (int i = 0; i < c->rows * c->cols; i++) {
            if (!c->pt[i].ok) continue;
            sx += c->pt[i].x;
            sy += c->pt[i].y;
            sz += c->pt[i].z;
            n++;
        }
        if (n < 80) return 0;
        L->axis_x = (float)(sx / n);
        L->axis_y = (float)(sy / n);
        L->axis_z = (float)(sz / n);
        L->have_axis = 1;
    }
    if (commit && !cloud_touches(L, c, yaw)) return -1;
    if (commit) {
        if (++L->shell_gen >= 65535u) {
            memset(L->cell, 0, (size_t)L->ncell * sizeof(unsigned));
            L->shell_gen = 1;
        }
    }
    return mesh_sheet(L, c, yaw, commit, commit && L->nmodel > 80, preview, preview_cap, preview_n);
}

int openscan_live_push(openscan_live *live, const uint8_t *ya, const uint8_t *yb,
                       int width, int height, openscan_live_status *status) {
    if (status) memset(status, 0, sizeof *status);
    if (!live || !ya || !yb || width < 16 || height < 16) return -1;
    size_t n = (size_t)width * height;
    uint8_t *clean = malloc(n);
    uint8_t *pattern = malloc(n);
    if (!clean || !pattern) {
        free(clean);
        free(pattern);
        return -1;
    }
    memcpy(clean, yb, n);
    memcpy(pattern, ya, n);
    if (live->build.flip) {
        flip180(clean, (int)n);
        flip180(pattern, (int)n);
    }
    /* The top of the outline moves farther in the picture than the middle
     * when the object turns. The middle alone stays put on a centered spin. */
    if (live->row_n != height || !live->row_mid) {
        free(live->row_mid);
        live->row_mid = malloc((size_t)height * sizeof(int));
        live->row_n = height;
        live->have_rows = 0;
    }
    double sx = 0, sn = 0, topx = 0;
    int ntop = 0, ymin = height, ymax = -1, wide = 0;
    for (int y = 0; y < height; y += 2) {
        int l = -1, r = -1;
        const uint8_t *row = clean + y * width;
        for (int x = 0; x < width; x++)
            if (row[x] > 28) { l = x; break; }
        if (l >= 0) {
            for (int x = width - 1; x >= 0; x--)
                if (row[x] > 28) { r = x; break; }
        }
        int mid = l >= 0 ? (l + r) / 2 : -1;
        if (mid >= 0) {
            if (y < ymin) ymin = y;
            if (y > ymax) ymax = y;
            if (r - l > wide) wide = r - l;
            sx += mid;
            sn += 1;
        }
        if (live->row_mid) live->row_mid[y] = mid;
        if ((y & 1) == 0 && y + 1 < height && live->row_mid) live->row_mid[y + 1] = mid;
    }
    int span = ymax - ymin;
    float cx = sn > 8 ? (float)(sx / sn) : width * 0.5f;
    if (live->row_mid && span > 16) {
        int y1 = ymin + span / 5;
        for (int y = ymin; y < y1; y += 2) {
            int mid = live->row_mid[y];
            if (mid < 0) continue;
            topx += mid;
            ntop++;
        }
    }
    float cx_top = ntop > 4 ? (float)(topx / ntop) : cx;
    int scanning = live->mode == OPENSCAN_MODE_SCAN;
    int measured = live->shape == OPENSCAN_SHAPE_MEASURED;
    int cloud_mode = live->shape == OPENSCAN_SHAPE_CLOUD;
    int fuse = 0;
    float dx = 0.f;
    float radius = wide * 0.5f;
    if (radius < 20.f) radius = 20.f;
    if (live->have_prev && sn > 8 && live->have_rows) {
        float dx_body = cx - live->prev_cx;
        float dx_top = cx_top - live->prev_top;
        dx = os_fabs(dx_top) > os_fabs(dx_body) ? dx_top : dx_body;
    }
    if (measured && live->prev_ready) {
        int mot = 0, mw = 0;
        float mdx = motion_shift(live->prev_gray, clean, width, height, &mot, &mw);
        if (mot && os_fabs(mdx) > os_fabs(dx)) {
            dx = mdx;
            if (mw > 16) radius = mw * 0.5f;
        }
    }
    float yaw0 = live->turn.yaw;
    if (live->have_prev && sn > 8 && live->have_rows)
        os_turn_step(&live->turn, dx, radius, scanning, &fuse);
    live->prev_cx = cx;
    live->prev_top = cx_top;
    live->have_prev = sn > 8;
    live->have_rows = live->row_mid != NULL;
    int before = live->nmodel;
    int nok = 0;
    int commit = scanning && (before == 0 || fuse);
    int cloud_got = 0;
    int cloud_tracked = -1;
    Tri *shown = NULL;
    int nshown = 0;
    /* Measured meshes the projector surface. A moved view is kept only where
     * it meets the body. Cloud and mold store the points. Shapes in the
     * matrix locate the turn. A cell molds only after it is seen again. */
    if (cloud_mode) {
        os_stripe_cloud sc;
        memset(&sc, 0, sizeof sc);
        if (live->has_cal &&
            os_stripe_make(&live->cal, pattern, clean, width, height, live->build.flip,
                           live->distance, live->build.near_mm, live->build.far_mm,
                           live->build.stride, &sc) == 0)
            nok = sc.n;
        if (scanning && nok >= 80 && live->cloud) {
            if (!live->have_axis) {
                double sx = 0, sy = 0, sz = 0;
                int n = 0, i;
                for (i = 0; i < sc.rows * sc.cols; i++) {
                    if (!sc.pt[i].ok) continue;
                    sx += sc.pt[i].x;
                    sy += sc.pt[i].y;
                    sz += sc.pt[i].z;
                    n++;
                }
                if (n >= 80) {
                    live->axis_x = (float)(sx / n);
                    live->axis_y = (float)(sy / n);
                    live->axis_z = (float)(sz / n);
                    live->have_axis = 1;
                }
            }
            if (live->have_axis) {
                float yaw = live->turn.yaw;
                cloud_got = os_cloud_integrate(live->cloud, sc.pt, sc.rows * sc.cols,
                                               &yaw, live->axis_x, live->axis_z);
                if (cloud_got < 0) {
                    live->turn.yaw = yaw0;
                    cloud_tracked = 0;
                } else {
                    live->turn.yaw = yaw;
                    rebuild_cloud(live);
                    cloud_tracked = 1;
                }
            }
        } else if (os_cloud_points(live->cloud) > 0) {
            cloud_tracked = 1;
        }
        if (nok >= 80 && live->nmodel < 80) {
            if (!live->held_preview) live->held_preview = malloc(sizeof(Tri) * 80000);
            if (live->held_preview) {
                live->nheld = 0;
                commit_depth(live, &sc, 0.f, 0, live->held_preview, 80000, &live->nheld);
                live->cache_ok = 0;
                shown = live->held_preview;
                nshown = live->nheld;
            }
        }
        os_stripe_free(&sc);
    } else if (measured) {
        os_stripe_cloud cloud;
        memset(&cloud, 0, sizeof cloud);
        if (os_stripe_make(&live->cal, pattern, clean, width, height, live->build.flip,
                           live->distance, live->build.near_mm, live->build.far_mm,
                           live->build.stride, &cloud) == 0) {
            nok = cloud.n;
        }
        if (live->nmodel < 80) {
            /* The facing sheet is tens of thousands of triangles. 12000 kept
             * only the top of the head and drew that band as the whole model. */
            if (!live->held_preview) live->held_preview = malloc(sizeof(Tri) * 80000);
            if (live->held_preview) {
                live->nheld = 0;
                commit_depth(live, &cloud, 0.f, 0, live->held_preview, 80000, &live->nheld);
                live->cache_ok = 0;
            }
            shown = live->held_preview;
            nshown = live->nheld;
        }
        if (commit && live->build.solid) {
            int got = commit_depth(live, &cloud, live->turn.yaw, 1, NULL, 0, NULL);
            if (got < 0) live->turn.yaw = yaw0;
        }
        os_stripe_free(&cloud);
    } else {
        os_stripe_cloud sc;
        memset(&sc, 0, sizeof sc);
        if (live->has_cal &&
            os_stripe_make(&live->cal, pattern, clean, width, height, live->build.flip,
                           live->distance, live->build.near_mm, live->build.far_mm,
                           live->build.stride, &sc) == 0)
            nok = sc.n;
        if (scanning && nok >= 80 && live->cloud) {
            if (!live->have_axis) {
                double sx = 0, sy = 0, sz = 0;
                int n = 0, i;
                for (i = 0; i < sc.rows * sc.cols; i++) {
                    if (!sc.pt[i].ok) continue;
                    sx += sc.pt[i].x;
                    sy += sc.pt[i].y;
                    sz += sc.pt[i].z;
                    n++;
                }
                if (n >= 80) {
                    live->axis_x = (float)(sx / n);
                    live->axis_y = (float)(sy / n);
                    live->axis_z = (float)(sz / n);
                    live->have_axis = 1;
                }
            }
            if (live->have_axis) {
                float yaw = live->turn.yaw;
                int got = os_cloud_integrate(live->cloud, sc.pt, sc.rows * sc.cols, &yaw,
                                             live->axis_x, live->axis_z);
                if (got < 0) {
                    live->turn.yaw = yaw0;
                    cloud_tracked = 0;
                } else {
                    live->turn.yaw = yaw;
                    rebuild_cloud(live);
                    cloud_tracked = 1;
                }
            }
        }
        if (nok >= 80 && live->nmodel < 80) {
            if (!live->held_preview) live->held_preview = malloc(sizeof(Tri) * 80000);
            if (live->held_preview) {
                live->nheld = 0;
                commit_depth(live, &sc, 0.f, 0, live->held_preview, 80000, &live->nheld);
                live->cache_ok = 0;
                shown = live->held_preview;
                nshown = live->nheld;
            }
        }
        os_stripe_free(&sc);
    }
    if (commit) live->cache_ok = 0;
    if (cloud_mode) {
        if (scanning && cloud_tracked == 1 && nok >= 80) live->fused++;
        else if (scanning && cloud_tracked == 0) live->lost++;
    } else if (scanning && live->nmodel > before) live->fused++;
    else if (scanning && nok < 80 && before == 0) live->lost++;
    draw_panels(live, clean, pattern, width, height, shown, nshown);
    /* Mold reports the sweep. Measured reports the kept turn.
     * Cloud reports how far around the object the points reach. */
    int covered = cloud_mode ? os_cloud_degrees(live->cloud)
        : measured ? (int)(os_fabs(live->turn.yaw) * (180.f / 3.14159265f) + 0.5f)
        : os_scanned_deg(live->build.sweep_deg, live->turn.yaw, count_bins(live), live->nmodel);
    if (live->prev_gw != width || live->prev_gh != height || !live->prev_gray) {
        free(live->prev_gray);
        live->prev_gray = malloc(n);
        live->prev_gw = width;
        live->prev_gh = height;
        live->prev_ready = 0;
    }
    if (live->prev_gray) {
        memcpy(live->prev_gray, clean, n);
        live->prev_ready = 1;
    }
    if (status) {
        status->fused = live->fused;
        status->lost = live->lost;
        status->valid_pixels = cloud_mode ? os_cloud_points(live->cloud) : nok;
        status->median_mm = live->distance;
        status->tracking = cloud_mode ? cloud_tracked : (live->nmodel > 80 ? 1 : -1);
        status->mode = live->mode;
        status->points = live->nmodel;
        status->scanned_deg = covered;
        status->detail = cloud_mode ? os_cloud_coverage(live->cloud) : 1;
        status->shapes = cloud_mode ? os_cloud_shape_count(live->cloud) : 0;
    }
    free(clean);
    free(pattern);
    return 0;
}

const uint8_t *openscan_live_preview_bgr(const openscan_live *live, int *width, int *height) {
    if (!live || !live->preview) return NULL;
    if (width) *width = live->pw;
    if (height) *height = live->ph;
    return live->preview;
}

const uint8_t *openscan_live_panel_bgr(const openscan_live *live, int panel, int *width, int *height) {
    if (!live || panel < 0 || panel > 4 || !live->panel[panel]) return NULL;
    if (width) *width = live->pw / 2;
    if (height) *height = live->ph;
    return live->panel[panel];
}

static void tri_normal(const Tri *t, float n[3]) {
    float ux = t->b[0] - t->a[0], uy = t->b[1] - t->a[1], uz = t->b[2] - t->a[2];
    float vx = t->c[0] - t->a[0], vy = t->c[1] - t->a[1], vz = t->c[2] - t->a[2];
    float len;
    n[0] = uy * vz - uz * vy;
    n[1] = uz * vx - ux * vz;
    n[2] = ux * vy - uy * vx;
    len = os_sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    if (len < 1e-12f) {
        n[0] = 0.f;
        n[1] = 0.f;
        n[2] = 1.f;
        return;
    }
    n[0] /= len;
    n[1] /= len;
    n[2] /= len;
}

static int suffix_is(const char *path, const char *ext) {
    size_t n = strlen(path), e = strlen(ext), i;
    if (n < e) return 0;
    for (i = 0; i < e; i++) {
        char a = path[n - e + i], b = ext[i];
        if (a >= 'A' && a <= 'Z') a = (char)(a + 32);
        if (b >= 'A' && b <= 'Z') b = (char)(b + 32);
        if (a != b) return 0;
    }
    return 1;
}

/* Geometry only. No serial, no calibration text. */
static int write_stl(const char *path, const Tri *tris, int count) {
    FILE *f = fopen(path, "wb");
    char header[80];
    uint32_t n;
    int i;
    if (!f) return -1;
    memset(header, 0, sizeof header);
    memcpy(header, "openscan mesh", 13);
    if (fwrite(header, 1, 80, f) != 80) {
        fclose(f);
        return -1;
    }
    n = (uint32_t)count;
    if (fwrite(&n, 4, 1, f) != 1) {
        fclose(f);
        return -1;
    }
    for (i = 0; i < count; i++) {
        float nn[3], rec[12];
        uint16_t attr = 0;
        tri_normal(&tris[i], nn);
        rec[0] = nn[0]; rec[1] = nn[1]; rec[2] = nn[2];
        rec[3] = tris[i].a[0]; rec[4] = tris[i].a[1]; rec[5] = tris[i].a[2];
        rec[6] = tris[i].b[0]; rec[7] = tris[i].b[1]; rec[8] = tris[i].b[2];
        rec[9] = tris[i].c[0]; rec[10] = tris[i].c[1]; rec[11] = tris[i].c[2];
        if (fwrite(rec, 4, 12, f) != 12 || fwrite(&attr, 2, 1, f) != 1) {
            fclose(f);
            return -1;
        }
    }
    if (fclose(f) != 0) return -1;
    return 0;
}

static int write_obj(const char *path, const Tri *tris, int count) {
    FILE *f = fopen(path, "w");
    int i, k, idx = 1;
    if (!f) return -1;
    fprintf(f, "# openscan mesh\n");
    for (i = 0; i < count; i++) {
        float nn[3];
        const float *p[3] = {tris[i].a, tris[i].b, tris[i].c};
        tri_normal(&tris[i], nn);
        for (k = 0; k < 3; k++) {
            fprintf(f, "v %.6f %.6f %.6f\n", p[k][0], p[k][1], p[k][2]);
            fprintf(f, "vn %.6f %.6f %.6f\n", nn[0], nn[1], nn[2]);
        }
        fprintf(f, "f %d//%d %d//%d %d//%d\n", idx, idx, idx + 1, idx + 1, idx + 2, idx + 2);
        idx += 3;
    }
    if (fclose(f) != 0) return -1;
    return 0;
}

static int write_ply(const char *path, const Tri *tris, int count) {
    FILE *f = fopen(path, "w");
    int i, k;
    if (!f) return -1;
    fprintf(f, "ply\nformat ascii 1.0\n");
    fprintf(f, "element vertex %d\n", count * 3);
    fprintf(f, "property float x\nproperty float y\nproperty float z\n");
    fprintf(f, "property float nx\nproperty float ny\nproperty float nz\n");
    fprintf(f, "element face %d\n", count);
    fprintf(f, "property list uchar int vertex_indices\n");
    fprintf(f, "end_header\n");
    for (i = 0; i < count; i++) {
        float nn[3];
        const float *p[3] = {tris[i].a, tris[i].b, tris[i].c};
        tri_normal(&tris[i], nn);
        for (k = 0; k < 3; k++)
            fprintf(f, "%.6f %.6f %.6f %.6f %.6f %.6f\n",
                    p[k][0], p[k][1], p[k][2], nn[0], nn[1], nn[2]);
    }
    for (i = 0; i < count; i++)
        fprintf(f, "3 %d %d %d\n", i * 3, i * 3 + 1, i * 3 + 2);
    if (fclose(f) != 0) return -1;
    return 0;
}

static int write_mesh(const char *path, const Tri *tris, int count, int *triangles_out) {
    int rc;
    if (!path || !tris || count < 1) return -1;
    if (suffix_is(path, ".obj")) rc = write_obj(path, tris, count);
    else if (suffix_is(path, ".ply")) rc = write_ply(path, tris, count);
    else if (suffix_is(path, ".stl") || !strrchr(path, '.')) rc = write_stl(path, tris, count);
    else return -1;
    if (rc == 0 && triangles_out) *triangles_out = count;
    return rc;
}

int openscan_live_write(openscan_live *live, const char *path, int *triangles_out) {
    if (!live || !path) return -1;
    if (live->nmodel > 0) return write_mesh(path, live->model, live->nmodel, triangles_out);
    return write_mesh(path, live->held_preview, live->nheld, triangles_out);
}

static void model_bounds(const openscan_live *L, float *zlo, float *zhi, float *meanx) {
    float lo = 1e9f, hi = -1e9f;
    double sx = 0;
    int n = 0;
    for (int i = 0; i < L->nmodel; i++) {
        const float *p[3] = {L->model[i].a, L->model[i].b, L->model[i].c};
        for (int k = 0; k < 3; k++) {
            if (p[k][2] < lo) lo = p[k][2];
            if (p[k][2] > hi) hi = p[k][2];
            sx += p[k][0];
            n++;
        }
    }
    if (zlo) *zlo = n ? lo : 0;
    if (zhi) *zhi = n ? hi : 0;
    if (meanx) *meanx = n ? (float)(sx / n) : 0;
}

static void rig_cal(openscan_calib *cal) {
    memset(cal, 0, sizeof *cal);
    cal->width = 160;
    cal->height = 120;
    for (int i = 0; i < 3; i++) {
        cal->cam[i].fx = cal->cam[i].fy = 400;
        cal->cam[i].cx = 80;
        cal->cam[i].cy = 60;
    }
    cal->cam[2].tvec[0] = 30;
    cal->nplanes = 40;
    for (int i = 0; i < 40; i++) {
        cal->plane[i].index = i;
        cal->plane[i].slope = 0;
        cal->plane[i].offset = 10 + i * 3;
    }
}

static void paint_disk(uint8_t *img, int w, int h, int cx) {
    memset(img, 0, (size_t)w * (size_t)h);
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            float nx = (x - cx) / 36.f;
            float ny = (y - 60) / 30.f;
            if (nx * nx + ny * ny < 1.f) img[y * w + x] = 140;
        }
    }
}

static int stripe_live_self_test(void) {
    openscan_calib cal;
    openscan_live *L;
    openscan_live_status st;
    uint8_t *disk;
    float zlo = 0, zhi = 0, meanx = 0;
    int n0;
    if (os_stripe_self_test() != 0) return 1;
    rig_cal(&cal);
    disk = calloc(160 * 120, 1);
    if (!disk) return 1;
    paint_disk(disk, 160, 120, 80);
    L = openscan_live_create(&cal, NULL);
    if (!L) {
        free(disk);
        return 1;
    }
    L->build.flip = 0;
    openscan_live_set_shape(L, OPENSCAN_SHAPE_MOLD);
    openscan_live_set_distance_mm(L, 180.f);
    openscan_live_set_mode(L, OPENSCAN_MODE_SCAN);
    if (openscan_live_push(L, disk, disk, 160, 120, &st) != 0 || st.points != 0) {
        fprintf(stderr, "c mold: unconfirmed molded %d tris\n", st.points);
        openscan_live_destroy(L);
        free(disk);
        return 1;
    }
    if (openscan_live_push(L, disk, disk, 160, 120, &st) != 0) {
        openscan_live_destroy(L);
        free(disk);
        return 1;
    }
    model_bounds(L, &zlo, &zhi, &meanx);
    if (st.points > 0 && zhi - zlo > 8.f) {
        fprintf(stderr, "c mold: outline came back span %.2f tris %d\n", zhi - zlo, st.points);
        openscan_live_destroy(L);
        free(disk);
        return 1;
    }
    fprintf(stderr, "c mold: confirmed %d tris\n", st.points);
    openscan_live_destroy(L);
    L = openscan_live_create(&cal, NULL);
    if (!L) {
        free(disk);
        return 1;
    }
    L->build.flip = 0;
    openscan_live_set_shape(L, OPENSCAN_SHAPE_MEASURED);
    openscan_live_set_distance_mm(L, 180.f);
    openscan_live_set_mode(L, OPENSCAN_MODE_STOP);
    /* A flat wall of projector ridges. Wrapping that picture would thicken it. */
    os_stripe_paint_wall(disk, &cal, 180.f);
    if (openscan_live_push(L, disk, disk, 160, 120, &st) != 0 || st.points != 0) {
        fprintf(stderr, "c measured: idle committed %d\n", st.points);
        openscan_live_destroy(L);
        free(disk);
        return 1;
    }
    openscan_live_set_mode(L, OPENSCAN_MODE_SCAN);
    if (openscan_live_push(L, disk, disk, 160, 120, &st) != 0 || st.points < 400 ||
        st.scanned_deg > 30 || st.fused != 1) {
        fprintf(stderr, "c measured: first frame %d tris %d deg %d frames\n",
                st.points, st.scanned_deg, st.fused);
        openscan_live_destroy(L);
        free(disk);
        return 1;
    }
    model_bounds(L, &zlo, &zhi, &meanx);
    if (zhi - zlo > 8.f) {
        fprintf(stderr, "c measured: wall became a barrel span %.2f\n", zhi - zlo);
        openscan_live_destroy(L);
        free(disk);
        return 1;
    }
    n0 = st.points;
    if (openscan_live_push(L, disk, disk, 160, 120, &st) != 0 || st.points > n0 + 40 ||
        st.fused != 1) {
        fprintf(stderr, "c measured: still grew %d -> %d frames %d\n", n0, st.points, st.fused);
        openscan_live_destroy(L);
        free(disk);
        return 1;
    }
    for (int y = 0; y < 120; y++) {
        uint8_t *row = disk + y * 160;
        memmove(row + 6, row, 154);
        memset(row, 30, 6);
    }
    if (openscan_live_push(L, disk, disk, 160, 120, &st) != 0 || st.points > n0 + 40 ||
        st.points >= n0 * 2) {
        fprintf(stderr, "c measured: shifted wall stacked %d -> %d frames %d\n",
                n0, st.points, st.fused);
        openscan_live_destroy(L);
        free(disk);
        return 1;
    }
    model_bounds(L, &zlo, &zhi, &meanx);
    if (zhi - zlo > 8.f) {
        fprintf(stderr, "c measured: shift thickened span %.2f\n", zhi - zlo);
        openscan_live_destroy(L);
        free(disk);
        return 1;
    }
    fprintf(stderr, "c measured: wall %d tris span %.1f\n", n0, zhi - zlo);
    openscan_live_destroy(L);
    free(disk);
    return 0;
}


int openscan_mesh_self_test(void) {
    return openscan_turn_self_test();
}

int openscan_turn_self_test(void) {
    int logic = os_logic_self_test();
    if (logic != 0) {
        fprintf(stderr, "c logic: check %d failed\n", logic);
        return 1;
    }
    openscan_calib cal;
    memset(&cal, 0, sizeof cal);
    cal.width = 320;
    cal.height = 240;
    cal.cam[1].fx = 420;
    cal.cam[1].fy = 420;
    cal.cam[1].cx = 160;
    cal.cam[1].cy = 120;
    openscan_live *L = openscan_live_create(&cal, NULL);
    if (!L) return 1;
    L->build.flip = 0;
    L->build.sweep_deg = 80;
    openscan_live_set_mode(L, OPENSCAN_MODE_SCAN);
    uint8_t *g = calloc(320 * 240, 1);
    if (!g) return 1;
    for (int y = 40; y < 200; y++) {
        for (int x = 80; x < 240; x++) {
            float nx = (x - 160) / 70.f, ny = (y - 120) / 70.f;
            if (nx * nx + ny * ny < 1.f) g[y * 320 + x] = 180;
        }
    }
    openscan_live_status st;
    if (openscan_live_push(L, g, g, 320, 240, &st) != 0 || st.points != 0) {
        fprintf(stderr, "c mold: silhouette molded %d tris\n", st.points);
        return 1;
    }
    {
        int pw = 0, ph = 0, lit = 0;
        const uint8_t *pv = openscan_live_preview_bgr(L, &pw, &ph);
        int cam_w = pw * 42 / 100;
        int mid_w = pw * 42 / 100;
        if (pv && pw > 8 && ph > 8) {
            for (int y = 0; y < ph; y++)
                for (int x = cam_w; x < cam_w + mid_w && x < pw; x++)
                    if (pv[(y * pw + x) * 3] > 30) lit++;
        }
        if (lit > 40) {
            fprintf(stderr, "c view invented a solid lit %d\n", lit);
            return 1;
        }
    }
    int n0 = st.points;
    float y0 = L->turn.yaw;
    if (openscan_live_push(L, g, g, 320, 240, &st) != 0 || st.points != 0 ||
        os_fabs(L->turn.yaw - y0) > 1e-4f) {
        fprintf(stderr, "c mold: still grew %d -> %d\n", n0, st.points);
        return 1;
    }
    uint8_t *s = calloc(320 * 240, 1);
    for (int y = 40; y < 200; y++) {
        for (int x = 100; x < 260; x++) {
            float nx = (x - 180) / 70.f, ny = (y - 120) / 70.f;
            if (nx * nx + ny * ny < 1.f) s[y * 320 + x] = 180;
        }
    }
    if (openscan_live_push(L, s, s, 320, 240, &st) != 0 ||
        os_fabs(L->turn.yaw) < 2.f * 3.14159265f / 180.f || st.points != 0) {
        fprintf(stderr, "c mold: shift not tracked yaw %.3f tris %d\n", L->turn.yaw, st.points);
        return 1;
    }
    fprintf(stderr, "c mold: silhouette stays empty, turn yaw %.1f°\n",
            L->turn.yaw * 180.f / 3.14159265f);
    openscan_live *R = openscan_live_create(&cal, NULL);
    if (!R) return 1;
    R->build.flip = 0;
    R->build.sweep_deg = 80;
    openscan_live_set_mode(R, OPENSCAN_MODE_SCAN);
    uint8_t *base = calloc(320 * 240, 1);
    uint8_t *spin = calloc(320 * 240, 1);
    if (!base || !spin) return 1;
    for (int y = 40; y < 200; y++) {
        for (int x = 70; x < 250; x++) {
            float nx = (x - 160) / 80.f, ny = (y - 120) / 70.f;
            if (nx * nx + ny * ny < 1.f) base[y * 320 + x] = 180;
        }
    }
    if (openscan_live_push(R, base, base, 320, 240, &st) != 0 || st.points != 0) return 1;
    int nr = st.points;
    for (int y = 40; y < 200; y++) {
        int shift = y < 120 ? 18 : -18;
        for (int x = 70; x < 250; x++) {
            float nx = (x - 160) / 80.f, ny = (y - 120) / 70.f;
            if (nx * nx + ny * ny >= 1.f) continue;
            int xx = x + shift;
            if (xx >= 0 && xx < 320) spin[y * 320 + xx] = 180;
        }
    }
    if (openscan_live_push(R, spin, spin, 320, 240, &st) != 0 ||
        os_fabs(R->turn.yaw) < 2.f * 3.14159265f / 180.f || st.points != 0) {
        fprintf(stderr, "c mold: spin not tracked yaw %.3f tris %d -> %d\n",
                R->turn.yaw, nr, st.points);
        return 1;
    }
    fprintf(stderr, "c mold: spin yaw %.1f°\n", R->turn.yaw * 180.f / 3.14159265f);
    free(base);
    free(spin);
    openscan_live_destroy(R);
    free(g);
    free(s);
    openscan_live_destroy(L);
    return stripe_live_self_test();
}
