/* Mold solid and the live session. No image library: the outline, the
 * sweep, and the picture are ordinary arrays. */
#include "openscan/openscan_live.h"
#include "os_logic.h"

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
    int scanned_bins;
    uint8_t seen[72];
    uint8_t *preview;
    uint8_t *panel[5];
    int pw, ph;
    int quit_flag;
    int save_flag;
    os_turn turn;
    float prev_cx;
    int have_prev;
    float axis_x, axis_y, axis_z;
    int have_axis;
    int fused, lost;
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
    L->ncell = 1 << 16;
    L->cell = calloc((size_t)L->ncell, sizeof(unsigned));
    L->pw = 960;
    L->ph = 540;
    L->preview = calloc((size_t)L->pw * L->ph * 3, 1);
    for (int i = 0; i < 5; i++) L->panel[i] = calloc((size_t)L->pw * L->ph * 3, 1);
    if (!L->model || !L->cell || !L->preview) {
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
    memset(live->seen, 0, sizeof live->seen);
    memset(live->cell, 0, (size_t)live->ncell * sizeof(unsigned));
}

void openscan_live_set_mode(openscan_live *live, int mode) {
    if (live) live->mode = mode;
}
int openscan_live_mode(const openscan_live *live) { return live ? live->mode : OPENSCAN_MODE_STOP; }

void openscan_live_set_shape(openscan_live *live, int shape) {
    if (!live) return;
    if (live->shape != shape) openscan_live_reset(live);
    live->shape = shape == OPENSCAN_SHAPE_MEASURED ? OPENSCAN_SHAPE_MEASURED : OPENSCAN_SHAPE_MOLD;
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

static int cell_new(openscan_live *L, float x, float y, float z) {
    int ix = os_lround(x / 3.f);
    int iy = os_lround(y / 3.f);
    int iz = os_lround(z / 3.f);
    unsigned h = (unsigned)(ix * 73856093 ^ iy * 19349663 ^ iz * 83492791);
    h &= (unsigned)(L->ncell - 1);
    for (int n = 0; n < 32; n++) {
        unsigned i = (h + (unsigned)n) & (unsigned)(L->ncell - 1);
        if (L->cell[i] == 0) {
            L->cell[i] = h + 1;
            return 1;
        }
        if (L->cell[i] == h + 1) return 0;
    }
    return 0;
}

static void mark_bin(openscan_live *L, float x, float z) {
    float ang = os_atan2(x - L->axis_x, -(z - L->axis_z));
    int b = (int)((ang + 3.14159265f) / (2.f * 3.14159265f) * 72.f);
    if (b < 0) b = 0;
    if (b > 71) b = 71;
    L->seen[b] = 1;
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

static void paint_model(uint8_t *dst, int dw, int dh, const Tri *tris, int n,
                        float yaw, float ax, float ay, float az) {
    memset(dst, 12, (size_t)dw * dh * 3);
    float *zbuf = malloc((size_t)dw * dh * sizeof(float));
    if (!zbuf) return;
    for (int i = 0; i < dw * dh; i++) zbuf[i] = 1e9f;
    float minx = 1e9f, maxx = -1e9f, miny = 1e9f, maxy = -1e9f;
    for (int i = 0; i < n; i++) {
        const float *p[3] = {tris[i].a, tris[i].b, tris[i].c};
        for (int k = 0; k < 3; k++) {
            float x = p[k][0], z = p[k][2];
            rot_y(yaw, ax, az, &x, &z);
            if (x < minx) minx = x;
            if (x > maxx) maxx = x;
            if (p[k][1] < miny) miny = p[k][1];
            if (p[k][1] > maxy) maxy = p[k][1];
            (void)z;
        }
    }
    float ext = maxx - minx;
    if (maxy - miny > ext) ext = maxy - miny;
    if (ext < 1.f) ext = 1.f;
    float scale = 0.86f * (float)dh / ext;
    float cx = 0.5f * (minx + maxx);
    float cy = 0.5f * (miny + maxy);
    for (int i = 0; i < n; i++) {
        float xs[3], ys[3], zs[3];
        const float *p[3] = {tris[i].a, tris[i].b, tris[i].c};
        for (int k = 0; k < 3; k++) {
            float x = p[k][0], z = p[k][2];
            rot_y(yaw, ax, az, &x, &z);
            xs[k] = dw * 0.5f + (x - cx) * scale;
            ys[k] = dh * 0.5f - (p[k][1] - cy) * scale;
            zs[k] = z;
        }
        float area = (xs[1] - xs[0]) * (ys[2] - ys[0]) - (ys[1] - ys[0]) * (xs[2] - xs[0]);
        if (os_fabs(area) < 0.2f) continue;
        int minpx = (int)os_min(xs[0], os_min(xs[1], xs[2]));
        int maxpx = (int)os_max(xs[0], os_max(xs[1], xs[2]));
        int minpy = (int)os_min(ys[0], os_min(ys[1], ys[2]));
        int maxpy = (int)os_max(ys[0], os_max(ys[1], ys[2]));
        minpx = clampi(minpx, 0, dw - 1);
        maxpx = clampi(maxpx, 0, dw - 1);
        minpy = clampi(minpy, 0, dh - 1);
        maxpy = clampi(maxpy, 0, dh - 1);
        float e1x = xs[1] - xs[0], e1y = ys[1] - ys[0], e1z = zs[1] - zs[0];
        float e2x = xs[2] - xs[0], e2y = ys[2] - ys[0], e2z = zs[2] - zs[0];
        float nx = e1y * e2z - e1z * e2y;
        float ny = e1z * e2x - e1x * e2z;
        float nz = e1x * e2y - e1y * e2x;
        float ln = os_sqrt(nx * nx + ny * ny + nz * nz) + 1e-6f;
        float lam = os_fabs((-0.2f * nx + 0.35f * ny - 0.9f * nz) / ln);
        lam = lam * 0.75f + 0.25f;
        int shade = (int)(255.f * (tris[i].tone * 0.45f + lam * 0.55f));
        if (shade < 0) shade = 0;
        if (shade > 255) shade = 255;
        for (int py = minpy; py <= maxpy; py++) {
            for (int px = minpx; px <= maxpx; px++) {
                float w0 = (xs[1] - px) * (ys[2] - py) - (ys[1] - py) * (xs[2] - px);
                float w1 = (xs[2] - px) * (ys[0] - py) - (ys[2] - py) * (xs[0] - px);
                float w2 = (xs[0] - px) * (ys[1] - py) - (ys[0] - py) * (xs[1] - px);
                int pos = w0 >= 0 && w1 >= 0 && w2 >= 0;
                int neg = w0 <= 0 && w1 <= 0 && w2 <= 0;
                if (!pos && !neg) continue;
                float zc = (w0 * zs[0] + w1 * zs[1] + w2 * zs[2]) / area;
                int id = py * dw + px;
                if (zc < zbuf[id]) {
                    zbuf[id] = zc;
                    dst[id * 3] = dst[id * 3 + 1] = dst[id * 3 + 2] = (uint8_t)shade;
                }
            }
        }
    }
    free(zbuf);
    (void)ay;
}

static int build_shell(openscan_live *L, const uint8_t *gray, int w, int h, int *ntri_out) {
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
            if (L->shape == OPENSCAN_SHAPE_MEASURED) v->z = z0;
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
    if (nok > 80 && L->build.solid) {
        if (!L->have_axis) {
            L->axis_x = sx / (float)nok;
            L->axis_y = sy / (float)nok;
            L->axis_z = z0;
            L->have_axis = 1;
        }
        for (int iy = 0; iy < gh - 1; iy++) {
            for (int ix = 0; ix < gw - 1; ix++) {
                Vert *v00 = &grid[iy * gw + ix];
                Vert *v10 = &grid[iy * gw + ix + 1];
                Vert *v01 = &grid[(iy + 1) * gw + ix];
                Vert *v11 = &grid[(iy + 1) * gw + ix + 1];
                if (v00->ok && v10->ok && v01->ok) {
                    add_tri(L, v00, v10, v01, L->turn.yaw);
                    ntri++;
                }
                if (v10->ok && v11->ok && v01->ok) {
                    add_tri(L, v10, v11, v01, L->turn.yaw);
                    ntri++;
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

static void draw_panels(openscan_live *L, const uint8_t *clean, const uint8_t *pattern, int w, int h) {
    int dw = L->pw / 2, dh = L->ph;
    gray_bgr(L->panel[0], dw, dh, clean, w, h);
    gray_bgr(L->panel[2], dw, dh, pattern, w, h);
    gray_bgr(L->panel[3], dw, dh, clean, w, h);
    paint_model(L->panel[1], dw, dh, L->model, L->nmodel, L->turn.yaw, L->axis_x, L->axis_y, L->axis_z);
    paint_model(L->panel[4], dw, dh, L->model, L->nmodel, L->turn.yaw + 1.15f, L->axis_x, L->axis_y, L->axis_z);
    memset(L->preview, 16, (size_t)L->pw * L->ph * 3);
    for (int y = 0; y < dh; y++) {
        memcpy(L->preview + (y * L->pw) * 3, L->panel[0] + (y * dw) * 3, (size_t)dw * 3);
        memcpy(L->preview + (y * L->pw + dw) * 3, L->panel[1] + (y * dw) * 3, (size_t)dw * 3);
    }
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
    /* Centroid of the bright object. A sideways move is a turn. */
    double sx = 0, sn = 0;
    for (int y = 0; y < height; y += 2) {
        for (int x = 0; x < width; x += 2) {
            if (clean[y * width + x] > 28) {
                sx += x;
                sn += 1;
            }
        }
    }
    float cx = sn > 20 ? (float)(sx / sn) : width * 0.5f;
    int scanning = live->mode == OPENSCAN_MODE_SCAN;
    int fuse = 0;
    if (live->have_prev && sn > 20) {
        float radius = os_sqrt((float)sn) * 2.f;
        os_turn_step(&live->turn, cx - live->prev_cx, radius, scanning, &fuse);
    }
    live->prev_cx = cx;
    live->have_prev = sn > 20;
    int before = live->nmodel;
    int added = 0;
    int nok = 0;
    if (scanning && (before == 0 || fuse))
        nok = build_shell(live, clean, width, height, &added);
    if (scanning && before == 0) live->fused++;
    else if (scanning && added > 0) live->fused++;
    else if (scanning && nok < 80 && before == 0) live->lost++;
    draw_panels(live, clean, pattern, width, height);
    int covered = os_scanned_deg(live->build.sweep_deg, live->turn.yaw, count_bins(live), live->nmodel);
    if (status) {
        status->fused = live->fused;
        status->lost = live->lost;
        status->valid_pixels = nok;
        status->median_mm = live->distance;
        status->tracking = live->nmodel > 80 ? 1 : -1;
        status->mode = live->mode;
        status->points = live->nmodel;
        status->scanned_deg = covered;
        status->detail = 1;
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

int openscan_live_write(openscan_live *live, const char *stl_path, int *triangles_out) {
    if (!live || !stl_path || live->nmodel < 1) return -1;
    FILE *f = fopen(stl_path, "wb");
    if (!f) return -1;
    char hdr[80];
    memset(hdr, 0, sizeof hdr);
    snprintf(hdr, sizeof hdr, "OpenScan mold");
    fwrite(hdr, 1, 80, f);
    uint32_t n = (uint32_t)live->nmodel;
    fwrite(&n, 4, 1, f);
    for (int i = 0; i < live->nmodel; i++) {
        float rec[12];
        memset(rec, 0, sizeof rec);
        rec[3] = live->model[i].a[0]; rec[4] = live->model[i].a[1]; rec[5] = live->model[i].a[2];
        rec[6] = live->model[i].b[0]; rec[7] = live->model[i].b[1]; rec[8] = live->model[i].b[2];
        rec[9] = live->model[i].c[0]; rec[10] = live->model[i].c[1]; rec[11] = live->model[i].c[2];
        fwrite(rec, 4, 12, f);
        uint16_t attr = 0;
        fwrite(&attr, 2, 1, f);
    }
    fclose(f);
    if (triangles_out) *triangles_out = live->nmodel;
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
    if (openscan_live_push(L, g, g, 320, 240, &st) != 0 || st.points < 400 || st.scanned_deg < 120) {
        fprintf(stderr, "c mold: front %d tris %d°\n", st.points, st.scanned_deg);
        return 1;
    }
    int n0 = st.points;
    float y0 = L->turn.yaw;
    if (openscan_live_push(L, g, g, 320, 240, &st) != 0 || st.points > n0 + 40 || os_fabs(L->turn.yaw - y0) > 1e-4f) {
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
    if (openscan_live_push(L, s, s, 320, 240, &st) != 0 || os_fabs(L->turn.yaw) < 2.f * 3.14159265f / 180.f) {
        fprintf(stderr, "c mold: shift not tracked yaw %.3f\n", L->turn.yaw);
        return 1;
    }
    if (st.points < n0) {
        fprintf(stderr, "c mold: turn dropped triangles\n");
        return 1;
    }
    fprintf(stderr, "c mold: front %d tris %d°, turn yaw %.1f° tris %d\n",
            n0, 160, L->turn.yaw * 180.f / 3.14159265f, st.points);
    free(g);
    free(s);
    openscan_live_destroy(L);
    return 0;
}
