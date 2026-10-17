/* Projector-plane depth. A bright column in the pattern camera is one stripe.
 * Streaming camera A is what turns that pattern on.
 * When the calibration has a spatial code, the stripe's phase selects the
 * projector plane. The lattice rank remains when that match fails, and the
 * distance hint is only that lattice's center column.
 * The synthetic wall is a small frame: each painted stripe is one plane
 * at the hint. docs/projector.md. */
#include "os_stripe.h"
#include "os_logic.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define OS_STRIPE_MAX_ROWS 400
#define OS_STRIPE_MAX_PEAKS 96

typedef struct {
    double nc0, nc1, nc2, num;
    int index;
} Peq;

static double dabs(double x) { return x < 0.0 ? -x : x; }

static void rodriques(const double r[3], double R[9]) {
    double th = os_sqrt((float)(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]));
    if (th < 1e-12) {
        R[0] = 1; R[1] = 0; R[2] = 0;
        R[3] = 0; R[4] = 1; R[5] = 0;
        R[6] = 0; R[7] = 0; R[8] = 1;
        return;
    }
    double kx = r[0] / th, ky = r[1] / th, kz = r[2] / th;
    double c = os_cos((float)th), s = os_sin((float)th), C = 1.0 - c;
    R[0] = c + kx * kx * C;
    R[1] = kx * ky * C - kz * s;
    R[2] = kx * kz * C + ky * s;
    R[3] = ky * kx * C + kz * s;
    R[4] = c + ky * ky * C;
    R[5] = ky * kz * C - kx * s;
    R[6] = kz * kx * C - ky * s;
    R[7] = kz * ky * C + kx * s;
    R[8] = c + kz * kz * C;
}

static void mul_rot(const double R[9], double x, double y, double z, double o[3]) {
    o[0] = R[0] * x + R[1] * y + R[2] * z;
    o[1] = R[3] * x + R[4] * y + R[5] * z;
    o[2] = R[6] * x + R[7] * y + R[8] * z;
}

/* R^T n, row-major R. */
static void mul_rot_t(const double R[9], double x, double y, double z, double o[3]) {
    o[0] = R[0] * x + R[3] * y + R[6] * z;
    o[1] = R[1] * x + R[4] * y + R[7] * z;
    o[2] = R[2] * x + R[5] * y + R[8] * z;
}

static int prepare_planes(const openscan_calib *cal, Peq *peq, int *n_out) {
    const openscan_pinhole *proj = &cal->cam[2];
    if (!(proj->fx > 1.0) || !(proj->fy > 1.0) || cal->nplanes < 4) return 0;
    int n = cal->nplanes;
    if (n > OPENSCAN_MAX_PLANES) n = OPENSCAN_MAX_PLANES;
    double R[9];
    rodriques(proj->rvec, R);
    for (int i = 0; i < n; i++) {
        double slope = cal->plane[i].slope;
        double offset = cal->plane[i].offset;
        double nx = proj->fx;
        double ny = -slope * proj->fy;
        double nz = proj->cx - slope * proj->cy - offset;
        double nc[3];
        mul_rot_t(R, nx, ny, nz, nc);
        peq[i].nc0 = nc[0];
        peq[i].nc1 = nc[1];
        peq[i].nc2 = nc[2];
        peq[i].num = -(nx * proj->tvec[0] + ny * proj->tvec[1] + nz * proj->tvec[2]);
        peq[i].index = cal->plane[i].index;
    }
    for (int i = 1; i < n; i++) {
        Peq key = peq[i];
        int j = i;
        while (j > 0 && peq[j - 1].index > key.index) {
            peq[j] = peq[j - 1];
            j--;
        }
        peq[j] = key;
    }
    *n_out = n;
    return 1;
}

static void undistort(const openscan_pinhole *cam, double u, double v, double *xu, double *yu) {
    double xd = (u - cam->cx) / cam->fx;
    double yd = (v - cam->cy) / cam->fy;
    double x = xd, y = yd;
    for (int i = 0; i < 6; i++) {
        double r2 = x * x + y * y;
        double r4 = r2 * r2;
        double rad = 1.0 + cam->k1 * r2 + cam->k2 * r4 + cam->k3 * r4 * r2;
        if (dabs(rad) < 1e-8) break;
        double dx = 2.0 * cam->p1 * x * y + cam->p2 * (r2 + 2.0 * x * x);
        double dy = cam->p1 * (r2 + 2.0 * y * y) + 2.0 * cam->p2 * x * y;
        x = (xd - dx) / rad;
        y = (yd - dy) / rad;
    }
    *xu = x;
    *yu = y;
}

static int undistort_ok(const openscan_pinhole *cam, double u, double v, double *xu, double *yu) {
    if (!(cam->fx > 1.0) || !(cam->fy > 1.0)) return 0;
    undistort(cam, u, v, xu, yu);
    return 1;
}

/* d is along the ray (xu, yu, 1), so Z = d. *Y is stored up. */
static int plane_at(const Peq *pl, double xu, double yu, double *z, double *X, double *Y) {
    double den = pl->nc0 * xu + pl->nc1 * yu + pl->nc2;
    if (dabs(den) < 1e-8) return 0;
    double d = pl->num / den;
    if (!(d > 1.0) || d > 8000.0) return 0;
    *z = d;
    *X = d * xu;
    *Y = -d * yu;
    return 1;
}

static void distort_project(const openscan_pinhole *cam, double X, double Y, double Z,
                            double *u, double *v) {
    double xn = X / Z, yn = Y / Z;
    double r2 = xn * xn + yn * yn;
    double r4 = r2 * r2;
    double rad = 1.0 + cam->k1 * r2 + cam->k2 * r4 + cam->k3 * r4 * r2;
    double xd = xn * rad + 2.0 * cam->p1 * xn * yn + cam->p2 * (r2 + 2.0 * xn * xn);
    double yd = yn * rad + cam->p1 * (r2 + 2.0 * yn * yn) + 2.0 * cam->p2 * xn * yn;
    *u = cam->fx * xd + cam->cx;
    *v = cam->fy * yd + cam->cy;
}

static int project_clean(const openscan_pinhole *cam, double Xp, double Yp, double Zp,
                          double *u, double *v) {
    double R[9], Xc[3];
    rodriques(cam->rvec, R);
    mul_rot(R, Xp, Yp, Zp, Xc);
    Xc[0] += cam->tvec[0];
    Xc[1] += cam->tvec[1];
    Xc[2] += cam->tvec[2];
    if (!(Xc[2] > 1.0)) return 0;
    distort_project(cam, Xc[0], Xc[1], Xc[2], u, v);
    return 1;
}

static int clean_flat(const uint8_t *img, int w, int h) {
    if (!img || w < 16 || h < 16) return 1;
    double acc = 0;
    int n = 0;
    for (int k = 1; k <= 6; k++) {
        int y = h * k / 7;
        for (int x = w / 8; x < w * 7 / 8; x += 2) {
            int s = 0, c = 0;
            for (int t = x - 8; t <= x + 8; t++) {
                if (t < 0 || t >= w) continue;
                s += img[y * w + t];
                c++;
            }
            double d = img[y * w + x] - (c ? (double)s / c : 0.0);
            acc += d < 0 ? -d : d;
            n++;
        }
    }
    return n == 0 || acc / n < 4.0;
}

static double clean_ridge(const uint8_t *img, int w, int h, int flipped, double u, double v) {
    int x = (int)(u >= 0 ? u + 0.5 : u - 0.5);
    int y = (int)(v >= 0 ? v + 0.5 : v - 0.5);
    if (flipped) {
        x = (w - 1) - x;
        y = (h - 1) - y;
    }
    if (x < 1 || y < 0 || x >= w - 1 || y >= h) return 0;
    int s = 0, c = 0;
    for (int t = x - 8; t <= x + 8; t++) {
        if (t < 0 || t >= w) continue;
        s += img[y * w + t];
        c++;
    }
    double hp = img[y * w + x] - (c ? (double)s / c : 0.0);
    return hp > 0 ? hp : 0;
}

typedef struct {
    int x;
    int pi;
    float z, X, Y, tone;
    int ok;
} Hit;

/* Camera pixels between this projector plane and the next, at this ray.
 * Higher index is the plane whose stripe sits to the right. */
static double plane_pitch_px(const openscan_pinhole *cam, const Peq *peq, int nplanes,
                             int pi, double xu, double yu) {
    double z0, z1, zu, X, Y, dzx, dzp, px;
    if (pi < 0) pi = 0;
    if (pi + 1 >= nplanes) pi = nplanes - 2;
    if (pi < 0) return 6.0;
    if (!plane_at(&peq[pi], xu, yu, &z0, &X, &Y)) return 6.0;
    if (!plane_at(&peq[pi + 1], xu, yu, &z1, &X, &Y)) return 6.0;
    if (!plane_at(&peq[pi], xu + 0.002, yu, &zu, &X, &Y)) return 6.0;
    dzx = (zu - z0) / 0.002;
    dzp = z1 - z0;
    if (dabs(dzx) < 1e-4) return 6.0;
    px = dabs(dzp / dzx) * cam->fx;
    if (px < 2.0 || px > 40.0) return 6.0;
    return px;
}

/* Move the running plane by the pixel gap. The fraction is kept, so a nose
 * that shifts a stripe changes the plane. Snapping to the nearest depth
 * holds every stripe on the hint and the toy becomes a flat sheet. */
static int plane_ok(const Peq *peq, int nplanes, int pi, double xu, double yu,
                    double near_mm, double far_mm, double *z, double *X, double *Y) {
    if (pi < 0 || pi >= nplanes) return 0;
    if (!plane_at(&peq[pi], xu, yu, z, X, Y)) return 0;
    if (*z < near_mm || *z > far_mm) return 0;
    return 1;
}

static int step_along(const Peq *peq, int nplanes, double pf, int dir, int gap_px,
                      double px_per, double cur_z, double xu, double yu,
                      double near_mm, double far_mm, double *pf_out, int *pi_out,
                      double *z_out, double *X_out, double *Y_out) {
    double add, next, z, X, Y, zg, Xg, Yg;
    int pi, geom, off;
    if (px_per < 2.0) px_per = 6.0;
    if (gap_px < 1) gap_px = (int)(px_per + 0.5);
    add = (double)gap_px / px_per;
    if (add > 8.0) add = 8.0;
    if (add < 0.65) add = 0.0;
    next = pf + (double)dir * add;
    geom = (int)(next >= 0.0 ? next + 0.5 : next - 0.5);
    if (!plane_ok(peq, nplanes, geom, xu, yu, near_mm, far_mm, &zg, &Xg, &Yg)) {
        for (off = 1; off >= -1; off -= 2) {
            if (!plane_ok(peq, nplanes, geom + off, xu, yu, near_mm, far_mm, &z, &X, &Y))
                continue;
            *pf_out = (double)(geom + off);
            *pi_out = geom + off;
            *z_out = z;
            *X_out = X;
            *Y_out = Y;
            return 1;
        }
        return 0;
    }
    /* A one-plane miss sits a few millimetres off a surface that is still here.
     * A real change of depth does not have that neighbour. */
    if (dabs(zg - cur_z) > 2.0) {
        for (off = -1; off <= 1; off += 2) {
            if (!plane_ok(peq, nplanes, geom + off, xu, yu, near_mm, far_mm, &z, &X, &Y))
                continue;
            if (dabs(z - cur_z) > 2.0) continue;
            if (dabs(z - cur_z) >= dabs(zg - cur_z)) continue;
            *pf_out = (double)(geom + off);
            *pi_out = geom + off;
            *z_out = z;
            *X_out = X;
            *Y_out = Y;
            return 1;
        }
    }
    pi = geom;
    *pf_out = next;
    *pi_out = pi;
    *z_out = zg;
    *X_out = Xg;
    *Y_out = Yg;
    return 1;
}

static void walk_peaks(Hit *h, const int *peaks, int np, const uint8_t *row, int start,
                       int dir_peak, double pf0, double px_per, const openscan_calib *cal,
                       const Peq *peq, int nplanes, int y, int width, int height, int flipped,
                       double near_mm, double far_mm, double z_anchor) {
    int cur = start;
    int guard = 0;
    int dir_plane = flipped ? -dir_peak : dir_peak;
    double pf = pf0;
    while (guard++ < OS_STRIPE_MAX_PEAKS) {
        int nxt = cur + dir_peak;
        int pi = 0, gap, peek;
        double pf_next = pf, xu, yu, z, X, Y, u, v;
        if (nxt < 0 || nxt >= np || h[nxt].ok) break;
        gap = peaks[nxt] - peaks[cur];
        if (gap < 0) gap = -gap;
        u = peaks[nxt];
        v = y;
        if (flipped) {
            u = (width - 1) - u;
            v = (height - 1) - v;
        }
        if (!undistort_ok(&cal->cam[0], u, v, &xu, &yu) ||
            !step_along(peq, nplanes, pf, dir_plane, gap, px_per, h[cur].z, xu, yu, near_mm,
                        far_mm, &pf_next, &pi, &z, &X, &Y)) {
            peek = nxt + dir_peak;
            if (peek < 0 || peek >= np || h[peek].ok) break;
            gap = peaks[peek] - peaks[cur];
            if (gap < 0) gap = -gap;
            if (gap > 56) break;
            u = peaks[peek];
            v = y;
            if (flipped) {
                u = (width - 1) - u;
                v = (height - 1) - v;
            }
            if (!undistort_ok(&cal->cam[0], u, v, &xu, &yu) ||
                !step_along(peq, nplanes, pf, dir_plane, gap, px_per, h[cur].z, xu, yu, near_mm,
                            far_mm, &pf_next, &pi, &z, &X, &Y))
                break;
            nxt = peek;
        }
        if (dabs(z - z_anchor) > 80.0) break;
        h[nxt].x = peaks[nxt];
        h[nxt].pi = pi;
        h[nxt].z = (float)z;
        h[nxt].X = (float)X;
        h[nxt].Y = (float)Y;
        h[nxt].tone = row[peaks[nxt]] / 255.f;
        h[nxt].ok = 1;
        pf = pf_next;
        cur = nxt;
    }
}

/* Both edges of one stripe alternate short, long, short, long. Real stripes
 * do not. Merge only that alternation, and keep the stronger edge. */
static int merge_double_edges(int *peaks, double *hp, int np) {
    int even_s = 0, odd_s = 0, even_n = 0, odd_n = 0, i, lim, w, ae, ao, small, big;
    if (np < 8) return np;
    for (i = 1; i < np; i++) {
        int g = peaks[i] - peaks[i - 1];
        if ((i & 1) == 0) {
            even_s += g;
            even_n++;
        } else {
            odd_s += g;
            odd_n++;
        }
    }
    if (even_n < 3 || odd_n < 3) return np;
    ae = even_s / even_n;
    ao = odd_s / odd_n;
    small = ae < ao ? ae : ao;
    big = ae > ao ? ae : ao;
    if (small < 3 || big < small * 8 / 5) return np;
    lim = (small + big) / 2;
    if (lim < small + 1) lim = small + 1;
    w = 0;
    for (i = 0; i < np; i++) {
        if (w > 0 && peaks[i] - peaks[w - 1] < lim) {
            if (hp[i] > hp[w - 1]) {
                peaks[w - 1] = peaks[i];
                hp[w - 1] = hp[i];
            }
            continue;
        }
        peaks[w] = peaks[i];
        hp[w] = hp[i];
        w++;
    }
    return w;
}

static int lock_hit(const openscan_calib *cal, const Peq *peq, int nplanes,
                   const uint8_t *clean, int width, int height, int flipped, int flat,
                   float hint_mm, float near_mm, float far_mm,
                   int x, int y, float tone, Hit *h) {
    double u = x, v = y, xu, yu, best_s = 1e300, bz = 0, bX = 0, bY = 0;
    int best_pi = -1, in_band = 0, i;
    if (flipped) {
        u = (width - 1) - u;
        v = (height - 1) - v;
    }
    if (!undistort_ok(&cal->cam[0], u, v, &xu, &yu)) return 0;
    for (i = 0; i < nplanes; i++) {
        double z, X, Y;
        if (!plane_at(&peq[i], xu, yu, &z, &X, &Y)) continue;
        if (z < near_mm || z > far_mm) continue;
        if (dabs(z - hint_mm) <= 40.0) in_band = 1;
    }
    for (i = 0; i < nplanes; i++) {
        double z, X, Y, score, cu, cv;
        if (!plane_at(&peq[i], xu, yu, &z, &X, &Y)) continue;
        if (z < near_mm || z > far_mm) continue;
        score = dabs(z - hint_mm);
        if (in_band && score > 40.0) continue;
        if (!flat && clean && project_clean(&cal->cam[1], z * xu, z * yu, z, &cu, &cv)) {
            if (clean_ridge(clean, width, height, flipped, cu, cv) > 12.0) score -= 3.0;
        }
        if (score < best_s) {
            best_s = score;
            best_pi = i;
            bz = z;
            bX = X;
            bY = Y;
        }
    }
    if (best_pi < 0) return 0;
    h->x = x;
    h->pi = best_pi;
    h->z = (float)bz;
    h->X = (float)bX;
    h->Y = (float)bY;
    h->tone = tone;
    h->ok = 1;
    return 1;
}

static void heapsort(float *a, int n) {
    for (int i = 1; i < n; i++) {
        float v = a[i];
        int j = i;
        while (j > 0) {
            int p = (j - 1) / 2;
            if (a[p] >= v) break;
            a[j] = a[p];
            j = p;
        }
        a[j] = v;
    }
    for (int end = n - 1; end > 0; end--) {
        float v = a[end];
        a[end] = a[0];
        int j = 0;
        for (;;) {
            int c = j * 2 + 1;
            if (c >= end) break;
            if (c + 1 < end && a[c + 1] > a[c]) c++;
            if (a[c] <= v) break;
            a[j] = a[c];
            j = c;
        }
        a[j] = v;
    }
}

static int uf_find(int *p, int i) {
    while (p[i] != i) {
        p[i] = p[p[i]];
        i = p[i];
    }
    return i;
}

static void uf_join(int *p, int *sz, int a, int b) {
    a = uf_find(p, a);
    b = uf_find(p, b);
    if (a == b) return;
    if (sz[a] < sz[b]) {
        int t = a;
        a = b;
        b = t;
    }
    p[b] = a;
    sz[a] += sz[b];
}

static int link_ok(const os_stripe_pt *a, const os_stripe_pt *b) {
    int dp;
    float dx, dy, dz;
    if (!a->ok || !b->ok) return 0;
    dp = a->plane - b->plane;
    if (dp < 0) dp = -dp;
    /* Neighbours on this lattice are about two factory planes apart.
     * A rounding step can land three apart. Four still will not bridge a hole. */
    if (dp > 4) return 0;
    dx = a->x - b->x;
    dy = a->y - b->y;
    dz = a->z - b->z;
    return dx * dx + dy * dy + dz * dz < 14.f * 14.f;
}

static void drop_specks(os_stripe_cloud *c) {
    int n, maxc, i, r, k;
    int *parent, *sz;
    if (!c || c->n < 16 || c->rows < 1 || c->cols < 1) return;
    n = c->rows * c->cols;
    parent = malloc((size_t)n * sizeof(int));
    sz = malloc((size_t)n * sizeof(int));
    if (!parent || !sz) {
        free(parent);
        free(sz);
        return;
    }
    for (i = 0; i < n; i++) {
        parent[i] = i;
        sz[i] = c->pt[i].ok ? 1 : 0;
    }
    for (r = 0; r < c->rows; r++) {
        os_stripe_pt *row = c->pt + r * c->cols;
        for (k = 0; k < c->cols - 1; k++) {
            if (link_ok(&row[k], &row[k + 1]))
                uf_join(parent, sz, r * c->cols + k, r * c->cols + k + 1);
        }
        if (r + 1 >= c->rows) continue;
        for (k = 0; k < c->cols; k++) {
            int t;
            if (!row[k].ok) continue;
            for (t = 0; t < c->cols; t++) {
                os_stripe_pt *q = c->pt + (r + 1) * c->cols + t;
                if (q->ok && q->plane == row[k].plane && link_ok(&row[k], q)) {
                    uf_join(parent, sz, r * c->cols + k, (r + 1) * c->cols + t);
                    break;
                }
            }
        }
    }
    maxc = 0;
    for (i = 0; i < n; i++) {
        if (c->pt[i].ok && parent[i] == i && sz[i] > maxc) maxc = sz[i];
    }
    if (maxc >= 8) {
        c->n = 0;
        for (i = 0; i < n; i++) {
            if (!c->pt[i].ok) continue;
            /* A snout is its own component once a shadow breaks the columns.
             * Only a speck much smaller than the face is dropped. */
            if (sz[uf_find(parent, i)] * 16 < maxc) {
                c->pt[i].ok = 0;
                continue;
            }
            c->n++;
        }
    }
    free(parent);
    free(sz);
}

/* The face survives as one component. The hood is the same toy, a few
 * millimetres away, and its plane index has drifted so the speck filter
 * cut it off. Put back a point that still lands on that toy in the clean
 * picture and still touches the face. */
static void adopt_plush(os_stripe_cloud *c, const openscan_calib *cal, const uint8_t *pattern,
                        const uint8_t *clean, int width, int height) {
    int ncell, i, seeds, bright, qh, qt, pass;
    uint8_t *mask, *seen;
    int *qx, *qy, *qd;
    if (!c || !c->pt || !cal || !pattern || !clean || c->n < 80) return;
    /* The wall rig is a small synthetic frame. This photo test is the camera. */
    if (width < 640 || height < 400) return;
    if (clean_flat(clean, width, height)) return;
    ncell = width * height;
    mask = calloc((size_t)ncell, 1);
    seen = calloc((size_t)ncell, 1);
    qx = malloc((size_t)ncell * sizeof(int));
    qy = malloc((size_t)ncell * sizeof(int));
    qd = malloc((size_t)ncell * sizeof(int));
    if (!mask || !seen || !qx || !qy || !qd) {
        free(mask);
        free(seen);
        free(qx);
        free(qy);
        free(qd);
        return;
    }
    qt = 0;
    seeds = 0;
    for (i = 0; i < c->rows * c->cols; i++) {
        double cu, cv;
        int ix, iy, id, g;
        if (!c->pt[i].ok) continue;
        if (!project_clean(&cal->cam[1], c->pt[i].x, -c->pt[i].y, c->pt[i].z, &cu, &cv)) continue;
        ix = (int)(cu >= 0 ? cu + 0.5 : cu - 0.5);
        iy = (int)(cv >= 0 ? cv + 0.5 : cv - 0.5);
        if (ix < 0 || iy < 0 || ix >= width || iy >= height) continue;
        g = clean[iy * width + ix];
        if (g < 140) continue;
        id = iy * width + ix;
        if (seen[id]) continue;
        seen[id] = 1;
        mask[id] = 1;
        qx[qt] = ix;
        qy[qt] = iy;
        qd[qt] = 0;
        qt++;
        seeds++;
    }
    bright = 0;
    qh = 0;
    while (qh < qt) {
        int x = qx[qh];
        int y = qy[qh];
        int n;
        qh++;
        bright++;
        for (n = 0; n < 8; n++) {
            static const int ox[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
            static const int oy[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
            int xx = x + ox[n];
            int yy = y + oy[n];
            int id;
            if (xx < 0 || yy < 0 || xx >= width || yy >= height) continue;
            id = yy * width + xx;
            if (seen[id] || clean[id] < 140) continue;
            seen[id] = 1;
            mask[id] = 1;
            qx[qt] = xx;
            qy[qt] = yy;
            qd[qt] = 0;
            qt++;
        }
    }
    /* A bright face enclosed by the hood must not flood into the heater.
     * The hood is the dark cloth touching that face, and only a short way out. */
    if (seeds >= 30 && bright >= 2000) {
        qh = qt = 0;
        for (i = 0; i < ncell; i++) {
            if (!mask[i]) continue;
            qx[qt] = i % width;
            qy[qt] = i / width;
            qd[qt] = 0;
            qt++;
        }
        while (qh < qt) {
            int x = qx[qh];
            int y = qy[qh];
            int d = qd[qh];
            int n;
            qh++;
            if (d >= 110) continue;
            for (n = 0; n < 4; n++) {
                int xx = x + (n == 0 ? -1 : n == 1 ? 1 : 0);
                int yy = y + (n == 2 ? -1 : n == 3 ? 1 : 0);
                int id, g;
                if (xx < 0 || yy < 0 || xx >= width || yy >= height) continue;
                id = yy * width + xx;
                if (seen[id]) continue;
                g = clean[id];
                if (g < 24 || g >= 140) continue;
                seen[id] = 1;
                mask[id] = 1;
                qx[qt] = xx;
                qy[qt] = yy;
                qd[qt] = d + 1;
                qt++;
            }
        }
        float med = 0.f;
        int have_med = 0;
        {
            float *zs = malloc((size_t)c->n * sizeof(float));
            int nz = 0;
            if (zs) {
                for (i = 0; i < c->rows * c->cols; i++)
                    if (c->pt[i].ok) zs[nz++] = c->pt[i].z;
                if (nz >= 16) {
                    heapsort(zs, nz);
                    med = zs[nz / 2];
                    have_med = 1;
                }
                free(zs);
            }
        }
        /* Eight millimetres of depth per pass walks onto the heater.
         * The face median is the toy. A recovered point stays on that depth. */
        if (!have_med) {
            free(mask);
            free(seen);
            free(qx);
            free(qy);
            free(qd);
            return;
        }
        /* The clean picture's bright face runs into the heater behind the toy.
         * The stripe picture still has a dark gap there. A recovered point has
         * to sit on the same stripes as the face, not across that gap. */
        {
            uint8_t *toy = calloc((size_t)ncell, 1);
            int *gs, ns = 0, thr = 8, qh2, qt2;
            if (!toy) {
                free(mask);
                free(seen);
                free(qx);
                free(qy);
                free(qd);
                return;
            }
            gs = malloc((size_t)c->n * sizeof(int));
            if (gs) {
                for (i = 0; i < c->rows * c->cols; i++) {
                    int u, v;
                    if (!c->pt[i].ok) continue;
                    u = c->pt[i].u;
                    v = c->pt[i].v;
                    if (u < 0 || v < 0 || u >= width || v >= height) continue;
                    gs[ns++] = pattern[v * width + u];
                }
                if (ns >= 20) {
                    /* 5th percentile, as a byte sort via the float heapsort. */
                    float *fv = malloc((size_t)ns * sizeof(float));
                    if (fv) {
                        int k;
                        for (k = 0; k < ns; k++) fv[k] = (float)gs[k];
                        heapsort(fv, ns);
                        thr = (int)fv[ns / 20];
                        free(fv);
                    }
                }
                free(gs);
            }
            if (thr < 4) thr = 4;
            /* The peak is brighter than the cloth between stripes. A cut at the
             * face's own dim tail never crosses that cloth, so the hood stays
             * out. Eight is still darker than this toy and darker than the gap
             * in front of the heater. */
            if (thr > 8) thr = 8;
            qt2 = 0;
            for (i = 0; i < c->rows * c->cols; i++) {
                int u, v, id;
                if (!c->pt[i].ok) continue;
                u = c->pt[i].u;
                v = c->pt[i].v;
                if (u < 0 || v < 0 || u >= width || v >= height) continue;
                id = v * width + u;
                if (toy[id]) continue;
                toy[id] = 1;
                qx[qt2] = u;
                qy[qt2] = v;
                qt2++;
            }
            qh2 = 0;
            while (qh2 < qt2) {
                int x = qx[qh2];
                int y = qy[qh2];
                int n;
                qh2++;
                for (n = 0; n < 4; n++) {
                    int xx = x + (n == 0 ? -1 : n == 1 ? 1 : 0);
                    int yy = y + (n == 2 ? -1 : n == 3 ? 1 : 0);
                    int id;
                    if (xx < 0 || yy < 0 || xx >= width || yy >= height) continue;
                    id = yy * width + xx;
                    if (toy[id] || pattern[id] < thr) continue;
                    toy[id] = 1;
                    qx[qt2] = xx;
                    qy[qt2] = yy;
                    qt2++;
                }
            }
            for (pass = 0; pass < 8; pass++) {
                int added = 0, nb = 0, b = 0, k;
                int *bucket = NULL;
                for (i = 0; i < c->rows * c->cols; i++)
                    if (c->pt[i].ok) nb++;
                bucket = malloc((size_t)nb * sizeof(int));
                if (!bucket) break;
                for (i = 0; i < c->rows * c->cols; i++)
                    if (c->pt[i].ok) bucket[b++] = i;
                for (i = 0; i < c->rows * c->cols; i++) {
                    os_stripe_pt *p = &c->pt[i];
                    double cu, cv;
                    int ix, iy, id, hit, pu, pv;
                    if (p->ok || !(p->z > 20.f)) continue;
                    if (os_fabs(p->z - med) > 14.f) continue;
                    pu = p->u;
                    pv = p->v;
                    if (pu < 1 || pv < 1 || pu >= width - 1 || pv >= height - 1) continue;
                    id = pv * width + pu;
                    if (!toy[id] && !toy[id - 1] && !toy[id + 1] && !toy[id - width] &&
                        !toy[id + width])
                        continue;
                    if (!project_clean(&cal->cam[1], p->x, -p->y, p->z, &cu, &cv)) continue;
                    ix = (int)(cu >= 0 ? cu + 0.5 : cu - 0.5);
                    iy = (int)(cv >= 0 ? cv + 0.5 : cv - 0.5);
                    if (ix < 1 || iy < 1 || ix >= width - 1 || iy >= height - 1) continue;
                    id = iy * width + ix;
                    if (!mask[id] && !mask[id - 1] && !mask[id + 1] && !mask[id - width] &&
                        !mask[id + width])
                        continue;
                    hit = 0;
                    for (k = 0; k < nb; k++) {
                        os_stripe_pt *q = &c->pt[bucket[k]];
                        float dx = p->x - q->x, dy = p->y - q->y, dz = p->z - q->z;
                        if (dz < 0.f) dz = -dz;
                        if (dz > 8.f) continue;
                        if (dx * dx + dy * dy + dz * dz <= 8.f * 8.f) {
                            hit = 1;
                            break;
                        }
                    }
                    if (!hit) continue;
                    p->ok = 1;
                    c->n++;
                    added++;
                }
                free(bucket);
                if (!added) break;
            }
            free(toy);
        }
    }
    free(mask);
    free(seen);
    free(qx);
    free(qy);
    free(qd);
}

/* One frame keeps the skin within 70 mm of the median depth.
 * That is enough for a snout in front of the face, and it still drops
 * a second surface behind the toy. */
static void keep_band(os_stripe_cloud *c) {
    float *zs;
    int n = 0, i;
    float med;
    if (!c || !c->pt || c->n < 16) return;
    zs = malloc((size_t)c->n * sizeof(float));
    if (!zs) return;
    for (i = 0; i < c->rows * c->cols; i++)
        if (c->pt[i].ok) zs[n++] = c->pt[i].z;
    if (n < 16) {
        free(zs);
        return;
    }
    heapsort(zs, n);
    med = zs[n / 2];
    free(zs);
    c->n = 0;
    for (i = 0; i < c->rows * c->cols; i++) {
        if (!c->pt[i].ok) continue;
        if (os_fabs(c->pt[i].z - med) > 70.f) {
            c->pt[i].ok = 0;
            continue;
        }
        c->n++;
    }
    drop_specks(c);
}

/* A few samples of the same plane disagree by a fraction of a millimetre.
 * Mixing another plane's depth is the terrace. The point stays on its ray. */
static void smooth_rays(os_stripe_cloud *c) {
    float *nz;
    int ncell, r, k, i;
    if (!c || !c->pt || c->n < 16 || c->cols < 1) return;
    ncell = c->rows * c->cols;
    nz = malloc((size_t)ncell * sizeof(float));
    if (!nz) return;
    memset(nz, 0, (size_t)ncell * sizeof(float));
    for (r = 0; r < c->rows; r++) {
        int r0 = r - 2, r1 = r + 2, rr;
        if (r0 < 0) r0 = 0;
        if (r1 >= c->rows) r1 = c->rows - 1;
        for (k = 0; k < c->cols; k++) {
            float win[32];
            int m = 0;
            i = r * c->cols + k;
            if (!c->pt[i].ok) continue;
            for (rr = r0; rr <= r1; rr++) {
                int t, t0 = k - 2, t1 = k + 2;
                if (t0 < 0) t0 = 0;
                if (t1 >= c->cols) t1 = c->cols - 1;
                for (t = t0; t <= t1; t++) {
                    float dx, dy;
                    os_stripe_pt *q = &c->pt[rr * c->cols + t];
                    if (!q->ok || q->plane != c->pt[i].plane) continue;
                    dx = c->pt[i].x - q->x;
                    dy = c->pt[i].y - q->y;
                    if (dx * dx + dy * dy > 6.f * 6.f) continue;
                    if (m < 32) win[m++] = q->z;
                }
            }
            if (m >= 3) {
                heapsort(win, m);
                nz[i] = win[m / 2];
            } else {
                nz[i] = c->pt[i].z;
            }
        }
    }
    for (i = 0; i < ncell; i++) {
        float z0, s;
        if (!c->pt[i].ok) continue;
        z0 = c->pt[i].z;
        if (!(z0 > 1.f) || !(nz[i] > 1.f)) continue;
        s = nz[i] / z0;
        c->pt[i].z = nz[i];
        c->pt[i].x *= s;
        c->pt[i].y *= s;
    }
    free(nz);
}

/* Half-width of the pattern high-pass. The projected columns repeat about
 * every 13 px. A 17 px average cancels that lattice. 7 px leaves a column
 * brighter than its neighbors, and a one-pixel stripe still peaks. */
enum { OS_HP = 3 };

static double row_hp(const uint8_t *row, int width, int x) {
    int a, b, t, s = 0, c = 0;
    if (x < 0 || x >= width) return 0;
    a = x - OS_HP;
    b = x + OS_HP;
    if (a < 0) a = 0;
    if (b >= width) b = width - 1;
    for (t = a; t <= b; t++) {
        s += row[t];
        c++;
    }
    return row[x] - (c ? (double)s / c : 0.0);
}

/* Narrow minus wide. The columns repeat about every 13 px, so a 3 px mean
 * minus a 13 px mean peaks once per column. A one-pixel stripe still peaks. */
static double lattice_hp(const uint8_t *row, int width, int x) {
    int t, s1 = 0, c1 = 0, s6 = 0, c6 = 0;
    if (x < 0 || x >= width) return 0;
    for (t = x - 1; t <= x + 1; t++) {
        if (t < 0 || t >= width) continue;
        s1 += row[t];
        c1++;
    }
    for (t = x - 6; t <= x + 6; t++) {
        if (t < 0 || t >= width) continue;
        s6 += row[t];
        c6++;
    }
    if (!c1 || !c6) return 0;
    return (double)s1 / c1 - (double)s6 / c6;
}

static int median_gap(const int *xs, int n) {
    int gap[OS_STRIPE_MAX_PEAKS];
    int i, g = 0;
    if (n < 2) return 0;
    for (i = 1; i < n && g < OS_STRIPE_MAX_PEAKS; i++) gap[g++] = xs[i] - xs[i - 1];
    for (i = 1; i < g; i++) {
        int v = gap[i], j = i;
        while (j > 0 && gap[j - 1] > v) {
            gap[j] = gap[j - 1];
            j--;
        }
        gap[j] = v;
    }
    return gap[g / 2];
}

/* One peak per projected column. Returns 0 when this row is not that lattice,
 * so the caller can fall back to the narrow stripe finder. */
static int lattice_peaks(const uint8_t *row, int width, int *peaks, double *hp_out, int cap) {
    int np = 0, last = -100, x, g;
    if (cap > OS_STRIPE_MAX_PEAKS) cap = OS_STRIPE_MAX_PEAKS;
    for (x = 8; x < width - 8 && np < cap; x++) {
        double hp = lattice_hp(row, width, x);
        if (hp < 6.0) continue;
        if (!(hp >= lattice_hp(row, width, x - 1) && hp > lattice_hp(row, width, x + 1) &&
              hp >= lattice_hp(row, width, x - 2) && hp > lattice_hp(row, width, x + 2)))
            continue;
        if (x - last < 8) {
            if (np > 0 && hp > hp_out[np - 1]) {
                peaks[np - 1] = x;
                hp_out[np - 1] = hp;
                last = x;
            }
            continue;
        }
        peaks[np] = x;
        hp_out[np] = hp;
        np++;
        last = x;
    }
    if (np < 12) return 0;
    g = median_gap(peaks, np);
    if (g < 9 || g > 20) return 0;
    return np;
}

static int find_columns(const uint8_t *row, int width, int *peaks, double *hp_out, int cap) {
    int n = lattice_peaks(row, width, peaks, hp_out, cap);
    int np = 0, last = -100, x;
    if (n >= 12) return n;
    if (cap > OS_STRIPE_MAX_PEAKS) cap = OS_STRIPE_MAX_PEAKS;
    for (x = OS_HP + 1; x < width - OS_HP - 1 && np < cap; x++) {
        double hp = row_hp(row, width, x);
        if (hp < 1.0) continue;
        if (!(hp >= row_hp(row, width, x - 1) && hp > row_hp(row, width, x + 1))) continue;
        if (x - last < 3) continue;
        peaks[np] = x;
        hp_out[np] = hp;
        np++;
        last = x;
    }
    return np;
}

/* The factory file stops at 130 lines. The snout still carries the same
 * lattice past that edge. Slope and offset change smoothly, so the next
 * lines are the same step continued. A short synthetic rig is left alone. */
#define OS_PLANE_CAP 480

static void fill_peq(const openscan_pinhole *proj, const double R[9], double slope, double offset,
                     int index, Peq *o) {
    double nx = proj->fx;
    double ny = -slope * proj->fy;
    double nz = proj->cx - slope * proj->cy - offset;
    double nc[3];
    mul_rot_t(R, nx, ny, nz, nc);
    o->nc0 = nc[0];
    o->nc1 = nc[1];
    o->nc2 = nc[2];
    o->num = -(nx * proj->tvec[0] + ny * proj->tvec[1] + nz * proj->tvec[2]);
    o->index = index;
}

static int build_extended(const openscan_calib *cal, Peq *peq, int cap) {
    const openscan_pinhole *proj = &cal->cam[2];
    int n, i, extra;
    int ord[OPENSCAN_MAX_PLANES];
    double R[9], dsl, dof, sl1, of1;
    if (!cal || cap < 4) return 0;
    n = cal->nplanes;
    if (n < 4) return 0;
    if (n > OPENSCAN_MAX_PLANES) n = OPENSCAN_MAX_PLANES;
    if (n > cap) n = cap;
    for (i = 0; i < n; i++) ord[i] = i;
    for (i = 1; i < n; i++) {
        int key = ord[i], j = i;
        while (j > 0 && cal->plane[ord[j - 1]].index > cal->plane[key].index) {
            ord[j] = ord[j - 1];
            j--;
        }
        ord[j] = key;
    }
    rodriques(proj->rvec, R);
    for (i = 0; i < n; i++) {
        const openscan_plane *pl = &cal->plane[ord[i]];
        fill_peq(proj, R, pl->slope, pl->offset, pl->index, &peq[i]);
    }
    if (n < 80 || n + 8 > cap) return n;
    sl1 = cal->plane[ord[n - 1]].slope;
    of1 = cal->plane[ord[n - 1]].offset;
    dsl = (sl1 - cal->plane[ord[n - 8]].slope) / 7.0;
    dof = (of1 - cal->plane[ord[n - 8]].offset) / 7.0;
    if (!(dof > 1.0 && dof < 30.0)) return n;
    extra = cap - n;
    if (extra > 260) extra = 260;
    for (i = 1; i <= extra; i++)
        fill_peq(proj, R, sl1 + (double)i * dsl, of1 + (double)i * dof,
                 cal->plane[ord[n - 1]].index + i, &peq[n + i - 1]);
    return n + extra;
}

/* Continuous plane index. The stored slot is the nearest factory line.
 * The intersection uses the line between the two neighbours, so a column
 * that falls halfway between stored lines does not sawtooth in depth. */
static int blend_at(const Peq *peq, int nplanes, double p, double xu, double yu, double near_mm,
                    double far_mm, int *pi_out, double *z, double *X, double *Y) {
    int i0;
    double f;
    Peq m;
    if (p >= 0.0) i0 = (int)p;
    else {
        i0 = (int)p;
        if ((double)i0 != p) i0--;
    }
    if (i0 < 0 || i0 >= nplanes) return 0;
    f = p - (double)i0;
    if (f < 1e-6 || i0 + 1 >= nplanes) {
        if (!plane_at(&peq[i0], xu, yu, z, X, Y)) return 0;
        *pi_out = i0;
    } else {
        m.nc0 = peq[i0].nc0 * (1.0 - f) + peq[i0 + 1].nc0 * f;
        m.nc1 = peq[i0].nc1 * (1.0 - f) + peq[i0 + 1].nc1 * f;
        m.nc2 = peq[i0].nc2 * (1.0 - f) + peq[i0 + 1].nc2 * f;
        m.num = peq[i0].num * (1.0 - f) + peq[i0 + 1].num * f;
        m.index = peq[i0].index;
        if (!plane_at(&m, xu, yu, z, X, Y)) return 0;
        *pi_out = f < 0.5 ? i0 : i0 + 1;
    }
    if (*z < near_mm || *z > far_mm) return 0;
    return 1;
}

typedef struct {
    float *xs;
    int n, cap;
    int last_r;
    float last_x;
    int alive;
    int mid_y;
    float mid_x;
    float mx;
    int rank;
} Col;

static float median_of(const float *src, int n) {
    float *a, m;
    int i;
    if (n < 1) return 0.f;
    a = malloc((size_t)n * sizeof(float));
    if (!a) return src[n / 2];
    for (i = 0; i < n; i++) a[i] = src[i];
    heapsort(a, n);
    m = a[n / 2];
    free(a);
    return m;
}

static int col_add(Col *c, int r, int y, float x, int height) {
    if (c->n >= c->cap) {
        int ncap = c->cap > 0 ? c->cap * 2 : 32;
        float *nx = realloc(c->xs, (size_t)ncap * sizeof(float));
        if (!nx) return 0;
        c->xs = nx;
        c->cap = ncap;
    }
    c->xs[c->n++] = x;
    c->last_x = x;
    c->last_r = r;
    c->alive = 1;
    if (c->n == 1 || abs(y - height / 2) < abs(c->mid_y - height / 2)) {
        c->mid_y = y;
        c->mid_x = x;
    }
    return 1;
}

/* Full-frame lattice. Returns 1 when hits are the measured surface.
 * The slider is the center column only. Chasing the median depth walks
 * onto a flatter sheet, so the period is taken once, at that center. */
static int lattice_fill(const openscan_calib *cal, const uint8_t *pattern, int width, int height,
                        int flipped, float hint_mm, float near_mm, float far_mm, const int *ys,
                        int nrows, Hit *hits, int *counts) {
    enum { COLS = 192, GATE = 7 };
    Peq ext[OS_PLANE_CAP];
    Col *cols;
    int *cid;
    int nplanes, nc, r, i, total, ns, ic;
    int surv[COLS], fold_of[COLS];
    float gap;
    double period, base, xu, yu;
    int best_pi;
    if (!cal || !pattern || !ys || !hits || !counts) return 0;
    if (width < 640 || height < 400 || nrows < 8) return 0;
    nplanes = build_extended(cal, ext, OS_PLANE_CAP);
    if (nplanes < 8) return 0;
    cols = calloc(COLS, sizeof(Col));
    cid = malloc((size_t)nrows * OS_STRIPE_MAX_PEAKS * sizeof(int));
    if (!cols || !cid) {
        free(cols);
        free(cid);
        return 0;
    }
    for (i = 0; i < nrows * OS_STRIPE_MAX_PEAKS; i++) cid[i] = -1;
    nc = 0;
    for (r = 0; r < nrows; r++) {
        int y = ys[r];
        const uint8_t *row = pattern + (size_t)y * width;
        int peaks[OS_STRIPE_MAX_PEAKS];
        double hp[OS_STRIPE_MAX_PEAKS];
        int np = lattice_peaks(row, width, peaks, hp, OS_STRIPE_MAX_PEAKS);
        int taken[COLS];
        Hit *h = hits + r * OS_STRIPE_MAX_PEAKS;
        np = merge_double_edges(peaks, hp, np);
        if (np < 12) {
            for (i = 0; i < nc; i++)
                if (cols[i].alive && cols[i].last_r < r && ys[r] - ys[cols[i].last_r] > 48)
                    cols[i].alive = 0;
            continue;
        }
        memset(taken, 0, sizeof taken);
        {
            /* Peaks and columns run left to right. A peak may not jump
             * back to a column a peak on its left already passed. */
            float floor_x = -1.0e9f;
            for (i = 0; i < np; i++) {
                int c, best = -1;
                double bd = GATE;
                h[i].x = peaks[i];
                h[i].ok = 0;
                for (c = 0; c < nc; c++) {
                    double d;
                    if (!cols[c].alive || taken[c]) continue;
                    if (cols[c].last_x + 0.5f < floor_x) continue;
                    d = dabs((double)peaks[i] - cols[c].last_x);
                    if (d < bd) {
                        bd = d;
                        best = c;
                    }
                }
                if (best < 0) continue;
                taken[best] = 1;
                floor_x = cols[best].last_x;
                col_add(&cols[best], r, y, (float)peaks[i], height);
                cid[r * OS_STRIPE_MAX_PEAKS + i] = best;
            }
        }
        for (i = 0; i < np; i++) {
            if (cid[r * OS_STRIPE_MAX_PEAKS + i] >= 0) continue;
            if (nc >= COLS) break;
            col_add(&cols[nc], r, y, (float)peaks[i], height);
            cid[r * OS_STRIPE_MAX_PEAKS + i] = nc;
            nc++;
        }
        for (i = 0; i < nc; i++)
            if (cols[i].alive && cols[i].last_r < r && ys[r] - ys[cols[i].last_r] > 48)
                cols[i].alive = 0;
    }
    ns = 0;
    for (i = 0; i < nc; i++) {
        fold_of[i] = -1;
        if (cols[i].n < 8) continue;
        cols[i].mx = median_of(cols[i].xs, cols[i].n);
        surv[ns++] = i;
    }
    for (i = 1; i < ns; i++) {
        int key = surv[i], j = i;
        while (j > 0 && cols[surv[j - 1]].mx > cols[key].mx) {
            surv[j] = surv[j - 1];
            j--;
        }
        surv[j] = key;
    }
    gap = 12.f;
    {
        float gbuf[COLS];
        int ng = 0;
        for (i = 1; i < ns; i++) {
            float dx = cols[surv[i]].mx - cols[surv[i - 1]].mx;
            if (dx > 0.5f) gbuf[ng++] = dx;
        }
        if (ng >= 8) {
            heapsort(gbuf, ng);
            gap = gbuf[ng / 2];
        }
    }
    {
        /* Two tracks closer than one column are the same stripe split.
         * Folding only a 4 px pair left those splits, and each one added a plane. */
        float near_dx = gap * 0.6f;
        int w = 0;
        if (near_dx < 4.f) near_dx = 4.f;
        if (near_dx > 8.f) near_dx = 8.f;
        for (i = 0; i < ns; i++) {
            int id = surv[i];
            if (w > 0 && cols[id].mx - cols[surv[w - 1]].mx <= near_dx) {
                Col *dst = &cols[surv[w - 1]];
                Col *src = &cols[id];
                int nn = dst->n + src->n;
                float *nx = realloc(dst->xs, (size_t)nn * sizeof(float));
                if (nx) {
                    dst->xs = nx;
                    memcpy(dst->xs + dst->n, src->xs, (size_t)src->n * sizeof(float));
                    dst->n = nn;
                    dst->cap = nn;
                    dst->mx = median_of(dst->xs, dst->n);
                    if (abs(src->mid_y - height / 2) < abs(dst->mid_y - height / 2)) {
                        dst->mid_y = src->mid_y;
                        dst->mid_x = src->mid_x;
                    }
                }
                free(src->xs);
                src->xs = NULL;
                src->n = 0;
                fold_of[id] = surv[w - 1];
                continue;
            }
            surv[w++] = id;
            fold_of[id] = id;
        }
        ns = w;
    }
    if (ns < 12) goto fail;
    {
        float gbuf[COLS];
        int ng = 0;
        for (i = 1; i < ns; i++) {
            float dx = cols[surv[i]].mx - cols[surv[i - 1]].mx;
            if (dx > 0.5f) gbuf[ng++] = dx;
        }
        if (ng < 8) goto fail;
        heapsort(gbuf, ng);
        gap = gbuf[ng / 2];
        if (gap < 9.f || gap > 20.f) goto fail;
    }
    ic = surv[ns / 2];
    {
        double u = cols[ic].mid_x, v = cols[ic].mid_y;
        double best_s = 1e300;
        best_pi = -1;
        if (flipped) {
            u = (width - 1) - u;
            v = (height - 1) - v;
        }
        if (!undistort_ok(&cal->cam[0], u, v, &xu, &yu)) goto fail;
        for (i = 0; i < nplanes; i++) {
            double z, X, Y, score;
            if (!plane_at(&ext[i], xu, yu, &z, &X, &Y)) continue;
            if (z < near_mm || z > far_mm) continue;
            score = dabs(z - hint_mm);
            if (score < best_s) {
                best_s = score;
                best_pi = i;
            }
        }
        if (best_pi < 0) goto fail;
        {
            double pitch = plane_pitch_px(&cal->cam[0], ext, nplanes, best_pi, xu, yu);
            if (!(pitch >= 2.0)) goto fail;
            period = (double)gap / pitch;
        }
        if (period < 1.2 || period > 4.0) goto fail;
        /* Rank is how many periods this column sits from the center.
         * A chain of steps counts a split stripe and then shifts the snout. */
        {
            float cx = cols[ic].mx;
            for (i = 0; i < ns; i++) {
                float t = (cols[surv[i]].mx - cx) / gap;
                cols[surv[i]].rank = (int)(t >= 0.f ? t + 0.5f : t - 0.5f);
            }
        }
        base = (double)best_pi - (double)cols[ic].rank * period;
    }
    total = 0;
    for (r = 0; r < nrows; r++) {
        Hit *h = hits + r * OS_STRIPE_MAX_PEAKS;
        int y = ys[r];
        counts[r] = 0;
        for (i = 0; i < OS_STRIPE_MAX_PEAKS; i++) {
            int id, rk, pi = 0;
            double u, v, z, X, Y, p;
            if (h[i].x < 0) continue;
            id = cid[r * OS_STRIPE_MAX_PEAKS + i];
            if (id < 0 || id >= nc) continue;
            id = fold_of[id];
            if (id < 0) continue;
            rk = cols[id].rank;
            u = h[i].x;
            v = y;
            if (flipped) {
                u = (width - 1) - u;
                v = (height - 1) - v;
            }
            if (!undistort_ok(&cal->cam[0], u, v, &xu, &yu)) continue;
            p = base + (double)rk * period;
            if (!blend_at(ext, nplanes, p, xu, yu, near_mm, far_mm, &pi, &z, &X, &Y)) continue;
            h[i].pi = pi;
            h[i].z = (float)z;
            h[i].X = (float)X;
            h[i].Y = (float)Y;
            h[i].tone = pattern[y * width + h[i].x] / 255.f;
            h[i].ok = 1;
            counts[r]++;
            total++;
        }
    }
    if (total < 200) goto fail;
    for (i = 0; i < nc; i++) free(cols[i].xs);
    free(cols);
    free(cid);
    return 1;
fail:
    for (i = 0; i < nc; i++) free(cols[i].xs);
    free(cols);
    free(cid);
    return 0;
}

/* Distance of one row. The columns are about two planes apart and a long run
 * fills the calibrated set, so the rightmost column is the last plane that
 * fits. Used only when the frame is too small for the lattice, or the lattice
 * is not there. The synthetic wall is that small frame. */
static int row_filled(const openscan_calib *cal, const Peq *peq, int nplanes,
                      const uint8_t *row, int width, int height, int y, int flipped,
                      float *med_out) {
    int peaks[OS_STRIPE_MAX_PEAKS];
    double hp[OS_STRIPE_MAX_PEAKS];
    int np, b = 0, best_b = 0, best_n = 0, i, count, stride, off, nz = 0;
    float zs[OS_STRIPE_MAX_PEAKS];
    double acc = 0;
    float med, std;
    np = lattice_peaks(row, width, peaks, hp, OS_STRIPE_MAX_PEAKS);
    if (np < 36) return 0;
    for (i = 1; i <= np; i++) {
        if (i < np && peaks[i] - peaks[i - 1] <= 20) continue;
        if (i - b > best_n) {
            best_b = b;
            best_n = i - b;
        }
        b = i;
    }
    if (best_n < 36) return 0;
    if (peaks[best_b + best_n - 1] - peaks[best_b] < width * 2 / 5) return 0;
    if (peaks[best_b + best_n - 1] < width / 2) return 0;
    stride = 2;
    count = best_n;
    while (count >= 36 && nplanes - stride * (count - 1) < 1) count--;
    off = nplanes - stride * (count - 1) - 1;
    if (off < 0) return 0;
    for (i = 0; i < count; i++) {
        double u = peaks[best_b + i], v = y, xu, yu, z, X, Y;
        int pi = off + stride * i;
        if (flipped) {
            u = (width - 1) - u;
            v = (height - 1) - v;
        }
        if (pi < 0 || pi >= nplanes) continue;
        if (!undistort_ok(&cal->cam[0], u, v, &xu, &yu)) continue;
        if (!plane_at(&peq[pi], xu, yu, &z, &X, &Y)) continue;
        if (z < 100.0 || z > 700.0) continue;
        zs[nz++] = (float)z;
    }
    if (nz < count * 9 / 10) return 0;
    heapsort(zs, nz);
    med = zs[nz / 2];
    if (!(med > 200.f && med < 450.f)) return 0;
    for (i = 0; i < nz; i++) {
        double d = zs[i] - med;
        acc += d * d;
    }
    std = (float)os_sqrt((float)(acc / nz));
    if (!(std < 42.f)) return 0;
    *med_out = med;
    return 1;
}

/* Distance of the projected columns. The slider is only a fallback.
 * The synthetic wall is a small frame and already knows its distance. */
static float ridges_past(const openscan_calib *cal, const Peq *peq, int nplanes,
                         const uint8_t *pattern, int width, int height, int flipped,
                         float hint, float far_lim) {
    float meds[48];
    int n = 0, y;
    (void)far_lim;
    if (!pattern || width < 640 || height < 400) return hint;
    for (y = 16; y < height - 16 && n < 48; y += 16) {
        float med = 0;
        if (!row_filled(cal, peq, nplanes, pattern + (size_t)y * width, width, height, y, flipped,
                        &med))
            continue;
        meds[n++] = med;
    }
    if (n < 3) return hint;
    heapsort(meds, n);
    return meds[n / 2];
}

/* Half the median gap between ridges on this row. A wider jump is the next stripe. */
static int row_lim(const Hit *h) {
    int gap[OS_STRIPE_MAX_PEAKS];
    int n = 0, i, prev = -1, a;
    for (i = 0; i < OS_STRIPE_MAX_PEAKS; i++) {
        if (h[i].x < 0) continue;
        if (prev >= 0) gap[n++] = h[i].x - prev;
        prev = h[i].x;
    }
    if (n < 1) return 3;
    for (a = 1; a < n; a++) {
        int v = gap[a], b = a;
        while (b > 0 && gap[b - 1] > v) {
            gap[b] = gap[b - 1];
            b--;
        }
        gap[b] = v;
    }
    a = gap[n / 2] / 2;
    if (a < 2) a = 2;
    if (a > 4) a = 4;
    return a;
}

/* Put this peak on one projector plane. have_ref rejects a jump off the stripe. */
static int place_on(const openscan_calib *cal, const Peq *peq, int nplanes,
                    Hit *h, int y, int width, int height, int flipped,
                    int pi, float near_mm, float far_mm, float zref, int have_ref) {
    double u, v, xu, yu, z, X, Y;
    if (h->x < 0 || pi < 0 || pi >= nplanes) return 0;
    u = h->x;
    v = y;
    if (flipped) {
        u = (width - 1) - u;
        v = (height - 1) - v;
    }
    if (!undistort_ok(&cal->cam[0], u, v, &xu, &yu)) return 0;
    if (!plane_at(&peq[pi], xu, yu, &z, &X, &Y)) return 0;
    if (z < near_mm || z > far_mm) return 0;
    if (have_ref && dabs(z - (double)zref) > 14.0) return 0;
    h->pi = pi;
    h->z = (float)z;
    h->X = (float)X;
    h->Y = (float)Y;
    h->ok = 1;
    return 1;
}

/* A column that jumps off a smooth row is the wrong plane. The neighbours
 * already name the stripe. Snap back when they agree and this hit does not. */
static void repair_streaks(const openscan_calib *cal, const Peq *peq, int nplanes,
                           Hit *hits, const int *ys, int nrows,
                           int width, int height, int flipped,
                           float near_mm, float far_mm) {
    int r, i;
    for (r = 0; r < nrows; r++) {
        Hit *h = hits + r * OS_STRIPE_MAX_PEAKS;
        for (i = 0; i < OS_STRIPE_MAX_PEAKS; i++) {
            int p = -1, n = -1, k, lo, hi, best, pi;
            double mid, best_dz, z, X, Y, xu, yu, u, v;
            if (!h[i].ok) continue;
            for (k = i - 1; k >= 0; k--)
                if (h[k].ok) {
                    p = k;
                    break;
                }
            for (k = i + 1; k < OS_STRIPE_MAX_PEAKS; k++)
                if (h[k].ok) {
                    n = k;
                    break;
                }
            if (p < 0 || n < 0) continue;
            if (h[i].x - h[p].x > 24 || h[n].x - h[i].x > 24) continue;
            if (dabs((double)h[p].z - (double)h[n].z) > 6.0) continue;
            mid = 0.5 * ((double)h[p].z + (double)h[n].z);
            if (dabs((double)h[i].z - mid) <= 12.0) continue;
            lo = h[p].pi < h[n].pi ? h[p].pi : h[n].pi;
            hi = h[p].pi > h[n].pi ? h[p].pi : h[n].pi;
            if (lo > 0) lo--;
            if (hi + 1 < nplanes) hi++;
            u = h[i].x;
            v = ys[r];
            if (flipped) {
                u = (width - 1) - u;
                v = (height - 1) - v;
            }
            if (!undistort_ok(&cal->cam[0], u, v, &xu, &yu)) continue;
            best = -1;
            best_dz = 1e300;
            for (pi = lo; pi <= hi; pi++) {
                double dz;
                if (!plane_at(&peq[pi], xu, yu, &z, &X, &Y)) continue;
                if (z < near_mm || z > far_mm) continue;
                dz = dabs(z - mid);
                if (dz >= best_dz) continue;
                best_dz = dz;
                best = pi;
            }
            if (best >= 0 && best_dz <= 6.0)
                place_on(cal, peq, nplanes, &h[i], ys[r], width, height, flipped, best, near_mm,
                         far_mm, (float)mid, 1);
        }
    }
}

/* One projector stripe keeps one plane down the image. Peaks bridge a row
 * that missed its lock, and the plane is the median of the locks on that stripe. */
static void continue_stripes(const openscan_calib *cal, const Peq *peq, int nplanes,
                             Hit *hits, const int *ys, int nrows,
                             int width, int height, int flipped,
                             float near_mm, float far_mm) {
    int maxn, r, i;
    int *parent, *child, *pis;
    if (nrows < 2 || !hits || !ys) return;
    maxn = nrows * OS_STRIPE_MAX_PEAKS;
    parent = malloc((size_t)maxn * sizeof(int));
    child = malloc((size_t)maxn * sizeof(int));
    pis = malloc((size_t)nrows * sizeof(int));
    if (!parent || !child || !pis) {
        free(parent);
        free(child);
        free(pis);
        return;
    }
    for (i = 0; i < maxn; i++) {
        parent[i] = -1;
        child[i] = -1;
    }
    for (r = 1; r < nrows; r++) {
        Hit *prev = hits + (r - 1) * OS_STRIPE_MAX_PEAKS;
        Hit *cur = hits + r * OS_STRIPE_MAX_PEAKS;
        int lim, taken_p[OS_STRIPE_MAX_PEAKS], taken_c[OS_STRIPE_MAX_PEAKS];
        lim = row_lim(prev);
        i = row_lim(cur);
        if (i < lim) lim = i;
        memset(taken_p, 0, sizeof taken_p);
        memset(taken_c, 0, sizeof taken_c);
        for (;;) {
            int best_d = 1000, bi = -1, bj = -1;
            for (i = 0; i < OS_STRIPE_MAX_PEAKS; i++) {
                int j;
                if (cur[i].x < 0 || taken_c[i]) continue;
                for (j = 0; j < OS_STRIPE_MAX_PEAKS; j++) {
                    int d;
                    if (prev[j].x < 0 || taken_p[j]) continue;
                    d = cur[i].x - prev[j].x;
                    if (d < 0) d = -d;
                    if (d <= lim && d < best_d) {
                        best_d = d;
                        bi = i;
                        bj = j;
                    }
                }
            }
            if (bi < 0) break;
            parent[r * OS_STRIPE_MAX_PEAKS + bi] = bj;
            child[(r - 1) * OS_STRIPE_MAX_PEAKS + bj] = bi;
            taken_c[bi] = 1;
            taken_p[bj] = 1;
        }
    }
    for (r = 0; r < nrows; r++) {
        Hit *row = hits + r * OS_STRIPE_MAX_PEAKS;
        for (i = 0; i < OS_STRIPE_MAX_PEAKS; i++) {
            int n = 0, rr, ii, a, med, have;
            float zref;
            if (row[i].x < 0) continue;
            if (parent[r * OS_STRIPE_MAX_PEAKS + i] >= 0) continue;
            rr = r;
            ii = i;
            while (n < nrows) {
                int c;
                if (hits[rr * OS_STRIPE_MAX_PEAKS + ii].ok)
                    pis[n++] = hits[rr * OS_STRIPE_MAX_PEAKS + ii].pi;
                c = child[rr * OS_STRIPE_MAX_PEAKS + ii];
                if (c < 0 || rr + 1 >= nrows) break;
                rr++;
                ii = c;
            }
            if (n < 1) continue;
            for (a = 1; a < n; a++) {
                int v = pis[a], b = a;
                while (b > 0 && pis[b - 1] > v) {
                    pis[b] = pis[b - 1];
                    b--;
                }
                pis[b] = v;
            }
            med = pis[n / 2];
            rr = r;
            ii = i;
            have = 0;
            zref = 0.f;
            for (;;) {
                int c;
                Hit *h = &hits[rr * OS_STRIPE_MAX_PEAKS + ii];
                if (h->ok && place_on(cal, peq, nplanes, h, ys[rr], width, height, flipped,
                                      med, near_mm, far_mm, zref, have)) {
                    zref = h->z;
                    have = 1;
                }
                c = child[rr * OS_STRIPE_MAX_PEAKS + ii];
                if (c < 0 || rr + 1 >= nrows) break;
                rr++;
                ii = c;
            }
        }
    }
    free(parent);
    free(child);
    free(pis);
}

/* One code step is sx raw projector pixels. Code pattern_w lies on the
 * projector principal point, so the line for code k is
 * offset = cx + pattern_w * sx - sx * k.
 * Slope is the factory line at that offset, extended past either end. */
static double db_slope(const double *off, const double *sl, int n, double o) {
    int i, k;
    double dsl, t;
    if (n < 2) return 0.0;
    if (o <= off[0]) {
        k = n >= 8 ? 7 : 1;
        dsl = (sl[k] - sl[0]) / (off[k] - off[0]);
        return sl[0] + (o - off[0]) * dsl;
    }
    if (o >= off[n - 1]) {
        k = n >= 8 ? n - 8 : 0;
        dsl = (sl[n - 1] - sl[k]) / (off[n - 1] - off[k]);
        return sl[n - 1] + (o - off[n - 1]) * dsl;
    }
    for (i = 1; i < n; i++) {
        if (o > off[i]) continue;
        t = (o - off[i - 1]) / (off[i] - off[i - 1]);
        return sl[i - 1] * (1.0 - t) + sl[i] * t;
    }
    return sl[n - 1];
}

static int db_slope_table(const openscan_calib *cal, double *off, double *sl, int *index) {
    int n, i, j;
    n = cal->nplanes;
    if (n < 4) return 0;
    if (n > OPENSCAN_MAX_PLANES) n = OPENSCAN_MAX_PLANES;
    for (i = 0; i < n; i++) {
        off[i] = cal->plane[i].offset;
        sl[i] = cal->plane[i].slope;
        index[i] = cal->plane[i].index;
    }
    for (i = 1; i < n; i++) {
        double oo = off[i], ss = sl[i];
        int ii = index[i];
        j = i;
        while (j > 0 && off[j - 1] > oo) {
            off[j] = off[j - 1];
            sl[j] = sl[j - 1];
            index[j] = index[j - 1];
            j--;
        }
        off[j] = oo;
        sl[j] = ss;
        index[j] = ii;
    }
    j = 1;
    for (i = 1; i < n; i++) {
        if (!(off[i] > off[j - 1] + 1e-3)) continue;
        off[j] = off[i];
        sl[j] = sl[i];
        index[j] = index[i];
        j++;
    }
    return j;
}

/* Peaks of the vertical carrier. Threshold 12 keeps a real column and
 * leaves the lattice gate (6, gap 9..20) untouched for the fallback. */
static int db_peaks(const uint8_t *row, int width, int *peaks, int cap) {
    int np = 0, last = -100, x;
    if (cap > OS_STRIPE_MAX_PEAKS) cap = OS_STRIPE_MAX_PEAKS;
    for (x = 8; x < width - 8 && np < cap; x++) {
        double hp = lattice_hp(row, width, x);
        if (hp < 12.0) continue;
        if (!(hp >= lattice_hp(row, width, x - 1) && hp > lattice_hp(row, width, x + 1) &&
              hp >= lattice_hp(row, width, x - 2) && hp > lattice_hp(row, width, x + 2)))
            continue;
        if (x - last < 8) {
            if (np > 0 && hp > lattice_hp(row, width, peaks[np - 1])) {
                peaks[np - 1] = x;
                last = x;
            }
            continue;
        }
        peaks[np++] = x;
        last = x;
    }
    return np;
}

enum { DB_CAP = 192 };

typedef struct {
    int *ys;
    float *xs;
    int n, cap;
    int last_y;
    float mx;
} DbCol;

static int db_add(DbCol *c, int y, float x) {
    if (c->n >= c->cap) {
        int ncap = c->cap > 0 ? c->cap * 2 : 16;
        int *ny = realloc(c->ys, (size_t)ncap * sizeof(int));
        float *nx;
        if (!ny) return 0;
        c->ys = ny;
        nx = realloc(c->xs, (size_t)ncap * sizeof(float));
        if (!nx) return 0;
        c->xs = nx;
        c->cap = ncap;
    }
    c->ys[c->n] = y;
    c->xs[c->n] = x;
    c->n++;
    c->last_y = y;
    return 1;
}

static void db_release(DbCol *cols, int nc) {
    int i;
    if (!cols) return;
    for (i = 0; i < nc; i++) {
        free(cols[i].ys);
        free(cols[i].xs);
    }
    free(cols);
}

static void db_sort(DbCol *c) {
    int i, j;
    if (c->n <= 1) return;
    for (i = 1; i < c->n; i++) {
        int y = c->ys[i];
        float x = c->xs[i];
        j = i;
        while (j > 0 && c->ys[j - 1] > y) {
            c->ys[j] = c->ys[j - 1];
            c->xs[j] = c->xs[j - 1];
            j--;
        }
        c->ys[j] = y;
        c->xs[j] = x;
    }
    j = 1;
    for (i = 1; i < c->n; i++) {
        if (c->ys[i] == c->ys[j - 1]) {
            c->xs[j - 1] = 0.5f * (c->xs[j - 1] + c->xs[i]);
            continue;
        }
        c->ys[j] = c->ys[i];
        c->xs[j] = c->xs[i];
        j++;
    }
    c->n = j;
}

static int db_x_at(const DbCol *c, int y, float *xo) {
    int lo, hi, mid;
    float t;
    if (c->n < 1 || y < c->ys[0] || y > c->ys[c->n - 1]) return 0;
    lo = 0;
    hi = c->n - 1;
    while (lo + 1 < hi) {
        mid = (lo + hi) >> 1;
        if (c->ys[mid] <= y) lo = mid;
        else hi = mid;
    }
    if (hi == lo || c->ys[hi] == c->ys[lo]) {
        *xo = c->xs[lo];
        return 1;
    }
    t = (float)(y - c->ys[lo]) / (float)(c->ys[hi] - c->ys[lo]);
    *xo = c->xs[lo] * (1.f - t) + c->xs[hi] * t;
    return 1;
}

static int db_pix(const uint8_t *img, int width, int height, const DbCol *c, int y) {
    float x;
    int xi;
    if (y < 0 || y >= height) return -1;
    if (!db_x_at(c, y, &x)) return -1;
    xi = (int)(x >= 0.f ? x + 0.5f : x - 0.5f);
    if (xi < 0) xi = 0;
    if (xi >= width) xi = width - 1;
    return img[(size_t)y * width + xi];
}

/* Integer period of the vertical brightness carrier. It grows when the
 * wall is closer, so the lag is measured on this frame. */
static int db_period(const uint8_t *img, int width, int height, DbCol *cols, const int *ord, int ns) {
    double acc[121];
    int cnt[121];
    float *sig;
    int i, lag, best, used;
    double best_s;
    memset(acc, 0, sizeof acc);
    memset(cnt, 0, sizeof cnt);
    sig = malloc((size_t)height * sizeof(float));
    if (!sig) return 0;
    used = 0;
    for (i = 0; i < ns && used < 32; i++) {
        DbCol *c = &cols[ord[i]];
        int y0, y1, y, len, n;
        double mean;
        if (c->n < 16) continue;
        y0 = c->ys[0];
        y1 = c->ys[c->n - 1];
        if (y1 - y0 < 80) continue;
        len = 0;
        mean = 0;
        for (y = y0; y <= y1; y++) {
            int g = db_pix(img, width, height, c, y);
            if (g < 0) continue;
            sig[len++] = (float)g;
            mean += g;
        }
        if (len < 80) continue;
        mean /= (double)len;
        for (n = 0; n < len; n++) sig[n] = (float)(sig[n] - mean);
        for (lag = 12; lag <= 120 && lag < len; lag++) {
            double dot = 0;
            int a;
            for (a = 0; a < len - lag; a++) dot += (double)sig[a] * (double)sig[a + lag];
            acc[lag] += dot;
            cnt[lag] += len - lag;
        }
        used++;
    }
    free(sig);
    if (used < 1) return 0;
    best = 0;
    best_s = -1e300;
    for (lag = 12; lag <= 120; lag++) {
        double s;
        if (!cnt[lag]) continue;
        s = acc[lag] / (double)cnt[lag];
        if (s > best_s) {
            best_s = s;
            best = lag;
        }
    }
    if (best < 12 || !(best_s > 0.0)) return 0;
    return best;
}

/* Symbol 0..5 is the phase of that carrier. 255 means this stripe had
 * too little column to measure; it still keeps its place in the order. */
static int db_symbol(const uint8_t *img, int width, int height, const DbCol *c, int period) {
    int y0, y1, y, cnt, sym;
    double sum, mean, ss, sc;
    float ph, cyc;
    if (c->n < 2) return 255;
    y0 = c->ys[0];
    y1 = c->ys[c->n - 1];
    if (y0 < 8) y0 = 8;
    if (y1 > height - 9) y1 = height - 9;
    if (y1 - y0 < period) return 255;
    sum = 0;
    cnt = 0;
    for (y = y0; y <= y1; y += 2) {
        int g = db_pix(img, width, height, c, y);
        if (g < 0) continue;
        sum += g;
        cnt++;
    }
    if (cnt < period / 2) return 255;
    mean = sum / (double)cnt;
    ss = 0;
    sc = 0;
    for (y = y0; y <= y1; y += 2) {
        int g = db_pix(img, width, height, c, y);
        double d;
        float ang;
        if (g < 0) continue;
        d = (double)g - mean;
        ang = 6.28318530718f * (float)y / (float)period;
        ss += d * (double)os_sin(ang);
        sc += d * (double)os_cos(ang);
    }
    ph = os_atan2((float)ss, (float)sc);
    cyc = ph / 6.28318530718f;
    if (cyc < 0.f) cyc += 1.f;
    sym = (int)(cyc * 6.f + 0.5f);
    if (sym >= 6) sym = 0;
    if (sym < 0) sym = 0;
    return sym;
}

static void db_perm(int idx, int out[6]) {
    int pool[6];
    int n, i, j, f, q;
    for (i = 0; i < 6; i++) pool[i] = i;
    n = 6;
    for (i = 0; i < 6; i++) {
        f = 1;
        for (j = 2; j < n; j++) f *= j;
        q = n <= 1 ? 0 : idx / f;
        if (n > 1) idx %= f;
        out[i] = pool[q];
        for (j = q; j < n - 1; j++) pool[j] = pool[j + 1];
        n--;
    }
}

/* The same word can sit in several places, so a short window is not a name.
 * The whole run is. A permutation absorbs the carrier's phase offset, and
 * the code index is this stripe's place in that run. */
static int db_align(const unsigned char *sym, int n, const uint8_t *seq, int ncode,
                    int *rev_out, int *off_out) {
    int best_h = -1, second_h = -1, best_rev = 0, best_off = 0, ties = 0;
    int rev, perm_i;
    if (n < 24 || n > ncode || n > DB_CAP) return 0;
    for (rev = 0; rev < 2; rev++) {
        for (perm_i = 0; perm_i < 720; perm_i++) {
            int perm[6];
            unsigned char mapped[DB_CAP];
            int i, lim;
            db_perm(perm_i, perm);
            for (i = 0; i < n; i++) {
                int s = rev ? sym[n - 1 - i] : sym[i];
                mapped[i] = s < 6 ? (unsigned char)perm[s] : 255;
            }
            lim = ncode - n;
            for (int off = 0; off <= lim; off++) {
                int h = 0;
                const uint8_t *w = seq + off;
                for (i = 0; i < n; i++) {
                    h += mapped[i] == w[i];
                    if (second_h >= 0 && h + (n - i - 1) <= second_h) break;
                }
                if (h > best_h) {
                    if (best_h >= 0 && (rev != best_rev || off != best_off)) second_h = best_h;
                    best_h = h;
                    best_rev = rev;
                    best_off = off;
                    ties = 1;
                } else if (rev == best_rev && off == best_off) {
                    continue;
                } else if (h == best_h) {
                    ties++;
                } else if (h > second_h) {
                    second_h = h;
                }
            }
        }
    }
    if (best_h * 100 < n * 60) return 0;
    if (ties != 1) return 0;
    if (second_h >= 0 && best_h - second_h < 8) return 0;
    *rev_out = best_rev;
    *off_out = best_off;
    return 1;
}

static int debruijn_fill(const openscan_calib *cal, const uint8_t *pattern, int width, int height,
                         float near_mm, float far_mm, const int *ys, int nrows, Hit *hits,
                         int *counts) {
    DbCol *cols;
    int ord[DB_CAP];
    unsigned char sym[DB_CAP];
    Peq pls[DB_CAP];
    double soff[OPENSCAN_MAX_PLANES], ssl[OPENSCAN_MAX_PLANES];
    int sindex[OPENSCAN_MAX_PLANES];
    double R[9], dof;
    const openscan_pinhole *proj;
    int nc, ns, y, i, nt, period, rev, off, total, seen[6], nuniq;
    if (!cal || !pattern || !ys || !hits || !counts) return 0;
    if (width < 640 || height < 400 || nrows < 8) return 0;
    if (cal->ncode < 32 || cal->ncode > OPENSCAN_MAX_CODE) return 0;
    if (!(cal->sx > 1.0) || cal->pattern_w < 8) return 0;
    proj = &cal->cam[2];
    if (!(proj->fx > 1.0) || !(proj->fy > 1.0)) return 0;
    memset(seen, 0, sizeof seen);
    nuniq = 0;
    for (i = 0; i < cal->ncode; i++) {
        int id = cal->code_id[i];
        if (id > 5) return 0;
        if (!seen[id]) {
            seen[id] = 1;
            nuniq++;
        }
    }
    if (nuniq != 6) return 0;
    nt = db_slope_table(cal, soff, ssl, sindex);
    if (nt < 4) return 0;
    if (sindex[nt - 1] == sindex[0]) return 0;
    dof = (soff[nt - 1] - soff[0]) / (double)(sindex[nt - 1] - sindex[0]);
    if (!(dof > 1.0 && dof < 30.0)) return 0;
    cols = calloc(DB_CAP, sizeof(DbCol));
    if (!cols) return 0;
    nc = 0;
    for (y = 32; y < height - 32; y += 12) {
        int peaks[OS_STRIPE_MAX_PEAKS];
        int assigned[OS_STRIPE_MAX_PEAKS];
        unsigned char taken[DB_CAP];
        int np = db_peaks(pattern + (size_t)y * width, width, peaks, OS_STRIPE_MAX_PEAKS);
        int p;
        memset(taken, 0, (size_t)nc);
        for (p = 0; p < np; p++) {
            int best = -1, bd = 7, c, d, lx;
            assigned[p] = -1;
            for (c = 0; c < nc; c++) {
                if (taken[c] || cols[c].n < 1) continue;
                if (y - cols[c].last_y > 48) continue;
                lx = (int)(cols[c].xs[cols[c].n - 1] + 0.5f);
                d = peaks[p] - lx;
                if (d < 0) d = -d;
                if (d < bd) {
                    bd = d;
                    best = c;
                }
            }
            if (best < 0) continue;
            taken[best] = 1;
            if (db_add(&cols[best], y, (float)peaks[p])) assigned[p] = best;
        }
        for (p = 0; p < np; p++) {
            if (assigned[p] >= 0) continue;
            if (nc >= DB_CAP) break;
            if (db_add(&cols[nc], y, (float)peaks[p])) nc++;
        }
    }
    ns = 0;
    for (i = 0; i < nc; i++) {
        if (cols[i].n < 8) continue;
        cols[i].mx = median_of(cols[i].xs, cols[i].n);
        ord[ns++] = i;
    }
    for (i = 1; i < ns; i++) {
        int key = ord[i], j = i;
        while (j > 0 && cols[ord[j - 1]].mx > cols[key].mx) {
            ord[j] = ord[j - 1];
            j--;
        }
        ord[j] = key;
    }
    {
        int w = 0;
        for (i = 0; i < ns; i++) {
            int id = ord[i];
            DbCol *src, *dst;
            int nn;
            int *ny;
            float *nx;
            if (w > 0 && cols[id].mx - cols[ord[w - 1]].mx <= 6.f) {
                dst = &cols[ord[w - 1]];
                src = &cols[id];
                nn = dst->n + src->n;
                ny = realloc(dst->ys, (size_t)nn * sizeof(int));
                if (!ny) goto fail;
                dst->ys = ny;
                nx = realloc(dst->xs, (size_t)nn * sizeof(float));
                if (!nx) goto fail;
                dst->xs = nx;
                memcpy(dst->ys + dst->n, src->ys, (size_t)src->n * sizeof(int));
                memcpy(dst->xs + dst->n, src->xs, (size_t)src->n * sizeof(float));
                dst->n = nn;
                dst->cap = nn;
                dst->mx = median_of(dst->xs, dst->n);
                free(src->ys);
                free(src->xs);
                src->ys = NULL;
                src->xs = NULL;
                src->n = 0;
                continue;
            }
            ord[w++] = id;
        }
        ns = w;
    }
    if (ns < 24) goto fail;
    for (i = 0; i < ns; i++) db_sort(&cols[ord[i]]);
    period = db_period(pattern, width, height, cols, ord, ns);
    if (period < 12) goto fail;
    for (i = 0; i < ns; i++)
        sym[i] = (unsigned char)db_symbol(pattern, width, height, &cols[ord[i]], period);
    if (!db_align(sym, ns, cal->code_id, cal->ncode, &rev, &off)) goto fail;
    rodriques(proj->rvec, R);
    for (i = 0; i < ns; i++) {
        int code = rev ? off + (ns - 1 - i) : off + i;
        double offset, slope;
        int slot;
        if (code < 0 || code >= cal->ncode) goto fail;
        offset = proj->cx + (double)cal->pattern_w * cal->sx - cal->sx * (double)code;
        slope = db_slope(soff, ssl, nt, offset);
        slot = sindex[0] + os_lround((float)((offset - soff[0]) / dof));
        fill_peq(proj, R, slope, offset, slot, &pls[i]);
    }
    total = 0;
    for (i = 0; i < nrows; i++) {
        Hit *h = hits + i * OS_STRIPE_MAX_PEAKS;
        int row_y = ys[i];
        int k = 0, s;
        counts[i] = 0;
        for (s = 0; s < OS_STRIPE_MAX_PEAKS; s++) {
            h[s].ok = 0;
            h[s].x = -1;
        }
        for (s = 0; s < ns && k < OS_STRIPE_MAX_PEAKS; s++) {
            float xf;
            int xi;
            double xu, yu, z, X, Y;
            if (!db_x_at(&cols[ord[s]], row_y, &xf)) continue;
            xi = (int)(xf >= 0.f ? xf + 0.5f : xf - 0.5f);
            if (xi < 0 || xi >= width) continue;
            if (!undistort_ok(&cal->cam[0], xi, row_y, &xu, &yu)) continue;
            if (!plane_at(&pls[s], xu, yu, &z, &X, &Y)) continue;
            if (z < near_mm || z > far_mm) continue;
            h[k].x = xi;
            h[k].pi = pls[s].index;
            h[k].z = (float)z;
            h[k].X = (float)X;
            h[k].Y = (float)Y;
            h[k].tone = pattern[(size_t)row_y * width + xi] / 255.f;
            h[k].ok = 1;
            k++;
            total++;
        }
        counts[i] = k;
    }
    if (total < 200) {
        for (i = 0; i < nrows; i++) {
            Hit *h = hits + i * OS_STRIPE_MAX_PEAKS;
            int s;
            counts[i] = 0;
            for (s = 0; s < OS_STRIPE_MAX_PEAKS; s++) {
                h[s].ok = 0;
                h[s].x = -1;
            }
        }
        goto fail;
    }
    db_release(cols, nc);
    return 1;
fail:
    db_release(cols, nc);
    return 0;
}

void os_stripe_free(os_stripe_cloud *cloud) {
    if (!cloud) return;
    free(cloud->pt);
    cloud->pt = NULL;
    cloud->rows = cloud->cols = cloud->n = 0;
    cloud->median_z = 0;
}

int os_stripe_make(const openscan_calib *cal,
                   const uint8_t *pattern, const uint8_t *clean,
                   int width, int height, int flipped,
                   float hint_mm, float near_mm, float far_mm, int stride,
                   os_stripe_cloud *out) {
    /* Peak finding keeps the left edge of a ridge. A 180° buffer reverses
     * that choice, and the plane walk then drifts. Measure the calibration
     * pose and put the pixels back in the caller's orientation. */
    if (flipped) {
        size_t n;
        uint8_t *pat, *cln;
        int rc, i, npt;
        if (!out || !cal || !pattern || width < 16 || height < 16) return -1;
        n = (size_t)width * (size_t)height;
        pat = malloc(n);
        cln = clean ? malloc(n) : NULL;
        if (!pat || (clean && !cln)) {
            free(pat);
            free(cln);
            memset(out, 0, sizeof *out);
            return -1;
        }
        for (i = 0; i < width * height; i++) {
            pat[i] = pattern[n - 1 - i];
            if (cln) cln[i] = clean[n - 1 - i];
        }
        rc = os_stripe_make(cal, pat, cln, width, height, 0, hint_mm, near_mm, far_mm, stride, out);
        free(pat);
        free(cln);
        if (rc == 0 && out->pt) {
            npt = out->rows * out->cols;
            for (i = 0; i < npt; i++) {
                if (!out->pt[i].ok) continue;
                out->pt[i].u = (width - 1) - out->pt[i].u;
                out->pt[i].v = (height - 1) - out->pt[i].v;
            }
        }
        return rc;
    }
    Peq peq[OPENSCAN_MAX_PLANES];
    int nplanes = 0;
    int step, nrows, r, flat;
    int *ys;
    Hit *hits;
    int *counts;
    int cols, total, decoded;
    float *zs;
    if (out) memset(out, 0, sizeof *out);
    if (!out || !cal || !pattern || width < 16 || height < 16) return -1;
    if (!(cal->cam[0].fx > 1.0) || !(cal->cam[0].fy > 1.0)) return -1;
    if (near_mm < 1.f) near_mm = 1.f;
    if (far_mm < near_mm + 1.f) far_mm = near_mm + 1.f;
    if (!prepare_planes(cal, peq, &nplanes)) return -1;
    step = stride < 1 ? 2 : stride * 2;
    if (step < 2) step = 2;
    if ((height + step - 1) / step > OS_STRIPE_MAX_ROWS) {
        step = (height + OS_STRIPE_MAX_ROWS - 1) / OS_STRIPE_MAX_ROWS;
        if (step < 2) step = 2;
    }
    ys = malloc((size_t)OS_STRIPE_MAX_ROWS * sizeof(int));
    hits = calloc((size_t)OS_STRIPE_MAX_ROWS * OS_STRIPE_MAX_PEAKS, sizeof(Hit));
    counts = calloc((size_t)OS_STRIPE_MAX_ROWS, sizeof(int));
    if (!ys || !hits || !counts) {
        free(ys);
        free(hits);
        free(counts);
        return -1;
    }
    nrows = 0;
    for (int y = 0; y < height && nrows < OS_STRIPE_MAX_ROWS; y += step) ys[nrows++] = y;
    flat = clean_flat(clean, width, height);
    for (r = 0; r < nrows * OS_STRIPE_MAX_PEAKS; r++) hits[r].x = -1;
    /* The code names each stripe. The lattice remains when that match fails.
     * A full frame that failed both is noise around the distance hint.
     * Emitting it paints a sheet at the slider depth. The small wall has no
     * lattice, so it keeps the plane walk. */
    decoded = 0;
    if (cal->ncode >= 32 && cal->sx > 1.0 && cal->pattern_w >= 8)
        decoded = debruijn_fill(cal, pattern, width, height, near_mm, far_mm, ys, nrows, hits,
                                counts);
    if (!decoded && !lattice_fill(cal, pattern, width, height, flipped, hint_mm, near_mm, far_mm,
                                  ys, nrows, hits, counts)) {
        if (width >= 640 && height >= 400) {
            free(ys);
            free(hits);
            free(counts);
            return -1;
        }
        float seen = ridges_past(cal, peq, nplanes, pattern, width, height, flipped, hint_mm, far_mm);
        if (seen > near_mm + 40.f && seen < far_mm - 5.f && dabs(seen - hint_mm) > 20.0) {
            hint_mm = seen;
            if (near_mm > seen - 80.f) near_mm = seen - 80.f;
            if (far_mm < seen + 80.f) far_mm = seen + 80.f;
        }
        memset(hits, 0, (size_t)nrows * OS_STRIPE_MAX_PEAKS * sizeof(Hit));
        memset(counts, 0, (size_t)nrows * sizeof(int));
        for (r = 0; r < nrows * OS_STRIPE_MAX_PEAKS; r++) hits[r].x = -1;
    for (r = 0; r < nrows; r++) {
        int y = ys[r];
        const uint8_t *row = pattern + y * width;
        int peaks[OS_STRIPE_MAX_PEAKS];
        double peak_hp[OS_STRIPE_MAX_PEAKS];
        int np = 0;
        Hit *h = hits + r * OS_STRIPE_MAX_PEAKS;
        int anchor, i;
        double px_per;
        np = find_columns(row, width, peaks, peak_hp, OS_STRIPE_MAX_PEAKS);
        np = merge_double_edges(peaks, peak_hp, np);
        if (np < 1) continue;
        for (i = 0; i < np; i++) h[i].x = peaks[i];
        px_per = 6.0;
        anchor = np / 2;
        {
            int med = peaks[np / 2];
            int best = 0, bd = width;
            for (i = 0; i < np; i++) {
                int d = peaks[i] - med;
                if (d < 0) d = -d;
                if (d < bd) {
                    bd = d;
                    best = i;
                }
            }
            anchor = best;
        }
        {
            double u = peaks[anchor];
            double v = y;
            double xu, yu, best_s = 1e300;
            int best_pi = -1;
            int in_band = 0;
            double bz = 0, bX = 0, bY = 0;
            if (flipped) {
                u = (width - 1) - u;
                v = (height - 1) - v;
            }
            if (!undistort_ok(&cal->cam[0], u, v, &xu, &yu)) continue;
            if (r > 0) {
                Hit *prev = hits + (r - 1) * OS_STRIPE_MAX_PEAKS;
                int best = -1, bd = 4, nprev = 0;
                for (i = 0; i < OS_STRIPE_MAX_PEAKS; i++)
                    if (prev[i].ok) nprev++;
                if (nprev >= 12) {
                    for (i = 0; i < OS_STRIPE_MAX_PEAKS; i++) {
                        int d;
                        if (!prev[i].ok) continue;
                        d = peaks[anchor] - prev[i].x;
                        if (d < 0) d = -d;
                        if (d < bd) {
                            bd = d;
                            best = i;
                        }
                    }
                }
                if (best >= 0 && plane_at(&peq[prev[best].pi], xu, yu, &bz, &bX, &bY) &&
                    bz >= near_mm && bz <= far_mm)
                    best_pi = prev[best].pi;
            }
            if (best_pi < 0) {
            for (i = 0; i < nplanes; i++) {
                double z, X, Y;
                if (!plane_at(&peq[i], xu, yu, &z, &X, &Y)) continue;
                if (z < near_mm || z > far_mm) continue;
                if (dabs(z - hint_mm) <= 40.0) in_band = 1;
            }
            for (i = 0; i < nplanes; i++) {
                double z, X, Y, score, cu, cv;
                if (!plane_at(&peq[i], xu, yu, &z, &X, &Y)) continue;
                if (z < near_mm || z > far_mm) continue;
                score = dabs(z - hint_mm);
                if (in_band && score > 40.0) continue;
                if (!flat && clean && project_clean(&cal->cam[1], z * xu, z * yu, z, &cu, &cv)) {
                    if (clean_ridge(clean, width, height, flipped, cu, cv) > 12.0) score -= 3.0;
                }
                if (score < best_s) {
                    best_s = score;
                    best_pi = i;
                    bz = z;
                    bX = X;
                    bY = Y;
                }
            }
            }
            if (best_pi < 0) continue;
            h[anchor].x = peaks[anchor];
            h[anchor].pi = best_pi;
            h[anchor].z = (float)bz;
            h[anchor].X = (float)bX;
            h[anchor].Y = (float)bY;
            h[anchor].tone = row[peaks[anchor]] / 255.f;
            h[anchor].ok = 1;
            {
                double au = peaks[anchor], av = y, axu, ayu;
                if (flipped) {
                    au = (width - 1) - au;
                    av = (height - 1) - av;
                }
                if (undistort_ok(&cal->cam[0], au, av, &axu, &ayu))
                    px_per = plane_pitch_px(&cal->cam[0], peq, nplanes, best_pi, axu, ayu);
            }
        }
        walk_peaks(h, peaks, np, row, anchor, +1, (double)h[anchor].pi, px_per, cal, peq, nplanes,
                   y, width, height, flipped, near_mm, far_mm, h[anchor].z);
        walk_peaks(h, peaks, np, row, anchor, -1, (double)h[anchor].pi, px_per, cal, peq, nplanes,
                   y, width, height, flipped, near_mm, far_mm, h[anchor].z);
        /* A gap ends that segment. The next ridge is still the toy, so it
         * starts again from the distance hint instead of being discarded. */
        for (i = 0; i < np; i++) {
            if (h[i].ok) continue;
            {
                int src = -1, k, seeded = 0;
                for (k = i - 1; k >= 0; k--)
                    if (h[k].ok) { src = k; break; }
                if (src < 0)
                    for (k = i + 1; k < np; k++)
                        if (h[k].ok) { src = k; break; }
                if (src >= 0) {
                    double su, sv, sxu, syu, sz, sX, sY, spf;
                    int spi, sgap, sdir;
                    sgap = peaks[i] - peaks[src];
                    if (sgap < 0) sgap = -sgap;
                    sdir = peaks[i] >= peaks[src] ? (flipped ? -1 : 1) : (flipped ? 1 : -1);
                    su = peaks[i];
                    sv = y;
                    if (flipped) {
                        su = (width - 1) - su;
                        sv = (height - 1) - sv;
                    }
                    if (undistort_ok(&cal->cam[0], su, sv, &sxu, &syu) &&
                        step_along(peq, nplanes, (double)h[src].pi, sdir, sgap, px_per, h[src].z,
                                   sxu, syu, near_mm, far_mm, &spf, &spi, &sz, &sX, &sY)) {
                        h[i].x = peaks[i];
                        h[i].pi = spi;
                        h[i].z = (float)sz;
                        h[i].X = (float)sX;
                        h[i].Y = (float)sY;
                        h[i].tone = row[peaks[i]] / 255.f;
                        h[i].ok = 1;
                        seeded = 1;
                    }
                }
                if (!seeded &&
                    !lock_hit(cal, peq, nplanes, clean, width, height, flipped, flat,
                              hint_mm, near_mm, far_mm, peaks[i], y, row[peaks[i]] / 255.f, &h[i]))
                    continue;
            }
            walk_peaks(h, peaks, np, row, i, +1, (double)h[i].pi, px_per, cal, peq, nplanes, y,
                       width, height, flipped, near_mm, far_mm, h[i].z);
        }
        for (i = 1; i < np - 1; i++) {
            float a, b, m;
            if (!h[i].ok || !h[i - 1].ok || !h[i + 1].ok) continue;
            if (h[i].pi != h[i - 1].pi || h[i].pi != h[i + 1].pi) continue;
            if (os_fabs(h[i - 1].z - h[i].z) > 14.f || os_fabs(h[i + 1].z - h[i].z) > 14.f)
                continue;
            a = h[i - 1].z;
            b = h[i].z;
            m = h[i + 1].z;
            if (a > b) {
                float t = a;
                a = b;
                b = t;
            }
            if (m < a) h[i].z = a;
            else if (m > b) h[i].z = b;
            else h[i].z = m;
        }
        counts[r] = 0;
        for (i = 0; i < np; i++)
            if (h[i].ok) counts[r]++;
    }
    continue_stripes(cal, peq, nplanes, hits, ys, nrows, width, height, flipped, near_mm, far_mm);
    repair_streaks(cal, peq, nplanes, hits, ys, nrows, width, height, flipped, near_mm, far_mm);
    }
    cols = 1;
    for (r = 0; r < nrows; r++)
        if (counts[r] > cols) cols = counts[r];
    if (cols > OS_STRIPE_MAX_PEAKS) cols = OS_STRIPE_MAX_PEAKS;
    out->pt = calloc((size_t)nrows * (size_t)cols, sizeof(os_stripe_pt));
    if (!out->pt) {
        free(ys);
        free(hits);
        free(counts);
        return -1;
    }
    out->rows = nrows;
    out->cols = cols;
    total = 0;
    for (r = 0; r < nrows; r++) {
        Hit *h = hits + r * OS_STRIPE_MAX_PEAKS;
        int k = 0;
        int y = ys[r];
        for (int i = 0; i < OS_STRIPE_MAX_PEAKS && k < cols; i++) {
            os_stripe_pt *p;
            double u, v, xu, yu;
            if (!h[i].ok) continue;
            p = &out->pt[r * cols + k];
            u = h[i].x;
            v = y;
            if (flipped) {
                u = (width - 1) - u;
                v = (height - 1) - v;
            }
            p->x = h[i].X;
            p->y = h[i].Y;
            p->z = h[i].z;
            p->tone = h[i].tone;
            p->plane = h[i].pi;
            p->ok = 1;
            p->u = h[i].x;
            p->v = y;
            /* Median filter slides the sample along its ray. */
            if (undistort_ok(&cal->cam[0], u, v, &xu, &yu)) {
                double cu, cv;
                int ix, iy;
                p->x = (float)(h[i].z * xu);
                p->y = (float)(-h[i].z * yu);
                if (clean && project_clean(&cal->cam[1], p->x, -p->y, p->z, &cu, &cv)) {
                    ix = (int)(cu >= 0 ? cu + 0.5 : cu - 0.5);
                    iy = (int)(cv >= 0 ? cv + 0.5 : cv - 0.5);
                    if (flipped) {
                        ix = (width - 1) - ix;
                        iy = (height - 1) - iy;
                    }
                    if (ix >= 0 && iy >= 0 && ix < width && iy < height)
                        p->tone = clean[iy * width + ix] / 255.f;
                }
            }
            k++;
            total++;
        }
    }
    out->n = total;
    free(ys);
    free(hits);
    free(counts);
    drop_specks(out);
    smooth_rays(out);
    keep_band(out);
    adopt_plush(out, cal, pattern, clean, width, height);
    if (out->n > 0) {
        zs = malloc((size_t)out->n * sizeof(float));
        if (zs) {
            int n = 0;
            for (int i = 0; i < out->rows * out->cols; i++)
                if (out->pt[i].ok) zs[n++] = out->pt[i].z;
            heapsort(zs, n);
            out->median_z = zs[n / 2];
            free(zs);
        }
    }
    return 0;
}

static void flip_buf(uint8_t *p, int n) {
    for (int i = 0; i < n / 2; i++) {
        uint8_t t = p[i];
        p[i] = p[n - 1 - i];
        p[n - 1 - i] = t;
    }
}

static void synth_rig(openscan_calib *cal) {
    memset(cal, 0, sizeof *cal);
    cal->width = 160;
    cal->height = 120;
    cal->cam[0].fx = cal->cam[0].fy = 400;
    cal->cam[0].cx = 80;
    cal->cam[0].cy = 60;
    cal->cam[1] = cal->cam[0];
    cal->cam[2] = cal->cam[0];
    cal->cam[2].tvec[0] = 30;
    cal->nplanes = 40;
    for (int i = 0; i < 40; i++) {
        cal->plane[i].index = i;
        cal->plane[i].slope = 0;
        cal->plane[i].offset = 10 + i * 3;
    }
}

/* Paint the projector stripe wherever that plane meets a flat wall. */
void os_stripe_paint_wall(uint8_t *img, const openscan_calib *cal, float wall_z) {
    double zwall = wall_z;
    Peq peq[OPENSCAN_MAX_PLANES];
    int n = 0;
    int w = cal->width, h = cal->height;
    memset(img, 30, (size_t)w * h);
    if (!prepare_planes(cal, peq, &n)) return;
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            double xu, yu;
            undistort(&cal->cam[0], x, y, &xu, &yu);
            for (int i = 0; i < n; i++) {
                double z, X, Y;
                if (!plane_at(&peq[i], xu, yu, &z, &X, &Y)) continue;
                if (dabs(z - zwall) < 1.5) {
                    img[y * w + x] = 220;
                    break;
                }
            }
        }
    }
}

static int cloud_stats(const os_stripe_cloud *c, float *mean_x, int *n_close, int *plo, int *phi) {
    double sx = 0;
    int n = 0, close = 0, lo = 1000, hi = -1;
    for (int i = 0; i < c->rows * c->cols; i++) {
        if (!c->pt[i].ok) continue;
        sx += c->pt[i].x;
        if (c->pt[i].plane < lo) lo = c->pt[i].plane;
        if (c->pt[i].plane > hi) hi = c->pt[i].plane;
        if (os_fabs(c->pt[i].z - 180.f) < 3.f) close++;
        n++;
    }
    if (mean_x) *mean_x = n ? (float)(sx / n) : 0;
    if (n_close) *n_close = close;
    if (plo) *plo = lo;
    if (phi) *phi = hi;
    return n;
}

int os_stripe_self_test(void) {
    double R[9], o[3], rvec[3];
    openscan_pinhole barrel;
    openscan_calib cal;
    Peq peq[OPENSCAN_MAX_PLANES];
    int nplanes = 0;
    double xu, yu, z = 0, X = 0, Y = 0;
    uint8_t *img, *clean, *turned;
    os_stripe_cloud cloud;
    float mean_x = 0;
    int close = 0, lo = 0, hi = 0, n;
    rvec[0] = 0;
    rvec[1] = 0.2;
    rvec[2] = 0;
    rodriques(rvec, R);
    mul_rot(R, 1, 0, 0, o);
    if (dabs(o[0] - 0.9800666) > 2e-4 || dabs(o[1]) > 2e-4 || dabs(o[2] + 0.1986693) > 2e-4) {
        fprintf(stderr, "c stripe: rotation %.5f %.5f %.5f\n", o[0], o[1], o[2]);
        return 2;
    }
    memset(&barrel, 0, sizeof barrel);
    barrel.fx = barrel.fy = 400;
    barrel.cx = 80;
    barrel.cy = 60;
    undistort(&barrel, 120, 60, &xu, &yu);
    if (dabs(xu - 0.1) > 1e-6 || dabs(yu) > 1e-6) return 3;
    barrel.k1 = -0.3;
    undistort(&barrel, 120, 60, &xu, &yu);
    if (!(xu > 0.1)) return 3;

    synth_rig(&cal);
    if (!prepare_planes(&cal, peq, &nplanes)) return 4;
    undistort(&cal.cam[0], 33, 60, &xu, &yu);
    if (!plane_at(&peq[30], xu, yu, &z, &X, &Y) || dabs(z - 179.104477) > 0.05) {
        fprintf(stderr, "c stripe: ray z %.3f\n", z);
        return 4;
    }

    img = calloc(160 * 120, 1);
    clean = calloc(160 * 120, 1);
    turned = calloc(160 * 120, 1);
    if (!img || !clean || !turned) return 1;
    memset(clean, 40, 160 * 120);
    os_stripe_paint_wall(img, &cal, 180.f);
    memset(&cloud, 0, sizeof cloud);
    if (os_stripe_make(&cal, img, clean, 160, 120, 0, 180.f, 80.f, 500.f, 1, &cloud) != 0) return 5;
    n = cloud_stats(&cloud, &mean_x, &close, &lo, &hi);
    if (n < 400) {
        fprintf(stderr, "c stripe: %d pts median %.2f\n", n, cloud.median_z);
        os_stripe_free(&cloud);
        return 5;
    }
    if (os_fabs(cloud.median_z - 179.1f) > 1.5f) {
        fprintf(stderr, "c stripe: median %.2f\n", cloud.median_z);
        os_stripe_free(&cloud);
        return 6;
    }
    if (close * 10 < n * 8) {
        fprintf(stderr, "c stripe: %d/%d near the wall\n", close, n);
        os_stripe_free(&cloud);
        return 7;
    }
    if (!(mean_x < -5.f) || lo > 24 || hi < 35) {
        fprintf(stderr, "c stripe: mean x %.1f planes %d..%d\n", mean_x, lo, hi);
        os_stripe_free(&cloud);
        return 8;
    }
    os_stripe_free(&cloud);
    memcpy(turned, img, 160 * 120);
    flip_buf(turned, 160 * 120);
    if (os_stripe_make(&cal, turned, clean, 160, 120, 1, 180.f, 80.f, 500.f, 1, &cloud) != 0) return 9;
    n = cloud_stats(&cloud, &mean_x, &close, &lo, &hi);
    if (n < 400 || os_fabs(cloud.median_z - 179.1f) > 1.5f || !(mean_x < -5.f)) {
        fprintf(stderr, "c stripe: flip %d pts median %.2f x %.1f\n", n, cloud.median_z, mean_x);
        os_stripe_free(&cloud);
        return 9;
    }
    fprintf(stderr, "c stripe: %d pts median %.1f mm\n", n, cloud.median_z);
    os_stripe_free(&cloud);
    free(img);
    free(clean);
    free(turned);
    return 0;
}
