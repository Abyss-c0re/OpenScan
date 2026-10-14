/* Cloud mode.
 * The measured points are grouped into shapes. Each shape is written into
 * a 3D matrix so a new view can find the object without a silhouette prior.
 * A location molds only after a second view lands on the same cell. */
#include "os_cloud.h"
#include "os_logic.h"
#include "openscan/openscan_live.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define OS_CLOUD_MAX 32768
#define OS_CLOUD_HASH 65536
#define OS_CLOUD_SHAPES 36
#define OS_VOXEL 2.f
#define OS_CELL 14.f
#define OS_CONFIRM 2
#define OS_MAT_N 32
#define OS_MAT_CELL 14.f
#define OS_MAT_CAP 4

typedef struct {
    float x, y, z;
    short nobs;
    short shape;
    unsigned short seen;
    int ix, iy, iz;
} Pt;

typedef struct {
    float c[3];
    float n[3];
    float u[3];
    float v[3];
    float rad;
    int count;
} Shape;

struct os_cloud {
    Pt *pt;
    int *slot;
    int n;
    int nshape;
    Shape shape[OS_CLOUD_SHAPES];
    float axis_x, axis_z;
    int have_axis;
    int cover;
    int degrees;
    unsigned short frame_gen;
    /* Cell (x, y, z) holds a short array of shape ids. Lookup is the pose. */
    unsigned char *mat;
};

typedef struct {
    float x, y, z;
    int ix, iy, iz;
    int bin;
    int shape;
} Sample;

typedef struct {
    int ix, iy, iz;
    int n;
    float sx, sy, sz;
} Bin;

static unsigned mix_hash(int x, int y, int z) {
    return (unsigned)x * 73856093u ^ (unsigned)y * 19349663u ^ (unsigned)z * 83492791u;
}

static void rot_y(float yaw, float ax, float az, float *x, float *z) {
    float dx = *x - ax, dz = *z - az;
    float c = os_cos(yaw), s = os_sin(yaw);
    *x = ax + c * dx + s * dz;
    *z = az - s * dx + c * dz;
}

static void normalize3(float v[3]) {
    float len = os_sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (len < 1e-8f) {
        v[0] = 0.f;
        v[1] = 1.f;
        v[2] = 0.f;
        return;
    }
    v[0] /= len;
    v[1] /= len;
    v[2] /= len;
}

static void basis_of(const float n[3], float u[3], float v[3]) {
    float ax = 0.f, ay = 1.f, az = 0.f;
    if (os_fabs(n[1]) > 0.85f) {
        ax = 1.f;
        ay = 0.f;
        az = 0.f;
    }
    u[0] = n[1] * az - n[2] * ay;
    u[1] = n[2] * ax - n[0] * az;
    u[2] = n[0] * ay - n[1] * ax;
    normalize3(u);
    v[0] = n[1] * u[2] - n[2] * u[1];
    v[1] = n[2] * u[0] - n[0] * u[2];
    v[2] = n[0] * u[1] - n[1] * u[0];
    normalize3(v);
}

static void normal_of(const float C[9], float n[3]) {
    float tr = C[0] + C[4] + C[8] + 1e-3f;
    float S[9];
    float v[3];
    int i;
    S[0] = tr - C[0];
    S[1] = -C[1];
    S[2] = -C[2];
    S[3] = -C[3];
    S[4] = tr - C[4];
    S[5] = -C[5];
    S[6] = -C[6];
    S[7] = -C[7];
    S[8] = tr - C[8];
    v[0] = 0.31f;
    v[1] = 0.72f;
    v[2] = 0.41f;
    for (i = 0; i < 10; i++) {
        float o0 = S[0] * v[0] + S[1] * v[1] + S[2] * v[2];
        float o1 = S[3] * v[0] + S[4] * v[1] + S[5] * v[2];
        float o2 = S[6] * v[0] + S[7] * v[1] + S[8] * v[2];
        float len = os_sqrt(o0 * o0 + o1 * o1 + o2 * o2);
        if (len < 1e-8f) break;
        v[0] = o0 / len;
        v[1] = o1 / len;
        v[2] = o2 / len;
    }
    n[0] = v[0];
    n[1] = v[1];
    n[2] = v[2];
    normalize3(n);
}

os_cloud *os_cloud_create(void) {
    os_cloud *c = calloc(1, sizeof *c);
    if (!c) return NULL;
    c->pt = calloc(OS_CLOUD_MAX, sizeof(Pt));
    c->slot = calloc(OS_CLOUD_HASH, sizeof(int));
    c->mat = calloc((size_t)OS_MAT_N * OS_MAT_N * OS_MAT_N * (1 + OS_MAT_CAP), 1);
    if (!c->pt || !c->slot || !c->mat) {
        os_cloud_free(c);
        return NULL;
    }
    return c;
}

void os_cloud_free(os_cloud *c) {
    if (!c) return;
    free(c->pt);
    free(c->slot);
    free(c->mat);
    free(c);
}

void os_cloud_reset(os_cloud *c) {
    if (!c) return;
    c->n = 0;
    c->nshape = 0;
    c->have_axis = 0;
    c->cover = 0;
    c->degrees = 0;
    c->frame_gen = 0;
    memset(c->shape, 0, sizeof c->shape);
    memset(c->slot, 0, OS_CLOUD_HASH * sizeof(int));
    if (c->mat)
        memset(c->mat, 0, (size_t)OS_MAT_N * OS_MAT_N * OS_MAT_N * (1 + OS_MAT_CAP));
}

int os_cloud_points(const os_cloud *c) { return c ? c->n : 0; }
int os_cloud_shape_count(const os_cloud *c) { return c ? c->nshape : 0; }
int os_cloud_coverage(const os_cloud *c) { return c ? c->cover : 0; }
int os_cloud_degrees(const os_cloud *c) { return c ? c->degrees : 0; }

static int slot_of(const os_cloud *c, int ix, int iy, int iz, int *empty) {
    unsigned h = mix_hash(ix, iy, iz);
    unsigned mask = OS_CLOUD_HASH - 1;
    int n;
    if (empty) *empty = -1;
    for (n = 0; n < 48; n++) {
        unsigned s = (h + (unsigned)n) & mask;
        int id = c->slot[s];
        if (id == 0) {
            if (empty) *empty = (int)s;
            return -1;
        }
        if (c->pt[id - 1].ix == ix && c->pt[id - 1].iy == iy && c->pt[id - 1].iz == iz)
            return id - 1;
    }
    return -2;
}

static void voxel_of(float x, float y, float z, int *ix, int *iy, int *iz) {
    *ix = os_lround(x / OS_VOXEL);
    *iy = os_lround(y / OS_VOXEL);
    *iz = os_lround(z / OS_VOXEL);
}

static int find_voxel(const os_cloud *c, int ix, int iy, int iz);

/* A live frame jitters by a few millimetres, so the same surface point
 * does not repeat an exact 2 mm cell. A neighbour within 4 mm is that cell. */
static int nearest_held(const os_cloud *c, float x, float y, float z, int ix, int iy, int iz) {
    int dx, dy, dz, best = -1;
    float best_d = 4.f * 4.f;
    for (dz = -2; dz <= 2; dz++) {
        for (dy = -2; dy <= 2; dy++) {
            for (dx = -2; dx <= 2; dx++) {
                int id = find_voxel(c, ix + dx, iy + dy, iz + dz);
                float ex, ey, ez, d;
                if (id < 0) continue;
                ex = c->pt[id].x - x;
                ey = c->pt[id].y - y;
                ez = c->pt[id].z - z;
                d = ex * ex + ey * ey + ez * ez;
                if (d <= best_d) {
                    best_d = d;
                    best = id;
                }
            }
        }
    }
    return best;
}

static int insert_point(os_cloud *c, float x, float y, float z) {
    int ix, iy, iz, empty = -1, found;
    Pt *p;
    voxel_of(x, y, z, &ix, &iy, &iz);
    found = slot_of(c, ix, iy, iz, &empty);
    if (found < 0) {
        int nb = nearest_held(c, x, y, z, ix, iy, iz);
        /* A cell born in this frame is not a confirmation of itself. */
        if (nb >= 0 && c->pt[nb].seen != c->frame_gen) found = nb;
    }
    if (found == -2 || (found < 0 && empty < 0)) return 0;
    if (found >= 0) {
        p = &c->pt[found];
        if (p->seen == c->frame_gen) return 0;
        if (p->nobs < 12) {
            float k = 1.f / (float)(p->nobs + 1);
            p->x += (x - p->x) * k;
            p->y += (y - p->y) * k;
            p->z += (z - p->z) * k;
            voxel_of(p->x, p->y, p->z, &p->ix, &p->iy, &p->iz);
        }
        if (p->nobs < 30000) p->nobs++;
        p->seen = c->frame_gen;
        return 0;
    }
    if (c->n >= OS_CLOUD_MAX) return 0;
    p = &c->pt[c->n];
    p->x = x;
    p->y = y;
    p->z = z;
    p->nobs = 1;
    p->seen = c->frame_gen;
    p->shape = -1;
    p->ix = ix;
    p->iy = iy;
    p->iz = iz;
    c->slot[empty] = c->n + 1;
    c->n++;
    return 1;
}

static int bin_touch(Bin *bins, int n, int ix, int iy, int iz) {
    int i;
    for (i = 0; i < n; i++)
        if (bins[i].ix == ix && bins[i].iy == iy && bins[i].iz == iz) return i;
    return -1;
}

static void fit_shape(os_cloud *c, int sid, const int *ids, int count) {
    Shape *s = &c->shape[sid];
    float mx = 0, my = 0, mz = 0;
    float C[9];
    float rad = 0, acc = 0;
    int i, k;
    memset(s, 0, sizeof *s);
    s->n[1] = 1.f;
    s->u[0] = 1.f;
    s->v[2] = 1.f;
    if (count < 1) return;
    for (i = 0; i < count; i++) {
        Pt *p = &c->pt[ids[i]];
        mx += p->x;
        my += p->y;
        mz += p->z;
    }
    mx /= (float)count;
    my /= (float)count;
    mz /= (float)count;
    memset(C, 0, sizeof C);
    for (i = 0; i < count; i++) {
        Pt *p = &c->pt[ids[i]];
        float dx = p->x - mx, dy = p->y - my, dz = p->z - mz;
        C[0] += dx * dx;
        C[1] += dx * dy;
        C[2] += dx * dz;
        C[4] += dy * dy;
        C[5] += dy * dz;
        C[8] += dz * dz;
    }
    k = count > 0 ? count : 1;
    C[0] /= (float)k;
    C[1] /= (float)k;
    C[2] /= (float)k;
    C[4] /= (float)k;
    C[5] /= (float)k;
    C[8] /= (float)k;
    C[3] = C[1];
    C[6] = C[2];
    C[7] = C[5];
    s->c[0] = mx;
    s->c[1] = my;
    s->c[2] = mz;
    normal_of(C, s->n);
    basis_of(s->n, s->u, s->v);
    for (i = 0; i < count; i++) {
        Pt *p = &c->pt[ids[i]];
        float dx = p->x - mx, dy = p->y - my, dz = p->z - mz;
        float ru = dx * s->u[0] + dy * s->u[1] + dz * s->u[2];
        float rv = dx * s->v[0] + dy * s->v[1] + dz * s->v[2];
        float r2 = ru * ru + rv * rv;
        acc += r2;
        if (r2 > rad) rad = r2;
    }
    rad = os_sqrt(rad);
    acc = os_sqrt(acc / (float)k) * 1.7f;
    if (acc < rad) rad = acc;
    if (rad < 6.f) rad = 6.f;
    if (rad > 22.f) rad = 22.f;
    s->rad = rad;
    s->count = count;
}

static float shape_plane(const Shape *s, float x, float y, float z, float *radial) {
    float dx = x - s->c[0], dy = y - s->c[1], dz = z - s->c[2];
    float ru = dx * s->u[0] + dy * s->u[1] + dz * s->u[2];
    float rv = dx * s->v[0] + dy * s->v[1] + dz * s->v[2];
    if (radial) *radial = os_sqrt(ru * ru + rv * rv);
    return os_fabs(dx * s->n[0] + dy * s->n[1] + dz * s->n[2]);
}

static int nearest_shape(const os_cloud *c, float x, float y, float z, float *plane, float *radial) {
    int i, best = -1;
    float best_s = 1e9f;
    for (i = 0; i < c->nshape; i++) {
        float rad, pl, score;
        pl = shape_plane(&c->shape[i], x, y, z, &rad);
        score = pl + (rad > c->shape[i].rad ? (rad - c->shape[i].rad) * 0.35f : 0.f);
        if (score < best_s) {
            best_s = score;
            best = i;
            if (plane) *plane = pl;
            if (radial) *radial = rad;
        }
    }
    return best;
}

static void refit(os_cloud *c) {
    Bin bins[512];
    int order[512];
    int nb, i, k, nkeep = 0;
    int *ids = NULL;
    int *used;
    if (c->n < 8) {
        c->nshape = 0;
        return;
    }
    nb = 0;
    for (i = 0; i < c->n; i++) {
        int ix = os_lround(c->pt[i].x / OS_CELL);
        int iy = os_lround(c->pt[i].y / OS_CELL);
        int iz = os_lround(c->pt[i].z / OS_CELL);
        int b = bin_touch(bins, nb, ix, iy, iz);
        if (b < 0) {
            if (nb >= 512) continue;
            bins[nb].ix = ix;
            bins[nb].iy = iy;
            bins[nb].iz = iz;
            bins[nb].n = 1;
            bins[nb].sx = c->pt[i].x;
            bins[nb].sy = c->pt[i].y;
            bins[nb].sz = c->pt[i].z;
            nb++;
        } else {
            bins[b].n++;
            bins[b].sx += c->pt[i].x;
            bins[b].sy += c->pt[i].y;
            bins[b].sz += c->pt[i].z;
        }
    }
    for (i = 0; i < nb; i++) order[i] = i;
    for (i = 0; i < nb; i++) {
        int best = i;
        for (k = i + 1; k < nb; k++)
            if (bins[order[k]].n > bins[order[best]].n) best = k;
        k = order[i];
        order[i] = order[best];
        order[best] = k;
    }
    c->nshape = 0;
    for (i = 0; i < nb && c->nshape < OS_CLOUD_SHAPES; i++) {
        Bin *b = &bins[order[i]];
        float cx, cy, cz;
        int keep = 1, s;
        if (b->n < 8) break;
        cx = b->sx / (float)b->n;
        cy = b->sy / (float)b->n;
        cz = b->sz / (float)b->n;
        for (s = 0; s < c->nshape; s++) {
            float dx = c->shape[s].c[0] - cx;
            float dy = c->shape[s].c[1] - cy;
            float dz = c->shape[s].c[2] - cz;
            if (dx * dx + dy * dy + dz * dz < 10.f * 10.f) {
                keep = 0;
                break;
            }
        }
        if (!keep) continue;
        c->shape[c->nshape].c[0] = cx;
        c->shape[c->nshape].c[1] = cy;
        c->shape[c->nshape].c[2] = cz;
        c->shape[c->nshape].count = b->n;
        c->nshape++;
    }
    ids = malloc((size_t)c->n * sizeof(int));
    used = calloc((size_t)c->nshape, sizeof(int));
    if (!ids || (c->nshape && !used)) {
        free(ids);
        free(used);
        return;
    }
    for (k = 0; k < c->nshape; k++) {
        int m = 0;
        float cx = c->shape[k].c[0], cy = c->shape[k].c[1], cz = c->shape[k].c[2];
        for (i = 0; i < c->n; i++) {
            float dx = c->pt[i].x - cx, dy = c->pt[i].y - cy, dz = c->pt[i].z - cz;
            if (dx * dx + dy * dy + dz * dz > (OS_CELL * 1.25f) * (OS_CELL * 1.25f)) continue;
            if (m < c->n) ids[m++] = i;
        }
        fit_shape(c, k, ids, m);
    }
    for (i = 0; i < c->n; i++) {
        float plane = 0, radial = 0;
        int s = nearest_shape(c, c->pt[i].x, c->pt[i].y, c->pt[i].z, &plane, &radial);
        if (s >= 0 && plane < 7.f && radial < c->shape[s].rad + 3.f) {
            c->pt[i].shape = (short)s;
            used[s]++;
        } else {
            c->pt[i].shape = -1;
        }
    }
    nkeep = 0;
    for (k = 0; k < c->nshape; k++)
        if (used[k] >= 8) nkeep++;
    if (nkeep == 0) c->nshape = 0;
    free(ids);
    free(used);
    (void)nkeep;
}

static int shape_on_disk(const Shape *s, float ru, float rv) {
    return ru * ru + rv * rv <= s->rad * s->rad * 0.90f;
}

static void measure_cover(os_cloud *c) {
    int s, filled = 0, cells = 0;
    unsigned char seen[72];
    int bins = 0, i;
    memset(seen, 0, sizeof seen);
    for (s = 0; s < c->nshape; s++) {
        Shape *sh = &c->shape[s];
        int gw, u, v, ncell;
        unsigned char *occ;
        if (sh->rad < 4.f) continue;
        gw = (int)(sh->rad * 2.f / OS_VOXEL) + 3;
        if (gw < 5) gw = 5;
        if (gw > 40) gw = 40;
        ncell = gw * gw;
        occ = calloc((size_t)ncell, 1);
        if (!occ) break;
        for (i = 0; i < c->n; i++) {
            float dx, dy, dz, ru, rv;
            int gu, gv;
            if (c->pt[i].shape != s || c->pt[i].nobs < OS_CONFIRM) continue;
            dx = c->pt[i].x - sh->c[0];
            dy = c->pt[i].y - sh->c[1];
            dz = c->pt[i].z - sh->c[2];
            ru = dx * sh->u[0] + dy * sh->u[1] + dz * sh->u[2];
            rv = dx * sh->v[0] + dy * sh->v[1] + dz * sh->v[2];
            gu = os_lround(ru / OS_VOXEL) + gw / 2;
            gv = os_lround(rv / OS_VOXEL) + gw / 2;
            if (gu < 0 || gv < 0 || gu >= gw || gv >= gw) continue;
            occ[gv * gw + gu] = 1;
        }
        for (v = 0; v < gw; v++) {
            for (u = 0; u < gw; u++) {
                int near = 0, du, dv;
                float ru = (float)(u - gw / 2) * OS_VOXEL;
                float rv = (float)(v - gw / 2) * OS_VOXEL;
                if (!shape_on_disk(sh, ru, rv)) continue;
                for (dv = -2; dv <= 2; dv++) {
                    for (du = -2; du <= 2; du++) {
                        int uu = u + du, vv = v + dv;
                        if (uu < 0 || vv < 0 || uu >= gw || vv >= gw) continue;
                        if (occ[vv * gw + uu]) near++;
                    }
                }
                /* The empty rim of the disk is not a hole. A cell counts when
                 * the patch already has points around it. */
                if (near < 4) continue;
                cells++;
                if (occ[v * gw + u]) filled++;
            }
        }
        free(occ);
    }
    c->cover = cells > 0 ? filled * 100 / cells : 0;
    if (c->cover > 100) c->cover = 100;
    for (i = 0; i < c->n; i++) {
        float dx, dz;
        int b;
        if (c->pt[i].nobs < OS_CONFIRM) continue;
        dx = c->pt[i].x - c->axis_x;
        dz = c->pt[i].z - c->axis_z;
        b = (int)((os_atan2(dx, dz) + 3.14159265f) / (2.f * 3.14159265f) * 72.f);
        if (b < 0) b = 0;
        if (b > 71) b = 71;
        seen[b] = 1;
    }
    for (i = 0; i < 72; i++) bins += seen[i] ? 1 : 0;
    c->degrees = bins * 5;
}

static int frame_bins(const Sample *s, int n, Bin *bins, int cap) {
    int i, nb = 0;
    for (i = 0; i < n; i++) {
        int b = bin_touch(bins, nb, s[i].ix, s[i].iy, s[i].iz);
        if (b < 0) {
            if (nb >= cap) continue;
            bins[nb].ix = s[i].ix;
            bins[nb].iy = s[i].iy;
            bins[nb].iz = s[i].iz;
            bins[nb].n = 1;
            bins[nb].sx = s[i].x;
            bins[nb].sy = s[i].y;
            bins[nb].sz = s[i].z;
            nb++;
        } else {
            bins[b].n++;
            bins[b].sx += s[i].x;
            bins[b].sy += s[i].y;
            bins[b].sz += s[i].z;
        }
    }
    return nb;
}

static int find_voxel(const os_cloud *c, int ix, int iy, int iz) {
    unsigned h = mix_hash(ix, iy, iz);
    unsigned mask = OS_CLOUD_HASH - 1;
    int n;
    for (n = 0; n < 32; n++) {
        unsigned s = (h + (unsigned)n) & mask;
        int id = c->slot[s];
        if (id == 0) return -1;
        if (c->pt[id - 1].ix == ix && c->pt[id - 1].iy == iy && c->pt[id - 1].iz == iz)
            return id - 1;
    }
    return -1;
}

/* Distance in millimetres to the nearest stored point. 5 means none nearby. */
static float point_gap(const os_cloud *c, float x, float y, float z) {
    int ix, iy, iz, dx, dy, dz;
    float best = 5.f;
    voxel_of(x, y, z, &ix, &iy, &iz);
    for (dz = -2; dz <= 2; dz++) {
        for (dy = -2; dy <= 2; dy++) {
            for (dx = -2; dx <= 2; dx++) {
                int id = find_voxel(c, ix + dx, iy + dy, iz + dz);
                float ex, ey, ez, d;
                if (id < 0) continue;
                ex = c->pt[id].x - x;
                ey = c->pt[id].y - y;
                ez = c->pt[id].z - z;
                d = os_sqrt(ex * ex + ey * ey + ez * ez);
                if (d < best) best = d;
            }
        }
    }
    return best;
}

static float score_yaw(const os_cloud *c, const Sample *sample, int ns, float yaw) {
    int i, used = 0, step;
    float sum = 0;
    if (ns < 8 || c->n < 8) return 1e9f;
    step = ns > 160 ? ns / 160 : 1;
    for (i = 0; i < ns; i += step) {
        float x = sample[i].x, z = sample[i].z, d;
        rot_y(-yaw, c->axis_x, c->axis_z, &x, &z);
        d = point_gap(c, x, sample[i].y, z);
        if (d > 5.f) d = 5.f;
        sum += d;
        used++;
    }
    if (used < 4) return 1e9f;
    return sum / (float)used;
}

static float wrap_pi(float a) {
    const float pi = 3.14159265f;
    while (a > pi) a -= 2.f * pi;
    while (a < -pi) a += 2.f * pi;
    return a;
}

static float xz_ang(float x, float z, float ax, float az) {
    return os_atan2(x - ax, -(z - az));
}

static float xz_rad(float x, float z, float ax, float az) {
    float dx = x - ax, dz = z - az;
    return os_sqrt(dx * dx + dz * dz);
}

static int push_yaw(float *cand, int n, int cap, float yaw) {
    int i;
    yaw = wrap_pi(yaw);
    for (i = 0; i < n; i++)
        if (os_fabs(wrap_pi(cand[i] - yaw)) < 0.08f) return n;
    if (n >= cap) return n;
    cand[n] = yaw;
    return n + 1;
}

/* Shape angles vote for a yaw anywhere around the object. The outline prior
 * is one more candidate. A round body does not outvote a repeated shape. */
static int lock_yaw(const os_cloud *c, const Sample *sample, int ns, const Bin *frame, int nframe,
                    float prior, float *yaw_out) {
    float cand[96];
    int nc, i, s, v;
    int votes[128];
    float best, prior_score, pick, y;
    if (c->n < 8) {
        *yaw_out = prior;
        return 1;
    }
    memset(votes, 0, sizeof votes);
    nc = 0;
    nc = push_yaw(cand, nc, 96, prior);
    for (i = 0; i < nframe; i++) {
        float x, yb, z, an;
        if (frame[i].n < 6) continue;
        x = frame[i].sx / (float)frame[i].n;
        yb = frame[i].sy / (float)frame[i].n;
        z = frame[i].sz / (float)frame[i].n;
        an = xz_ang(x, z, c->axis_x, c->axis_z);
        if (xz_rad(x, z, c->axis_x, c->axis_z) < 18.f) continue;
        for (s = 0; s < c->nshape; s++) {
            float as, yaw, px, py, pz, dx, dy, dz;
            int bin;
            if (xz_rad(c->shape[s].c[0], c->shape[s].c[2], c->axis_x, c->axis_z) < 18.f) continue;
            as = xz_ang(c->shape[s].c[0], c->shape[s].c[2], c->axis_x, c->axis_z);
            yaw = wrap_pi(an - as);
            px = x;
            py = yb;
            pz = z;
            rot_y(-yaw, c->axis_x, c->axis_z, &px, &pz);
            dx = px - c->shape[s].c[0];
            dy = py - c->shape[s].c[1];
            dz = pz - c->shape[s].c[2];
            if (dx * dx + dy * dy + dz * dz > 14.f * 14.f) continue;
            bin = (int)((yaw + 3.14159265f) / (2.f * 3.14159265f) * 128.f);
            if (bin < 0) bin = 0;
            if (bin > 127) bin = 127;
            votes[bin] += 1;
        }
    }
    prior_score = score_yaw(c, sample, ns, prior);
    pick = prior;
    best = prior_score;
    for (y = prior - 0.35f; y <= prior + 0.35f + 1e-4f; y += 0.0175f) {
        float sc = score_yaw(c, sample, ns, y);
        if (sc < best) {
            best = sc;
            pick = y;
        }
    }
    if (prior_score <= best + 0.15f) {
        pick = prior;
        best = prior_score;
    }
    {
        int top = 0, top_i = -1;
        for (v = 0; v < 128; v++) {
            if (votes[v] > top && votes[v] >= votes[(v + 127) & 127] &&
                votes[v] >= votes[(v + 1) & 127]) {
                top = votes[v];
                top_i = v;
            }
        }
        if (top >= 3 && top_i >= 0) {
            float center = -3.14159265f + ((float)top_i + 0.5f) * (2.f * 3.14159265f / 128.f);
            float far_best = 1e9f, far_pick = center;
            for (y = center - 0.10f; y <= center + 0.10f + 1e-4f; y += 0.0175f) {
                float sc = score_yaw(c, sample, ns, y);
                if (sc < far_best) {
                    far_best = sc;
                    far_pick = y;
                }
            }
            if (far_best + 0.3f < best) {
                pick = far_pick;
                best = far_best;
            }
        }
    }
    (void)nc;
    (void)cand;
    if (best > 4.2f) return 0;
    *yaw_out = pick;
    return 1;
}

static int patch_anchored(const os_cloud *c, float x, float y, float z) {
    int s;
    float best = 1e9f;
    for (s = 0; s < c->nshape; s++) {
        float dx = x - c->shape[s].c[0];
        float dy = y - c->shape[s].c[1];
        float dz = z - c->shape[s].c[2];
        float d = dx * dx + dy * dy + dz * dz;
        if (d < best) best = d;
    }
    return best < 26.f * 26.f;
}

static int point_on_shape(const os_cloud *c, float x, float y, float z) {
    float plane = 0, radial = 0;
    int s = nearest_shape(c, x, y, z, &plane, &radial);
    if (s < 0) return 0;
    return plane < 6.f && radial < c->shape[s].rad + 14.f;
}

static unsigned char *mat_cell(os_cloud *c, int ix, int iy, int iz) {
    size_t n = (size_t)OS_MAT_N;
    if (!c->mat || ix < 0 || iy < 0 || iz < 0 || ix >= OS_MAT_N || iy >= OS_MAT_N || iz >= OS_MAT_N)
        return NULL;
    return c->mat + (((size_t)iy * n + (size_t)iz) * n + (size_t)ix) * (1 + OS_MAT_CAP);
}

static void mat_add(os_cloud *c, int ix, int iy, int iz, int sid) {
    unsigned char *cell = mat_cell(c, ix, iy, iz);
    int k;
    if (!cell || sid < 0 || sid > 255) return;
    for (k = 0; k < cell[0]; k++)
        if (cell[1 + k] == (unsigned char)sid) return;
    if (cell[0] >= OS_MAT_CAP) return;
    cell[1 + cell[0]] = (unsigned char)sid;
    cell[0]++;
}

/* Shapes sit in the matrix as short arrays. A new cloud reads those cells. */
static void fill_matrix(os_cloud *c) {
    int s;
    size_t bytes = (size_t)OS_MAT_N * OS_MAT_N * OS_MAT_N * (1 + OS_MAT_CAP);
    if (!c->mat) return;
    memset(c->mat, 0, bytes);
    for (s = 0; s < c->nshape; s++) {
        int ix, iy, iz, r, dx, dy, dz;
        ix = os_lround((c->shape[s].c[0] - c->axis_x) / OS_MAT_CELL) + OS_MAT_N / 2;
        iy = os_lround(c->shape[s].c[1] / OS_MAT_CELL) + OS_MAT_N / 2;
        iz = os_lround((c->shape[s].c[2] - c->axis_z) / OS_MAT_CELL) + OS_MAT_N / 2;
        r = 0;
        for (dz = -r; dz <= r; dz++)
            for (dy = -r; dy <= r; dy++)
                for (dx = -r; dx <= r; dx++) mat_add(c, ix + dx, iy + dy, iz + dz, s);
    }
}

static void finish(os_cloud *c) {
    /* Shapes locate the next view. They are not a substitute for the points. */
    refit(c);
    fill_matrix(c);
    measure_cover(c);
}

int os_cloud_integrate(os_cloud *c, const os_stripe_pt *pts, int n,
                       float *yaw_io, float axis_x, float axis_z) {
    Sample *sample = NULL;
    Bin frame[256];
    int i, ns = 0, cap, step, added = 0, nframe;
    float prior, yaw;
    if (!c || !yaw_io) return -1;
    prior = *yaw_io;
    if (!c->have_axis) {
        c->axis_x = axis_x;
        c->axis_z = axis_z;
        c->have_axis = 1;
    }
    if (!pts || n < 1) {
        *yaw_io = prior;
        return 0;
    }
    cap = 0;
    for (i = 0; i < n; i++)
        if (pts[i].ok) cap++;
    if (cap < 8) return 0;
    if (c->frame_gen < 65535) c->frame_gen++;
    else c->frame_gen = 1;
    step = cap > 6000 ? cap / 6000 : 1;
    sample = malloc((size_t)(cap / step + 8) * sizeof(Sample));
    if (!sample) return -1;
    {
        int seen = 0;
        int limit = cap / step + 4;
        for (i = 0; i < n && ns < limit; i++) {
            if (!pts[i].ok) continue;
            if ((seen++ % step) != 0) continue;
            if (pts[i].z != pts[i].z) continue;
            sample[ns].x = pts[i].x;
            sample[ns].y = pts[i].y;
            sample[ns].z = pts[i].z;
            sample[ns].ix = os_lround(pts[i].x / OS_CELL);
            sample[ns].iy = os_lround(pts[i].y / OS_CELL);
            sample[ns].iz = os_lround(pts[i].z / OS_CELL);
            sample[ns].bin = -1;
            sample[ns].shape = -1;
            ns++;
        }
    }
    if (c->n == 0) {
        for (i = 0; i < ns; i++)
            added += insert_point(c, sample[i].x, sample[i].y, sample[i].z);
        finish(c);
        *yaw_io = prior;
        free(sample);
        return added;
    }
    nframe = frame_bins(sample, ns, frame, 256);
    for (i = 0; i < ns; i++)
        sample[i].bin = bin_touch(frame, nframe, sample[i].ix, sample[i].iy, sample[i].iz);
    if (!lock_yaw(c, sample, ns, frame, nframe, prior, &yaw)) {
        free(sample);
        return -1;
    }
    for (i = 0; i < nframe; i++) {
        float x, y, z;
        if (frame[i].n < 1) continue;
        x = frame[i].sx / (float)frame[i].n;
        y = frame[i].sy / (float)frame[i].n;
        z = frame[i].sz / (float)frame[i].n;
        rot_y(-yaw, c->axis_x, c->axis_z, &x, &z);
        frame[i].sx = x;
        frame[i].sy = y;
        frame[i].sz = z;
    }
    for (i = 0; i < ns; i++) {
        float x = sample[i].x, y = sample[i].y, z = sample[i].z;
        int b = sample[i].bin;
        int anchored = 0;
        rot_y(-yaw, c->axis_x, c->axis_z, &x, &z);
        if (b >= 0 && frame[b].n >= 8)
            anchored = patch_anchored(c, frame[b].sx, frame[b].sy, frame[b].sz);
        else
            anchored = point_on_shape(c, x, y, z);
        if (!anchored && !point_on_shape(c, x, y, z)) continue;
        added += insert_point(c, x, y, z);
    }
    finish(c);
    *yaw_io = yaw;
    free(sample);
    return added;
}

static int grid_next(const int *own, int gw, int u, int v, int du, int dv, int reach) {
    int k;
    for (k = 1; k <= reach; k++) {
        int uu = u + du * k, vv = v + dv * k;
        if (uu < 0 || vv < 0 || uu >= gw || vv >= gw) return -1;
        if (own[vv * gw + uu] >= 0) return own[vv * gw + uu];
    }
    return -1;
}

static int grid_diag(const int *own, int gw, int u, int v, int reach) {
    int best = -1, bd = reach * reach * 2, du, dv;
    for (dv = 1; dv <= reach; dv++) {
        for (du = 1; du <= reach; du++) {
            int uu = u + du, vv = v + dv, d;
            if (uu >= gw || vv >= gw) continue;
            if (own[vv * gw + uu] < 0) continue;
            d = du * du + dv * dv;
            if (d < bd) {
                bd = d;
                best = own[vv * gw + uu];
            }
        }
    }
    return best;
}

static int edge_ok(const os_cloud *c, int a, int b) {
    float dx, dy, dz;
    if (a < 0 || b < 0 || a == b) return 0;
    dx = c->pt[a].x - c->pt[b].x;
    dy = c->pt[a].y - c->pt[b].y;
    dz = c->pt[a].z - c->pt[b].z;
    return dx * dx + dy * dy + dz * dz < 64.f;
}

static int emit_tri(os_cloud_tri *out, int n, int cap, float ax, float ay, float az,
                    float bx, float by, float bz, float cx, float cy, float cz) {
    float e1x, e1y, e1z, e2x, e2y, e2z, cxp, cyp, czp;
    if (n >= cap) return n;
    e1x = bx - ax;
    e1y = by - ay;
    e1z = bz - az;
    e2x = cx - ax;
    e2y = cy - ay;
    e2z = cz - az;
    cxp = e1y * e2z - e1z * e2y;
    cyp = e1z * e2x - e1x * e2z;
    czp = e1x * e2y - e1y * e2x;
    if (cxp * cxp + cyp * cyp + czp * czp < 0.04f) return n;
    out[n].a[0] = ax;
    out[n].a[1] = ay;
    out[n].a[2] = az;
    out[n].b[0] = bx;
    out[n].b[1] = by;
    out[n].b[2] = bz;
    out[n].c[0] = cx;
    out[n].c[1] = cy;
    out[n].c[2] = cz;
    return n + 1;
}

#define OS_BODY_A 96

static void body_at(const os_cloud *c, int ia, int iy, float ymin, float step, float r, float o[3]) {
    float ang = -3.14159265f + ((float)ia + 0.5f) * (6.2831853f / (float)OS_BODY_A);
    o[0] = c->axis_x + r * os_sin(ang);
    o[1] = ymin + ((float)iy + 0.5f) * step;
    o[2] = c->axis_z - r * os_cos(ang);
}

int os_cloud_body(os_cloud *c, os_cloud_tri *out, int cap) {
    float ymin = 1e9f, ymax = -1e9f, step, *rad;
    unsigned char *cnt, *ncol;
    int i, ia, iy, ny, n = 0, cols = 0, filled = 0;
    int confirmed = 0;
    if (!c || !out || cap < 1 || !c->have_axis || c->n < 24) return 0;
    for (i = 0; i < c->n; i++) {
        if (c->pt[i].nobs < OS_CONFIRM) continue;
        confirmed++;
        if (c->pt[i].y < ymin) ymin = c->pt[i].y;
        if (c->pt[i].y > ymax) ymax = c->pt[i].y;
    }
    if (confirmed < 24) return 0;
    if (ymax - ymin < 3.f) ymax = ymin + 3.f;
    step = 2.f;
    ny = (int)((ymax - ymin) / step) + 1;
    if (ny < 2) ny = 2;
    if (ny > 180) {
        step = (ymax - ymin) / 179.f;
        ny = 180;
    }
    rad = calloc((size_t)OS_BODY_A * (size_t)ny, sizeof(float));
    cnt = calloc((size_t)OS_BODY_A * (size_t)ny, 1);
    ncol = calloc((size_t)OS_BODY_A, 1);
    if (!rad || !cnt || !ncol) {
        free(rad);
        free(cnt);
        free(ncol);
        return 0;
    }
    for (i = 0; i < c->n; i++) {
        float dx = c->pt[i].x - c->axis_x;
        float dz = c->pt[i].z - c->axis_z;
        float r = os_sqrt(dx * dx + dz * dz);
        float ang;
        int k;
        if (c->pt[i].nobs < OS_CONFIRM) continue;
        if (r < 2.f) continue;
        ang = os_atan2(dx, -dz);
        ia = (int)((ang + 3.14159265f) / 6.2831853f * (float)OS_BODY_A);
        if (ia < 0) ia = 0;
        if (ia >= OS_BODY_A) ia = OS_BODY_A - 1;
        iy = (int)((c->pt[i].y - ymin) / step);
        if (iy < 0) iy = 0;
        if (iy >= ny) iy = ny - 1;
        k = iy * OS_BODY_A + ia;
        if (!cnt[k]) {
            rad[k] = r;
            cnt[k] = 1;
        } else if (r > rad[k] && r < rad[k] + 8.f) {
            rad[k] = rad[k] * 0.35f + r * 0.65f;
            if (cnt[k] < 255) cnt[k]++;
        } else if (os_fabs(r - rad[k]) < 4.f && cnt[k] < 255) {
            rad[k] = (rad[k] * cnt[k] + r) / (float)(cnt[k] + 1);
            cnt[k]++;
        }
    }
    for (ia = 0; ia < OS_BODY_A; ia++) {
        for (iy = 0; iy < ny; iy++)
            if (cnt[iy * OS_BODY_A + ia]) {
                ncol[ia] = 1;
                filled++;
            }
        cols += ncol[ia] ? 1 : 0;
    }
    c->cover = cols > 0 ? filled * 100 / (cols * ny) : 0;
    if (c->cover > 100) c->cover = 100;
    for (iy = 0; iy < ny - 1 && n + 2 <= cap; iy++) {
        for (ia = 0; ia < OS_BODY_A && n + 2 <= cap; ia++) {
            int ir = (ia + 1) % OS_BODY_A;
            int k00 = iy * OS_BODY_A + ia;
            int k10 = iy * OS_BODY_A + ir;
            int k01 = (iy + 1) * OS_BODY_A + ia;
            int k11 = (iy + 1) * OS_BODY_A + ir;
            float p00[3], p10[3], p01[3], p11[3];
            if (!cnt[k00] || !cnt[k10] || !cnt[k01] || !cnt[k11]) continue;
            if (os_fabs(rad[k00] - rad[k10]) > 10.f || os_fabs(rad[k00] - rad[k01]) > 10.f ||
                os_fabs(rad[k10] - rad[k11]) > 10.f || os_fabs(rad[k01] - rad[k11]) > 10.f)
                continue;
            body_at(c, ia, iy, ymin, step, rad[k00], p00);
            body_at(c, ir, iy, ymin, step, rad[k10], p10);
            body_at(c, ia, iy + 1, ymin, step, rad[k01], p01);
            body_at(c, ir, iy + 1, ymin, step, rad[k11], p11);
            n = emit_tri(out, n, cap, p00[0], p00[1], p00[2], p10[0], p10[1], p10[2], p01[0], p01[1], p01[2]);
            n = emit_tri(out, n, cap, p10[0], p10[1], p10[2], p11[0], p11[1], p11[2], p01[0], p01[1], p01[2]);
        }
    }
    free(ncol);
    free(cnt);
    free(rad);
    return n;
}

int os_cloud_mesh(const os_cloud *c, os_cloud_tri *out, int cap) {
    int s, n = 0, i;
    if (!c || !out || cap < 1 || c->nshape < 1) return 0;
    for (s = 0; s < c->nshape && n < cap; s++) {
        const Shape *sh = &c->shape[s];
        int gw, u, v, ncell;
        int *own;
        if (sh->count < 6 || sh->rad < 3.f) continue;
        gw = (int)(sh->rad * 2.f / OS_VOXEL) + 3;
        if (gw < 4) gw = 4;
        if (gw > 48) gw = 48;
        ncell = gw * gw;
        own = malloc((size_t)ncell * sizeof(int));
        if (!own) break;
        for (i = 0; i < ncell; i++) own[i] = -1;
        for (i = 0; i < c->n; i++) {
            float dx, dy, dz, ru, rv, plane;
            int gu, gv, slot;
            if (c->pt[i].shape != s || c->pt[i].nobs < OS_CONFIRM) continue;
            dx = c->pt[i].x - sh->c[0];
            dy = c->pt[i].y - sh->c[1];
            dz = c->pt[i].z - sh->c[2];
            ru = dx * sh->u[0] + dy * sh->u[1] + dz * sh->u[2];
            rv = dx * sh->v[0] + dy * sh->v[1] + dz * sh->v[2];
            plane = os_fabs(dx * sh->n[0] + dy * sh->n[1] + dz * sh->n[2]);
            gu = os_lround(ru / OS_VOXEL) + gw / 2;
            gv = os_lround(rv / OS_VOXEL) + gw / 2;
            if (gu < 0 || gv < 0 || gu >= gw || gv >= gw) continue;
            slot = gv * gw + gu;
            if (own[slot] < 0) own[slot] = i;
            else {
                float odx = c->pt[own[slot]].x - sh->c[0];
                float ody = c->pt[own[slot]].y - sh->c[1];
                float odz = c->pt[own[slot]].z - sh->c[2];
                float op = os_fabs(odx * sh->n[0] + ody * sh->n[1] + odz * sh->n[2]);
                if (plane < op) own[slot] = i;
            }
        }
        for (v = 0; v < gw && n < cap; v++) {
            for (u = 0; u < gw && n < cap; u++) {
                int a = own[v * gw + u];
                int b, d, e;
                if (a < 0) continue;
                b = grid_next(own, gw, u, v, 1, 0, 4);
                d = grid_next(own, gw, u, v, 0, 1, 4);
                e = grid_diag(own, gw, u, v, 4);
                if (edge_ok(c, a, b) && edge_ok(c, a, d) && edge_ok(c, b, d))
                    n = emit_tri(out, n, cap, c->pt[a].x, c->pt[a].y, c->pt[a].z,
                                 c->pt[b].x, c->pt[b].y, c->pt[b].z, c->pt[d].x, c->pt[d].y, c->pt[d].z);
                if (edge_ok(c, b, e) && edge_ok(c, b, d) && edge_ok(c, e, d))
                    n = emit_tri(out, n, cap, c->pt[b].x, c->pt[b].y, c->pt[b].z,
                                 c->pt[e].x, c->pt[e].y, c->pt[e].z, c->pt[d].x, c->pt[d].y, c->pt[d].z);
            }
        }
        free(own);
    }
    return n;
}

int os_cloud_specks(const os_cloud *c, os_cloud_tri *out, int cap) {
    int i, n = 0, step;
    if (!c || !out || cap < 1 || c->n < 1) return 0;
    step = c->n > cap ? c->n / cap : 1;
    if (step < 1) step = 1;
    for (i = 0; i < c->n && n + 2 <= cap; i += step) {
        float x, y, z;
        if (c->pt[i].nobs < OS_CONFIRM) continue;
        x = c->pt[i].x;
        y = c->pt[i].y;
        z = c->pt[i].z;
        n = emit_tri(out, n, cap, x, y, z, x + 0.8f, y, z, x, y + 0.8f, z);
        n = emit_tri(out, n, cap, x + 0.8f, y, z, x + 0.8f, y + 0.8f, z, x, y + 0.8f, z);
    }
    return n;
}

static int near_point(const os_cloud *c, float x, float y, float z, float r) {
    int i;
    float r2 = r * r;
    for (i = 0; i < c->n; i++) {
        float dx = c->pt[i].x - x, dy = c->pt[i].y - y, dz = c->pt[i].z - z;
        if (dx * dx + dy * dy + dz * dz <= r2) return 1;
    }
    return 0;
}

static void object_to_cam(float yaw, float ax, float az, float ox, float oy, float oz, os_stripe_pt *p) {
    p->x = ox + ax;
    p->y = oy;
    p->z = oz + az;
    rot_y(yaw, ax, az, &p->x, &p->z);
    p->ok = 1;
    p->tone = 1.f;
    p->plane = 0;
    p->u = 0;
    p->v = 0;
}

static int paint_object(os_stripe_pt *dst, int cap, float yaw, int with_hole, int with_outlier) {
    const float R = 34.f;
    const float ax = 0.f, az = 180.f;
    int n = 0, lat, lon;
    for (lat = 0; lat < 28 && n < cap; lat++) {
        float phi = -1.05f + 2.1f * (float)lat / 27.f;
        for (lon = 0; lon < 56 && n < cap; lon++) {
            float th = -3.14159265f + 6.2831853f * (float)lon / 56.f;
            float ox = R * os_cos(phi) * os_sin(th);
            float oy = R * os_sin(phi);
            float oz = R * os_cos(phi) * os_cos(th);
            float vx = ox, vz = oz;
            rot_y(-yaw, 0.f, 0.f, &vx, &vz);
            if (vz > 6.f) continue;
            if (with_hole && os_fabs(ox) < 3.2f && os_fabs(oy) < 3.2f && oz < 0.f) continue;
            object_to_cam(yaw, ax, az, ox, oy, oz, &dst[n]);
            n++;
        }
    }
    for (lat = 0; lat < 6 && n < cap; lat++) {
        float phi = -0.6f + 1.2f * (float)lat / 5.f;
        for (lon = 0; lon < 8 && n < cap; lon++) {
            float th = 6.2831853f * (float)lon / 8.f;
            float ox = 28.f + 7.f * os_cos(phi) * os_cos(th);
            float oy = 14.f + 7.f * os_sin(phi);
            float oz = -8.f + 7.f * os_cos(phi) * os_sin(th);
            float vx = ox, vz = oz;
            rot_y(-yaw, 0.f, 0.f, &vx, &vz);
            if (vz > 8.f) continue;
            object_to_cam(yaw, ax, az, ox, oy, oz, &dst[n]);
            n++;
        }
    }
    if (with_outlier && n < cap) {
        object_to_cam(yaw, ax, az, 160.f, 0.f, 0.f, &dst[n]);
        n++;
    }
    return n;
}

static int front_shell(const os_cloud *c, const os_cloud_tri *tris, int n) {
    int seen = 0, bad = 0, i, k;
    for (i = 0; i < n; i++) {
        const float *p[3];
        p[0] = tris[i].a;
        p[1] = tris[i].b;
        p[2] = tris[i].c;
        for (k = 0; k < 3; k++) {
            float dx = p[k][0] - c->axis_x;
            float dy = p[k][1];
            float dz = p[k][2] - c->axis_z;
            float r, ang;
            if (os_fabs(dy) > 8.f) continue;
            r = os_sqrt(dx * dx + dz * dz);
            ang = os_atan2(dx, -dz);
            if (os_fabs(ang) > 0.45f) continue;
            seen++;
            if (r < 26.f || r > 42.f) bad++;
        }
    }
    return seen >= 12 && bad * 4 < seen;
}

int openscan_cloud_self_test(void) {
    os_cloud *c;
    os_stripe_pt *view;
    os_cloud_tri *tris;
    float yaw;
    int n, added, n0, n1, tris_n, i;
    openscan_live *live;
    uint8_t *img;
    openscan_live_status st;
    if (openscan_shape_resolve(OPENSCAN_SHAPE_CLOUD, 1) != OPENSCAN_SHAPE_CLOUD) return 1;
    if (openscan_shape_resolve(OPENSCAN_SHAPE_MEASURED, 1) != OPENSCAN_SHAPE_MEASURED) return 1;
    if (openscan_shape_resolve(9, 1) != OPENSCAN_SHAPE_MOLD) return 1;
    c = os_cloud_create();
    view = calloc(8000, sizeof *view);
    tris = calloc(12000, sizeof *tris);
    if (!c || !view || !tris) return 1;

    n = paint_object(view, 8000, 0.f, 0, 0);
    yaw = 0.f;
    added = os_cloud_integrate(c, view, n, &yaw, 0.f, 180.f);
    n0 = os_cloud_points(c);
    if (added < 120 || n0 < 120 || os_cloud_shape_count(c) < 2) {
        fprintf(stderr, "cloud: first view %d pts %d shapes\n", n0, os_cloud_shape_count(c));
        return 1;
    }
    tris_n = os_cloud_body(c, tris, 12000);
    if (tris_n != 0) {
        fprintf(stderr, "cloud: unconfirmed molded %d tris\n", tris_n);
        return 1;
    }

    added = os_cloud_integrate(c, view, n, &yaw, 0.f, 180.f);
    n1 = os_cloud_points(c);
    if (added > 40 || n1 > n0 + 40) {
        fprintf(stderr, "cloud: same view stacked %d -> %d (%d new)\n", n0, n1, added);
        return 1;
    }
    tris_n = os_cloud_body(c, tris, 12000);
    if (tris_n < 40 || !front_shell(c, tris, tris_n)) {
        fprintf(stderr, "cloud: body %d tris\n", tris_n);
        return 1;
    }

    n = paint_object(view, 8000, 0.f, 0, 1);
    yaw = 0.f;
    added = os_cloud_integrate(c, view, n, &yaw, 0.f, 180.f);
    if (near_point(c, 160.f, 0.f, 180.f, 12.f)) {
        fprintf(stderr, "cloud: outlier joined the body\n");
        return 1;
    }

    n = paint_object(view, 8000, 0.50f, 0, 0);
    yaw = 0.50f - 0.22f;
    added = os_cloud_integrate(c, view, n, &yaw, 0.f, 180.f);
    if (added < 0 || os_fabs(yaw - 0.50f) > 0.12f) {
        fprintf(stderr, "cloud: track yaw %.3f added %d\n", yaw, added);
        return 1;
    }
    if (os_cloud_points(c) < n1 + 30) {
        fprintf(stderr, "cloud: turn added too little %d -> %d\n", n1, os_cloud_points(c));
        return 1;
    }

    for (i = 1; i <= 5; i++) {
        float true_yaw = (float)i * 0.85f;
        int pass;
        n = paint_object(view, 8000, true_yaw, 0, 0);
        for (pass = 0; pass < 2; pass++) {
            yaw = true_yaw;
            if (os_cloud_integrate(c, view, n, &yaw, 0.f, 180.f) < 0) {
                fprintf(stderr, "cloud: lost the turn at %.2f\n", true_yaw);
                return 1;
            }
        }
    }
    tris_n = os_cloud_body(c, tris, 12000);
    if (os_cloud_degrees(c) < 200 || tris_n < 120 || !front_shell(c, tris, tris_n)) {
        fprintf(stderr, "cloud: body %d%% %d deg %d tris %d pts %d shapes\n",
                os_cloud_coverage(c), os_cloud_degrees(c), tris_n,
                os_cloud_points(c), os_cloud_shape_count(c));
        return 1;
    }
    os_cloud_free(c);
    free(view);
    free(tris);

    live = openscan_live_create(NULL, NULL);
    img = calloc(160 * 120, 1);
    if (!live || !img) return 1;
    for (i = 0; i < 160 * 120; i++) img[i] = 0;
    for (i = 0; i < 120; i++) {
        int x;
        for (x = 0; x < 160; x++) {
            float nx = (x - 70) / 28.f, ny = (i - 60) / 24.f;
            if (nx * nx + ny * ny < 1.f) img[i * 160 + x] = 160;
        }
    }
    openscan_live_set_shape(live, OPENSCAN_SHAPE_CLOUD);
    openscan_live_set_mode(live, OPENSCAN_MODE_SCAN);
    if (openscan_live_push(live, img, img, 160, 120, &st) != 0) return 1;
    for (i = 0; i < 120; i++) {
        int x;
        for (x = 0; x < 160; x++) {
            float nx = (x - 96) / 28.f, ny = (i - 60) / 24.f;
            img[i * 160 + x] = (nx * nx + ny * ny < 1.f) ? 160 : 0;
        }
    }
    if (openscan_live_push(live, img, img, 160, 120, &st) != 0 || st.points != 0) {
        fprintf(stderr, "cloud: outline became a mold %d\n", st.points);
        return 1;
    }
    openscan_live_destroy(live);
    free(img);
    return 0;
}
