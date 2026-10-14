#include "lizard_mold.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define LZ_VOXEL 0.003f
#define LZ_MAX 160000
#define LZ_HASH 1048576
#define LZ_CONFIRM_M 0.025f
#define LZ_HAND_PAD 0.008f

typedef struct {
    int ix, iy, iz;
    float x, y, z;
    float r, g, b;
    float cx, cy, cz;
    unsigned short n;
    unsigned char confirmed;
} Cell;

struct LzMold {
    Cell *cell;
    int *slot;
    int n;
    int rejected;
};

/* Parent of each OpenXR hand joint. Palm has none. Wrist hangs off the palm. */
static const int k_parent[26] = {
    -1, 0, 1, 2, 3, 4,
    1, 6, 7, 8, 9,
    1, 11, 12, 13, 14,
    1, 16, 17, 18, 19,
    1, 21, 22, 23, 24
};

static unsigned mix_hash(int x, int y, int z) {
    return (unsigned)x * 73856093u ^ (unsigned)y * 19349663u ^ (unsigned)z * 83492791u;
}

static float dist3(float ax, float ay, float az, float bx, float by, float bz) {
    float dx = ax - bx, dy = ay - by, dz = az - bz;
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

static float seg_dist(float px, float py, float pz,
                      float ax, float ay, float az,
                      float bx, float by, float bz) {
    float abx = bx - ax, aby = by - ay, abz = bz - az;
    float apx = px - ax, apy = py - ay, apz = pz - az;
    float ab2 = abx * abx + aby * aby + abz * abz;
    float t = 0.f;
    if (ab2 > 1e-12f) {
        t = (apx * abx + apy * aby + apz * abz) / ab2;
        if (t < 0.f) t = 0.f;
        if (t > 1.f) t = 1.f;
    }
    return dist3(px, py, pz, ax + abx * t, ay + aby * t, az + abz * t);
}

LzMold *lz_mold_create(void) {
    LzMold *m = (LzMold *)calloc(1, sizeof *m);
    if (!m) return NULL;
    m->cell = (Cell *)calloc(LZ_MAX, sizeof(Cell));
    m->slot = (int *)calloc(LZ_HASH, sizeof(int));
    if (!m->cell || !m->slot) {
        lz_mold_free(m);
        return NULL;
    }
    return m;
}

void lz_mold_free(LzMold *m) {
    if (!m) return;
    free(m->cell);
    free(m->slot);
    free(m);
}

void lz_mold_reset(LzMold *m) {
    if (!m) return;
    memset(m->cell, 0, (size_t)LZ_MAX * sizeof(Cell));
    memset(m->slot, 0, (size_t)LZ_HASH * sizeof(int));
    m->n = 0;
    m->rejected = 0;
}

int lz_hand_hits(const LzJoint *joints, int njoints, float x, float y, float z) {
    int i;
    if (!joints || njoints < 1) return 0;
    for (i = 0; i < njoints; i++) {
        float rad = joints[i].radius;
        int p;
        if (!(rad > 0.f)) continue;
        if (dist3(x, y, z, joints[i].x, joints[i].y, joints[i].z) <= rad + LZ_HAND_PAD)
            return 1;
        p = (i < 26) ? k_parent[i] : k_parent[i - 26];
        if (p < 0) continue;
        if (i >= 26) p += 26;
        if (p >= njoints || !(joints[p].radius > 0.f)) continue;
        if (seg_dist(x, y, z, joints[p].x, joints[p].y, joints[p].z,
                     joints[i].x, joints[i].y, joints[i].z) <=
            (joints[p].radius > rad ? joints[p].radius : rad) + LZ_HAND_PAD)
            return 1;
    }
    return 0;
}

static Cell *lookup(LzMold *m, int ix, int iy, int iz, int create) {
    unsigned h = mix_hash(ix, iy, iz) & (LZ_HASH - 1);
    int n;
    for (n = 0; n < 96; n++) {
        unsigned s = (h + (unsigned)n) & (LZ_HASH - 1);
        int id = m->slot[s];
        Cell *c;
        if (id == 0) {
            if (!create || m->n >= LZ_MAX) return NULL;
            c = &m->cell[m->n];
            memset(c, 0, sizeof *c);
            c->ix = ix;
            c->iy = iy;
            c->iz = iz;
            m->n++;
            m->slot[s] = m->n;
            return c;
        }
        c = &m->cell[id - 1];
        if (c->ix == ix && c->iy == iy && c->iz == iz) return c;
    }
    return NULL;
}

static const Cell *lookup_c(const LzMold *m, int ix, int iy, int iz) {
    unsigned h = mix_hash(ix, iy, iz) & (LZ_HASH - 1);
    int n;
    for (n = 0; n < 96; n++) {
        unsigned s = (h + (unsigned)n) & (LZ_HASH - 1);
        int id = m->slot[s];
        const Cell *c;
        if (id == 0) return NULL;
        c = &m->cell[id - 1];
        if (c->ix == ix && c->iy == iy && c->iz == iz) return c;
    }
    return NULL;
}

int lz_mold_integrate(LzMold *m, const LzSample *pts, int n,
                      float camx, float camy, float camz,
                      const LzJoint *hands, int nhands) {
    int i, grew = 0;
    if (!m || !pts || n < 1) return 0;
    for (i = 0; i < n; i++) {
        int ix, iy, iz;
        Cell *c;
        float w;
        const LzSample *p = &pts[i];
        if (p->z != p->z || p->x != p->x || p->y != p->y) continue;
        if (lz_hand_hits(hands, nhands, p->x, p->y, p->z)) {
            m->rejected++;
            continue;
        }
        ix = (int)floorf(p->x / LZ_VOXEL);
        iy = (int)floorf(p->y / LZ_VOXEL);
        iz = (int)floorf(p->z / LZ_VOXEL);
        c = lookup(m, ix, iy, iz, 1);
        if (!c) continue;
        if (c->n == 0) {
            c->x = p->x;
            c->y = p->y;
            c->z = p->z;
            c->r = p->r;
            c->g = p->g;
            c->b = p->b;
            c->cx = camx;
            c->cy = camy;
            c->cz = camz;
            c->n = 1;
            continue;
        }
        w = c->n < 12 ? (float)c->n : 12.f;
        c->x = (c->x * w + p->x) / (w + 1.f);
        c->y = (c->y * w + p->y) / (w + 1.f);
        c->z = (c->z * w + p->z) / (w + 1.f);
        c->r = (c->r * w + p->r) / (w + 1.f);
        c->g = (c->g * w + p->g) / (w + 1.f);
        c->b = (c->b * w + p->b) / (w + 1.f);
        if (c->n < 60000) c->n++;
        if (!c->confirmed && dist3(camx, camy, camz, c->cx, c->cy, c->cz) >= LZ_CONFIRM_M) {
            c->confirmed = 1;
            grew++;
        }
    }
    return grew;
}

int lz_mold_confirmed(const LzMold *m) {
    int i, n = 0;
    if (!m) return 0;
    for (i = 0; i < m->n; i++)
        if (m->cell[i].confirmed) n++;
    return n;
}

int lz_mold_stored(const LzMold *m) { return m ? m->n : 0; }
int lz_mold_rejected(const LzMold *m) { return m ? m->rejected : 0; }

static void push_vert(LzMesh *o, int *cap, float x, float y, float z, float r, float g, float b) {
    if (o->nvert >= *cap) {
        int ncap = *cap ? *cap * 2 : 1024;
        float *p = (float *)realloc(o->vert, (size_t)ncap * 6 * sizeof(float));
        if (!p) return;
        o->vert = p;
        *cap = ncap;
    }
    o->vert[o->nvert * 6 + 0] = x;
    o->vert[o->nvert * 6 + 1] = y;
    o->vert[o->nvert * 6 + 2] = z;
    o->vert[o->nvert * 6 + 3] = r;
    o->vert[o->nvert * 6 + 4] = g;
    o->vert[o->nvert * 6 + 5] = b;
    o->nvert++;
}

static void push_idx(LzMesh *o, int *cap, unsigned a, unsigned b, unsigned c) {
    if (o->nidx + 3 > *cap) {
        int ncap = *cap ? *cap * 2 : 1024;
        unsigned *p = (unsigned *)realloc(o->idx, (size_t)ncap * sizeof(unsigned));
        if (!p) return;
        o->idx = p;
        *cap = ncap;
    }
    o->idx[o->nidx++] = a;
    o->idx[o->nidx++] = b;
    o->idx[o->nidx++] = c;
}

static int cell_kept(const Cell *c, int confirmed_only) {
    if (!c || c->n < 1) return 0;
    if (confirmed_only && !c->confirmed) return 0;
    return 1;
}

static int mesh_cells(const LzMold *m, LzMesh *out, int confirmed_only) {
    static const int off[6][3] = {
        {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
    int i, vcap = 0, icap = 0;
    const float h = LZ_VOXEL * 0.5f;
    if (!out) return -1;
    memset(out, 0, sizeof *out);
    if (!m) return 0;
    for (i = 0; i < m->n; i++) {
        const Cell *c = &m->cell[i];
        int f, neigh = 0;
        float cx, cy, cz;
        if (!cell_kept(c, confirmed_only)) continue;
        if (out->nidx > 180000) break;
        for (f = 0; f < 6; f++) {
            const Cell *nb = lookup_c(m, c->ix + off[f][0], c->iy + off[f][1], c->iz + off[f][2]);
            if (cell_kept(nb, confirmed_only)) neigh++;
        }
        if (neigh < 1) continue;
        cx = ((float)c->ix + 0.5f) * LZ_VOXEL;
        cy = ((float)c->iy + 0.5f) * LZ_VOXEL;
        cz = ((float)c->iz + 0.5f) * LZ_VOXEL;
        for (f = 0; f < 6; f++) {
            const Cell *nb = lookup_c(m, c->ix + off[f][0], c->iy + off[f][1], c->iz + off[f][2]);
            float nx = (float)off[f][0], ny = (float)off[f][1], nz = (float)off[f][2];
            unsigned base;
            float q[4][3];
            int k;
            if (cell_kept(nb, confirmed_only)) continue;
            base = (unsigned)out->nvert;
            if (off[f][0] != 0) {
                q[0][0] = cx + nx * h; q[0][1] = cy - h; q[0][2] = cz - h;
                q[1][0] = cx + nx * h; q[1][1] = cy - h; q[1][2] = cz + h;
                q[2][0] = cx + nx * h; q[2][1] = cy + h; q[2][2] = cz + h;
                q[3][0] = cx + nx * h; q[3][1] = cy + h; q[3][2] = cz - h;
            } else if (off[f][1] != 0) {
                q[0][0] = cx - h; q[0][1] = cy + ny * h; q[0][2] = cz - h;
                q[1][0] = cx + h; q[1][1] = cy + ny * h; q[1][2] = cz - h;
                q[2][0] = cx + h; q[2][1] = cy + ny * h; q[2][2] = cz + h;
                q[3][0] = cx - h; q[3][1] = cy + ny * h; q[3][2] = cz + h;
            } else {
                q[0][0] = cx - h; q[0][1] = cy - h; q[0][2] = cz + nz * h;
                q[1][0] = cx + h; q[1][1] = cy - h; q[1][2] = cz + nz * h;
                q[2][0] = cx + h; q[2][1] = cy + h; q[2][2] = cz + nz * h;
                q[3][0] = cx - h; q[3][1] = cy + h; q[3][2] = cz + nz * h;
            }
            for (k = 0; k < 4; k++)
                push_vert(out, &vcap, q[k][0], q[k][1], q[k][2], c->r, c->g, c->b);
            if (out->nvert < (int)base + 4) continue;
            push_idx(out, &icap, base, base + 1, base + 2);
            push_idx(out, &icap, base, base + 2, base + 3);
        }
    }
    return 0;
}

int lz_mold_mesh(const LzMold *m, LzMesh *out) { return mesh_cells(m, out, 1); }

int lz_mold_shell(const LzMold *m, LzMesh *out) { return mesh_cells(m, out, 0); }

int lz_mold_cloud(const LzMold *m, float *xyzrgb, int cap) {
    int i, n = 0;
    if (!m || !xyzrgb || cap < 1) return 0;
    for (i = 0; i < m->n && n < cap; i++) {
        const Cell *c = &m->cell[i];
        if (c->n < 1) continue;
        xyzrgb[n * 6 + 0] = c->x;
        xyzrgb[n * 6 + 1] = c->y;
        xyzrgb[n * 6 + 2] = c->z;
        xyzrgb[n * 6 + 3] = c->r;
        xyzrgb[n * 6 + 4] = c->g;
        xyzrgb[n * 6 + 5] = c->b;
        n++;
    }
    return n;
}

void lz_mesh_free(LzMesh *out) {
    if (!out) return;
    free(out->vert);
    free(out->idx);
    out->vert = NULL;
    out->idx = NULL;
    out->nvert = out->nidx = 0;
}
