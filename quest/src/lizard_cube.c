#include "lizard_cube.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/* While the cube follows the head, its near face sits 0.20 m in front of the eyes
 * and its centre is 0.03 m below the look direction. A pinch closer than 25 mm
 * grabs; it stays grabbed until the fingertips open past 40 mm. */
#define LZ_PINCH_ON 0.025f
#define LZ_PINCH_OFF 0.040f
#define LZ_GRAB_MARGIN 0.06f
#define LZ_CORNER_REACH 0.045f
#define LZ_BEAM_HALF 0.0035f

typedef struct {
    int active;
    float x, y, z;
} Pinch;

static float dist3(float ax, float ay, float az, float bx, float by, float bz) {
    float dx = ax - bx, dy = ay - by, dz = az - bz;
    return sqrtf(dx * dx + dy * dy + dz * dz);
}

void lz_cube_init(LzCube *c, float half_m) {
    if (!c) return;
    memset(c, 0, sizeof *c);
    if (half_m < 0.04f) half_m = 0.04f;
    if (half_m > 0.25f) half_m = 0.25f;
    c->half = half_m;
    c->grab_hand = -1;
}

int lz_cube_scanning(const LzCube *c) {
    return c && c->ready && c->placed && !c->grabbed;
}

void lz_cube_to_local(const LzCube *c, float x, float y, float z,
                      float *ox, float *oy, float *oz) {
    *ox = x - c->cx;
    *oy = y - c->cy;
    *oz = z - c->cz;
}

void lz_cube_to_world(const LzCube *c, float x, float y, float z,
                      float *ox, float *oy, float *oz) {
    *ox = c->cx + x;
    *oy = c->cy + y;
    *oz = c->cz + z;
}

int lz_cube_contains(const LzCube *c, float x, float y, float z, float margin) {
    float h;
    if (!c || !c->ready || !(c->half > 0.f)) return 0;
    h = c->half + margin;
    return fabsf(x - c->cx) <= h && fabsf(y - c->cy) <= h && fabsf(z - c->cz) <= h;
}

void lz_joints_into_cube(const LzCube *c, const LzJoint *in, int n, LzJoint *out) {
    int i;
    if (!out || n < 1) return;
    for (i = 0; i < n; i++) {
        out[i] = in ? in[i] : (LzJoint){0};
        if (!in || !c || !c->ready) continue;
        lz_cube_to_local(c, in[i].x, in[i].y, in[i].z, &out[i].x, &out[i].y, &out[i].z);
    }
}

void lz_cube_model(const LzCube *c, float m[16]) {
    memset(m, 0, sizeof(float) * 16);
    m[0] = m[5] = m[10] = m[15] = 1.f;
    if (!c) return;
    m[12] = c->cx;
    m[13] = c->cy;
    m[14] = c->cz;
}

static void pose_follow(LzCube *c, const LzPose *head) {
    float ahead = 0.20f + c->half;
    float hx, hy, hz;
    lz_quat_rotate(head->qx, head->qy, head->qz, head->qw, 0.f, -0.03f, -ahead, &hx, &hy, &hz);
    c->cx = head->px + hx;
    c->cy = head->py + hy;
    c->cz = head->pz + hz;
}

static void pinch_of(LzCube *c, const LzJoint *j, int n, int hand, Pinch *out) {
    int base = hand ? 26 : 0;
    int thumb = base + 5;
    int index = base + 10;
    float d;
    out->active = 0;
    out->x = out->y = out->z = 0.f;
    if (!j || thumb >= n || index >= n || !(j[thumb].radius > 0.f) || !(j[index].radius > 0.f)) {
        c->pinch_latched[hand] = 0;
        return;
    }
    d = dist3(j[thumb].x, j[thumb].y, j[thumb].z, j[index].x, j[index].y, j[index].z);
    if (c->pinch_latched[hand]) out->active = d < LZ_PINCH_OFF;
    else out->active = d < LZ_PINCH_ON;
    c->pinch_latched[hand] = out->active;
    if (!out->active) return;
    out->x = 0.5f * (j[thumb].x + j[index].x);
    out->y = 0.5f * (j[thumb].y + j[index].y);
    out->z = 0.5f * (j[thumb].z + j[index].z);
}

static void corner_world(const LzCube *c, int i, float *x, float *y, float *z) {
    float lx = (i & 1) ? c->half : -c->half;
    float ly = (i & 2) ? c->half : -c->half;
    float lz = (i & 4) ? c->half : -c->half;
    lz_cube_to_world(c, lx, ly, lz, x, y, z);
}

static int near_corner(const LzCube *c, float x, float y, float z) {
    int i;
    for (i = 0; i < 8; i++) {
        float cx, cy, cz;
        corner_world(c, i, &cx, &cy, &cz);
        if (dist3(x, y, z, cx, cy, cz) <= LZ_CORNER_REACH) return 1;
    }
    return 0;
}

static int pinch_reaches(const LzCube *c, float x, float y, float z) {
    if (!c->placed) return lz_cube_contains(c, x, y, z, LZ_GRAB_MARGIN);
    return near_corner(c, x, y, z);
}

int lz_cube_step(LzCube *c, const LzPose *head, int head_ok,
                 const LzJoint *hands, int nhands) {
    Pinch pin[2];
    int hand, best;
    float best_d;
    if (!c) return 0;
    /* Follow only until the first lock. A placed cube stays where the hand left it. */
    if (head_ok && head && !c->grabbed && !c->placed) {
        pose_follow(c, head);
        c->ready = 1;
    }
    pinch_of(c, hands, nhands, 0, &pin[0]);
    pinch_of(c, hands, nhands, 1, &pin[1]);
    if (!c->ready) return 0;
    if (c->grabbed) {
        int h = c->grab_hand;
        if (h >= 0 && h <= 1 && pin[h].active) {
            c->cx = pin[h].x + c->grab_ox;
            c->cy = pin[h].y + c->grab_oy;
            c->cz = pin[h].z + c->grab_oz;
            return 0;
        }
        c->grabbed = 0;
        c->grab_hand = -1;
        c->placed = 1;
        return 1;
    }
    best = -1;
    best_d = 1e9f;
    for (hand = 0; hand < 2; hand++) {
        float d;
        if (!pin[hand].active) continue;
        if (!pinch_reaches(c, pin[hand].x, pin[hand].y, pin[hand].z)) continue;
        d = dist3(pin[hand].x, pin[hand].y, pin[hand].z, c->cx, c->cy, c->cz);
        if (d < best_d) {
            best_d = d;
            best = hand;
        }
    }
    if (best < 0) return 0;
    c->grab_hand = best;
    c->grab_ox = c->cx - pin[best].x;
    c->grab_oy = c->cy - pin[best].y;
    c->grab_oz = c->cz - pin[best].z;
    c->grabbed = 1;
    c->placed = 0;
    return 1;
}

#define LZ_AIM_PLACE 0.40f

void lz_cube_ray(float ax, float ay, float az, float dx, float dy, float dz, float dist, float *x, float *y,
                 float *z) {
    float len;
    if (!x || !y || !z) return;
    len = sqrtf(dx * dx + dy * dy + dz * dz);
    if (len < 1e-6f) {
        *x = ax;
        *y = ay;
        *z = az;
        return;
    }
    *x = ax + dx / len * dist;
    *y = ay + dy / len * dist;
    *z = az + dz / len * dist;
}

static void clamp_half(LzCube *c) {
    if (c->half < 0.04f) c->half = 0.04f;
    if (c->half > 0.25f) c->half = 0.25f;
}

int lz_cube_drive(LzCube *c, const LzPose *head, int head_ok, const LzCubeDrive *d) {
    int clear = 0;
    int aim;
    float px = 0.f, py = 0.f, pz = 0.f;
    if (!c || !d) return 0;
    if (d->bigger) c->half += 0.02f;
    if (d->smaller) c->half -= 0.02f;
    clamp_half(c);
    if (d->reset_scan) clear = 1;
    aim = d->aim_ok;
    if (!aim) {
        if (c->grabbed) {
            c->grabbed = 0;
            c->grab_hand = -1;
            c->placed = 1;
            clear = 1;
        } else if (!c->placed && !c->had_aim && head_ok && head) {
            pose_follow(c, head);
            c->ready = 1;
        }
    } else {
        lz_cube_ray(d->ax, d->ay, d->az, d->dx, d->dy, d->dz, LZ_AIM_PLACE, &px, &py, &pz);
        c->had_aim = 1;
        c->ready = 1;
        if (d->grip) {
            if (!c->grabbed) {
                c->grab_ox = c->cx - px;
                c->grab_oy = c->cy - py;
                c->grab_oz = c->cz - pz;
                c->grabbed = 1;
                c->placed = 0;
                c->grab_hand = -1;
            }
            c->cx = px + c->grab_ox;
            c->cy = py + c->grab_oy;
            c->cz = pz + c->grab_oz;
        } else if (c->grabbed) {
            c->grabbed = 0;
            c->grab_hand = -1;
            c->placed = 1;
            clear = 1;
        } else if (!c->placed) {
            c->cx = px;
            c->cy = py;
            c->cz = pz;
        }
    }
    if (d->lock_toggle) {
        if (c->grabbed) {
            c->grabbed = 0;
            c->grab_hand = -1;
        }
        if (c->placed) {
            c->placed = 0;
            c->settle_cool = LZ_CUBE_SETTLE;
            c->still_n = 0;
            if (aim) {
                c->cx = px;
                c->cy = py;
                c->cz = pz;
            } else if (!c->had_aim && head_ok && head) {
                pose_follow(c, head);
                c->ready = 1;
            }
        } else {
            c->placed = 1;
            c->ready = 1;
            c->still_n = 0;
            clear = 1;
        }
    }
    /* No controller: hold the head still and the cube locks where the user is looking. */
    if (c->settle_cool > 0) c->settle_cool--;
    if (aim || c->placed || c->grabbed || !head_ok || !head) {
        if (c->settle_cool < 1) c->still_n = 0;
    } else {
        float dx = head->px - c->still_px;
        float dy = head->py - c->still_py;
        float dz = head->pz - c->still_pz;
        float dist = sqrtf(dx * dx + dy * dy + dz * dz);
        float dot = head->qx * c->still_qx + head->qy * c->still_qy + head->qz * c->still_qz +
                    head->qw * c->still_qw;
        if (dot < 0.f) dot = -dot;
        if (c->still_n < 1 || dist > 0.015f || dot < 0.999f) {
            c->still_px = head->px;
            c->still_py = head->py;
            c->still_pz = head->pz;
            c->still_qx = head->qx;
            c->still_qy = head->qy;
            c->still_qz = head->qz;
            c->still_qw = head->qw;
            c->still_n = 1;
        } else if (c->still_n < LZ_CUBE_SETTLE) {
            c->still_n++;
            if (c->still_n >= LZ_CUBE_SETTLE) {
                c->placed = 1;
                c->ready = 1;
                c->grabbed = 0;
                c->still_n = 0;
                clear = 1;
            }
        }
    }
    return clear;
}

static void cube_rgb(const LzCube *c, float *r, float *g, float *b) {
    if (c && c->grabbed) {
        *r = 1.f;
        *g = 0.82f;
        *b = 0.15f;
        return;
    }
    if (c && c->placed) {
        *r = 0.20f;
        *g = 0.85f;
        *b = 1.f;
        return;
    }
    *r = 0.15f;
    *g = 1.f;
    *b = 0.45f;
}

static void emit(float *o, int *n, float x, float y, float z, float r, float g, float b) {
    float *p;
    if (*n >= LZ_CUBE_BEAM_VERTS) return;
    p = o + (*n) * 6;
    p[0] = x;
    p[1] = y;
    p[2] = z;
    p[3] = r;
    p[4] = g;
    p[5] = b;
    (*n)++;
}

static void emit_beam(float *o, int *n, float ax, float ay, float az, float bx, float by, float bz,
                      float r, float g, float b) {
    float dx = bx - ax, dy = by - ay, dz = bz - az;
    float len = sqrtf(dx * dx + dy * dy + dz * dz);
    float sx, sy, sz, ux, uy, uz, sl;
    float off[4][3];
    float va[4][3], vb[4][3];
    int i;
    if (len < 1e-6f) return;
    dx /= len;
    dy /= len;
    dz /= len;
    if (fabsf(dy) < 0.9f) {
        sx = -dz;
        sy = 0.f;
        sz = dx;
    } else {
        sx = 0.f;
        sy = dz;
        sz = -dy;
    }
    sl = sqrtf(sx * sx + sy * sy + sz * sz);
    if (sl < 1e-6f) return;
    sx /= sl;
    sy /= sl;
    sz /= sl;
    ux = sy * dz - sz * dy;
    uy = sz * dx - sx * dz;
    uz = sx * dy - sy * dx;
    off[0][0] = (sx + ux) * LZ_BEAM_HALF;
    off[0][1] = (sy + uy) * LZ_BEAM_HALF;
    off[0][2] = (sz + uz) * LZ_BEAM_HALF;
    off[1][0] = (sx - ux) * LZ_BEAM_HALF;
    off[1][1] = (sy - uy) * LZ_BEAM_HALF;
    off[1][2] = (sz - uz) * LZ_BEAM_HALF;
    off[2][0] = (-sx - ux) * LZ_BEAM_HALF;
    off[2][1] = (-sy - uy) * LZ_BEAM_HALF;
    off[2][2] = (-sz - uz) * LZ_BEAM_HALF;
    off[3][0] = (-sx + ux) * LZ_BEAM_HALF;
    off[3][1] = (-sy + uy) * LZ_BEAM_HALF;
    off[3][2] = (-sz + uz) * LZ_BEAM_HALF;
    for (i = 0; i < 4; i++) {
        va[i][0] = ax + off[i][0];
        va[i][1] = ay + off[i][1];
        va[i][2] = az + off[i][2];
        vb[i][0] = bx + off[i][0];
        vb[i][1] = by + off[i][1];
        vb[i][2] = bz + off[i][2];
    }
    for (i = 0; i < 4; i++) {
        int k = (i + 1) & 3;
        emit(o, n, va[i][0], va[i][1], va[i][2], r, g, b);
        emit(o, n, va[k][0], va[k][1], va[k][2], r, g, b);
        emit(o, n, vb[i][0], vb[i][1], vb[i][2], r, g, b);
        emit(o, n, va[k][0], va[k][1], va[k][2], r, g, b);
        emit(o, n, vb[k][0], vb[k][1], vb[k][2], r, g, b);
        emit(o, n, vb[i][0], vb[i][1], vb[i][2], r, g, b);
    }
}

int lz_cube_beams(const LzCube *c, float *out) {
    static const int e[12][2] = {
        {0, 1}, {2, 3}, {4, 5}, {6, 7},
        {0, 2}, {1, 3}, {4, 6}, {5, 7},
        {0, 4}, {1, 5}, {2, 6}, {3, 7},
    };
    float r, g, b, h;
    int i, n = 0;
    if (!c || !out || !(c->half > 0.f)) return 0;
    h = c->half;
    cube_rgb(c, &r, &g, &b);
    for (i = 0; i < 12; i++) {
        int a = e[i][0], bidx = e[i][1];
        float ax = (a & 1) ? h : -h;
        float ay = (a & 2) ? h : -h;
        float az = (a & 4) ? h : -h;
        float bx = (bidx & 1) ? h : -h;
        float by = (bidx & 2) ? h : -h;
        float bz = (bidx & 4) ? h : -h;
        emit_beam(out, &n, ax, ay, az, bx, by, bz, r, g, b);
    }
    return n;
}

int lz_cube_corners(const LzCube *c, float *out) {
    float r, g, b, h;
    int i;
    if (!c || !out || !(c->half > 0.f)) return 0;
    h = c->half;
    cube_rgb(c, &r, &g, &b);
    for (i = 0; i < 8; i++) {
        float *p = out + i * 6;
        p[0] = (i & 1) ? h : -h;
        p[1] = (i & 2) ? h : -h;
        p[2] = (i & 4) ? h : -h;
        p[3] = r;
        p[4] = g;
        p[5] = b;
    }
    return 8;
}

void lz_cube_status(const LzCube *c, const LzJoint *hands, int nhands, char *buf, int cap) {
    const char *st = "wait";
    int found = -1, i;
    char tag = '-';
    float hx = 0.f, hy = 0.f, hz = 0.f;
    float cx = 0.f, cy = 0.f, cz = 0.f;
    if (!buf || cap < 8) return;
    if (c) {
        cx = c->cx;
        cy = c->cy;
        cz = c->cz;
        if (c->ready) st = c->grabbed ? "G" : c->placed ? "L" : "F";
    }
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
    if (found >= 0 && c && c->ready) {
        tag = found >= 26 ? 'R' : 'L';
        lz_cube_to_local(c, hands[found].x, hands[found].y, hands[found].z, &hx, &hy, &hz);
        snprintf(buf, (size_t)cap, "cube %s %.2f,%.2f,%.2f h%c %.2f,%.2f,%.2f", st, cx, cy, cz, tag, hx, hy, hz);
        return;
    }
    snprintf(buf, (size_t)cap, "cube %s %.2f,%.2f,%.2f h-", st, cx, cy, cz);
}
