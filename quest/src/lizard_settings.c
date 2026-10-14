#include "lizard_settings.h"

#include "openscan/openscan_live.h"
#include "openscan/openscan_v4l2.h"

#include <stdio.h>
#include <string.h>

void lz_settings_default(LzSettings *s) {
    openscan_build b = openscan_default_build();
    if (!s) return;
    s->ea = openscan_default_exposure_a();
    s->ga = openscan_default_gain_a();
    s->eb = openscan_default_exposure_b();
    s->gb = openscan_default_gain_b();
    s->distance = openscan_default_distance_mm();
    s->near_mm = (int)b.near_mm;
    s->far_mm = (int)b.far_mm;
    s->stride = b.stride;
    s->smooth = b.smooth;
    s->sweep = b.sweep_deg;
    s->relief = b.relief;
    s->solid = b.solid ? 1 : 0;
    s->flip = b.flip ? 1 : 0;
}

static int clamp_int(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void lz_settings_clamp(LzSettings *s) {
    openscan_build in, b;
    if (!s) return;
    s->ea = clamp_int(s->ea, 1, 200);
    s->eb = clamp_int(s->eb, 1, 200);
    s->ga = clamp_int(s->ga, 0, 100);
    s->gb = clamp_int(s->gb, 0, 100);
    s->distance = (int)openscan_distance_clamped((float)s->distance);
    memset(&in, 0, sizeof in);
    in.near_mm = (float)s->near_mm;
    in.far_mm = (float)s->far_mm;
    in.stride = s->stride;
    in.smooth = s->smooth;
    in.sweep_deg = s->sweep;
    in.relief = s->relief;
    in.solid = s->solid;
    in.flip = s->flip;
    b = openscan_build_clamped(&in);
    s->near_mm = (int)b.near_mm;
    s->far_mm = (int)b.far_mm;
    s->stride = b.stride;
    s->smooth = b.smooth;
    s->sweep = b.sweep_deg;
    s->relief = b.relief;
    s->solid = b.solid ? 1 : 0;
    s->flip = b.flip ? 1 : 0;
}

int lz_settings_nudge(LzSettings *s, int field, int dir) {
    if (!s || dir == 0 || field < 0 || field >= LZ_SET_COUNT) return 0;
    if (dir > 1) dir = 1;
    if (dir < -1) dir = -1;
    switch (field) {
    case LZ_SET_EA: s->ea += dir; break;
    case LZ_SET_GA: s->ga += dir; break;
    case LZ_SET_EB: s->eb += dir; break;
    case LZ_SET_GB: s->gb += dir; break;
    case LZ_SET_DIST: s->distance += dir; break;
    case LZ_SET_NEAR: s->near_mm += dir; break;
    case LZ_SET_FAR: s->far_mm += dir; break;
    case LZ_SET_STRIDE: s->stride += dir; break;
    case LZ_SET_SMOOTH: s->smooth += dir; break;
    case LZ_SET_SWEEP: s->sweep += dir; break;
    case LZ_SET_RELIEF: s->relief += dir; break;
    default: return 0;
    }
    lz_settings_clamp(s);
    return 1;
}

const char *lz_settings_key(int field) {
    static const char *keys[LZ_SET_COUNT] = {"ea",  "ga",     "eb",    "gb",     "distance", "near",
                                             "far", "stride", "smooth", "sweep",  "relief"};
    if (field < 0 || field >= LZ_SET_COUNT) return "?";
    return keys[field];
}

static void apply_key(LzSettings *s, const char *key, int v) {
    if (!strcmp(key, "ea")) s->ea = v;
    else if (!strcmp(key, "ga")) s->ga = v;
    else if (!strcmp(key, "eb")) s->eb = v;
    else if (!strcmp(key, "gb")) s->gb = v;
    else if (!strcmp(key, "distance")) s->distance = v;
    else if (!strcmp(key, "near")) s->near_mm = v;
    else if (!strcmp(key, "far")) s->far_mm = v;
    else if (!strcmp(key, "stride")) s->stride = v;
    else if (!strcmp(key, "smooth")) s->smooth = v;
    else if (!strcmp(key, "sweep")) s->sweep = v;
    else if (!strcmp(key, "relief")) s->relief = v;
    else if (!strcmp(key, "solid")) s->solid = v;
    else if (!strcmp(key, "flip")) s->flip = v;
}

int lz_settings_load(const char *path, LzSettings *s) {
    FILE *f;
    char key[32];
    int v;
    LzSettings c;
    if (!path || !s) return -1;
    f = fopen(path, "r");
    if (!f) return -1;
    c = *s;
    while (fscanf(f, "%31s %d", key, &v) == 2) apply_key(&c, key, v);
    fclose(f);
    lz_settings_clamp(&c);
    *s = c;
    return 0;
}

int lz_settings_save(const char *path, const LzSettings *s) {
    FILE *f;
    LzSettings c;
    if (!path || !s) return -1;
    c = *s;
    lz_settings_clamp(&c);
    f = fopen(path, "w");
    if (!f) return -1;
    fprintf(f, "ea %d\nga %d\neb %d\ngb %d\n", c.ea, c.ga, c.eb, c.gb);
    fprintf(f, "distance %d\nnear %d\nfar %d\nstride %d\nsmooth %d\n", c.distance, c.near_mm, c.far_mm,
            c.stride, c.smooth);
    fprintf(f, "sweep %d\nrelief %d\nsolid %d\nflip %d\n", c.sweep, c.relief, c.solid, c.flip);
    if (fclose(f) != 0) return -1;
    return 0;
}
