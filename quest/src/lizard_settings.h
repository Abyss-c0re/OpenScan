#ifndef LIZARD_SETTINGS_H
#define LIZARD_SETTINGS_H

/* Scanner knobs from the desktop window and the Android settings screen.
 * Exposure and gain are the seekbar units (exposure is 100 µs, so 22 is 2.2 ms).
 * Near, far, stride, smooth, sweep, relief, solid, and flip go through
 * openscan_build_clamped. Distance stays on 100..500.
 * Sweep below 60 is raised: the desktop slider and the engine both keep 60..140.
 * The Android seekbar draws 20, then the engine stores 60.
 * Smooth, sweep, and relief are stored with those ranges. The stripe measurement
 * does not read them, and the desktop stripe path does not either. */

enum {
    LZ_SET_EA = 0,
    LZ_SET_GA,
    LZ_SET_EB,
    LZ_SET_GB,
    LZ_SET_DIST,
    LZ_SET_NEAR,
    LZ_SET_FAR,
    LZ_SET_STRIDE,
    LZ_SET_SMOOTH,
    LZ_SET_SWEEP,
    LZ_SET_RELIEF,
    LZ_SET_COUNT
};

typedef struct {
    int ea, ga, eb, gb;
    int distance;
    int near_mm, far_mm;
    int stride, smooth, sweep, relief;
    int solid, flip;
} LzSettings;

void lz_settings_default(LzSettings *s);
void lz_settings_clamp(LzSettings *s);

/* dir is -1 or +1. The result is clamped, including far >= near + 40. */
int lz_settings_nudge(LzSettings *s, int field, int dir);

const char *lz_settings_key(int field);

/* 0 on success. A missing file leaves *s alone and returns -1. */
int lz_settings_load(const char *path, LzSettings *s);
int lz_settings_save(const char *path, const LzSettings *s);

#endif
