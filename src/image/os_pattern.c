#include "openscan/openscan.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Same column count the Quest scanner uses. A row of the projector lattice
 * has 12 or more peaks. Full-range gray is required: studio swing shrinks
 * the high-pass until this count falls under the stripe gate. */

static int column_score(const uint8_t *y, int w, int h) {
    int counts[48];
    int nrows = 0, r, i, acc, step;
    int *hp_at;
    if (!y || w < 32 || h < 32 || w > 1920) return 0;
    hp_at = (int *)malloc((size_t)w * sizeof(int));
    if (!hp_at) return 0;
    step = h / 40 > 0 ? h / 40 : 1;
    for (r = h / 8; r < (h * 7) / 8 && nrows < 48; r += step) {
        const uint8_t *row = y + (size_t)r * (size_t)w;
        int np = 0, last = -100, x;
        for (x = 0; x < w; x++) hp_at[x] = 0;
        for (x = 8; x < w - 8; x++) {
            int t, s1 = 0, s6 = 0;
            for (t = x - 1; t <= x + 1; t++) s1 += row[t];
            for (t = x - 6; t <= x + 6; t++) s6 += row[t];
            hp_at[x] = s1 / 3 - s6 / 13;
        }
        for (x = 8; x < w - 8; x++) {
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
    free(hp_at);
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

static void level_clip(const uint8_t *y, int w, int h, int *clip, int *p50, int *p95) {
    int hist[256];
    int row, col, n = 0, i, acc = 0, hi = 0, got50 = 0;
    memset(hist, 0, sizeof hist);
    *clip = 0;
    *p50 = 0;
    *p95 = 0;
    for (row = 0; row < h; row += 4) {
        const uint8_t *p = y + (size_t)row * (size_t)w;
        for (col = 0; col < w; col += 4) {
            hist[p[col]]++;
            n++;
        }
    }
    if (!n) return;
    for (i = 230; i < 256; i++) hi += hist[i];
    *clip = hi * 100 / n;
    acc = 0;
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

void openscan_pattern_meter(const uint8_t *y, int w, int h, int *cols, int *clip, int *p50, int *p95) {
    int c = 0, k = 0, mid = 0, hi = 0;
    if (cols) *cols = 0;
    if (clip) *clip = 0;
    if (p50) *p50 = 0;
    if (p95) *p95 = 0;
    if (!y || w < 32 || h < 32) return;
    c = column_score(y, w, h);
    level_clip(y, w, h, &k, &mid, &hi);
    if (cols) *cols = c;
    if (clip) *clip = k;
    if (p50) *p50 = mid;
    if (p95) *p95 = hi;
}

int openscan_pattern_rescue(int cols, int clip, int p95, int exposure, int *next) {
    int washed, dark, n;
    if (!next) return 0;
    if (p95 < 0) p95 = 0;
    if (p95 > 255) p95 = 255;
    /* Black borders hide a blown skull: clip stays under 18 while p95 is 250
     * and the 13 px lattice is flattened to a one-count ripple. */
    washed = (clip >= 8 || p95 >= 210);
    dark = (p95 < 150 && clip < 8);
    if (cols >= 12 && !washed) return 0;
    if (washed && exposure > 1) {
        /* Coarse while obviously blown, then one count so a sharp lattice
         * between two exposures is not stepped over. */
        n = exposure > 10 ? exposure - 2 : exposure - 1;
        if (n < 1) n = 1;
        if (n == exposure) return 0;
        *next = n;
        return 1;
    }
    if (cols < 12 && dark && exposure < 24) {
        n = exposure < 8 ? exposure + 2 : exposure + 1;
        if (n > 24) n = 24;
        if (n == exposure) return 0;
        *next = n;
        return 1;
    }
    return 0;
}

int openscan_pattern_self_test(void) {
    enum { W = 220, H = 80 };
    uint8_t *y = (uint8_t *)malloc((size_t)W * H);
    int cols = 0, clip = 0, p50 = 0, p95 = 0, next = 0, x, row, center;
    if (!y) return 1;
    memset(y, 25, (size_t)W * H);
    openscan_pattern_meter(y, W, H, &cols, &clip, &p50, &p95);
    if (cols != 0 || p95 > 40) {
        fprintf(stderr, "pattern: flat frame reported %d columns p95 %d\n", cols, p95);
        free(y);
        return 1;
    }
    for (center = 20; center < W - 20; center += 14) {
        for (row = 0; row < H; row++) {
            for (x = center - 2; x <= center + 2; x++) y[row * W + x] = 220;
        }
    }
    openscan_pattern_meter(y, W, H, &cols, &clip, &p50, &p95);
    free(y);
    if (cols < 12) {
        fprintf(stderr, "pattern: lattice reported %d columns\n", cols);
        return 1;
    }
    /* Dark frame lengthens. Washed frame shortens even when clip percent is
     * diluted by empty borders. A locked lattice is left alone. */
    if (openscan_pattern_rescue(0, 0, 20, 10, &next) != 1 || next != 11) return 1;
    if (openscan_pattern_rescue(20, 30, 250, 12, &next) != 1 || next != 10) return 1;
    if (openscan_pattern_rescue(1, 15, 249, 14, &next) != 1 || next != 12) return 1;
    if (openscan_pattern_rescue(20, 5, 160, 10, &next) != 0) return 1;
    if (openscan_pattern_rescue(4, 0, 40, 18, &next) != 1 || next != 19) return 1;
    if (openscan_pattern_rescue(4, 0, 40, 24, &next) != 0) return 1;
    return 0;
}
