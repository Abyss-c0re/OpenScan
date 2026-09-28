#include "openscan/openscan_calib.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

static int read_doubles(FILE *f, double *dst, int n) {
    for (int i = 0; i < n; i++) {
        if (fscanf(f, "%lf", &dst[i]) != 1) return -1;
    }
    return 0;
}

int openscan_calib_load(const char *path, openscan_calib *out) {
    memset(out, 0, sizeof *out);
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    if (fscanf(f, "%d %d", &out->width, &out->height) != 2) {
        fclose(f);
        return -1;
    }
    double skip[4];
    if (read_doubles(f, skip, 2) < 0) { fclose(f); return -1; }
    int iw, ih, code, off;
    if (fscanf(f, "%d %d", &iw, &ih) != 2) { fclose(f); return -1; }
    if (fscanf(f, "%d %d", &code, &off) != 2) { fclose(f); return -1; }
    (void)iw; (void)ih; (void)code; (void)off;

    for (int c = 0; c < 3; c++) {
        double v[15];
        if (read_doubles(f, v, 15) < 0) { fclose(f); return -1; }
        openscan_pinhole *p = &out->cam[c];
        p->fx = v[0]; p->fy = v[1]; p->cx = v[2]; p->cy = v[3];
        p->k1 = v[4]; p->k2 = v[5]; p->p1 = v[6]; p->p2 = v[7]; p->k3 = v[8];
        p->rvec[0] = v[9]; p->rvec[1] = v[10]; p->rvec[2] = v[11];
        p->tvec[0] = v[12]; p->tvec[1] = v[13]; p->tvec[2] = v[14];
    }

    /* 130 light planes, then the spatial-code table. Planes are what the stripe decoder
     * intersects with each decoded stripe. The code table is not needed to
     * cut the plane. */
    int np = 0;
    if (fscanf(f, "%d", &np) == 1 && np > 0) {
        if (np > OPENSCAN_MAX_PLANES) np = OPENSCAN_MAX_PLANES;
        for (int i = 0; i < np; i++) {
            int index = 0;
            double slope = 0, offset = 0;
            if (fscanf(f, "%d %lf %lf", &index, &slope, &offset) != 3) {
                np = i;
                break;
            }
            out->plane[i].index = index;
            out->plane[i].slope = slope;
            out->plane[i].offset = offset;
        }
        out->nplanes = np;
    }

    char line[512];
    while (fgets(line, sizeof line, f)) {
        char *dev = strstr(line, "DevID:");
        if (!dev) continue;
        dev += 6;
        size_t i = 0;
        while (dev[i] && dev[i] != '*' && i + 1 < sizeof out->serial) {
            out->serial[i] = dev[i];
            i++;
        }
        out->serial[i] = 0;
        char *date = strstr(line, "CalibrateDate:");
        if (date) {
            date += 14;
            i = 0;
            while (date[i] && date[i] != '*' && i + 1 < sizeof out->date) {
                out->date[i] = date[i];
                i++;
            }
            out->date[i] = 0;
        }
        break;
    }
    fclose(f);
    if (out->cam[0].fx < 100.0 || out->cam[1].fx < 100.0) return -1;
    return 0;
}

int openscan_calib_load_for(const char *path, const char *serial, openscan_calib *out) {
    if (!out) return -1;
    memset(out, 0, sizeof *out);
    if (!serial || !serial[0]) return -1;
    if (openscan_calib_load(path, out) != 0 || strcmp(out->serial, serial) != 0) {
        memset(out, 0, sizeof *out);
        return -1;
    }
    return 0;
}

#ifdef OPENSCAN_CALIB_SERIAL_TEST
int main(int argc, char **argv) {
    if (argc != 3) return 9;
    openscan_calib good, bad;
    if (openscan_calib_load_for(argv[1], "EXAMPLE01", &good) != 0) return 1;
    if (strcmp(good.serial, "EXAMPLE01") != 0) return 2;
    if (openscan_calib_load_for(argv[1], "JMM0000000", &good) == 0) return 3;
    if (good.serial[0] != 0) return 4;
    if (openscan_calib_load_for(argv[2], "EXAMPLE01", &bad) == 0) return 5;
    if (bad.serial[0] != 0) return 6;
    if (openscan_calib_load_for(argv[2], "JMM0000000", &bad) != 0) return 7;
    if (strcmp(bad.serial, "JMM0000000") != 0) return 8;
    return 0;
}
#endif
