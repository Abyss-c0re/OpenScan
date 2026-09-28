#include "openscan/openscan_calib.h"
#include "openscan/openscan_live.h"
#include "openscan/openscan_v4l2.h"
#include "ui/os_ui.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static void usage(const char *argv0) {
    fprintf(stderr,
            "OpenScan " OPENSCAN_VERSION "\n"
            "Not affiliated with any scanner maker. Bring your own calibration file.\n"
            "\n"
            "  %s                 window\n"
            "  %s devices\n"
            "  %s grab -o DIR\n"
            "  %s scan --no-window -o FILE.stl --seconds N\n"
            "  %s turn-test\n",
            argv0, argv0, argv0, argv0, argv0);
}

static int arg_int(int argc, char **argv, const char *name, int fallback) {
    for (int i = 1; i < argc - 1; i++)
        if (!strcmp(argv[i], name)) return atoi(argv[i + 1]);
    return fallback;
}

static const char *arg_str(int argc, char **argv, const char *name) {
    for (int i = 1; i < argc - 1; i++)
        if (!strcmp(argv[i], name)) return argv[i + 1];
    return NULL;
}

static int cmd_devices(void) {
    char a[512], b[512], sn[64];
    int rc = openscan_find_cameras(a, b, sizeof a, sn, sizeof sn);
    if (rc != 0) {
        fprintf(stderr, "no scanner pair (%d)\n", rc);
        return 1;
    }
    printf("%s\n%s\n%s\n", sn, a, b);
    return 0;
}

static int open_pair(openscan_calib *cal, int *have, openscan_cam **ca, openscan_cam **cb,
                     int ea, int ga, int eb, int gb) {
    char pa[1024], pb[1024], sn[64];
    *have = 0;
    memset(cal, 0, sizeof *cal);
    if (openscan_find_cameras(pa, pb, sizeof pa, sn, sizeof sn) != 0) return -1;
    char used[512];
    if (openscan_calib_ensure(sn, NULL, cal, used, sizeof used) == 0) *have = 1;
    int w = *have ? cal->width : 1280;
    int h = *have ? cal->height : 720;
    *ca = openscan_cam_open(pa, w, h, 10, 1);
    *cb = openscan_cam_open(pb, w, h, 10, 1);
    if (!*ca || !*cb) return -1;
    openscan_cam_set_exposure(*ca, ea, ga);
    openscan_cam_set_exposure(*cb, eb, gb);
    openscan_cam_start(*ca);
    openscan_cam_start(*cb);
    return 0;
}

static int cmd_grab(const char *dir, int ea, int ga, int eb, int gb) {
    openscan_calib cal;
    int have = 0;
    openscan_cam *a = NULL, *b = NULL;
    if (open_pair(&cal, &have, &a, &b, ea, ga, eb, gb) != 0) return 1;
    int w = openscan_cam_width(a), h = openscan_cam_height(a);
    uint8_t *store = malloc((size_t)w * h * 4);
    uint8_t *y = malloc((size_t)w * h);
    char path[512];
    snprintf(path, sizeof path, "%s/camB.pgm", dir);
    if (os_grab_gray(b, store, (size_t)w * h * 4, y) == 0) {
        FILE *f = fopen(path, "wb");
        if (f) {
            fprintf(f, "P5\n%d %d\n255\n", w, h);
            fwrite(y, 1, (size_t)w * h, f);
            fclose(f);
            printf("%s\n", path);
        }
    }
    snprintf(path, sizeof path, "%s/camA.pgm", dir);
    if (os_grab_gray(a, store, (size_t)w * h * 4, y) == 0) {
        FILE *f = fopen(path, "wb");
        if (f) {
            fprintf(f, "P5\n%d %d\n255\n", w, h);
            fwrite(y, 1, (size_t)w * h, f);
            fclose(f);
            printf("%s\n", path);
        }
    }
    openscan_cam_stop(a); openscan_cam_stop(b);
    openscan_cam_close(a); openscan_cam_close(b);
    free(store); free(y);
    return 0;
}

static int cmd_scan(const char *stl, int seconds, int ea, int ga, int eb, int gb) {
    openscan_calib cal;
    int have = 0;
    openscan_cam *a = NULL, *b = NULL;
    if (open_pair(&cal, &have, &a, &b, ea, ga, eb, gb) != 0) return 1;
    int w = openscan_cam_width(a), h = openscan_cam_height(a);
    uint8_t *sa = malloc((size_t)w * h * 4);
    uint8_t *sb = malloc((size_t)w * h * 4);
    uint8_t *ya = malloc((size_t)w * h);
    uint8_t *yb = malloc((size_t)w * h);
    openscan_live *live = openscan_live_create(have ? &cal : NULL, NULL);
    openscan_live_set_mode(live, OPENSCAN_MODE_SCAN);
    time_t t0 = time(NULL);
    while (time(NULL) - t0 < seconds) {
        if (os_grab_gray(a, sa, (size_t)w * h * 4, ya) != 0) continue;
        if (os_grab_gray(b, sb, (size_t)w * h * 4, yb) != 0) continue;
        openscan_live_status st;
        openscan_live_push(live, ya, yb, w, h, &st);
        fprintf(stderr, "tris %d  %d deg\n", st.points, st.scanned_deg);
    }
    int tris = 0;
    int rc = openscan_live_write(live, stl ? stl : "openscan.stl", &tris);
    fprintf(stderr, "wrote %s %d\n", stl ? stl : "openscan.stl", tris);
    openscan_live_destroy(live);
    openscan_cam_stop(a); openscan_cam_stop(b);
    openscan_cam_close(a); openscan_cam_close(b);
    free(sa); free(sb); free(ya); free(yb);
    return rc == 0 ? 0 : 1;
}

int main(int argc, char **argv) {
    if (argc >= 2 && (!strcmp(argv[1], "--version") || !strcmp(argv[1], "version"))) {
        printf("OpenScan %s (%s)\n", OPENSCAN_VERSION, OPENSCAN_HOST);
        return 0;
    }
    if (argc >= 2 && (!strcmp(argv[1], "-h") || !strcmp(argv[1], "--help"))) {
        usage(argv[0]);
        return 0;
    }
    if (argc >= 2 && !strcmp(argv[1], "turn-test")) return openscan_turn_self_test();
    if (argc >= 2 && !strcmp(argv[1], "mesh-test")) return openscan_mesh_self_test();
    if (argc >= 2 && !strcmp(argv[1], "devices")) return cmd_devices();
    int ea = arg_int(argc, argv, "--exposure-a", openscan_default_exposure_a());
    int ga = arg_int(argc, argv, "--gain-a", openscan_default_gain_a());
    int eb = arg_int(argc, argv, "--exposure-b", openscan_default_exposure_b());
    int gb = arg_int(argc, argv, "--gain-b", openscan_default_gain_b());
    if (argc >= 2 && !strcmp(argv[1], "grab")) {
        const char *dir = arg_str(argc, argv, "-o");
        if (!dir) dir = "grab";
        return cmd_grab(dir, ea, ga, eb, gb);
    }
    if (argc >= 2 && !strcmp(argv[1], "scan")) {
        int seconds = arg_int(argc, argv, "--seconds", 4);
        const char *out = arg_str(argc, argv, "-o");
        int window = 1;
        for (int i = 1; i < argc; i++)
            if (!strcmp(argv[i], "--no-window")) window = 0;
        if (!window) return cmd_scan(out, seconds, ea, ga, eb, gb);
    }
    openscan_calib cal;
    char pa[1024], pb[1024], sn[64], used[512];
    const openscan_calib *use = NULL;
    if (openscan_find_cameras(pa, pb, sizeof pa, sn, sizeof sn) == 0 &&
        openscan_calib_ensure(sn, NULL, &cal, used, sizeof used) == 0)
        use = &cal;
    return os_ui_main(use, ea, ga, eb, gb);
}
