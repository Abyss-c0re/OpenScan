#include "openscan/openscan.h"
#include "ui/os_ui.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#ifdef _WIN32
#include <direct.h>
#else
#include <sys/stat.h>
#endif

static void usage(const char *argv0) {
    fprintf(stderr,
            "OpenScan %s (%s)\n"
            "One object, two cameras. Bring your own calibration file.\n"
            "\n"
            "  %s                         open the window\n"
            "  %s devices                  print the camera pair\n"
            "  %s grab -o DIR              save camA.pgm and camB.pgm\n"
            "  %s scan -o FILE.stl|.obj|.ply   record a solid, no window\n"
            "  %s turn-test                check that a turn grows the solid\n"
            "  %s help                     this text\n"
            "\n"
            "scan and the window accept:\n"
            "  --seconds N                 scan length, default 4, at least 1\n"
            "  --shape mold|measured       mold is the solid, measured is the sheet\n"
            "  --distance MM               100..500, default 220\n"
            "  --sweep DEG                 60..140, default 80 (160 degrees of the object)\n"
            "  --exposure-a N  --gain-a N  pattern camera, exposure is 100 microsecond steps\n"
            "  --exposure-b N  --gain-b N  clean camera\n"
            "\n"
            "Another program links libopenscan and includes openscan/openscan.h.\n",
            openscan_version(), openscan_host(),
            argv0, argv0, argv0, argv0, argv0, argv0);
}

static const char *arg_str(int argc, char **argv, const char *name) {
    int i;
    for (i = 1; i < argc - 1; i++)
        if (!strcmp(argv[i], name)) return argv[i + 1];
    return NULL;
}

static int arg_present(int argc, char **argv, const char *name) {
    return arg_str(argc, argv, name) != NULL;
}

static int parse_int(const char *text, int *out) {
    char *end = NULL;
    long v;
    if (!text || !text[0]) return -1;
    v = strtol(text, &end, 10);
    if (!end || *end) return -1;
    *out = (int)v;
    return 0;
}

static int opt_int(int argc, char **argv, const char *name, int fallback, int *out) {
    const char *text = arg_str(argc, argv, name);
    if (!text) {
        *out = fallback;
        return 0;
    }
    if (parse_int(text, out) != 0) {
        fprintf(stderr, "%s needs a whole number, not \"%s\".\n", name, text);
        return -1;
    }
    return 0;
}

static int ensure_dir(const char *dir) {
#ifdef _WIN32
    if (_mkdir(dir) != 0 && errno != EEXIST) return -1;
#else
    if (mkdir(dir, 0755) != 0 && errno != EEXIST) return -1;
#endif
    return 0;
}

static int cmd_devices(void) {
    char a[1024], b[1024], sn[64];
    int rc = openscan_find_cameras(a, b, sizeof a, sn, sizeof sn);
    if (rc != 0) {
        fprintf(stderr, "No scanner pair. Plug in both cameras and try again.\n");
        return 1;
    }
    if (sn[0]) printf("serial  %s\n", sn);
    printf("camera A  %s\n", a);
    printf("camera B  %s\n", b);
    return 0;
}

static int open_pair(openscan_calib *cal, int *have, openscan_cam **ca, openscan_cam **cb,
                     int ea, int ga, int eb, int gb) {
    char pa[1024], pb[1024], sn[64], used[512];
    int w, h;
    *have = 0;
    *ca = NULL;
    *cb = NULL;
    memset(cal, 0, sizeof *cal);
    if (openscan_find_cameras(pa, pb, sizeof pa, sn, sizeof sn) != 0) {
        fprintf(stderr, "No scanner cameras. Plug in the scanner and run: openscan devices\n");
        return -1;
    }
    if (openscan_calib_ensure(sn, NULL, cal, used, sizeof used) == 0) *have = 1;
    w = *have ? cal->width : 1280;
    h = *have ? cal->height : 720;
    *ca = openscan_cam_open(pa, w, h, 10, 1);
    *cb = openscan_cam_open(pb, w, h, 10, 1);
    if (!*ca || !*cb) {
        fprintf(stderr, "A camera did not open.\n");
        openscan_cam_close(*ca);
        openscan_cam_close(*cb);
        return -1;
    }
    if (ea < 0) ea = openscan_default_exposure_a();
    if (ga < 0) ga = openscan_default_gain_a();
    if (eb < 0) eb = openscan_default_exposure_b();
    if (gb < 0) gb = openscan_default_gain_b();
    openscan_cam_set_exposure(*ca, ea, ga);
    openscan_cam_set_exposure(*cb, eb, gb);
    if (openscan_cam_start(*ca) != 0 || openscan_cam_start(*cb) != 0) {
        fprintf(stderr, "A camera did not start.\n");
        openscan_cam_close(*ca);
        openscan_cam_close(*cb);
        return -1;
    }
    return 0;
}

static int write_pgm(const char *path, const uint8_t *gray, int w, int h) {
    FILE *f = fopen(path, "wb");
    if (!f) {
        fprintf(stderr, "Could not write %s\n", path);
        return -1;
    }
    fprintf(f, "P5\n%d %d\n255\n", w, h);
    if (fwrite(gray, 1, (size_t)w * (size_t)h, f) != (size_t)w * (size_t)h) {
        fclose(f);
        fprintf(stderr, "Could not finish %s\n", path);
        return -1;
    }
    fclose(f);
    printf("%s\n", path);
    return 0;
}

static int cmd_grab(const char *dir, int ea, int ga, int eb, int gb) {
    openscan_calib cal;
    openscan_cam *a = NULL, *b = NULL;
    int have = 0, w, h, rc = 1;
    uint8_t *store = NULL, *y = NULL;
    char path[512];
    if (ensure_dir(dir) != 0) {
        fprintf(stderr, "Could not create %s\n", dir);
        return 1;
    }
    if (open_pair(&cal, &have, &a, &b, ea, ga, eb, gb) != 0) return 1;
    w = openscan_cam_width(a);
    h = openscan_cam_height(a);
    store = malloc((size_t)w * (size_t)h * 4);
    y = malloc((size_t)w * (size_t)h);
    if (!store || !y) {
        fprintf(stderr, "Out of memory.\n");
        goto done;
    }
    snprintf(path, sizeof path, "%s/camB.pgm", dir);
    if (openscan_frame_gray(b, store, (size_t)w * h * 4, y) == 0) write_pgm(path, y, w, h);
    snprintf(path, sizeof path, "%s/camA.pgm", dir);
    if (openscan_frame_gray(a, store, (size_t)w * h * 4, y) == 0) write_pgm(path, y, w, h);
    rc = 0;
done:
    openscan_cam_stop(a);
    openscan_cam_stop(b);
    openscan_cam_close(a);
    openscan_cam_close(b);
    free(store);
    free(y);
    return rc;
}

static void apply_scan_knobs(openscan_live *live, int distance, int sweep, int shape) {
    openscan_build b;
    openscan_live_get_build(live, &b);
    if (sweep > 0) {
        if (sweep < 60) sweep = 60;
        if (sweep > 140) sweep = 140;
        b.sweep_deg = sweep;
    }
    openscan_live_set_build(live, &b);
    if (distance > 0) openscan_live_set_distance_mm(live, (float)distance);
    if (shape >= 0) openscan_live_set_shape(live, shape);
}

static int cmd_scan(const char *stl, int seconds, int ea, int ga, int eb, int gb,
                    int distance, int sweep, int shape) {
    openscan_calib cal;
    openscan_cam *a = NULL, *b = NULL;
    openscan_live *live = NULL;
    int have = 0, w, h, tris = 0, rc = 1;
    uint8_t *sa = NULL, *sb = NULL, *ya = NULL, *yb = NULL;
    time_t t0;
    const char *out = stl ? stl : "openscan.stl";
    if (seconds < 1) {
        fprintf(stderr, "--seconds must be at least 1.\n");
        return 1;
    }
    if (open_pair(&cal, &have, &a, &b, ea, ga, eb, gb) != 0) return 1;
    w = openscan_cam_width(a);
    h = openscan_cam_height(a);
    sa = malloc((size_t)w * h * 4);
    sb = malloc((size_t)w * h * 4);
    ya = malloc((size_t)w * h);
    yb = malloc((size_t)w * h);
    live = openscan_live_create(have ? &cal : NULL, NULL);
    if (!sa || !sb || !ya || !yb || !live) {
        fprintf(stderr, "Out of memory.\n");
        goto done;
    }
    apply_scan_knobs(live, distance, sweep, shape);
    openscan_live_set_mode(live, OPENSCAN_MODE_SCAN);
    t0 = time(NULL);
    while (time(NULL) - t0 < seconds) {
        openscan_live_status st;
        if (openscan_frame_gray(a, sa, (size_t)w * h * 4, ya) != 0) continue;
        if (openscan_frame_gray(b, sb, (size_t)w * h * 4, yb) != 0) continue;
        if (openscan_live_push(live, ya, yb, w, h, &st) != 0) continue;
        fprintf(stderr, "%d triangles, %d degrees\n", st.points, st.scanned_deg);
    }
    if (openscan_live_write(live, out, &tris) != 0) {
        fprintf(stderr, "No solid to write. The object needs to be in frame.\n");
        goto done;
    }
    fprintf(stderr, "Saved %s (%d triangles)\n", out, tris);
    rc = 0;
done:
    openscan_live_destroy(live);
    if (a) openscan_cam_stop(a);
    if (b) openscan_cam_stop(b);
    openscan_cam_close(a);
    openscan_cam_close(b);
    free(sa);
    free(sb);
    free(ya);
    free(yb);
    return rc;
}

static int shape_arg(int argc, char **argv, int *shape) {
    const char *s = arg_str(argc, argv, "--shape");
    *shape = -1;
    if (!s) return 0;
    if (!strcmp(s, "mold")) {
        *shape = OPENSCAN_SHAPE_MOLD;
        return 0;
    }
    if (!strcmp(s, "measured")) {
        *shape = OPENSCAN_SHAPE_MEASURED;
        return 0;
    }
    fprintf(stderr, "--shape is mold or measured, not \"%s\".\n", s);
    return -1;
}

int main(int argc, char **argv) {
    const char *cmd;
    int ea = -1, ga = -1, eb = -1, gb = -1, seconds = 4, distance = 0, sweep = 0, shape = -1;
    if (argc >= 2 && (!strcmp(argv[1], "--version") || !strcmp(argv[1], "version"))) {
        printf("OpenScan %s (%s)\n", openscan_version(), openscan_host());
        return 0;
    }
    if (argc >= 2 && (!strcmp(argv[1], "-h") || !strcmp(argv[1], "--help") || !strcmp(argv[1], "help"))) {
        usage(argv[0]);
        return 0;
    }
    if (argc >= 2 && !strcmp(argv[1], "turn-test")) return openscan_turn_self_test();
    if (argc >= 2 && !strcmp(argv[1], "mesh-test")) return openscan_mesh_self_test();
    if (opt_int(argc, argv, "--exposure-a", -1, &ea) || opt_int(argc, argv, "--gain-a", -1, &ga) ||
        opt_int(argc, argv, "--exposure-b", -1, &eb) || opt_int(argc, argv, "--gain-b", -1, &gb) ||
        opt_int(argc, argv, "--seconds", 4, &seconds) || opt_int(argc, argv, "--distance", 0, &distance) ||
        opt_int(argc, argv, "--sweep", 0, &sweep) || shape_arg(argc, argv, &shape))
        return 2;
    if (!arg_present(argc, argv, "--exposure-a")) ea = -1;
    if (!arg_present(argc, argv, "--gain-a")) ga = -1;
    if (!arg_present(argc, argv, "--exposure-b")) eb = -1;
    if (!arg_present(argc, argv, "--gain-b")) gb = -1;
    cmd = argc >= 2 ? argv[1] : "window";
    if (!strcmp(cmd, "devices")) return cmd_devices();
    if (!strcmp(cmd, "grab")) {
        const char *dir = arg_str(argc, argv, "-o");
        if (!dir) dir = "grab";
        return cmd_grab(dir, ea, ga, eb, gb);
    }
    if (!strcmp(cmd, "scan")) return cmd_scan(arg_str(argc, argv, "-o"), seconds, ea, ga, eb, gb, distance, sweep, shape);
    if (!strcmp(cmd, "window") || argv[1] == NULL || cmd[0] == '-') {
        openscan_calib cal;
        char pa[1024], pb[1024], sn[64], used[512];
        const openscan_calib *use = NULL;
        if (openscan_find_cameras(pa, pb, sizeof pa, sn, sizeof sn) == 0 &&
            openscan_calib_ensure(sn, NULL, &cal, used, sizeof used) == 0)
            use = &cal;
        return os_ui_main(use, ea, ga, eb, gb);
    }
    fprintf(stderr, "Unknown command \"%s\".\n\n", cmd);
    usage(argv[0]);
    return 2;
}
