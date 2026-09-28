/* Scan window. The picture and the buttons are the same on every host.
 * os_surf_* is the only piece that knows X11, Win32, or Cocoa. */
#include "openscan/openscan.h"
#include "host/os_host.h"
#include "os_surf.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#ifdef _WIN32
#include <direct.h>
#define os_getcwd _getcwd
#else
#include <unistd.h>
#define os_getcwd getcwd
#endif

static int grab_gray(openscan_cam *cam, uint8_t *storage, size_t cap, uint8_t *gray) {
    return openscan_frame_gray(cam, storage, cap, gray);
}

static void release_cams(openscan_cam *a, openscan_cam *b) {
    if (a) {
        openscan_cam_stop(a);
        openscan_cam_close(a);
    }
    if (b) {
        openscan_cam_stop(b);
        openscan_cam_close(b);
    }
}

typedef struct {
    int ea, ga, eb, gb;
    int distance, near_mm, far_mm, stride, smooth, sweep, relief;
    int solid, flip, shape;
} Knobs;

static void knobs_default(Knobs *k) {
    openscan_build b = openscan_default_build();
    k->ea = openscan_default_exposure_a();
    k->ga = openscan_default_gain_a();
    k->eb = openscan_default_exposure_b();
    k->gb = openscan_default_gain_b();
    k->distance = openscan_default_distance_mm();
    k->near_mm = (int)b.near_mm;
    k->far_mm = (int)b.far_mm;
    k->stride = b.stride;
    k->smooth = b.smooth;
    k->sweep = b.sweep_deg;
    k->relief = b.relief;
    k->solid = b.solid;
    k->flip = b.flip;
    k->shape = OPENSCAN_SHAPE_MOLD;
}

static void prefs_path(char *out, size_t n) {
#ifdef _WIN32
    const char *b = getenv("APPDATA");
    snprintf(out, n, "%s/openscan-ui.txt", b && b[0] ? b : ".");
#elif defined(__APPLE__)
    const char *h = getenv("HOME");
    snprintf(out, n, "%s/Library/Application Support/openscan-ui.txt", h && h[0] ? h : ".");
#else
    const char *x = getenv("XDG_CONFIG_HOME");
    const char *h = getenv("HOME");
    if (x && x[0]) snprintf(out, n, "%s/openscan-ui.txt", x);
    else snprintf(out, n, "%s/.config/openscan-ui.txt", h && h[0] ? h : ".");
#endif
}

static void knobs_load(Knobs *k) {
    knobs_default(k);
    char path[512];
    prefs_path(path, sizeof path);
    FILE *f = fopen(path, "r");
    if (!f) return;
    char key[32];
    int v;
    while (fscanf(f, "%31s %d", key, &v) == 2) {
        if (!strcmp(key, "ea")) k->ea = v;
        else if (!strcmp(key, "ga")) k->ga = v;
        else if (!strcmp(key, "eb")) k->eb = v;
        else if (!strcmp(key, "gb")) k->gb = v;
        else if (!strcmp(key, "distance")) k->distance = v;
        else if (!strcmp(key, "near")) k->near_mm = v;
        else if (!strcmp(key, "far")) k->far_mm = v;
        else if (!strcmp(key, "stride")) k->stride = v;
        else if (!strcmp(key, "smooth")) k->smooth = v;
        else if (!strcmp(key, "sweep")) k->sweep = v;
        else if (!strcmp(key, "relief")) k->relief = v;
        else if (!strcmp(key, "solid")) k->solid = v;
        else if (!strcmp(key, "flip")) k->flip = v;
        else if (!strcmp(key, "shape")) k->shape = v;
    }
    fclose(f);
}

static void knobs_save(const Knobs *k) {
    char path[512];
    prefs_path(path, sizeof path);
    FILE *f = fopen(path, "w");
    if (!f) return;
    fprintf(f, "ea %d\nga %d\neb %d\ngb %d\n", k->ea, k->ga, k->eb, k->gb);
    fprintf(f, "distance %d\nnear %d\nfar %d\nstride %d\nsmooth %d\n",
            k->distance, k->near_mm, k->far_mm, k->stride, k->smooth);
    fprintf(f, "sweep %d\nrelief %d\nsolid %d\nflip %d\nshape %d\n",
            k->sweep, k->relief, k->solid, k->flip, k->shape);
    fclose(f);
}

static void knobs_clamp(Knobs *k) {
    if (k->ea < 1) k->ea = 1;
    if (k->ea > 200) k->ea = 200;
    if (k->eb < 1) k->eb = 1;
    if (k->eb > 200) k->eb = 200;
    if (k->ga < 0) k->ga = 0;
    if (k->ga > 100) k->ga = 100;
    if (k->gb < 0) k->gb = 0;
    if (k->gb > 100) k->gb = 100;
    if (k->distance < 100) k->distance = 100;
    if (k->distance > 500) k->distance = 500;
    if (k->near_mm < 60) k->near_mm = 60;
    if (k->far_mm > 800) k->far_mm = 800;
    if (k->far_mm < k->near_mm + 40) k->far_mm = k->near_mm + 40;
    if (k->stride < 1) k->stride = 1;
    if (k->stride > 6) k->stride = 6;
    if (k->smooth < 0) k->smooth = 0;
    if (k->smooth > 8) k->smooth = 8;
    if (k->sweep < 60) k->sweep = 60;
    if (k->sweep > 140) k->sweep = 140;
    if (k->relief < 0) k->relief = 0;
    if (k->relief > 100) k->relief = 100;
    k->solid = k->solid ? 1 : 0;
    k->flip = k->flip ? 1 : 0;
    k->shape = k->shape == OPENSCAN_SHAPE_MEASURED ? OPENSCAN_SHAPE_MEASURED : OPENSCAN_SHAPE_MOLD;
}

static void apply_knobs(openscan_live *live, const Knobs *k) {
    openscan_build b;
    openscan_live_get_build(live, &b);
    b.near_mm = (float)k->near_mm;
    b.far_mm = (float)k->far_mm;
    b.stride = k->stride;
    b.smooth = k->smooth;
    b.sweep_deg = k->sweep;
    b.relief = k->relief;
    b.solid = k->solid;
    b.flip = k->flip;
    openscan_live_set_build(live, &b);
    openscan_live_set_distance_mm(live, (float)k->distance);
}

static int slider_at(int mx, int lo, int hi) {
    int x0 = 200, x1 = 1040;
    if (mx < x0) mx = x0;
    if (mx > x1) mx = x1;
    if (hi <= lo) return lo;
    return lo + (hi - lo) * (mx - x0) / (x1 - x0);
}

static int frame_score(const uint8_t *g, int n) {
    int sum = 0, m = 0, clip = 0;
    for (int i = 0; i < n; i += 4) {
        if (g[i] > 16) {
            sum += g[i];
            m++;
            if (g[i] > 245) clip++;
        }
    }
    if (m < n / 80) return -100000;
    int mean = sum / m;
    int miss = mean - 110;
    if (miss < 0) miss = -miss;
    return -miss * 10 - clip * 40 / (m + 1);
}

static void auto_exposure(openscan_cam *cam, uint8_t *buf, uint8_t *gray, int w, int h, int *exp, int gain) {
    static const int try_e[] = {6, 10, 14, 18, 24, 32, 48, 64};
    int best_e = *exp, best_s = -1000000;
    size_t cap = (size_t)w * h * 4;
    for (int i = 0; i < 8; i++) {
        openscan_cam_set_exposure(cam, try_e[i], gain);
        os_host_sleep_ms(90);
        grab_gray(cam, buf, cap, gray);
        grab_gray(cam, buf, cap, gray);
        int s = frame_score(gray, w * h);
        if (s > best_s) {
            best_s = s;
            best_e = try_e[i];
        }
    }
    *exp = best_e;
    openscan_cam_set_exposure(cam, best_e, gain);
}

int os_ui_main(const openscan_calib *cal, int exp_a, int gain_a, int exp_b, int gain_b) {
    char path_a[1024], path_b[1024], serial[64];
    if (openscan_find_cameras(path_a, path_b, sizeof path_a, serial, sizeof serial) != 0) {
        fprintf(stderr, "no scanner cameras\n");
        return 1;
    }
    int w = cal && cal->width > 16 ? cal->width : 1280;
    int h = cal && cal->height > 16 ? cal->height : 720;
    openscan_cam *a = openscan_cam_open(path_a, w, h, 10, 1);
    openscan_cam *b = openscan_cam_open(path_b, w, h, 10, 1);
    if (!a || !b) {
        fprintf(stderr, "camera open failed\n");
        release_cams(a, b);
        return 1;
    }
    w = openscan_cam_width(a);
    h = openscan_cam_height(a);
    Knobs kn;
    knobs_load(&kn);
    if (exp_a >= 0) kn.ea = exp_a;
    if (gain_a >= 0) kn.ga = gain_a;
    if (exp_b >= 0) kn.eb = exp_b;
    if (gain_b >= 0) kn.gb = gain_b;
    knobs_clamp(&kn);
    openscan_cam_set_exposure(a, kn.ea, kn.ga);
    openscan_cam_set_exposure(b, kn.eb, kn.gb);
    if (openscan_cam_start(a) != 0 || openscan_cam_start(b) != 0) {
        fprintf(stderr, "camera start failed\n");
        release_cams(a, b);
        return 1;
    }
    openscan_live *live = openscan_live_create(cal, NULL);
    if (!live) {
        release_cams(a, b);
        return 1;
    }
    openscan_live_set_shape(live, kn.shape);
    apply_knobs(live, &kn);
    openscan_live_set_mode(live, OPENSCAN_MODE_STOP);
    const int win_w = 1280, win_h = 880, view_y = 340;
    os_surf *surf = os_surf_open(win_w, win_h);
    if (!surf) {
        fprintf(stderr, "no window\n");
        openscan_live_destroy(live);
        release_cams(a, b);
        return 1;
    }

    uint8_t *bufa = malloc((size_t)w * h * 4);
    uint8_t *bufb = malloc((size_t)w * h * 4);
    uint8_t *ya = malloc((size_t)w * h);
    uint8_t *yb = malloc((size_t)w * h);
    char status[640];
    char note[640];
    time_t note_until = 0;
    snprintf(status, sizeof status, "idle");
    note[0] = 0;
    int run = bufa && bufb && ya && yb;
    int was_held = 0, drag = -1, orbiting = 0, last_x = 0, last_y = 0;
    int sent_ea = kn.ea, sent_ga = kn.ga, sent_eb = kn.eb, sent_gb = kn.gb;
    static const char *names[] = {
        "Camera A exposure", "Camera A gain", "Camera B exposure", "Camera B gain",
        "Distance (mm)", "Near (mm)", "Far (mm)", "Stride", "Smooth",
        "Mold sweep", "Mold relief"};
    int *vals[] = {&kn.ea, &kn.ga, &kn.eb, &kn.gb, &kn.distance, &kn.near_mm, &kn.far_mm,
                   &kn.stride, &kn.smooth, &kn.sweep, &kn.relief};
    int los[] = {1, 0, 1, 0, 100, 60, 100, 1, 0, 60, 0};
    int his[] = {200, 100, 200, 100, 500, 400, 800, 6, 8, 140, 100};
    while (run && !openscan_live_quit(live)) {
        int mx, my, key, quit, held;
        while (os_surf_pump(surf, &mx, &my, &key, &quit, &held)) {
            if (quit) run = 0;
            int press = held && !was_held;
            was_held = held;
            if (!held) {
                drag = -1;
                orbiting = 0;
            }
            if (press && my >= 6 && my < 36) {
                if (mx < 90) openscan_live_set_mode(live, OPENSCAN_MODE_SCAN);
                else if (mx < 170) openscan_live_set_mode(live, OPENSCAN_MODE_PAUSE);
                else if (mx < 250) openscan_live_set_mode(live, OPENSCAN_MODE_STOP);
                else if (mx < 340) openscan_live_reset(live);
                else if (mx < 430) {
                    kn.shape = OPENSCAN_SHAPE_MOLD;
                    openscan_live_set_shape(live, kn.shape);
                } else if (mx < 560) {
                    kn.shape = OPENSCAN_SHAPE_MEASURED;
                    openscan_live_set_shape(live, kn.shape);
                } else if (mx < 720) {
                    snprintf(status, sizeof status, "calibrating");
                    auto_exposure(a, bufa, ya, w, h, &kn.ea, kn.ga);
                    auto_exposure(b, bufb, yb, w, h, &kn.eb, kn.gb);
                    sent_ea = sent_ga = sent_eb = sent_gb = -1;
                } else if (mx >= 1100 && mx < 1270) {
                    char cwd[360], stem[400];
                    char stl[480], obj[480], ply[480];
                    int tris = 0, ok = 0;
                    if (!os_getcwd(cwd, sizeof cwd)) snprintf(cwd, sizeof cwd, ".");
                    snprintf(stem, sizeof stem, "%s/openscan-last", cwd);
                    snprintf(stl, sizeof stl, "%s.stl", stem);
                    snprintf(obj, sizeof obj, "%s.obj", stem);
                    snprintf(ply, sizeof ply, "%s.ply", stem);
                    if (openscan_live_write(live, stl, &tris) == 0) ok++;
                    if (openscan_live_write(live, obj, &tris) == 0) ok++;
                    if (openscan_live_write(live, ply, &tris) == 0) ok++;
                    if (ok == 3) {
                        snprintf(note, sizeof note, "Saved %s .stl .obj .ply  (%d triangles)", stem, tris);
                        fprintf(stderr, "%s\n", note);
                    } else if (ok == 0) {
                        snprintf(note, sizeof note, "Nothing to save yet. Press Start, then Export.");
                        fprintf(stderr, "%s\n", note);
                    } else {
                        snprintf(note, sizeof note, "Saved %d of stl, obj, ply in %s", ok, cwd);
                        fprintf(stderr, "%s\n", note);
                    }
                    note_until = time(NULL) + 8;
                }
            }
            if (press && my >= 292 && my < 328) {
                if (mx < 180) kn.solid = !kn.solid;
                else if (mx < 360) kn.flip = !kn.flip;
                else if (mx > 1100) knobs_default(&kn);
            }
            if (press) {
                drag = -1;
                for (int i = 0; i < 11; i++) {
                    int y = 46 + i * 22;
                    if (my >= y && my < y + 20 && mx >= 190 && mx <= 1050) drag = i;
                }
            }
            if (held && drag >= 0) vals[drag][0] = slider_at(mx, los[drag], his[drag]);
            if (press && my >= view_y) orbiting = 1;
            if (held && orbiting && drag < 0 && !press && mx >= 0) {
                openscan_live_orbit(live, (mx - last_x) * 0.01f, (my - last_y) * 0.01f);
            }
            if (mx >= 0) {
                last_x = mx;
                last_y = my;
            }
            if (key == 'q' || key == 27) run = 0;
            if (key == 's') openscan_live_set_mode(live, OPENSCAN_MODE_SCAN);
            if (key == ' ') openscan_live_key(live, ' ');
            knobs_clamp(&kn);
        }
        apply_knobs(live, &kn);
        if (kn.ea != sent_ea || kn.ga != sent_ga) {
            openscan_cam_set_exposure(a, kn.ea, kn.ga);
            sent_ea = kn.ea;
            sent_ga = kn.ga;
        }
        if (kn.eb != sent_eb || kn.gb != sent_gb) {
            openscan_cam_set_exposure(b, kn.eb, kn.gb);
            sent_eb = kn.eb;
            sent_gb = kn.gb;
        }
        int pw = 0, ph = 0;
        if (grab_gray(a, bufa, (size_t)w * h * 4, ya) == 0 &&
            grab_gray(b, bufb, (size_t)w * h * 4, yb) == 0) {
            openscan_live_status st;
            openscan_live_push(live, ya, yb, w, h, &st);
            const char *mode = "idle";
            if (openscan_live_mode(live) == OPENSCAN_MODE_SCAN) mode = "scan";
            else if (openscan_live_mode(live) == OPENSCAN_MODE_PAUSE) mode = "pause";
            const char *shape = openscan_live_shape(live) == OPENSCAN_SHAPE_MEASURED ? "measured" : "mold";
            if (time(NULL) < note_until)
                snprintf(status, sizeof status, "%s", note);
            else
                snprintf(status, sizeof status, "%s  %s  %d mm  %d tris  %d deg",
                         mode, shape, kn.distance, st.points, st.scanned_deg);
            const uint8_t *bgr = openscan_live_preview_bgr(live, &pw, &ph);
            if (bgr && pw > 8 && ph > 8) os_surf_blit_bgr(surf, bgr, pw, ph, 8, view_y);
        }
        if (time(NULL) < note_until) snprintf(status, sizeof status, "%s", note);
        /* The window keeps whatever was drawn last. Wipe the controls first
         * so a moved thumb or a shorter label does not leave the old one. */
        os_surf_bar(surf, 0, 0, win_w, view_y, 0x12141a);
        os_surf_bar(surf, 0, view_y + (ph > 8 ? ph : 0), win_w, win_h - view_y - (ph > 8 ? ph : 0), 0x12141a);
        os_surf_bar(surf, 8, 6, 78, 26, 0x2a9d4a);
        os_surf_bar(surf, 92, 6, 72, 26, 0x3a3e48);
        os_surf_bar(surf, 170, 6, 72, 26, 0x3a3e48);
        os_surf_bar(surf, 248, 6, 84, 26, 0x3a3e48);
        os_surf_bar(surf, 344, 6, 80, 26, kn.shape == OPENSCAN_SHAPE_MOLD ? 0x2a6f9d : 0x3a3e48);
        os_surf_bar(surf, 430, 6, 120, 26, kn.shape == OPENSCAN_SHAPE_MEASURED ? 0x2a6f9d : 0x3a3e48);
        os_surf_bar(surf, 558, 6, 150, 26, 0x2a6f9d);
        os_surf_bar(surf, 1108, 6, 150, 26, 0xc4892a);
        os_surf_text(surf, 22, 24, "Start", 0xf2f2f2);
        os_surf_text(surf, 104, 24, "Pause", 0xf2f2f2);
        os_surf_text(surf, 188, 24, "Stop", 0xf2f2f2);
        os_surf_text(surf, 264, 24, "Reset", 0xf2f2f2);
        os_surf_text(surf, 362, 24, "Mold", 0xf2f2f2);
        os_surf_text(surf, 448, 24, "Measured", 0xf2f2f2);
        os_surf_text(surf, 574, 24, "Auto calibrate", 0xf2f2f2);
        os_surf_text(surf, 1148, 24, "Export", 0xf2f2f2);
        for (int i = 0; i < 11; i++) {
            int y = 46 + i * 22;
            int v = vals[i][0];
            int lo = los[i], hi = his[i];
            int x0 = 200, x1 = 1040;
            int thumb = x0 + (hi > lo ? (x1 - x0) * (v - lo) / (hi - lo) : 0);
            char line[64];
            os_surf_text(surf, 12, y + 14, names[i], 0xd0d0d0);
            os_surf_bar(surf, x0, y + 6, x1 - x0, 6, 0x2a2e36);
            os_surf_bar(surf, x0, y + 6, thumb - x0, 6, 0x3d7ea6);
            os_surf_bar(surf, thumb - 4, y + 2, 10, 14, 0xf2f2f2);
            if (i == 9)
                snprintf(line, sizeof line, "%d (%d of the object)", v, v * 2);
            else
                snprintf(line, sizeof line, "%d", v);
            os_surf_text(surf, 1052, y + 14, line, 0xf2f2f2);
        }
        os_surf_bar(surf, 12, 294, 16, 16, kn.solid ? 0x2a9d4a : 0x3a3e48);
        os_surf_text(surf, 34, 308, "Solid mesh", 0xf2f2f2);
        os_surf_bar(surf, 180, 294, 16, 16, kn.flip ? 0x2a9d4a : 0x3a3e48);
        os_surf_text(surf, 202, 308, "Flip scanner", 0xf2f2f2);
        os_surf_text(surf, 1110, 308, "Defaults", 0xf2f2f2);
        if (pw > 8) {
            int cam_w = pw * 42 / 100;
            int mid_w = pw * 42 / 100;
            os_surf_text(surf, 16, view_y + 16, "camera", 0xf2f2f2);
            os_surf_text(surf, 16 + cam_w, view_y + 16, "3D", 0xf2f2f2);
            os_surf_text(surf, 16 + cam_w + mid_w, view_y + 16, "stereo A", 0xf2f2f2);
            os_surf_text(surf, 16 + cam_w + mid_w, view_y + ph / 3 + 16, "stereo B", 0xf2f2f2);
            os_surf_text(surf, 16 + cam_w + mid_w, view_y + 2 * ph / 3 + 16, "side", 0xf2f2f2);
        }
        os_surf_text(surf, 12, view_y + ph + 22, status, 0xf2f2f2);
        os_surf_flush(surf);
        os_host_sleep_ms(30);
    }
    knobs_save(&kn);
    if (openscan_live_points(live) > 0) {
        int tris = 0;
        openscan_live_write(live, "openscan-last.stl", &tris);
        openscan_live_write(live, "openscan-last.obj", &tris);
        openscan_live_write(live, "openscan-last.ply", &tris);
        fprintf(stderr, "wrote openscan-last.stl %d tris\n", tris);
    }
    os_surf_close(surf);
    openscan_live_destroy(live);
    release_cams(a, b);
    free(bufa);
    free(bufb);
    free(ya);
    free(yb);
    return 0;
}
