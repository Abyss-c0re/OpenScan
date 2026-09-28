/* Scan window. The picture and the buttons are the same on every host.
 * os_surf_* is the only piece that knows X11, Win32, or Cocoa. */
#include "openscan/openscan_calib.h"
#include "openscan/openscan_live.h"
#include "openscan/openscan_v4l2.h"
#include "host/os_host.h"
#include "image/os_jpeg.h"
#include "os_surf.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int grab_gray(openscan_cam *cam, uint8_t *storage, size_t cap, uint8_t *gray) {
    size_t n = 0;
    int w = openscan_cam_width(cam), h = openscan_cam_height(cam);
    if (openscan_cam_grab_latest(cam, storage, cap, &n, NULL) != 0 || n < 16) return -1;
    if (!openscan_cam_mjpeg(cam)) {
        if (n < (size_t)w * h) return -1;
        memcpy(gray, storage, (size_t)w * h);
        return 0;
    }
    return os_jpeg_gray(storage, n, w, h, gray);
}

int os_grab_gray(openscan_cam *cam, uint8_t *storage, size_t cap, uint8_t *gray) {
    return grab_gray(cam, storage, cap, gray);
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
    openscan_cam_set_exposure(a, exp_a, gain_a);
    openscan_cam_set_exposure(b, exp_b, gain_b);
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
    openscan_live_set_mode(live, OPENSCAN_MODE_STOP);
    os_surf *surf = os_surf_open(1000, 620);
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
    char status[160];
    snprintf(status, sizeof status, "idle");
    int run = bufa && bufb && ya && yb;
    while (run && !openscan_live_quit(live)) {
        int mx, my, key, quit;
        while (os_surf_pump(surf, &mx, &my, &key, &quit)) {
            if (quit) run = 0;
            if (my >= 0 && my < 36) {
                if (mx < 90) openscan_live_set_mode(live, OPENSCAN_MODE_SCAN);
                else if (mx < 170) openscan_live_set_mode(live, OPENSCAN_MODE_STOP);
                else if (mx < 260) openscan_live_reset(live);
            }
            if (key == 'q' || key == 27) run = 0;
            if (key == 's') openscan_live_set_mode(live, OPENSCAN_MODE_SCAN);
            if (key == ' ') openscan_live_key(live, ' ');
        }
        if (grab_gray(a, bufa, (size_t)w * h * 4, ya) == 0 &&
            grab_gray(b, bufb, (size_t)w * h * 4, yb) == 0) {
            openscan_live_status st;
            openscan_live_push(live, ya, yb, w, h, &st);
            snprintf(status, sizeof status, "%s  %d tris  %d deg",
                     openscan_live_mode(live) == OPENSCAN_MODE_SCAN ? "scan" : "idle",
                     st.points, st.scanned_deg);
            int pw = 0, ph = 0;
            const uint8_t *bgr = openscan_live_preview_bgr(live, &pw, &ph);
            if (bgr && pw > 8 && ph > 8) os_surf_blit_bgr(surf, bgr, pw, ph, 0, 40);
        }
        os_surf_bar(surf, 8, 6, 78, 26, 0x2a9d4a);
        os_surf_bar(surf, 94, 6, 70, 26, 0x3a3e48);
        os_surf_bar(surf, 172, 6, 78, 26, 0x3a3e48);
        os_surf_text(surf, 22, 24, "Start", 0xf2f2f2);
        os_surf_text(surf, 112, 24, "Stop", 0xf2f2f2);
        os_surf_text(surf, 186, 24, "Reset", 0xf2f2f2);
        os_surf_text(surf, 270, 24, status, 0xf2f2f2);
        os_surf_flush(surf);
        os_host_sleep_ms(30);
    }
    if (openscan_live_points(live) > 0) {
        int tris = 0;
        openscan_live_write(live, "openscan-last.stl", &tris);
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
