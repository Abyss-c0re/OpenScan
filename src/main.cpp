#include "fox_calib.h"
#include "fox_live.h"
#include "fox_scan.h"
#include "fox_v4l2.h"

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

static volatile sig_atomic_t g_stop = 0;

static void on_sigint(int) { g_stop = 1; }

static void usage(const char *argv0) {
    fprintf(stderr,
            "fox3d — open-source scanner for the 3DMakerpro Fox (JMM8)\n"
            "\n"
            "  %s\n"
            "      Open the app. The cameras and 3D view come up idle.\n"
            "      Start scan when the object is in view. Export STL when\n"
            "      the model is ready. Closing the window quits.\n"
            "\n"
            "Usage:\n"
            "  %s\n"
            "  %s devices\n"
            "  %s grab  -o DIR [--exposure N] [--gain N]\n"
            "           [--exposure-a N] [--exposure-b N] [--gain-a N] [--gain-b N]\n"
            "  %s scan  [--no-window -o FILE.stl --seconds N]\n"
            "           With no --no-window, scan opens the same app.\n"
            "           [--calib PATH] [--scale S] [--min-mm Z] [--max-mm Z]\n"
            "           [--exposure N] [--gain N]\n"
            "           [--exposure-a N] [--exposure-b N] [--gain-a N] [--gain-b N]\n"
            "  %s snap  -o FILE.stl [--calib PATH] [--frames N] [--scale S]\n"
            "           [--exposure N] [--gain N] [--min-mm Z] [--max-mm Z]\n"
            "  %s asic-read HEXADDR\n"
            "  %s mesh-test\n"
            "\n"
            "exposure is UVC absolute exposure in units of 100 microseconds.\n"
            "scan adjusts it per camera unless --no-ae is set. gain is 0..100.\n"
            "Calibration is read from calib/<serial>.txt .\n",
            argv0, argv0, argv0, argv0, argv0, argv0, argv0, argv0);
}

static int arg_int(int argc, char **argv, int *i, int *dst) {
    if (*i + 1 >= argc) return -1;
    *dst = atoi(argv[++(*i)]);
    return 0;
}

static int arg_dbl(int argc, char **argv, int *i, double *dst) {
    if (*i + 1 >= argc) return -1;
    *dst = atof(argv[++(*i)]);
    return 0;
}

static int arg_str(int argc, char **argv, int *i, const char **dst) {
    if (*i + 1 >= argc) return -1;
    *dst = argv[++(*i)];
    return 0;
}

static int write_pgm(const char *path, const uint8_t *y, int w, int h) {
    FILE *f = fopen(path, "wb");
    if (!f) return -1;
    fprintf(f, "P5\n%d %d\n255\n", w, h);
    fwrite(y, 1, (size_t)w * h, f);
    fclose(f);
    return 0;
}

static double mean_y(const uint8_t *y, int n) {
    double s = 0;
    for (int i = 0; i < n; i++) s += y[i];
    return n ? s / n : 0;
}

static double hf_energy(const uint8_t *y, int w, int h) {
    double s = 0;
    int n = 0;
    for (int row = h / 5; row < 4 * h / 5; row += 6) {
        const uint8_t *line = y + row * w;
        for (int x = 0; x + 2 < w; x += 3) {
            int d = (int)line[x + 2] - (int)line[x];
            if (d < 0) d = -d;
            s += d;
            n++;
        }
    }
    return n ? s / n : 0;
}

static int open_pair(fox_cam **a, fox_cam **b, char *serial, size_t serial_n,
                     int exp_a, int gain_a, int exp_b, int gain_b) {
    char path_a[256], path_b[256];
    if (fox_find_cameras(path_a, path_b, sizeof path_a, serial, serial_n) < 0) {
        fprintf(stderr, "No Fox cameras found. Look for 0c45:636a and 0c45:636b in lsusb.\n");
        return -1;
    }
    printf("serial %s\n  A %s\n  B %s\n", serial, path_a, path_b);
    *a = fox_cam_open(path_a, 1280, 720, 10, 1);
    *b = fox_cam_open(path_b, 1280, 720, 10, 1);
    if (!*a || !*b) {
        fox_cam_close(*a);
        fox_cam_close(*b);
        *a = *b = NULL;
        return -1;
    }
    fox_cam_set_exposure(*a, exp_a, gain_a);
    fox_cam_set_exposure(*b, exp_b, gain_b);
    if (fox_cam_start(*a) < 0 || fox_cam_start(*b) < 0) {
        fox_cam_close(*a);
        fox_cam_close(*b);
        *a = *b = NULL;
        return -1;
    }
    return 0;
}

static int jpeg_to_gray(const uint8_t *jpg, size_t n, int w, int h, std::vector<uint8_t> &y) {
    cv::Mat encoded(1, (int)n, CV_8UC1, const_cast<uint8_t *>(jpg));
    cv::Mat gray = cv::imdecode(encoded, cv::IMREAD_GRAYSCALE);
    if (gray.empty() || gray.cols != w || gray.rows != h || !gray.isContinuous()) return -1;
    y.assign(gray.data, gray.data + (size_t)w * h);
    return 0;
}

static int grab_pair(fox_cam *a, fox_cam *b, std::vector<uint8_t> &ya, std::vector<uint8_t> &yb, int latest) {
    int w = fox_cam_width(a), h = fox_cam_height(a);
    std::vector<uint8_t> ja(2 * 1024 * 1024), jb(2 * 1024 * 1024);
    size_t na = 0, nb = 0;
    int (*take)(fox_cam *, uint8_t *, size_t, size_t *, uint64_t *) =
        latest ? fox_cam_grab_latest : fox_cam_grab;
    if (take(a, ja.data(), ja.size(), &na, NULL) < 0) return -1;
    if (take(b, jb.data(), jb.size(), &nb, NULL) < 0) return -1;
    if (!fox_cam_mjpeg(a)) {
        ya.assign(ja.begin(), ja.begin() + (ptrdiff_t)na);
        yb.assign(jb.begin(), jb.begin() + (ptrdiff_t)nb);
        return 0;
    }
    if (jpeg_to_gray(ja.data(), na, w, h, ya) < 0 || jpeg_to_gray(jb.data(), nb, w, h, yb) < 0) {
        fprintf(stderr, "jpeg decode failed (%zu / %zu bytes)\n", na, nb);
        return -1;
    }
    return 0;
}

static int cmd_devices(void) {
    char path_a[256], path_b[256], serial[64];
    if (fox_find_cameras(path_a, path_b, sizeof path_a, serial, sizeof serial) < 0) {
        fprintf(stderr, "Fox not found.\n");
        return 1;
    }
    printf("Fox serial %s\n", serial);
    printf("  camera A %s\n", path_a);
    printf("  camera B %s\n", path_b);
    if (strncmp(serial, "JMM8", 4) != 0)
        fprintf(stderr, "warning: serial prefix is not JMM8 (Fox). Other 3DMakerpro bodies share this USB ID.\n");
    return 0;
}

static int cmd_grab(const char *dir, int exp_a, int gain_a, int exp_b, int gain_b) {
    fox_cam *a = NULL, *b = NULL;
    char serial[64];
    if (open_pair(&a, &b, serial, sizeof serial, exp_a, gain_a, exp_b, gain_b) < 0) return 1;
    std::vector<uint8_t> ya, yb;
    for (int i = 0; i < 3; i++) {
        if (grab_pair(a, b, ya, yb, 1) < 0) {
            fox_cam_close(a);
            fox_cam_close(b);
            return 1;
        }
    }
    char pa[512], pb[512];
    snprintf(pa, sizeof pa, "%s/camA.pgm", dir);
    snprintf(pb, sizeof pb, "%s/camB.pgm", dir);
    int w = fox_cam_width(a), h = fox_cam_height(a);
    if (write_pgm(pa, ya.data(), w, h) < 0 || write_pgm(pb, yb.data(), w, h) < 0) {
        fprintf(stderr, "cannot write into %s\n", dir);
        fox_cam_close(a);
        fox_cam_close(b);
        return 1;
    }
    int n = w * h;
    printf("wrote %s (mean %.1f, hf %.1f) and %s (mean %.1f, hf %.1f)\n",
           pa, mean_y(ya.data(), n), hf_energy(ya.data(), w, h),
           pb, mean_y(yb.data(), n), hf_energy(yb.data(), w, h));
    printf("exposure A %d gain %d,  B %d gain %d\n", exp_a, gain_a, exp_b, gain_b);
    fox_cam_close(a);
    fox_cam_close(b);
    return 0;
}

static int load_calib_for(const char *serial, const char *explicit_path, fox_calib *cal) {
    if (explicit_path) {
        if (fox_calib_load(explicit_path, cal) == 0) {
            printf("calib %s  %dx%d  baseline %.2f mm  date %s\n",
                   explicit_path, cal->width, cal->height, cal->cam[1].tvec[0], cal->date);
            return 0;
        }
        fprintf(stderr, "cannot read calib %s\n", explicit_path);
        return -1;
    }
    char path[512];
    const char *cands[] = {
        "calib/%s.txt",
        "fox3d/calib/%s.txt",
        "/home/voldemar/Dev/lab/Fox3DScan/fox3d/calib/%s.txt",
    };
    for (const char *fmt : cands) {
        snprintf(path, sizeof path, fmt, serial);
        if (fox_calib_load(path, cal) == 0) {
            printf("calib %s  %dx%d  baseline %.2f mm  date %s\n",
                   path, cal->width, cal->height, cal->cam[1].tvec[0], cal->date);
            return 0;
        }
    }
    fprintf(stderr, "no calib for %s. Pass --calib, or place calib/%s.txt\n", serial, serial);
    return -1;
}

static void nudge_exposure(int *exp, int *gain, double mean) {
    const double target = 55;
    if (std::fabs(mean - target) < 10) return;
    double ratio = target / std::max(mean, 1.0);
    if (mean > 200) ratio = 0.45;
    ratio = std::max(0.72, std::min(ratio, 1.35));
    int next = (int)std::lround(*exp * ratio);
    if (next == *exp) next += (mean < target) ? 2 : -2;
    if (next < 1) {
        *exp = 1;
        if (mean > target) *gain = std::max(0, *gain - 2);
    } else if (next > 350 && mean < target) {
        *exp = 350;
        *gain = std::min(32, *gain + 2);
    } else {
        *exp = std::max(1, std::min(350, next));
    }
}

static int cmd_snap(const char *stl, const char *calib_path, int frames,
                    int exposure, int gain, double scale, double min_mm, double max_mm) {
    fox_cam *a = NULL, *b = NULL;
    char serial[64];
    if (open_pair(&a, &b, serial, sizeof serial, exposure, gain, exposure, gain) < 0) return 1;
    fox_calib cal;
    if (load_calib_for(serial, calib_path, &cal) < 0) {
        fox_cam_close(a);
        fox_cam_close(b);
        return 1;
    }
    std::vector<uint8_t> ya, yb, best_a, best_b;
    double best_score = -1;
    for (int i = 0; i < frames + 2; i++) {
        if (grab_pair(a, b, ya, yb, 0) < 0) {
            fox_cam_close(a);
            fox_cam_close(b);
            return 1;
        }
        if (i < 2) continue;
        double ma = mean_y(ya.data(), (int)ya.size());
        double mb = mean_y(yb.data(), (int)yb.size());
        double score = 0;
        if (ma > 8 && ma < 240) score += 1000 - std::fabs(ma - 80);
        if (mb > 8 && mb < 240) score += 1000 - std::fabs(mb - 80);
        printf("frame %d  mean A %.1f  B %.1f\n", i - 1, ma, mb);
        if (score > best_score) {
            best_score = score;
            best_a = ya;
            best_b = yb;
        }
    }
    fox_cam_close(a);
    fox_cam_close(b);
    if (best_a.empty()) return 1;
    fox_scan_opts opt;
    opt.scale = scale;
    opt.min_mm = min_mm;
    opt.max_mm = max_mm;
    opt.edge_mm = 4.0;
    opt.preview_png = NULL;
    int tris = 0;
    if (fox_scan_to_stl(best_a.data(), best_b.data(), cal.width, cal.height, &cal, &opt, stl, &tris) < 0)
        return 1;
    printf("wrote %s  (%d triangles, millimetres)\n", stl, tris);
    return tris > 0 ? 0 : 2;
}

static std::string preview_path_for(const char *stl) {
    std::string s(stl);
    auto dot = s.rfind('.');
    auto slash = s.find_last_of('/');
    if (dot != std::string::npos && (slash == std::string::npos || dot > slash)) s.resize(dot);
    return s + ".preview.png";
}

static int cmd_scan(const char *stl, const char *calib_path, int seconds, int no_window, int no_ae,
                    int exp_a, int gain_a, int exp_b, int gain_b,
                    double scale, double min_mm, double max_mm) {
    fox_cam *a = NULL, *b = NULL;
    char serial[64];
    if (open_pair(&a, &b, serial, sizeof serial, exp_a, gain_a, exp_b, gain_b) < 0) return 1;
    fox_calib cal;
    if (load_calib_for(serial, calib_path, &cal) < 0) {
        fox_cam_close(a);
        fox_cam_close(b);
        return 1;
    }
    fox_scan_opts opt;
    opt.scale = scale;
    opt.min_mm = min_mm;
    opt.max_mm = max_mm;
    opt.edge_mm = 4.0;
    opt.preview_png = NULL;
    fox_live *live = fox_live_create(&cal, &opt);
    if (!live) {
        fox_cam_close(a);
        fox_cam_close(b);
        return 1;
    }

    std::vector<uint8_t> ya, yb;
    for (int i = 0; i < 2 && !g_stop; i++) {
        if (grab_pair(a, b, ya, yb, 1) < 0) {
            fox_live_destroy(live);
            fox_cam_close(a);
            fox_cam_close(b);
            return 1;
        }
    }
    if (!no_ae) {
        printf("balancing exposure (target mean about 55 on each camera)\n");
        for (int step = 0; step < 8 && !g_stop; step++) {
            fox_cam_set_exposure(a, exp_a, gain_a);
            fox_cam_set_exposure(b, exp_b, gain_b);
            bool settled = true;
            for (int dump = 0; dump < 3; dump++) {
                if (grab_pair(a, b, ya, yb, 1) < 0) { settled = false; break; }
            }
            if (!settled) break;
            int n = (int)ya.size();
            double ma = mean_y(ya.data(), n);
            double mb = mean_y(yb.data(), (int)yb.size());
            printf("  ae %d  A mean %.0f exp %d gain %d    B mean %.0f exp %d gain %d\n",
                   step + 1, ma, exp_a, gain_a, mb, exp_b, gain_b);
            nudge_exposure(&exp_a, &gain_a, ma);
            nudge_exposure(&exp_b, &gain_b, mb);
        }
        fox_cam_set_exposure(a, exp_a, gain_a);
        fox_cam_set_exposure(b, exp_b, gain_b);
        if (grab_pair(a, b, ya, yb, 1) == 0) {
            int w = fox_cam_width(a), h = fox_cam_height(a);
            double ha = hf_energy(ya.data(), w, h);
            double hb = hf_energy(yb.data(), w, h);
            printf("row detail  A %.1f   B %.1f   (a projected pattern is usually above 8)\n", ha, hb);
            if (ha < 3 && hb < 3)
                printf("no fine pattern in the image. A plain object will not match until the projector is lighting it.\n");
        }
    }

    printf("\nThe large panel is a 3D view. It turns on its own; drag to orbit, wheel to zoom.\n");
    printf("Start keeps new surface as you rotate the object. Pause holds it. Stop ends the pass.\n");
    printf("Save writes the STL and leaves the window open. Close, Q, or Esc quits and stays closed.\n");
    printf("Space toggles scan/pause. S saves. r clears the model.\n");
    printf("[ ] exposure camera A,  - = exposure camera B.   Now A %d/%d  B %d/%d\n\n",
           exp_a, gain_a, exp_b, gain_b);
    fflush(stdout);

    int window = !no_window;
    if (window && !getenv("DISPLAY") && !getenv("WAYLAND_DISPLAY")) window = 0;
    if (window) {
        try {
            cv::namedWindow("Fox scan", cv::WINDOW_NORMAL);
            cv::resizeWindow("Fox scan", 1360, 700);
            cv::setMouseCallback("Fox scan", [](int event, int x, int y, int flags, void *ud) {
                fox_live_mouse(static_cast<fox_live *>(ud), event, x, y, flags);
            }, live);
        } catch (const cv::Exception &e) {
            fprintf(stderr, "preview window unavailable (%s). Running in the terminal.\n", e.what());
            window = 0;
        }
    }
    if (!window || seconds > 0) fox_live_set_mode(live, FOX_MODE_SCAN);

    std::string preview_path = preview_path_for(stl);
    auto t0 = std::chrono::steady_clock::now();
    int pushed = 0;
    while (!g_stop) {
        if (seconds > 0) {
            double el = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            if (el >= seconds) break;
        }
        if (grab_pair(a, b, ya, yb, 1) < 0) break;
        fox_live_status st;
        if (fox_live_push(live, ya.data(), yb.data(), cal.width, cal.height, &st) < 0) break;
        pushed++;
        const char *mode_name = st.mode == FOX_MODE_SCAN ? "scanning" :
                                st.mode == FOX_MODE_PAUSE ? "paused" : "stopped";
        printf("\r%-8s  model %5d  depth %6d px  z %5.0f mm  %4.0f ms   ",
               mode_name, st.points, st.valid_pixels, st.median_mm, st.match_ms);
        fflush(stdout);
        if (fox_live_take_save(live)) {
            int tris = 0;
            if (fox_live_write(live, stl, &tris) == 0)
                printf("\nsaved %s  (%d triangles)\n", stl, tris);
            else
                fprintf(stderr, "\ncould not write %s\n", stl);
        }
        if (fox_live_quit(live)) break;

        int pw = 0, ph = 0;
        const uint8_t *bg = fox_live_preview_bgr(live, &pw, &ph);
        if (bg && pw > 0 && (pushed % 8 == 0 || st.points == 1)) {
            cv::Mat view(ph, pw, CV_8UC3, const_cast<uint8_t *>(bg));
            cv::imwrite(preview_path, view);
        }
        if (!window) continue;
        try {
            if (bg && pw > 0) {
                cv::Mat view(ph, pw, CV_8UC3, const_cast<uint8_t *>(bg));
                cv::imshow("Fox scan", view);
            }
            int key = cv::waitKey(1);
            /* Closing the title-bar button must end the process. Another
             * imshow would create the window again. */
            double vis = cv::getWindowProperty("Fox scan", cv::WND_PROP_VISIBLE);
            if (vis < 1) {
                fprintf(stderr, "\nwindow closed\n");
                break;
            }
            if (key >= 0) fox_live_key(live, key & 0xff);
            if (fox_live_quit(live)) break;
            if (key < 0) continue;
            key &= 0xff;
            if (key == 'r' || key == 'R') {
                fox_live_reset(live);
                printf("\nmodel cleared\n");
            } else if (key == '[') {
                exp_a = std::max(1, exp_a - std::max(1, exp_a / 6));
                fox_cam_set_exposure(a, exp_a, gain_a);
                printf("\nexposure A %d\n", exp_a);
            } else if (key == ']') {
                exp_a = std::min(350, exp_a + std::max(1, exp_a / 6));
                fox_cam_set_exposure(a, exp_a, gain_a);
                printf("\nexposure A %d\n", exp_a);
            } else if (key == '-' || key == '_') {
                exp_b = std::max(1, exp_b - std::max(1, exp_b / 6));
                fox_cam_set_exposure(b, exp_b, gain_b);
                printf("\nexposure B %d\n", exp_b);
            } else if (key == '=' || key == '+') {
                exp_b = std::min(350, exp_b + std::max(1, exp_b / 6));
                fox_cam_set_exposure(b, exp_b, gain_b);
                printf("\nexposure B %d\n", exp_b);
            }
        } catch (const cv::Exception &e) {
            fprintf(stderr, "\npreview closed (%s)\n", e.what());
            break;
        }
    }
    printf("\n");
    int pw = 0, ph = 0;
    const uint8_t *bg = fox_live_preview_bgr(live, &pw, &ph);
    if (bg && pw > 0) {
        cv::Mat view(ph, pw, CV_8UC3, const_cast<uint8_t *>(bg));
        if (cv::imwrite(preview_path, view))
            printf("preview %s\n", preview_path.c_str());
    }
    int tris = 0;
    int rc = 0;
    int pts = fox_live_points(live);
    if (!window || pts > 0) {
        if (fox_live_write(live, stl, &tris) < 0) {
            fprintf(stderr, "could not write %s\n", stl);
            rc = 1;
        } else {
            printf("wrote %s  (%d triangles, millimetres)\n", stl, tris);
            if (!window && tris < 100) {
                fprintf(stderr,
                        "very few triangles. Keep a matte object inside %.0f–%.0f mm and turn it slowly\n"
                        "until the depth panel fills in. The mesh only grows where both cameras agree.\n",
                        min_mm, max_mm);
                rc = 2;
            }
        }
    }
    if (window) cv::destroyAllWindows();
    fox_live_destroy(live);
    fox_cam_close(a);
    fox_cam_close(b);
    return rc;
}

static int cmd_asic(const char *hexaddr) {
    char path_a[256], path_b[256], serial[64];
    if (fox_find_cameras(path_a, path_b, sizeof path_a, serial, sizeof serial) < 0) return 1;
    unsigned addr = (unsigned)strtoul(hexaddr, NULL, 16);
    fox_cam *a = fox_cam_open(path_a, 1280, 720, 10, 1);
    fox_cam *b = fox_cam_open(path_b, 1280, 720, 10, 1);
    if (!a || !b) {
        fox_cam_close(a);
        fox_cam_close(b);
        return 1;
    }
    uint8_t va = 0, vb = 0;
    int ra = fox_asic_read(a, addr, &va);
    int rb = fox_asic_read(b, addr, &vb);
    printf("asic 0x%04x  A=%s%02x  B=%s%02x\n", addr,
           ra ? "err " : "", va, rb ? "err " : "", vb);
    fox_cam_close(a);
    fox_cam_close(b);
    return (ra || rb) ? 1 : 0;
}

int fox_app_main(int argc, char **argv, const char *calib,
                 int exp_a, int gain_a, int exp_b, int gain_b,
                 double scale, double min_mm, double max_mm);

int main(int argc, char **argv) {
    if (argc < 2) {
        return fox_app_main(argc, argv, nullptr, 18, 4, 12, 4, 0.4, 80, 550);
    }
    const char *cmd = argv[1];
    if (!strcmp(cmd, "mesh-test")) return fox_mesh_self_test();

    const char *out = NULL;
    const char *calib = NULL;
    const char *asic_addr = NULL;
    if (!strcmp(cmd, "asic-read")) {
        if (argc < 3) {
            fprintf(stderr, "asic-read HEXADDR\n");
            return 2;
        }
        asic_addr = argv[2];
    }
    int exposure = -1, gain = -1;
    int exp_a = -1, exp_b = -1, gain_a = -1, gain_b = -1;
    int frames = 3;
    int seconds = 0;
    int no_window = 0;
    int no_ae = 0;
    int scale_set = 0;
    double scale = 0.5;
    double min_mm = 80;
    double max_mm = 550;
    int opt_from = asic_addr ? 3 : 2;
    for (int i = opt_from; i < argc; i++) {
        if (!strcmp(argv[i], "--exposure")) { if (arg_int(argc, argv, &i, &exposure)) return 2; }
        else if (!strcmp(argv[i], "--gain")) { if (arg_int(argc, argv, &i, &gain)) return 2; }
        else if (!strcmp(argv[i], "--exposure-a")) { if (arg_int(argc, argv, &i, &exp_a)) return 2; }
        else if (!strcmp(argv[i], "--exposure-b")) { if (arg_int(argc, argv, &i, &exp_b)) return 2; }
        else if (!strcmp(argv[i], "--gain-a")) { if (arg_int(argc, argv, &i, &gain_a)) return 2; }
        else if (!strcmp(argv[i], "--gain-b")) { if (arg_int(argc, argv, &i, &gain_b)) return 2; }
        else if (!strcmp(argv[i], "--frames")) { if (arg_int(argc, argv, &i, &frames)) return 2; }
        else if (!strcmp(argv[i], "--seconds")) { if (arg_int(argc, argv, &i, &seconds)) return 2; }
        else if (!strcmp(argv[i], "--scale")) { if (arg_dbl(argc, argv, &i, &scale)) return 2; scale_set = 1; }
        else if (!strcmp(argv[i], "--min-mm")) { if (arg_dbl(argc, argv, &i, &min_mm)) return 2; }
        else if (!strcmp(argv[i], "--max-mm")) { if (arg_dbl(argc, argv, &i, &max_mm)) return 2; }
        else if (!strcmp(argv[i], "--calib")) { if (arg_str(argc, argv, &i, &calib)) return 2; }
        else if (!strcmp(argv[i], "-o")) { if (arg_str(argc, argv, &i, &out)) return 2; }
        else if (!strcmp(argv[i], "--no-window")) no_window = 1;
        else if (!strcmp(argv[i], "--no-ae")) no_ae = 1;
        else if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) { usage(argv[0]); return 0; }
        else {
            fprintf(stderr, "unknown argument %s\n", argv[i]);
            return 2;
        }
    }

    int base_exp = exposure >= 0 ? exposure : 24;
    int base_gain = gain >= 0 ? gain : 6;
    if (exp_a < 0) exp_a = base_exp;
    if (exp_b < 0) exp_b = exposure >= 0 ? exposure : std::max(1, base_exp / 3);
    if (gain_a < 0) gain_a = base_gain;
    if (gain_b < 0) gain_b = gain >= 0 ? gain : std::max(0, base_gain - 2);

    signal(SIGINT, on_sigint);
    if (!strcmp(cmd, "devices")) return cmd_devices();
    if (!strcmp(cmd, "grab")) {
        if (!out) {
            fprintf(stderr, "grab needs -o DIR\n");
            return 2;
        }
        return cmd_grab(out, exp_a, gain_a, exp_b, gain_b);
    }
    if (!strcmp(cmd, "snap")) {
        if (!out) {
            fprintf(stderr, "snap needs -o FILE.stl\n");
            return 2;
        }
        if (frames < 1) frames = 1;
        if (!scale_set) scale = 0.5;
        return cmd_snap(out, calib, frames, base_exp, base_gain, scale, min_mm, max_mm);
    }
    if (!strcmp(cmd, "scan")) {
        if (!scale_set) scale = 0.4;
        if (no_window) {
            if (!out) {
                fprintf(stderr, "scan --no-window needs -o FILE.stl\n");
                return 2;
            }
            return cmd_scan(out, calib, seconds, 1, no_ae, exp_a, gain_a, exp_b, gain_b,
                            scale, min_mm, max_mm);
        }
        return fox_app_main(argc, argv, calib, exp_a, gain_a, exp_b, gain_b, scale, min_mm, max_mm);
    }
    if (!strcmp(cmd, "asic-read")) return cmd_asic(asic_addr);
    usage(argv[0]);
    return 2;
}
