#include "openscan/openscan_oneshot.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>

namespace {

struct PlaneEq {
    double nx, ny, nz;
    double rhs;
};

struct Dot {
    int x, y;
    int value;
};

void rodrigues(const double r[3], double R[9]) {
    double th = std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
    R[0] = 1;
    R[1] = 0;
    R[2] = 0;
    R[3] = 0;
    R[4] = 1;
    R[5] = 0;
    R[6] = 0;
    R[7] = 0;
    R[8] = 1;
    if (th < 1e-12) return;
    double x = r[0] / th, y = r[1] / th, z = r[2] / th;
    double c = std::cos(th), s = std::sin(th), C = 1.0 - c;
    R[0] = c + x * x * C;
    R[1] = x * y * C - z * s;
    R[2] = x * z * C + y * s;
    R[3] = y * x * C + z * s;
    R[4] = c + y * y * C;
    R[5] = y * z * C - x * s;
    R[6] = z * x * C - y * s;
    R[7] = z * y * C + x * s;
    R[8] = c + z * z * C;
}

/* X_projector = R * X_cameraA + t. The stripe is the plane through the
 * projector centre whose image is the line u = slope * v + offset.
 * n_camera = R^T * n_projector, with R stored row-major. */
PlaneEq plane_of(const openscan_pinhole &proj, double slope, double offset, const double *R) {
    double fx = proj.fx, fy = proj.fy, cx = proj.cx, cy = proj.cy;
    double npx = fx;
    double npy = -fy * slope;
    double npz = cx - cy * slope - offset;
    PlaneEq e;
    e.nx = R[0] * npx + R[3] * npy + R[6] * npz;
    e.ny = R[1] * npx + R[4] * npy + R[7] * npz;
    e.nz = R[2] * npx + R[5] * npy + R[8] * npz;
    e.rhs = -(npx * proj.tvec[0] + npy * proj.tvec[1] + npz * proj.tvec[2]);
    return e;
}

void undistort_ray(double x, double y, const openscan_pinhole &cam, double ray[3]) {
    double xd = (x - cam.cx) / cam.fx;
    double yd = (y - cam.cy) / cam.fy;
    double xu = xd, yu = yd;
    for (int i = 0; i < 8; i++) {
        double r2 = xu * xu + yu * yu;
        double r4 = r2 * r2;
        double r6 = r4 * r2;
        double rad = 1.0 + cam.k1 * r2 + cam.k2 * r4 + cam.k3 * r6;
        double dx = 2.0 * cam.p1 * xu * yu + cam.p2 * (r2 + 2.0 * xu * xu);
        double dy = cam.p1 * (r2 + 2.0 * yu * yu) + 2.0 * cam.p2 * xu * yu;
        xu = (xd - dx) / rad;
        yu = (yd - dy) / rad;
    }
    ray[0] = xu;
    ray[1] = yu;
    ray[2] = 1.0;
}

bool intersect_plane(const double ray[3], const PlaneEq &e, float out[3]) {
    double den = e.nx * ray[0] + e.ny * ray[1] + e.nz * ray[2];
    if (std::fabs(den) < 1e-9) return false;
    double lam = e.rhs / den;
    if (lam <= 0.0) return false;
    double X = lam * ray[0];
    double Y = lam * ray[1];
    double Z = lam * ray[2];
    if (!(Z > 40.0 && Z < 900.0)) return false;
    out[0] = (float)X;
    out[1] = (float)Y;
    out[2] = (float)Z;
    return true;
}

}  // namespace

int openscan_oneshot_points(const uint8_t *pattern, int width, int height,
                       const openscan_calib *cal, float target_z_mm, openscan_shot *out, int cap) {
    if (!pattern || !cal || !out || cap <= 0 || width < 32 || height < 32) return -1;
    if (cal->nplanes < 8 || cal->cam[0].fx < 100.0 || cal->cam[2].fx < 100.0) return -1;
    if (target_z_mm < 80.f) target_z_mm = 80.f;
    if (target_z_mm > 800.f) target_z_mm = 800.f;

    const openscan_pinhole &cam = cal->cam[0];
    const openscan_pinhole &proj = cal->cam[2];
    double R[9];
    rodrigues(proj.rvec, R);
    std::vector<PlaneEq> planes((size_t)cal->nplanes);
    for (int i = 0; i < cal->nplanes; i++)
        planes[(size_t)i] = plane_of(proj, cal->plane[i].slope, cal->plane[i].offset, R);

    std::vector<int> integral((size_t)(width + 1) * (height + 1), 0);
    for (int y = 0; y < height; y++) {
        int row = 0;
        const uint8_t *src = pattern + (size_t)y * width;
        int *dst = integral.data() + (size_t)(y + 1) * (width + 1);
        const int *prev = integral.data() + (size_t)y * (width + 1);
        for (int x = 0; x < width; x++) {
            row += src[x];
            dst[x + 1] = prev[x + 1] + row;
        }
    }
    auto box = [&](int x0, int y0, int x1, int y1) -> int {
        x0 = std::max(0, x0);
        y0 = std::max(0, y0);
        x1 = std::min(width, x1);
        y1 = std::min(height, y1);
        if (x1 <= x0 || y1 <= y0) return 0;
        const int stride = width + 1;
        return integral[(size_t)y1 * stride + x1] - integral[(size_t)y0 * stride + x1] -
               integral[(size_t)y1 * stride + x0] + integral[(size_t)y0 * stride + x0];
    };

    std::vector<Dot> dots;
    dots.reserve(12000);
    for (int y = 1; y < height - 1; y++) {
        const uint8_t *row = pattern + (size_t)y * width;
        const uint8_t *up = row - width;
        const uint8_t *dn = row + width;
        for (int x = 1; x < width - 1; x++) {
            int v = row[x];
            if (v <= 40) continue;
            if (v < up[x] || v < dn[x] || v < row[x - 1] || v < row[x + 1]) continue;
            if (v < up[x - 1] || v < up[x + 1] || v < dn[x - 1] || v < dn[x + 1]) continue;
            int sum = box(x - 8, y - 8, x + 8, y + 8);
            int area = (std::min(width, x + 8) - std::max(0, x - 8)) *
                       (std::min(height, y + 8) - std::max(0, y - 8));
            if (area < 16) continue;
            if (v * area <= sum + 12 * area) continue;
            dots.push_back(Dot{x, y, v});
        }
    }
    if (dots.size() < 40) return 0;
    std::sort(dots.begin(), dots.end(), [](const Dot &a, const Dot &b) {
        if (a.y != b.y) return a.y < b.y;
        return a.x < b.x;
    });

    /* A stripe bends as the surface turns. Follow it from row to row instead
     * of cutting it wherever it drifts from its first x. */
    struct Stripe {
        std::vector<Dot> pts;
        double sx = 0;
        double sy = 0;
        double si = 0;
        int last_x = 0;
        int last_y = -1000;
    };
    std::vector<Stripe> cols;
    std::vector<int> live;
    for (const Dot &d : dots) {
        int best = -1;
        int best_dx = 4;
        for (size_t n = 0; n < live.size();) {
            Stripe &s = cols[(size_t)live[n]];
            /* Dots on one stripe are about ten pixels apart down the image. */
            if (d.y - s.last_y > 18) {
                live[n] = live.back();
                live.pop_back();
                continue;
            }
            int dx = std::abs(d.x - s.last_x);
            if (dx < best_dx) {
                best_dx = dx;
                best = live[n];
            }
            n++;
        }
        if (best < 0) {
            Stripe s;
            s.pts.push_back(d);
            s.sx = d.x;
            s.sy = d.y;
            s.si = d.value;
            s.last_x = d.x;
            s.last_y = d.y;
            live.push_back((int)cols.size());
            cols.push_back(std::move(s));
        } else {
            Stripe &s = cols[(size_t)best];
            s.pts.push_back(d);
            s.sx += d.x;
            s.sy += d.y;
            s.si += d.value;
            /* Stay on this column. A second dot in the same row must not
             * drag the stripe sideways into the next one. */
            s.last_x = (int)std::lround(0.7 * s.last_x + 0.3 * d.x);
            s.last_y = d.y;
        }
    }
    cols.erase(std::remove_if(cols.begin(), cols.end(),
                              [](const Stripe &c) {
                                  if (c.pts.size() <= 20) return true;
                                  return c.si / (double)c.pts.size() <= 70.0;
                              }),
               cols.end());
    std::sort(cols.begin(), cols.end(), [](const Stripe &a, const Stripe &b) {
        return a.sx / (double)a.pts.size() < b.sx / (double)b.pts.size();
    });
    /* One projector column is a single bright line. Fragments detected a
     * few pixels apart belong to it. The real gap on this Fox is about
     * twice the calibrated plane pitch. */
    {
        std::vector<double> gaps;
        for (size_t i = 1; i < cols.size(); i++) {
            double a = cols[i - 1].sx / (double)cols[i - 1].pts.size();
            double b = cols[i].sx / (double)cols[i].pts.size();
            if (b - a >= 8.0) gaps.push_back(b - a);
        }
        double period = 12.0;
        if (!gaps.empty()) {
            std::nth_element(gaps.begin(), gaps.begin() + (ptrdiff_t)gaps.size() / 2, gaps.end());
            period = gaps[gaps.size() / 2];
        }
        double min_sep = std::max(4.5, period * 0.55);
        std::vector<Stripe> merged;
        for (Stripe &c : cols) {
            if (!merged.empty()) {
                double mx = c.sx / (double)c.pts.size();
                double px = merged.back().sx / (double)merged.back().pts.size();
                if (mx - px < min_sep) {
                    Stripe &m = merged.back();
                    m.pts.insert(m.pts.end(), c.pts.begin(), c.pts.end());
                    m.sx += c.sx;
                    m.sy += c.sy;
                    m.si += c.si;
                    continue;
                }
            }
            merged.push_back(std::move(c));
        }
        cols.swap(merged);
    }
    if (cols.size() < 4) return 0;

    /* Image position of each plane on a frontal surface at target_z.
     * The Fox draws a bright stripe on every other calibrated plane, so the
     * picture pitch is about twice the plane pitch. The ratio is measured. */
    auto frontal_u = [&](const PlaneEq &e, double Z) -> double {
        if (std::fabs(e.nx) < 1e-8 || Z < 1.0) return NAN;
        double X = (e.rhs - e.nz * Z) / e.nx;
        return cam.fx * X / Z + cam.cx;
    };
    std::vector<double> pu;
    pu.reserve(planes.size());
    for (const PlaneEq &e : planes) {
        double u = frontal_u(e, target_z_mm);
        if (std::isfinite(u)) pu.push_back(u);
    }
    std::vector<double> pstep;
    for (size_t i = 1; i < pu.size(); i++) {
        double d = pu[i] - pu[i - 1];
        if (d > 1.0 && d < 40.0) pstep.push_back(d);
    }
    double plane_px = 6.0;
    if (!pstep.empty()) {
        std::nth_element(pstep.begin(), pstep.begin() + (ptrdiff_t)pstep.size() / 2, pstep.end());
        plane_px = pstep[pstep.size() / 2];
    }
    std::vector<double> gstep;
    for (size_t i = 1; i < cols.size(); i++) {
        double a = cols[i - 1].sx / (double)cols[i - 1].pts.size();
        double b = cols[i].sx / (double)cols[i].pts.size();
        if (b > a) gstep.push_back(b - a);
    }
    double image_px = plane_px * 2.0;
    if (!gstep.empty()) {
        std::nth_element(gstep.begin(), gstep.begin() + (ptrdiff_t)gstep.size() / 2, gstep.end());
        image_px = gstep[gstep.size() / 2];
    }
    int step = (int)std::lround(image_px / std::max(1.0, plane_px));
    if (step < 1) step = 1;
    if (step > 3) step = 3;

    /* Neighbouring bright stripes are usually `step` planes apart, but a
     * slant or a missed stripe changes that. Dynamic programming keeps the
     * depth continuous and near the distance slider. */
    const int ns = (int)cols.size();
    const int np = (int)planes.size();
    std::vector<float> zplane((size_t)ns * np, -1.f);
    double ray[3];
    for (int k = 0; k < ns; k++) {
        const Stripe &c = cols[(size_t)k];
        double mx = c.sx / (double)c.pts.size();
        double my = c.sy / (double)c.pts.size();
        undistort_ray(mx, my, cam, ray);
        for (int i = 0; i < np; i++) {
            float X[3];
            if (!intersect_plane(ray, planes[(size_t)i], X)) continue;
            if (X[2] < 100.f || X[2] > 700.f) continue;
            zplane[(size_t)k * np + i] = X[2];
        }
    }
    const float big = 1e8f;
    std::vector<float> dp((size_t)ns * np, big);
    std::vector<int> prev((size_t)ns * np, -1);
    for (int i = 0; i < np; i++) {
        float z = zplane[i];
        if (z < 0.f) continue;
        float dz = std::fabs(z - target_z_mm);
        if (dz > 90.f) continue;
        dp[i] = dz;
    }
    for (int k = 1; k < ns; k++) {
        for (int i = 0; i < np; i++) {
            float z = zplane[(size_t)k * np + i];
            if (z < 0.f) continue;
            float best = big;
            int from = -1;
            for (int d = 1; d <= 4; d++) {
                int j = i - d;
                if (j < 0) break;
                float pz = zplane[(size_t)(k - 1) * np + j];
                if (pz < 0.f) continue;
                float pen = (d == step) ? 0.f : (std::abs(d - step) == 1 ? 4.f : 12.f);
                float c = dp[(size_t)(k - 1) * np + j] + 0.35f * std::fabs(z - pz) + pen;
                if (c < best) {
                    best = c;
                    from = j;
                }
            }
            if (from >= 0) {
                dp[(size_t)k * np + i] = best + 0.02f * std::fabs(z - target_z_mm);
                prev[(size_t)k * np + i] = from;
            }
        }
    }
    int last = -1;
    float last_c = big;
    for (int i = 0; i < np; i++) {
        if (dp[(size_t)(ns - 1) * np + i] < last_c) {
            last_c = dp[(size_t)(ns - 1) * np + i];
            last = i;
        }
    }
    std::vector<int> assign((size_t)ns, -1);
    for (int k = ns - 1; k >= 0 && last >= 0; k--) {
        assign[(size_t)k] = last;
        last = prev[(size_t)k * np + last];
    }
    std::vector<float> band;
    band.reserve((size_t)ns);
    for (int k = 0; k < ns; k++) {
        int pi = assign[(size_t)k];
        if (pi < 0) continue;
        band.push_back(zplane[(size_t)k * np + pi]);
    }
    if ((int)band.size() * 2 < ns) return 0;
    std::nth_element(band.begin(), band.begin() + (ptrdiff_t)band.size() / 2, band.end());
    float zmed = band[band.size() / 2];
    /* The last planes are nearly edge-on, so a stripe that runs off the
     * table is placed tens of millimetres behind the object. */
    for (int k = 0; k < ns; k++) {
        int pi = assign[(size_t)k];
        if (pi < 0) continue;
        float z = zplane[(size_t)k * np + pi];
        if (z > zmed + 55.f || z < zmed - 40.f) assign[(size_t)k] = -1;
    }
    if (const char *dbg = std::getenv("OPENSCAN_ONESHOT_DEBUG")) {
        if (dbg[0] == '1') {
            fprintf(stderr, "stripes %d step %d\n", ns, step);
            int prev_i = -1;
            for (int k = 0; k < ns; k++) {
                int pi = assign[(size_t)k];
                float z = (pi >= 0) ? zplane[(size_t)k * np + pi] : -1.f;
                double mx = cols[(size_t)k].sx / (double)cols[(size_t)k].pts.size();
                fprintf(stderr, "  k %02d x %6.1f plane %3d dplane %2d z %6.1f n %zu\n", k, mx, pi,
                        prev_i < 0 || pi < 0 ? 0 : pi - prev_i, z, cols[(size_t)k].pts.size());
                if (pi >= 0) prev_i = pi;
            }
        }
    }

    int written = 0;
    for (int k = 0; k < ns && written < cap; k++) {
        int pi = assign[(size_t)k];
        if (pi < 0 || pi >= np) continue;
        const PlaneEq &e = planes[(size_t)pi];
        for (const Dot &d : cols[(size_t)k].pts) {
            if (written >= cap) break;
            undistort_ray((double)d.x, (double)d.y, cam, ray);
            float X[3];
            if (!intersect_plane(ray, e, X)) continue;
            openscan_shot &s = out[written++];
            s.x = X[0];
            s.y = X[1];
            s.z = X[2];
            s.u = (float)d.x;
            s.v = (float)d.y;
        }
    }
    return written;
}

int openscan_oneshot_depth(const uint8_t *pattern, int width, int height, const openscan_calib *cal,
                      float target_z_mm, float *depth_out) {
    if (!depth_out || width < 32 || height < 32) return -1;
    std::vector<openscan_shot> shots((size_t)width * 8);
    int n = openscan_oneshot_points(pattern, width, height, cal, target_z_mm, shots.data(),
                               (int)shots.size());
    if (n < 0) return -1;
    std::fill(depth_out, depth_out + (size_t)width * height, 0.f);
    for (int i = 0; i < n; i++) {
        int x = (int)std::lround(shots[(size_t)i].u);
        int y = (int)std::lround(shots[(size_t)i].v);
        if (x < 0 || y < 0 || x >= width || y >= height) continue;
        depth_out[(size_t)y * width + x] = shots[(size_t)i].z;
    }
    return n;
}
