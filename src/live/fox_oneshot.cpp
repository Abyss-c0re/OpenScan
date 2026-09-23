#include "fox/fox_oneshot.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
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
PlaneEq plane_of(const fox_pinhole &proj, double slope, double offset, const double *R) {
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

void undistort_ray(double x, double y, const fox_pinhole &cam, double ray[3]) {
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

int fox_oneshot_points(const uint8_t *pattern, int width, int height,
                       const fox_calib *cal, float target_z_mm, fox_shot *out, int cap) {
    if (!pattern || !cal || !out || cap <= 0 || width < 32 || height < 32) return -1;
    if (cal->nplanes < 8 || cal->cam[0].fx < 100.0 || cal->cam[2].fx < 100.0) return -1;
    if (target_z_mm < 80.f) target_z_mm = 80.f;
    if (target_z_mm > 800.f) target_z_mm = 800.f;

    const fox_pinhole &cam = cal->cam[0];
    const fox_pinhole &proj = cal->cam[2];
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
    std::sort(dots.begin(), dots.end(), [](const Dot &a, const Dot &b) { return a.x < b.x; });

    struct Column {
        std::vector<Dot> pts;
        double sx = 0;
        double sy = 0;
        double si = 0;
    };
    std::vector<Column> cols;
    const double dx_max = 3.0;
    for (const Dot &d : dots) {
        if (!cols.empty()) {
            Column &c = cols.back();
            double mean = c.sx / (double)c.pts.size();
            if (std::fabs((double)d.x - mean) <= dx_max) {
                c.pts.push_back(d);
                c.sx += d.x;
                c.sy += d.y;
                c.si += d.value;
                continue;
            }
        }
        Column c;
        c.pts.push_back(d);
        c.sx = d.x;
        c.sy = d.y;
        c.si = d.value;
        cols.push_back(std::move(c));
    }
    cols.erase(std::remove_if(cols.begin(), cols.end(),
                              [](const Column &c) {
                                  if (c.pts.size() <= 20) return true;
                                  return c.si / (double)c.pts.size() <= 70.0;
                              }),
               cols.end());
    std::sort(cols.begin(), cols.end(), [](const Column &a, const Column &b) {
        return a.sx / (double)a.pts.size() < b.sx / (double)b.pts.size();
    });
    if (cols.size() < 4) return 0;

    const Column &mid = cols[cols.size() / 2];
    double mx = mid.sx / (double)mid.pts.size();
    double my = mid.sy / (double)mid.pts.size();
    double ray[3];
    undistort_ray(mx, my, cam, ray);
    int anchor = -1;
    double anchor_err = 1e18;
    for (int i = 0; i < (int)planes.size(); i++) {
        float X[3];
        if (!intersect_plane(ray, planes[(size_t)i], X)) continue;
        double err = std::fabs((double)X[2] - (double)target_z_mm);
        if (err < anchor_err) {
            anchor_err = err;
            anchor = i;
        }
    }
    if (anchor < 0) return 0;
    /* A close object can show more stripes than the file lists. Keep the
     * stripes that have a plane. Dropping the whole frame is an empty 3D view. */
    int i0 = anchor - (int)cols.size() / 2;

    int written = 0;
    for (size_t k = 0; k < cols.size() && written < cap; k++) {
        int pi = i0 + (int)k;
        if (pi < 0 || pi >= (int)planes.size()) continue;
        const PlaneEq &e = planes[(size_t)pi];
        for (const Dot &d : cols[k].pts) {
            if (written >= cap) break;
            undistort_ray((double)d.x, (double)d.y, cam, ray);
            float X[3];
            if (!intersect_plane(ray, e, X)) continue;
            fox_shot &s = out[written++];
            s.x = X[0];
            s.y = X[1];
            s.z = X[2];
            s.u = (float)d.x;
            s.v = (float)d.y;
        }
    }
    return written;
}

int fox_oneshot_depth(const uint8_t *pattern, int width, int height, const fox_calib *cal,
                      float target_z_mm, float *depth_out) {
    if (!depth_out || width < 32 || height < 32) return -1;
    std::vector<fox_shot> shots((size_t)width * 8);
    int n = fox_oneshot_points(pattern, width, height, cal, target_z_mm, shots.data(),
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
