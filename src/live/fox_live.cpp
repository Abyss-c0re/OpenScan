#include "fox/fox_live.h"
#include "fox/fox_oneshot.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/rgbd/kinfu.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <strings.h>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace {

#include "mc_tables.inc"

struct Tri {
    cv::Vec3f a, b, c;
    float tone = 0.6f; /* camera gray, 0..1, so the model matches the picture */
};

static const int kEdgeCorner[12][2] = {
    {0, 1}, {1, 2}, {2, 3}, {3, 0},
    {4, 5}, {5, 6}, {6, 7}, {7, 4},
    {0, 4}, {1, 5}, {2, 6}, {3, 7},
};

static const int kCorner[8][3] = {
    {0, 0, 0}, {1, 0, 0}, {1, 1, 0}, {0, 1, 0},
    {0, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 1, 1},
};

static cv::Vec3f tri_normal(const Tri &t) {
    cv::Vec3f n = (t.b - t.a).cross(t.c - t.a);
    float len = std::sqrt(n.dot(n));
    if (len < 1e-12f) return cv::Vec3f(0, 0, 1);
    return n / len;
}

/* Geometry only. No serial, no calibration text, no device id. */
static int write_stl(const char *path, const std::vector<Tri> &tris) {
    FILE *fp = fopen(path, "wb");
    if (!fp) return -1;
    char header[80];
    memset(header, 0, sizeof header);
    memcpy(header, "fox3d mesh", 10);
    fwrite(header, 1, 80, fp);
    uint32_t n = (uint32_t)tris.size();
    fwrite(&n, 4, 1, fp);
    for (const Tri &t : tris) {
        cv::Vec3f nn = tri_normal(t);
        float rec[12] = {
            nn[0], nn[1], nn[2],
            t.a[0], t.a[1], t.a[2],
            t.b[0], t.b[1], t.b[2],
            t.c[0], t.c[1], t.c[2],
        };
        fwrite(rec, 4, 12, fp);
        uint16_t attr = 0;
        fwrite(&attr, 2, 1, fp);
    }
    fclose(fp);
    return 0;
}

static int write_obj(const char *path, const std::vector<Tri> &tris) {
    FILE *fp = fopen(path, "w");
    if (!fp) return -1;
    fprintf(fp, "# fox3d mesh\n");
    int idx = 1;
    for (const Tri &t : tris) {
        cv::Vec3f n = tri_normal(t);
        const cv::Vec3f *p[3] = {&t.a, &t.b, &t.c};
        for (int i = 0; i < 3; i++) {
            fprintf(fp, "v %.6f %.6f %.6f\n", (*p[i])[0], (*p[i])[1], (*p[i])[2]);
            fprintf(fp, "vn %.6f %.6f %.6f\n", n[0], n[1], n[2]);
        }
        fprintf(fp, "f %d//%d %d//%d %d//%d\n", idx, idx, idx + 1, idx + 1, idx + 2, idx + 2);
        idx += 3;
    }
    fclose(fp);
    return 0;
}

static int write_ply(const char *path, const std::vector<Tri> &tris) {
    FILE *fp = fopen(path, "w");
    if (!fp) return -1;
    fprintf(fp, "ply\nformat ascii 1.0\n");
    fprintf(fp, "element vertex %zu\n", tris.size() * 3);
    fprintf(fp, "property float x\nproperty float y\nproperty float z\n");
    fprintf(fp, "property float nx\nproperty float ny\nproperty float nz\n");
    fprintf(fp, "element face %zu\n", tris.size());
    fprintf(fp, "property list uchar int vertex_indices\n");
    fprintf(fp, "end_header\n");
    for (const Tri &t : tris) {
        cv::Vec3f n = tri_normal(t);
        const cv::Vec3f *p[3] = {&t.a, &t.b, &t.c};
        for (int i = 0; i < 3; i++)
            fprintf(fp, "%.6f %.6f %.6f %.6f %.6f %.6f\n",
                    (*p[i])[0], (*p[i])[1], (*p[i])[2], n[0], n[1], n[2]);
    }
    for (size_t i = 0; i < tris.size(); i++)
        fprintf(fp, "3 %zu %zu %zu\n", i * 3, i * 3 + 1, i * 3 + 2);
    fclose(fp);
    return 0;
}

static int write_mesh(const char *path, const std::vector<Tri> &tris) {
    const char *dot = strrchr(path, '.');
    if (dot && !strcasecmp(dot, ".obj")) return write_obj(path, tris);
    if (dot && !strcasecmp(dot, ".ply")) return write_ply(path, tris);
    if (!dot || !strcasecmp(dot, ".stl")) return write_stl(path, tris);
    return -1;
}

/* sdf < 0 is inside. When weight is non-empty, a corner with weight <= 0 is
 * unknown and triangles that touch it are dropped, so the shell stays on the
 * measured surface instead of closing against empty space. */
static void marching_cubes(const std::vector<float> &sdf, const std::vector<float> *weight,
                           int nx, int ny, int nz,
                           float ox, float oy, float oz, float voxel,
                           std::vector<Tri> &tris) {
    auto id = [&](int x, int y, int z) -> size_t {
        return (size_t)((z * ny + y) * nx + x);
    };
    auto known = [&](size_t i) -> bool {
        return !weight || (*weight)[i] > 0.f;
    };
    for (int z = 0; z < nz - 1; z++) {
        for (int y = 0; y < ny - 1; y++) {
            for (int x = 0; x < nx - 1; x++) {
                float s[8];
                int known_c[8];
                int cube = 0;
                for (int c = 0; c < 8; c++) {
                    int cx = x + kCorner[c][0];
                    int cy = y + kCorner[c][1];
                    int cz = z + kCorner[c][2];
                    size_t i = id(cx, cy, cz);
                    known_c[c] = known(i);
                    s[c] = sdf[i];
                    if (known_c[c] && s[c] < 0.f) cube |= 1 << c;
                }
                int edges = edge_table[cube];
                if (!edges) continue;
                cv::Vec3f vert[12];
                bool vert_ok[12];
                memset(vert_ok, 0, sizeof vert_ok);
                for (int e = 0; e < 12; e++) {
                    if (!(edges & (1 << e))) continue;
                    int c0 = kEdgeCorner[e][0];
                    int c1 = kEdgeCorner[e][1];
                    if (!known_c[c0] || !known_c[c1]) continue;
                    float s0 = s[c0], s1 = s[c1];
                    float denom = s0 - s1;
                    float t = std::fabs(denom) < 1e-8f ? 0.5f : s0 / denom;
                    if (t < 0.f) t = 0.f;
                    if (t > 1.f) t = 1.f;
                    cv::Vec3f p0(ox + (x + kCorner[c0][0]) * voxel,
                                 oy + (y + kCorner[c0][1]) * voxel,
                                 oz + (z + kCorner[c0][2]) * voxel);
                    cv::Vec3f p1(ox + (x + kCorner[c1][0]) * voxel,
                                 oy + (y + kCorner[c1][1]) * voxel,
                                 oz + (z + kCorner[c1][2]) * voxel);
                    vert[e] = p0 + t * (p1 - p0);
                    vert_ok[e] = true;
                }
                const int *tri = triangle_table[cube];
                for (int t = 0; tri[t] != -1 && t < 15; t += 3) {
                    int e0 = tri[t], e1 = tri[t + 1], e2 = tri[t + 2];
                    if (!vert_ok[e0] || !vert_ok[e1] || !vert_ok[e2]) continue;
                    Tri face{vert[e0], vert[e1], vert[e2]};
                    cv::Vec3f cr = (face.b - face.a).cross(face.c - face.a);
                    if (cr.dot(cr) < 1e-12f) continue;
                    tris.push_back(face);
                }
            }
        }
    }
}

static cv::Mat pinhole_k(const fox_pinhole &p, double scale) {
    cv::Mat K(3, 3, CV_64F);
    K.at<double>(0, 0) = p.fx * scale;
    K.at<double>(0, 1) = 0;
    K.at<double>(0, 2) = p.cx * scale;
    K.at<double>(1, 0) = 0;
    K.at<double>(1, 1) = p.fy * scale;
    K.at<double>(1, 2) = p.cy * scale;
    K.at<double>(2, 0) = 0;
    K.at<double>(2, 1) = 0;
    K.at<double>(2, 2) = 1;
    return K;
}

static cv::Mat pinhole_d(const fox_pinhole &p) {
    cv::Mat D(1, 5, CV_64F);
    D.at<double>(0) = p.k1;
    D.at<double>(1) = p.k2;
    D.at<double>(2) = p.p1;
    D.at<double>(3) = p.p2;
    D.at<double>(4) = p.k3;
    return D;
}

struct Geom {
    cv::Mat map1x, map1y, map2x, map2y, Q;
    cv::Ptr<cv::StereoSGBM> sgbm;
    float fx = 0, fy = 0, cx = 0, cy = 0;
    float baseline = 0;
    cv::Size size;
};

static Geom build_geom(const fox_calib *cal, const fox_scan_opts &opt, cv::Size size) {
    Geom g;
    g.size = size;
    const fox_pinhole &c1 = cal->cam[0];
    const fox_pinhole &c2 = cal->cam[1];
    double scale_x = (double)size.width / cal->width;
    double scale_y = (double)size.height / cal->height;
    double scale = 0.5 * (scale_x + scale_y);
    cv::Mat K1 = pinhole_k(c1, scale);
    cv::Mat K2 = pinhole_k(c2, scale);
    /* Principal point was scaled by a single factor above; correct it if the
     * resize is not isotropic. */
    K1.at<double>(0, 2) = c1.cx * scale_x;
    K1.at<double>(1, 2) = c1.cy * scale_y;
    K1.at<double>(0, 0) = c1.fx * scale_x;
    K1.at<double>(1, 1) = c1.fy * scale_y;
    K2.at<double>(0, 2) = c2.cx * scale_x;
    K2.at<double>(1, 2) = c2.cy * scale_y;
    K2.at<double>(0, 0) = c2.fx * scale_x;
    K2.at<double>(1, 1) = c2.fy * scale_y;
    cv::Mat D1 = pinhole_d(c1);
    cv::Mat D2 = pinhole_d(c2);
    cv::Mat rvec(3, 1, CV_64F);
    rvec.at<double>(0) = c2.rvec[0];
    rvec.at<double>(1) = c2.rvec[1];
    rvec.at<double>(2) = c2.rvec[2];
    cv::Mat R, T(3, 1, CV_64F);
    cv::Rodrigues(rvec, R);
    T.at<double>(0) = c2.tvec[0];
    T.at<double>(1) = c2.tvec[1];
    T.at<double>(2) = c2.tvec[2];
    cv::Mat R1, R2, P1, P2;
    /* alpha 0 makes OpenCV zoom the shared field of view until the focal length
     * is several times too long, so the working-distance disparity falls outside
     * the search. alpha < 0 keeps the calibrated focal length. */
    /* alpha -1 keeps the calibrated focal length. alpha 0 zooms until the
     * picture is a few valid pixels wide and the pair is useless. */
    cv::stereoRectify(K1, D1, K2, D2, size, R, T, R1, R2, P1, P2, g.Q,
                      cv::STEREO_ZERO_DISPARITY, -1, size);
    cv::initUndistortRectifyMap(K1, D1, R1, P1, size, CV_32FC1, g.map1x, g.map1y);
    cv::initUndistortRectifyMap(K2, D2, R2, P2, size, CV_32FC1, g.map2x, g.map2y);
    g.fx = (float)P1.at<double>(0, 0);
    g.fy = (float)P1.at<double>(1, 1);
    g.cx = (float)P1.at<double>(0, 2);
    g.cy = (float)P1.at<double>(1, 2);
    g.baseline = (float)std::fabs(1.0 / g.Q.at<double>(3, 2));

    double baseline = g.baseline;
    double near_mm = std::max(40.0, opt.min_mm * 0.85);
    int block = 7;
    /* SGBM rejects a search that reaches the right edge of the image, and a
     * negative minimum disparity makes it return an empty map. Stay inside
     * both limits. 256 px is about 140 mm at this focal length. */
    int span = (int)std::lround(g.fx * baseline / near_mm);
    span = (span + 15) & ~15;
    if (span < 64) span = 64;
    /* With this rig the match sits at a negative disparity (the feature in
     * camera B is to the right of camera A). Searching [-span, 0) covers it.
     * A range that touches the right edge of the image makes SGBM abort. */
    if (span > 320) span = 320;
    g.sgbm = cv::StereoSGBM::create(-span, span, block);
    g.sgbm->setP1(8 * block * block);
    g.sgbm->setP2(32 * block * block);
    /* The projector draws a fine repeating grid. A strict uniqueness cut
     * treats that grid as ambiguous and deletes the surface. */
    g.sgbm->setUniquenessRatio(5);
    g.sgbm->setSpeckleWindowSize(40);
    g.sgbm->setSpeckleRange(2);
    g.sgbm->setDisp12MaxDiff(8);
    g.sgbm->setPreFilterCap(31);
    g.sgbm->setMode(cv::StereoSGBM::MODE_SGBM);
    return g;
}

struct DepthFrame {
    cv::Mat depth_m; /* CV_32F metres, 0 = no measurement */
    cv::Mat rect;    /* rectified left, 8-bit */
    int valid = 0;
    double median_mm = 0;
};

static DepthFrame match_depth(const Geom &g, const cv::Mat &left, const cv::Mat &right,
                              double min_mm, double max_mm) {
    DepthFrame f;
    cv::remap(left, f.rect, g.map1x, g.map1y, cv::INTER_LINEAR);
    cv::Mat recR;
    cv::remap(right, recR, g.map2x, g.map2y, cv::INTER_LINEAR);
    /* Knock the projected dots down to the shading underneath them.
     * Block matching cannot tell one identical dot from the next. */
    cv::Mat left_b, right_b;
    cv::GaussianBlur(f.rect, left_b, cv::Size(0, 0), 0.8);
    cv::GaussianBlur(recR, right_b, cv::Size(0, 0), 0.8);
    cv::Mat disp16;
    g.sgbm->compute(left_b, right_b, disp16);
    cv::Mat disp;
    disp16.convertTo(disp, CV_32F, 1.0 / 16.0);
    cv::Mat xyz;
    cv::reprojectImageTo3D(disp, xyz, g.Q, true);

    f.depth_m = cv::Mat(xyz.size(), CV_32F, cv::Scalar(0));
    std::vector<float> samples;
    samples.reserve((size_t)xyz.total() / 8);
    for (int y = 0; y < xyz.rows; y++) {
        const cv::Vec3f *row = xyz.ptr<cv::Vec3f>(y);
        const float *drow = disp.ptr<float>(y);
        float *out = f.depth_m.ptr<float>(y);
        for (int x = 0; x < xyz.cols; x++) {
            float d = std::fabs(drow[x]);
            if (d < 2.f) continue;
            float z = g.fx * g.baseline / d;
            if (!std::isfinite(z)) continue;
            if (z < min_mm || z > max_mm) continue;
            out[x] = z * 0.001f;
            f.valid++;
            if (((y * xyz.cols + x) & 7) == 0) samples.push_back(z);
        }
    }
    if (!samples.empty()) {
        size_t mid = samples.size() / 2;
        std::nth_element(samples.begin(), samples.begin() + (ptrdiff_t)mid, samples.end());
        f.median_mm = samples[mid];
    }
    static int report = 0;
    if (f.valid < 500 && (report++ % 20) == 0) {
        std::vector<float> zany;
        double dmax = 0;
        int dpos = 0;
        for (int y = 0; y < xyz.rows; y += 2) {
            const cv::Vec3f *row = xyz.ptr<cv::Vec3f>(y);
            const float *drow = disp.ptr<float>(y);
            for (int x = 0; x < xyz.cols; x += 2) {
                if (std::fabs(drow[x]) > dmax) dmax = std::fabs(drow[x]);
                float z = row[x][2];
                if (std::isfinite(z) && z < 0.f) z = -z;
                if (std::fabs(drow[x]) <= 2.f || !std::isfinite(z)) continue;
                dpos++;
                zany.push_back(z);
            }
        }
        float zlo = 0, zhi = 0, zmed = 0;
        if (!zany.empty()) {
            std::nth_element(zany.begin(), zany.begin() + (ptrdiff_t)zany.size() / 2, zany.end());
            zmed = zany[zany.size() / 2];
            zlo = *std::min_element(zany.begin(), zany.end());
            zhi = *std::max_element(zany.begin(), zany.end());
        }
        fprintf(stderr, "depth gate kept %d   disp>2 (sampled) %d  max-disp %.0f  Z %.0f..%.0f med %.0f mm\n",
                f.valid, dpos, dmax, zlo, zhi, zmed);
    }
    /* Keep KinFu's 3x3 bilateral kernel off the invalid zeros. */
    cv::Mat keep(f.depth_m.size(), CV_8U, cv::Scalar(0));
    for (int y = 1; y < f.depth_m.rows - 1; y++) {
        const float *r = f.depth_m.ptr<float>(y);
        uint8_t *k = keep.ptr<uint8_t>(y);
        for (int x = 1; x < f.depth_m.cols - 1; x++) {
            float z = r[x];
            if (z <= 0.f) continue;
            int n = 0;
            for (int dy = -1; dy <= 1; dy++) {
                const float *rr = f.depth_m.ptr<float>(y + dy);
                for (int dx = -1; dx <= 1; dx++) {
                    float o = rr[x + dx];
                    if (o > 0.f && std::fabs(o - z) < 0.012f) n++;
                }
            }
            if (n >= 4) k[x] = 255;
        }
    }
    f.depth_m.setTo(0, keep == 0);
    f.valid = cv::countNonZero(f.depth_m > 0);
    return f;
}

static bool read_cloud(const cv::Mat &pts, const cv::Mat &nrm,
                       std::vector<cv::Vec3f> &P, std::vector<cv::Vec3f> &N) {
    if (pts.empty()) return false;
    cv::Mat p = pts, n = nrm;
    if (p.channels() == 4 || p.channels() == 3)
        p = p.reshape(1, (int)p.total());
    if (!n.empty() && (n.channels() == 4 || n.channels() == 3))
        n = n.reshape(1, (int)n.total());
    if (p.type() != CV_32F || p.cols < 3) return false;
    int comp = std::min(p.cols, 3);
    bool have_n = !n.empty() && n.type() == CV_32F && n.rows == p.rows && n.cols >= 3;
    P.reserve((size_t)p.rows);
    N.reserve((size_t)p.rows);
    for (int i = 0; i < p.rows; i++) {
        const float *pr = p.ptr<float>(i);
        if (!std::isfinite(pr[0]) || !std::isfinite(pr[1]) || !std::isfinite(pr[2])) continue;
        cv::Vec3f point(pr[0], pr[1], pr[2]);
        cv::Vec3f normal(0, 0, -1);
        if (have_n) {
            const float *nr = n.ptr<float>(i);
            normal = cv::Vec3f(nr[0], nr[1], nr[2]);
            float ln = std::sqrt(normal.dot(normal));
            if (!std::isfinite(ln) || ln < 1e-4f) continue;
            normal /= ln;
        }
        (void)comp;
        P.push_back(point);
        N.push_back(normal);
    }
    return !P.empty();
}

static int mesh_points(const std::vector<cv::Vec3f> &P, const std::vector<cv::Vec3f> &N,
                       std::vector<Tri> &tris) {
    if (P.size() < 80) return 0;
    cv::Vec3f lo = P[0], hi = P[0];
    for (const cv::Vec3f &p : P) {
        lo[0] = std::min(lo[0], p[0]); lo[1] = std::min(lo[1], p[1]); lo[2] = std::min(lo[2], p[2]);
        hi[0] = std::max(hi[0], p[0]); hi[1] = std::max(hi[1], p[1]); hi[2] = std::max(hi[2], p[2]);
    }
    float ext = std::max(hi[0] - lo[0], std::max(hi[1] - lo[1], hi[2] - lo[2]));
    if (!(ext > 1.f) || ext > 2000.f) return 0;
    float voxel = std::max(1.5f, ext / 180.f);
    float trunc = voxel * 3.f;
    int margin = 3;
    int nx = (int)std::ceil((hi[0] - lo[0]) / voxel) + 1 + 2 * margin;
    int ny = (int)std::ceil((hi[1] - lo[1]) / voxel) + 1 + 2 * margin;
    int nz = (int)std::ceil((hi[2] - lo[2]) / voxel) + 1 + 2 * margin;
    if (nx < 4 || ny < 4 || nz < 4) return 0;
    if ((long)nx * ny * nz > 200L * 200 * 200) return 0;
    float ox = lo[0] - margin * voxel;
    float oy = lo[1] - margin * voxel;
    float oz = lo[2] - margin * voxel;
    std::vector<float> sdf((size_t)nx * ny * nz, 0.f);
    std::vector<float> w(sdf.size(), 0.f);
    int r = (int)std::ceil(trunc / voxel);
    auto id = [&](int x, int y, int z) -> size_t {
        return (size_t)((z * ny + y) * nx + x);
    };
    for (size_t i = 0; i < P.size(); i++) {
        const cv::Vec3f &p = P[i];
        const cv::Vec3f &n = N[i];
        int ix = (int)std::lround((p[0] - ox) / voxel);
        int iy = (int)std::lround((p[1] - oy) / voxel);
        int iz = (int)std::lround((p[2] - oz) / voxel);
        for (int dz = -r; dz <= r; dz++) {
            int z = iz + dz;
            if (z < 0 || z >= nz) continue;
            for (int dy = -r; dy <= r; dy++) {
                int y = iy + dy;
                if (y < 0 || y >= ny) continue;
                for (int dx = -r; dx <= r; dx++) {
                    int x = ix + dx;
                    if (x < 0 || x >= nx) continue;
                    cv::Vec3f c(ox + (x + 0.5f) * voxel, oy + (y + 0.5f) * voxel, oz + (z + 0.5f) * voxel);
                    float signed_mm = n.dot(c - p);
                    if (std::fabs(signed_mm) > trunc) continue;
                    float s = signed_mm / trunc;
                    size_t k = id(x, y, z);
                    float w0 = w[k];
                    if (w0 > 24.f) w0 = 24.f;
                    w[k] = w0 + 1.f;
                    sdf[k] = (sdf[k] * w0 + s) / w[k];
                }
            }
        }
    }
    marching_cubes(sdf, &w, nx, ny, nz, ox, oy, oz, voxel, tris);
    return (int)tris.size();
}

} // namespace

struct FoxButton {
    int x, y, w, h;
    int id;
};

struct fox_live {
    fox_calib calib;
    fox_scan_opts opt;
    Geom geom;
    cv::Size size;
    int locked = 0;          /* 0 unknown, 1 USB A is calib cam1, 2 USB B is cam1 */
    int fused = 0;
    int lost = 0;
    int mode = FOX_MODE_STOP;
    int quit = 0;
    int save_req = 0;
    cv::Ptr<cv::kinfu::KinFu> kf;
    cv::Mat last_model;
    cv::Mat align_gray;
    cv::Mat preview;
    int preview_w = 0;
    int preview_h = 0;
    cv::Mat depth_hist[3];
    int hist_i = 0;
    int hist_n = 0;
    char line1[160];
    char line2[160];
    std::vector<cv::Vec3f> cloud, cloud_n;
    std::unordered_map<uint64_t, uint32_t> vox;
    cv::Matx33f pose_R = cv::Matx33f::eye();
    cv::Vec3f pose_t = cv::Vec3f(0, 0, 0);
    float pose_s = 1.f;
    float distance_mm = 220.f;
    int shape = FOX_SHAPE_MOLD;
    int covered = 0;
    int tracked_frames = 0;
    std::vector<Tri> shell;
    size_t shell_mark = 0;
    cv::Matx33f shell_R = cv::Matx33f::eye();
    int shell_set = 0;
    struct Sample {
        cv::Vec3f p;
        float w = 0.f;
        int n = 0;
    };
    struct Wedge {
        std::vector<Tri> tris;
        float quality = 10.f;
        int filled = 0;
    };
    std::vector<Wedge> wedge;
    /* Cylindrical samples. A repeat pass blends into the same cells and
     * fills any that were still empty, so the surface gains detail. */
    std::unordered_map<uint64_t, Sample> cells;
    struct TriRef {
        cv::Vec3f p[3];
        uint64_t k[3];
        int bin;
        float tone = 0.6f;
    };
    std::vector<TriRef> refs;
    std::unordered_set<uint64_t> seen_cells;
    int detail = 1;
    char seen_dir[72] = {};
    float object_yaw = 0.f;
    cv::Mat track_ref;
    int track_have = 0;
    int track_miss = 0;
    float track_cx = 0.f;
    float track_cy = 0.f;
    int track_side = 0;
    std::vector<cv::Vec3f> track_pts;
    cv::Vec3f track_axis = cv::Vec3f(0, 0, 0);
    int last_track = -1;
    cv::Vec3f axis0 = cv::Vec3f(0, 0, 0);
    int have_axis = 0;
    float radius_mm = 27.f;
    float span_y0 = 0.f, span_y1 = 0.f;
    int scanned_bins = 0;
    float yaw = 0.18f;
    float pitch = -0.10f;
    float dist = -1.f;
    int spin = 0;
    int user_zoom = 0;
    int dragging = 0;
    int drag_x = 0;
    int drag_y = 0;
    int view_x = 0, view_y = 0, view_w = 0, view_h = 0;
    std::vector<FoxButton> buttons;
    /* Full-resolution undistort of each sensor, in the sensor's own orientation. */
    cv::Mat und_ax, und_ay, und_bx, und_by;
    double view_fx = 0, view_fy = 0, view_cx = 0, view_cy = 0;
    cv::Mat stereo_a, stereo_b;
};

/* Remove lens distortion without zooming the picture. alpha 1 keeps every
 * source pixel, so the skull is not cropped again to hide the bend. */
static void build_undistort(const fox_pinhole &cam, cv::Size size, cv::Mat &mx, cv::Mat &my,
                            double &fx, double &fy, double &cx, double &cy) {
    cv::Mat K = pinhole_k(cam, 1.0);
    cv::Mat D = pinhole_d(cam);
    cv::Mat neu = cv::getOptimalNewCameraMatrix(K, D, size, 1.0, size);
    cv::initUndistortRectifyMap(K, D, cv::Mat(), neu, size, CV_32FC1, mx, my);
    fx = neu.at<double>(0, 0);
    fy = neu.at<double>(1, 1);
    cx = neu.at<double>(0, 2);
    cy = neu.at<double>(1, 2);
}

fox_live *fox_live_create(const fox_calib *calib, const fox_scan_opts *opt) {
    fox_live *L = new fox_live();
    L->calib = *calib;
    L->opt = *opt;
    if (L->opt.scale < 0.25) L->opt.scale = 0.25;
    if (L->opt.scale > 1.0) L->opt.scale = 1.0;
    if (L->opt.min_mm < 50) L->opt.min_mm = 50;
    if (L->opt.max_mm <= L->opt.min_mm) L->opt.max_mm = L->opt.min_mm + 50;
    int sw = (int)std::lround(calib->width * L->opt.scale);
    int sh = (int)std::lround(calib->height * L->opt.scale);
    sw = std::max(32, sw & ~7);
    sh = std::max(32, sh & ~7);
    L->size = cv::Size(sw, sh);
    L->geom = build_geom(&L->calib, L->opt, L->size);
    if (calib->width > 16 && calib->height > 16) {
        cv::Size full(calib->width, calib->height);
        double afx = 0, afy = 0, acx = 0, acy = 0;
        build_undistort(calib->cam[0], full, L->und_ax, L->und_ay, afx, afy, acx, acy);
        build_undistort(calib->cam[1], full, L->und_bx, L->und_by, L->view_fx, L->view_fy, L->view_cx,
                        L->view_cy);
    }
    snprintf(L->line1, sizeof L->line1, "stopped");
    L->wedge.resize(72);
    snprintf(L->line2, sizeof L->line2, "Start begins the model. Drag it to orbit. Close stays closed.");
    /* The AMD OpenCL path in KinFu fails to build its raycast kernel. */
    cv::ocl::setUseOpenCL(false);
    fprintf(stderr, "stereo %dx%d  fx %.1f  disparity search %d px\n",
            sw, sh, L->geom.fx, L->geom.sgbm->getNumDisparities());
    return L;
}

void fox_live_destroy(fox_live *live) { delete live; }

void fox_live_reset(fox_live *live) {
    if (!live) return;
    live->kf.release();
    live->last_model.release();
    live->fused = 0;
    live->lost = 0;
    live->locked = 0;
    live->hist_n = 0;
    live->hist_i = 0;
    live->cloud.clear();
    live->cloud_n.clear();
    live->vox.clear();
    live->pose_R = cv::Matx33f::eye();
    live->pose_t = cv::Vec3f(0, 0, 0);
    live->pose_s = 1.f;
    live->covered = 0;
    live->tracked_frames = 0;
    live->shell.clear();
    live->shell_mark = 0;
    live->shell_R = cv::Matx33f::eye();
    live->shell_set = 0;
    live->wedge.assign(72, fox_live::Wedge{});
    live->cells.clear();
    live->refs.clear();
    live->seen_cells.clear();
    live->detail = 1;
    memset(live->seen_dir, 0, sizeof live->seen_dir);
    live->object_yaw = 0.f;
    live->track_ref.release();
    live->track_have = 0;
    live->track_miss = 0;
    live->track_cx = 0.f;
    live->track_cy = 0.f;
    live->track_pts.clear();
    live->track_side = 0;
    live->last_track = -1;
    live->have_axis = 0;
    live->scanned_bins = 0;
    live->dist = -1.f;
    live->user_zoom = 0;
    live->mode = FOX_MODE_STOP;
    snprintf(live->line1, sizeof live->line1,
             "STOPPED   scanned 0°   not scanned 360°   detail x1   0 tris");
}

void fox_live_set_mode(fox_live *live, int mode) {
    if (!live) return;
    if (mode != FOX_MODE_SCAN && mode != FOX_MODE_PAUSE) mode = FOX_MODE_STOP;
    live->mode = mode;
}

void fox_live_set_shape(fox_live *live, int shape) {
    if (!live) return;
    if (shape != FOX_SHAPE_MEASURED) shape = FOX_SHAPE_MOLD;
    if (live->shape == shape) return;
    int mode = live->mode;
    float dist_mm = live->distance_mm;
    float yaw = live->yaw;
    float pitch = live->pitch;
    float dist = live->dist;
    int zoom = live->user_zoom;
    fox_live_reset(live);
    live->shape = shape;
    live->mode = mode;
    live->distance_mm = dist_mm;
    live->yaw = yaw;
    live->pitch = pitch;
    live->dist = dist;
    live->user_zoom = zoom;
}

int fox_live_shape(const fox_live *live) {
    return live ? live->shape : FOX_SHAPE_MOLD;
}

static uint64_t voxel_key(int ix, int iy, int iz);

void fox_live_set_distance_mm(fox_live *live, float mm) {
    if (!live) return;
    if (mm < 80.f) mm = 80.f;
    if (mm > 500.f) mm = 500.f;
    float old = live->distance_mm > 1.f ? live->distance_mm : mm;
    float k = old > 1.f ? mm / old : 1.f;
    live->distance_mm = mm;
    /* Measured depth is already in millimetres. Scaling it would lie.
     * Mold size is the working distance, so the solid grows with the slider. */
    if (live->shape != FOX_SHAPE_MOLD) return;
    if (k < 0.995f || k > 1.005f) {
        auto scale_p = [k](cv::Vec3f &p) { p *= k; };
        if (live->have_axis) scale_p(live->axis0);
        live->radius_mm *= k;
        if (live->span_y0 < 1e8f && live->span_y1 > live->span_y0) {
            live->span_y0 *= k;
            live->span_y1 *= k;
        }
        for (auto &kv : live->cells) scale_p(kv.second.p);
        for (auto &ref : live->refs)
            for (int i = 0; i < 3; i++) scale_p(ref.p[i]);
        for (auto &wd : live->wedge)
            for (Tri &t : wd.tris) {
                scale_p(t.a);
                scale_p(t.b);
                scale_p(t.c);
            }
        for (cv::Vec3f &p : live->cloud) scale_p(p);
        scale_p(live->pose_t);
        live->vox.clear();
        const float voxel = 2.5f;
        for (size_t i = 0; i < live->cloud.size(); i++) {
            const cv::Vec3f &p = live->cloud[i];
            int ix = (int)std::floor(p[0] / voxel);
            int iy = (int)std::floor(p[1] / voxel);
            int iz = (int)std::floor(p[2] / voxel);
            live->vox.emplace(voxel_key(ix, iy, iz), (uint32_t)i);
        }
    }
}

float fox_live_distance_mm(const fox_live *live) {
    return live ? live->distance_mm : 220.f;
}

int fox_live_mode(const fox_live *live) { return live ? live->mode : FOX_MODE_STOP; }
int fox_live_points(const fox_live *live) {
    if (!live) return 0;
    int n = 0;
    for (const auto &w : live->wedge) n += (int)w.tris.size();
    if (n > 0) return n;
    if (live->shell.size() >= 80) return (int)live->shell.size();
    return (int)live->cloud.size();
}
int fox_live_take_save(fox_live *live) {
    if (!live || !live->save_req) return 0;
    live->save_req = 0;
    return 1;
}
int fox_live_quit(const fox_live *live) { return live && live->quit; }

void fox_live_key(fox_live *live, int key) {
    if (!live) return;
    if (key == 's' || key == 'S') live->save_req = 1;
    else if (key == ' ') live->mode = live->mode == FOX_MODE_SCAN ? FOX_MODE_PAUSE : FOX_MODE_SCAN;
    else if (key == 'q' || key == 'Q' || key == 27) live->quit = 1;
}

void fox_live_mouse(fox_live *live, int event, int x, int y, int flags) {
    if (!live) return;
    if (event == cv::EVENT_LBUTTONDOWN) {
        for (const FoxButton &b : live->buttons) {
            if (x >= b.x && x < b.x + b.w && y >= b.y && y < b.y + b.h) {
                if (b.id == 1) live->mode = FOX_MODE_SCAN;
                else if (b.id == 2) live->mode = FOX_MODE_PAUSE;
                else if (b.id == 3) live->mode = FOX_MODE_STOP;
                else if (b.id == 4) live->save_req = 1;
                else if (b.id == 5) live->quit = 1;
                return;
            }
        }
        if (live->view_w > 0 &&
            (x < live->view_x || x >= live->view_x + live->view_w ||
             y < live->view_y || y >= live->view_y + live->view_h))
            return;
        live->dragging = 1;
        live->spin = 0;
        live->drag_x = x;
        live->drag_y = y;
    } else if (event == cv::EVENT_MOUSEMOVE && live->dragging) {
        live->yaw += (x - live->drag_x) * 0.01f;
        live->pitch += (y - live->drag_y) * 0.01f;
        if (live->pitch < -1.2f) live->pitch = -1.2f;
        if (live->pitch > 1.2f) live->pitch = 1.2f;
        live->drag_x = x;
        live->drag_y = y;
    } else if (event == cv::EVENT_LBUTTONUP) {
        live->dragging = 0;
    } else if (event == cv::EVENT_MOUSEWHEEL) {
        int delta = cv::getMouseWheelDelta(flags);
        live->user_zoom = 1;
        if (live->dist < 30.f) live->dist = 160.f;
        live->dist *= delta > 0 ? 0.9f : 1.1f;
        if (live->dist < 30.f) live->dist = 30.f;
        if (live->dist > 2000.f) live->dist = 2000.f;
    }
}

static void label_panel(cv::Mat &im, const char *text) {
    cv::putText(im, text, cv::Point(8, 22), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
    cv::putText(im, text, cv::Point(8, 22), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(240, 240, 240), 1, cv::LINE_AA);
}

static cv::Mat to_bgr(const cv::Mat &src, cv::Size size) {
    cv::Mat bgr, out;
    if (src.empty()) return cv::Mat(size, CV_8UC3, cv::Scalar(16, 16, 16));
    if (src.channels() == 1) cv::cvtColor(src, bgr, cv::COLOR_GRAY2BGR);
    else if (src.channels() == 4) cv::cvtColor(src, bgr, cv::COLOR_BGRA2BGR);
    else bgr = src;
    if (bgr.size() != size) cv::resize(bgr, out, size, 0, 0, cv::INTER_LINEAR);
    else out = bgr;
    return out;
}

static cv::Mat depth_color(const cv::Mat &depth_m, double min_mm, double max_mm, cv::Size size) {
    cv::Mat mid(size, CV_8UC3, cv::Scalar(0, 0, 0));
    if (depth_m.empty()) return mid;
    cv::Mat u8(depth_m.size(), CV_8U, cv::Scalar(0));
    float z0 = (float)min_mm, z1 = (float)max_mm;
    for (int y = 0; y < depth_m.rows; y++) {
        const float *d = depth_m.ptr<float>(y);
        uint8_t *o = u8.ptr<uint8_t>(y);
        for (int x = 0; x < depth_m.cols; x++) {
            if (d[x] <= 0.f) continue;
            float t = (d[x] * 1000.f - z0) / (z1 - z0);
            if (t < 0.f) t = 0.f;
            if (t > 1.f) t = 1.f;
            o[x] = (uint8_t)(255.f - t * 255.f);
        }
    }
    cv::Mat colored;
    cv::applyColorMap(u8, colored, cv::COLORMAP_TURBO);
    colored.setTo(cv::Scalar(0, 0, 0), u8 == 0);
    if (colored.size() != size) cv::resize(colored, mid, size, 0, 0, cv::INTER_NEAREST);
    else mid = colored;
    return mid;
}

static uint64_t voxel_key(int ix, int iy, int iz) {
    uint32_t bx = (uint32_t)(ix + (1 << 20));
    uint32_t by = (uint32_t)(iy + (1 << 20));
    uint32_t bz = (uint32_t)(iz + (1 << 20));
    return ((uint64_t)bx << 42) | ((uint64_t)by << 21) | bz;
}

static float depth_median_m(const cv::Mat &depth) {
    std::vector<float> s;
    s.reserve((size_t)depth.total() / 16);
    for (int y = 0; y < depth.rows; y += 4) {
        const float *row = depth.ptr<float>(y);
        for (int x = 0; x < depth.cols; x += 4) {
            if (row[x] > 0.f) s.push_back(row[x]);
        }
    }
    if (s.empty()) return 0.f;
    size_t mid = s.size() / 2;
    std::nth_element(s.begin(), s.begin() + (ptrdiff_t)mid, s.end());
    return s[mid];
}

/* Depth is metres, Y down, Z forward. Stored points are millimetres, Y up.
 * Only the dominant surface is kept: samples far from the median depth are
 * the stereo outliers that otherwise draw a streak instead of an object. */
static void depth_samples(const cv::Mat &depth, float fx, float fy, float cx, float cy,
                          const cv::Affine3f *world_from_cam,
                          std::vector<cv::Vec3f> &P, std::vector<cv::Vec3f> &N) {
    P.clear();
    N.clear();
    if (depth.empty()) return;
    float med = depth_median_m(depth);
    for (int y = 1; y < depth.rows - 1; y += 2) {
        const float *row = depth.ptr<float>(y);
        const float *dn = depth.ptr<float>(y + 1);
        for (int x = 1; x < depth.cols - 1; x += 2) {
            float z = row[x];
            float zx = row[x + 1];
            float zy = dn[x];
            if (z <= 0.f || zx <= 0.f || zy <= 0.f) continue;
            if (med > 0.f && std::fabs(z - med) > 0.02f) continue;
            if (std::fabs(zx - z) > 0.008f || std::fabs(zy - z) > 0.008f) continue;
            cv::Point3f pc((x - cx) * z / fx, (y - cy) * z / fy, z);
            cv::Point3f px(((x + 1) - cx) * zx / fx, (y - cy) * zx / fy, zx);
            cv::Point3f py((x - cx) * zy / fx, ((y + 1) - cy) * zy / fy, zy);
            if (world_from_cam) {
                pc = (*world_from_cam) * pc;
                px = (*world_from_cam) * px;
                py = (*world_from_cam) * py;
            }
            cv::Vec3f p(pc.x * 1000.f, -pc.y * 1000.f, pc.z * 1000.f);
            cv::Vec3f n(px.x - pc.x, -(px.y - pc.y), px.z - pc.z);
            cv::Vec3f d(py.x - pc.x, -(py.y - pc.y), py.z - pc.z);
            n = n.cross(d);
            float ln = std::sqrt(n.dot(n));
            if (ln < 1e-8f) continue;
            n /= ln;
            P.push_back(p);
            N.push_back(n);
        }
    }
}

struct SurfTri {
    cv::Vec3f a, b, c, n;
};

/* One connected surface around the median depth. This is the object. */
static void object_mask(const cv::Mat &depth, float med, cv::Mat &labels, int &best) {
    cv::Mat keep(depth.size(), CV_8U, cv::Scalar(0));
    for (int y = 0; y < depth.rows; y++) {
        const float *row = depth.ptr<float>(y);
        uint8_t *k = keep.ptr<uint8_t>(y);
        for (int x = 0; x < depth.cols; x++) {
            if (row[x] > 0.f && std::fabs(row[x] - med) <= 0.04f) k[x] = 255;
        }
    }
    cv::Mat stats, centroids;
    int n = cv::connectedComponentsWithStats(keep, labels, stats, centroids, 8, CV_32S);
    best = 0;
    int area = 0;
    for (int i = 1; i < n; i++) {
        int a = stats.at<int>(i, cv::CC_STAT_AREA);
        if (a > area) {
            area = a;
            best = i;
        }
    }
}

static cv::Vec3f unproject_mm(int x, int y, float z, float fx, float fy, float cx, float cy) {
    return cv::Vec3f((x - cx) * z / fx * 1000.f, -((y - cy) * z / fy * 1000.f), z * 1000.f);
}

static void build_object(const cv::Mat &depth, float fx, float fy, float cx, float cy,
                         std::vector<cv::Vec3f> &P, std::vector<cv::Vec3f> &N,
                         std::vector<SurfTri> &tris) {
    P.clear();
    N.clear();
    tris.clear();
    if (depth.empty()) return;
    float med = depth_median_m(depth);
    if (med <= 0.f) return;
    cv::Mat labels;
    int best = 0;
    object_mask(depth, med, labels, best);
    if (best == 0) return;
    const int step = 2;
    cv::Mat used(depth.rows, depth.cols, CV_8U, cv::Scalar(0));
    auto ok_at = [&](int x, int y, cv::Vec3f &p) -> bool {
        if (x < 0 || y < 0 || x >= depth.cols || y >= depth.rows) return false;
        if (labels.at<int>(y, x) != best) return false;
        float z = depth.at<float>(y, x);
        if (z <= 0.f || std::fabs(z - med) > 0.04f) return false;
        p = unproject_mm(x, y, z, fx, fy, cx, cy);
        return true;
    };
    for (int y = 0; y < depth.rows - step; y += step) {
        for (int x = 0; x < depth.cols - step; x += step) {
            cv::Vec3f p00, p10, p01, p11;
            if (!ok_at(x, y, p00) || !ok_at(x + step, y, p10) || !ok_at(x, y + step, p01) ||
                !ok_at(x + step, y + step, p11))
                continue;
            auto edge_ok = [](const cv::Vec3f &a, const cv::Vec3f &b) {
                return cv::norm(a - b) < 12.f;
            };
            if (!edge_ok(p00, p10) || !edge_ok(p00, p01) || !edge_ok(p10, p11) || !edge_ok(p01, p11))
                continue;
            cv::Vec3f n = (p10 - p00).cross(p01 - p00);
            float ln = std::sqrt(n.dot(n));
            if (ln < 1e-4f) continue;
            n /= ln;
            tris.push_back(SurfTri{p00, p10, p01, n});
            tris.push_back(SurfTri{p10, p11, p01, n});
            if (!used.at<uint8_t>(y, x)) {
                used.at<uint8_t>(y, x) = 1;
                P.push_back(p00);
                N.push_back(n);
            }
        }
    }
}

static bool nearest_model(const fox_live *L, const cv::Vec3f &p, int &idx) {
    const float voxel = 2.5f;
    int ix = (int)std::floor(p[0] / voxel);
    int iy = (int)std::floor(p[1] / voxel);
    int iz = (int)std::floor(p[2] / voxel);
    float best = 18.f * 18.f;
    bool found = false;
    for (int dz = -6; dz <= 6; dz++) {
        for (int dy = -6; dy <= 6; dy++) {
            for (int dx = -6; dx <= 6; dx++) {
                auto it = L->vox.find(voxel_key(ix + dx, iy + dy, iz + dz));
                if (it == L->vox.end()) continue;
                cv::Vec3f d = L->cloud[it->second] - p;
                float dd = d.dot(d);
                if (dd < best) {
                    best = dd;
                    idx = (int)it->second;
                    found = true;
                }
            }
        }
    }
    return found;
}

/* Align the new object cloud to the model. Scale lets the scanner move
 * closer or farther. A frame that does not land on this object is rejected,
 * so the background is not fused. */
static bool track_object(fox_live *L, const std::vector<cv::Vec3f> &P, cv::Matx33f &R, cv::Vec3f &t,
                         float &scale) {
    if (L->cloud.size() < 80) {
        R = cv::Matx33f::eye();
        t = cv::Vec3f(0, 0, 0);
        scale = 1.f;
        return P.size() >= 80;
    }
    R = L->pose_R;
    t = L->pose_t;
    scale = L->pose_s > 0.2f ? L->pose_s : 1.f;
    int stride = std::max(1, (int)P.size() / 900);
    for (int iter = 0; iter < 10; iter++) {
        std::vector<cv::Vec3f> src, dst;
        for (size_t i = 0; i < P.size(); i += (size_t)stride) {
            cv::Vec3f pm = scale * (R * P[i]) + t;
            int id = 0;
            if (!nearest_model(L, pm, id)) continue;
            src.push_back(P[i]);
            dst.push_back(L->cloud[(size_t)id]);
        }
        if (src.size() < 50) return false;
        cv::Vec3d cs(0, 0, 0), cd(0, 0, 0);
        for (size_t i = 0; i < src.size(); i++) {
            cs += cv::Vec3d(src[i]);
            cd += cv::Vec3d(dst[i]);
        }
        cs *= 1.0 / src.size();
        cd *= 1.0 / src.size();
        cv::Mat H = cv::Mat::zeros(3, 3, CV_64F);
        for (size_t i = 0; i < src.size(); i++) {
            cv::Vec3d a = cv::Vec3d(src[i]) - cs;
            cv::Vec3d b = cv::Vec3d(dst[i]) - cd;
            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 3; c++) H.at<double>(r, c) += a[r] * b[c];
        }
        cv::SVD svd(H, cv::SVD::FULL_UV);
        cv::Mat Rm = svd.vt.t() * svd.u.t();
        if (cv::determinant(Rm) < 0) {
            cv::Mat vt = svd.vt.clone();
            vt.row(2) *= -1;
            Rm = vt.t() * svd.u.t();
        }
        R = cv::Matx33f((float)Rm.at<double>(0, 0), (float)Rm.at<double>(0, 1), (float)Rm.at<double>(0, 2),
                        (float)Rm.at<double>(1, 0), (float)Rm.at<double>(1, 1), (float)Rm.at<double>(1, 2),
                        (float)Rm.at<double>(2, 0), (float)Rm.at<double>(2, 1), (float)Rm.at<double>(2, 2));
        double denom = 0, numer = 0;
        for (size_t i = 0; i < src.size(); i++) {
            cv::Vec3d a = cv::Vec3d(src[i]) - cs;
            cv::Vec3d b = cv::Vec3d(dst[i]) - cd;
            cv::Vec3d ra(R(0, 0) * a[0] + R(0, 1) * a[1] + R(0, 2) * a[2],
                         R(1, 0) * a[0] + R(1, 1) * a[1] + R(1, 2) * a[2],
                         R(2, 0) * a[0] + R(2, 1) * a[1] + R(2, 2) * a[2]);
            numer += b.dot(ra);
            denom += a.dot(a);
        }
        float s = denom > 1e-6 ? (float)(numer / denom) : scale;
        if (s < 0.45f) s = 0.45f;
        if (s > 2.2f) s = 2.2f;
        if (L->pose_s > 0.2f) {
            float lo = L->pose_s * 0.97f, hi = L->pose_s * 1.03f;
            if (s < lo) s = lo;
            if (s > hi) s = hi;
        }
        scale = s;
        cv::Vec3f csf((float)cs[0], (float)cs[1], (float)cs[2]);
        cv::Vec3f cdf((float)cd[0], (float)cd[1], (float)cd[2]);
        t = cdf - scale * (R * csf);
    }
    double res = 0;
    int inliers = 0, tested = 0;
    for (size_t i = 0; i < P.size(); i += (size_t)stride) {
        tested++;
        cv::Vec3f pm = scale * (R * P[i]) + t;
        int id = 0;
        if (!nearest_model(L, pm, id)) continue;
        float d = cv::norm(pm - L->cloud[(size_t)id]);
        if (d > 8.f) continue;
        res += d;
        inliers++;
    }
    if (tested < 40 || inliers < 40) return false;
    return (float)inliers / (float)tested >= 0.42f && res / inliers < 5.5;
}

static void note_coverage(fox_live *L, const std::vector<cv::Vec3f> &N) {
    cv::Vec3f s(0, 0, 0);
    for (const cv::Vec3f &n : N) s += L->pose_R * n;
    if (s.dot(s) < 1e-6f) return;
    int axis = 0;
    float ax = std::fabs(s[0]), ay = std::fabs(s[1]), az = std::fabs(s[2]);
    if (ay >= ax && ay >= az) axis = 1;
    else if (az >= ax && az >= ay) axis = 2;
    int sign = s[axis] >= 0 ? 0 : 1;
    L->covered |= 1 << (axis * 2 + sign);
}

static int coverage_count(int mask) {
    int n = 0;
    for (int i = 0; i < 6; i++) if (mask & (1 << i)) n++;
    return n;
}

static void absorb_points(fox_live *L, const std::vector<cv::Vec3f> &P, const std::vector<cv::Vec3f> &N) {
    const float voxel = 2.5f;
    for (size_t i = 0; i < P.size(); i++) {
        if (L->cloud.size() >= 70000) return;
        int ix = (int)std::floor(P[i][0] / voxel);
        int iy = (int)std::floor(P[i][1] / voxel);
        int iz = (int)std::floor(P[i][2] / voxel);
        uint64_t k = voxel_key(ix, iy, iz);
        if (L->vox.find(k) != L->vox.end()) continue;
        L->vox.emplace(k, (uint32_t)L->cloud.size());
        L->cloud.push_back(P[i]);
        L->cloud_n.push_back(N[i]);
    }
}

static cv::Mat render_orbit(fox_live *L, const std::vector<cv::Vec3f> &P, const std::vector<cv::Vec3f> &N,
                            int w, int h, const char *title) {
    cv::Mat img(h, w, CV_8UC3, cv::Scalar(18, 20, 24));
    cv::putText(img, title, cv::Point(10, 24), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(230, 230, 230), 1, cv::LINE_AA);
    if (P.size() < 30) {
        cv::putText(img, "no surface yet", cv::Point(10, h / 2), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(160, 160, 160), 1, cv::LINE_AA);
        return img;
    }
    if (L->spin && !L->dragging) L->yaw += 0.018f;
    cv::Vec3f c(0, 0, 0);
    int stride = std::max(1, (int)P.size() / 3000);
    int cnt = 0;
    for (size_t i = 0; i < P.size(); i += (size_t)stride) {
        c += P[i];
        cnt++;
    }
    c *= 1.f / std::max(cnt, 1);
    cv::Vec3f lo = P[0], hi = P[0];
    cv::Vec3f nmean(0, 0, 0);
    for (size_t i = 0; i < P.size(); i++) {
        const cv::Vec3f &p = P[i];
        lo[0] = std::min(lo[0], p[0]); lo[1] = std::min(lo[1], p[1]); lo[2] = std::min(lo[2], p[2]);
        hi[0] = std::max(hi[0], p[0]); hi[1] = std::max(hi[1], p[1]); hi[2] = std::max(hi[2], p[2]);
        if (i < N.size()) nmean += N[i];
    }
    float extent = std::max(hi[0] - lo[0], std::max(hi[1] - lo[1], hi[2] - lo[2]));
    if (!L->user_zoom)
        L->dist = std::max(40.f, extent * 2.2f);
    float dist = L->dist;
    if (nmean.dot(nmean) < 1e-6f) nmean = cv::Vec3f(0, 0, 1);
    float nl = std::sqrt(nmean.dot(nmean));
    nmean /= nl;
    float cyaw = std::cos(L->yaw * 0.35f), syaw = std::sin(L->yaw * 0.35f);
    cv::Vec3f faced(nmean[0] * cyaw + nmean[2] * syaw, nmean[1], -nmean[0] * syaw + nmean[2] * cyaw);
    float flf = std::sqrt(faced.dot(faced));
    if (flf > 1e-6f) faced /= flf;
    cv::Vec3f eye = c + faced * dist;
    cv::Vec3f forward = c - eye;
    float fl = std::sqrt(forward.dot(forward));
    if (fl < 1e-3f) return img;
    forward /= fl;
    cv::Vec3f up(0, 1, 0);
    cv::Vec3f right = forward.cross(up);
    float rl = std::sqrt(right.dot(right));
    if (rl < 1e-4f) right = cv::Vec3f(1, 0, 0);
    else right /= rl;
    cv::Vec3f cam_up = right.cross(forward);
    float focal = h * 1.2f;
    std::vector<float> zbuf((size_t)w * h, 1e9f);
    int rad = (int)(focal * 1.8f / std::max(dist, 1.f));
    if (rad < 2) rad = 2;
    if (rad > 7) rad = 7;
    for (size_t i = 0; i < P.size(); i++) {
        cv::Vec3f d = P[i] - eye;
        float zc = d.dot(forward);
        if (zc < 15.f) continue;
        int sx = (int)(w * 0.5f + focal * d.dot(right) / zc);
        int sy = (int)(h * 0.5f - focal * d.dot(cam_up) / zc);
        cv::Vec3f light = eye - P[i];
        float ll = std::sqrt(light.dot(light)) + 1e-6f;
        float nd = N[i].dot(light) / ll;
        if (nd < 0.f) nd = 0.f;
        int g = (int)(36 + 205 * nd);
        cv::Vec3b col((uint8_t)(g * 0.72f), (uint8_t)(g * 0.88f), (uint8_t)std::min(255, g + 24));
        for (int dy = -rad; dy <= rad; dy++) {
            int yy = sy + dy;
            if ((unsigned)yy >= (unsigned)h) continue;
            for (int dx = -rad; dx <= rad; dx++) {
                int xx = sx + dx;
                if ((unsigned)xx >= (unsigned)w) continue;
                float &zslot = zbuf[(size_t)yy * w + xx];
                if (zc >= zslot) continue;
                zslot = zc;
                img.at<cv::Vec3b>(yy, xx) = col;
            }
        }
    }
    return img;
}

static void draw_solid(fox_live *L, cv::Mat &img, const std::vector<SurfTri> &tris) {
    if (tris.size() < 20) return;
    cv::Vec3f c(0, 0, 0);
    cv::Vec3f lo = tris[0].a, hi = tris[0].a;
    int cnt = 0;
    for (size_t i = 0; i < tris.size(); i += 3) {
        c += tris[i].a;
        cnt++;
        for (const cv::Vec3f &p : {tris[i].a, tris[i].b, tris[i].c}) {
            lo[0] = std::min(lo[0], p[0]); lo[1] = std::min(lo[1], p[1]); lo[2] = std::min(lo[2], p[2]);
            hi[0] = std::max(hi[0], p[0]); hi[1] = std::max(hi[1], p[1]); hi[2] = std::max(hi[2], p[2]);
        }
    }
    c *= 1.f / std::max(cnt, 1);
    cv::Vec3f nmean(0, 0, 0);
    for (const SurfTri &t : tris) nmean += t.n;
    float extent = std::max(hi[0] - lo[0], std::max(hi[1] - lo[1], hi[2] - lo[2]));
    if (!L->user_zoom) L->dist = std::max(40.f, extent * 2.4f);
    float dist = L->dist;
    if (L->spin && !L->dragging) L->yaw += 0.012f;
    if (nmean.dot(nmean) < 1e-6f) nmean = cv::Vec3f(0, 0, 1);
    nmean /= std::sqrt(nmean.dot(nmean));
    float cyaw = std::cos(L->yaw * 0.35f), syaw = std::sin(L->yaw * 0.35f);
    cv::Vec3f faced(nmean[0] * cyaw + nmean[2] * syaw, nmean[1], -nmean[0] * syaw + nmean[2] * cyaw);
    faced /= std::sqrt(faced.dot(faced));
    cv::Vec3f eye = c + faced * dist;
    cv::Vec3f forward = c - eye;
    float fl = std::sqrt(forward.dot(forward));
    if (fl < 1e-3f) return;
    forward /= fl;
    cv::Vec3f up(0, 1, 0);
    cv::Vec3f right = forward.cross(up);
    float rl = std::sqrt(right.dot(right));
    if (rl < 1e-4f) right = cv::Vec3f(1, 0, 0);
    else right /= rl;
    cv::Vec3f cam_up = right.cross(forward);
    int w = img.cols, h = img.rows;
    float focal = h * 1.15f;
    std::vector<float> zbuf((size_t)w * h, 1e9f);
    auto project = [&](const cv::Vec3f &p, float &sx, float &sy, float &zc) -> bool {
        cv::Vec3f d = p - eye;
        zc = d.dot(forward);
        if (zc < 8.f) return false;
        sx = w * 0.5f + focal * d.dot(right) / zc;
        sy = h * 0.5f - focal * d.dot(cam_up) / zc;
        return true;
    };
    for (const SurfTri &t : tris) {
        float ax, ay, az, bx, by, bz, cx, cy, cz;
        if (!project(t.a, ax, ay, az) || !project(t.b, bx, by, bz) || !project(t.c, cx, cy, cz)) continue;
        int minx = (int)std::floor(std::min(ax, std::min(bx, cx)));
        int maxx = (int)std::ceil(std::max(ax, std::max(bx, cx)));
        int miny = (int)std::floor(std::min(ay, std::min(by, cy)));
        int maxy = (int)std::ceil(std::max(ay, std::max(by, cy)));
        if (maxx < 0 || maxy < 0 || minx >= w || miny >= h) continue;
        minx = std::max(minx, 0);
        miny = std::max(miny, 0);
        maxx = std::min(maxx, w - 1);
        maxy = std::min(maxy, h - 1);
        float area = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
        if (std::fabs(area) < 1e-3f) continue;
        cv::Vec3f light = eye - t.a;
        float ll = std::sqrt(light.dot(light)) + 1e-6f;
        float nd = t.n.dot(light) / ll;
        if (nd < 0.f) nd = -nd * 0.35f;
        int g = (int)(48 + 190 * std::min(nd, 1.f));
        cv::Vec3b col((uint8_t)(g * 0.7f), (uint8_t)(g * 0.88f), (uint8_t)std::min(255, g + 30));
        for (int y = miny; y <= maxy; y++) {
            for (int x = minx; x <= maxx; x++) {
                float w0 = (bx - x) * (cy - y) - (by - y) * (cx - x);
                float w1 = (cx - x) * (ay - y) - (cy - y) * (ax - x);
                float w2 = (ax - x) * (by - y) - (ay - y) * (bx - x);
                if ((w0 >= 0 && w1 >= 0 && w2 >= 0) || (w0 <= 0 && w1 <= 0 && w2 <= 0)) {
                    float inv = 1.f / area;
                    float zc = (w0 * az + w1 * bz + w2 * cz) * inv;
                    float &slot = zbuf[(size_t)y * w + x];
                    if (zc < slot) {
                        slot = zc;
                        img.at<cv::Vec3b>(y, x) = col;
                    }
                }
            }
        }
    }
}

/* The solid is the light-plane surface of the stripes, in millimetres.
 * A missing stripe stays a hole. Nothing here is revolved into a tube. */
struct ShadeVert {
    float x, y, z, bump, albedo;
    bool ok;
};

struct BodyView {
    cv::Mat camera;
    cv::Mat solid;
    cv::Mat side;
    std::vector<cv::Vec3f> P, N;
    std::vector<Tri> tris;
    cv::Mat gray;
    cv::Point2f center;
    cv::Vec3f axis;
    float radius_px = 0;
    float median_mm = 0;
};

static void longest_runs(const cv::Mat &mask, std::vector<float> &left, std::vector<float> &right) {
    left.assign(mask.rows, -1.f);
    right.assign(mask.rows, -1.f);
    for (int y = 0; y < mask.rows; y++) {
        const uint8_t *m = mask.ptr<uint8_t>(y);
        int run_l = -1, best_l = -1, best_r = -1, best = 0;
        for (int x = 0; x <= mask.cols; x++) {
            bool on = x < mask.cols && m[x];
            if (on) {
                if (run_l < 0) run_l = x;
            } else if (run_l >= 0) {
                int len = x - run_l;
                if (len > best) {
                    best = len;
                    best_l = run_l;
                    best_r = x - 1;
                }
                run_l = -1;
            }
        }
        if (best >= 12) {
            left[y] = (float)best_l;
            right[y] = (float)best_r;
        }
    }
}

/* A run that ends on the picture edge is the frame, not the object. Mirror
 * the in-frame side about the axis of the rows that are fully visible so the
 * head is not sheared into that border. */
static void repair_clipped_sides(std::vector<float> &left, std::vector<float> &right, int cols) {
    const float border = 2.f;
    double sum = 0;
    int n = 0;
    for (int y = 0; y < (int)left.size(); y++) {
        if (left[y] < 0.f || right[y] < 0.f) continue;
        if (left[y] > border && right[y] < (float)cols - 1.f - border) {
            sum += 0.5 * (left[y] + right[y]);
            n++;
        }
    }
    if (n < 24) return;
    float axis = (float)(sum / n);
    for (int y = 0; y < (int)left.size(); y++) {
        if (left[y] < 0.f || right[y] < 0.f) continue;
        bool clip_l = left[y] <= border;
        bool clip_r = right[y] >= (float)cols - 1.f - border;
        if (clip_r && !clip_l) {
            float half = axis - left[y];
            if (half > 8.f) right[y] = axis + half;
        } else if (clip_l && !clip_r) {
            float half = right[y] - axis;
            if (half > 8.f) left[y] = axis - half;
        }
    }
}

static void gaussian_valid(std::vector<float> &v, float sigma) {
    int rad = (int)std::ceil(sigma * 3.f);
    std::vector<float> o(v.size(), -1.f);
    for (int i = 0; i < (int)v.size(); i++) {
        if (v[i] < 0.f) continue;
        float s = 0, wsum = 0;
        for (int k = -rad; k <= rad; k++) {
            int j = i + k;
            if (j < 0 || j >= (int)v.size() || v[j] < 0.f) continue;
            float w = std::exp(-(float)(k * k) / (2.f * sigma * sigma));
            s += w * v[j];
            wsum += w;
        }
        if (wsum > 0.f) o[i] = s / wsum;
    }
    v.swap(o);
}

static void fill_edge_gaps(std::vector<float> &v, int max_gap) {
    int n = (int)v.size();
    int i = 0;
    while (i < n) {
        if (v[i] >= 0.f) {
            i++;
            continue;
        }
        int a = i - 1;
        while (i < n && v[i] < 0.f) i++;
        int b = i;
        if (a < 0 || b >= n || b - a > max_gap) continue;
        for (int k = a + 1; k < b; k++) {
            float t = (float)(k - a) / (float)(b - a);
            v[k] = v[a] * (1.f - t) + v[b] * t;
        }
    }
}

static cv::Mat foreground_mask(const cv::Mat &gray) {
    cv::Mat blur;
    float rel = gray.cols / 1280.f;
    cv::GaussianBlur(gray, blur, cv::Size(0, 0), std::max(1.2f, 1.4f * rel));
    cv::Mat tmp;
    double otsu = cv::threshold(blur, tmp, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    double thr = std::max(14.0, std::min(otsu * 0.45, 28.0));
    cv::Mat bin;
    cv::threshold(blur, bin, thr, 255, cv::THRESH_BINARY);
    int ksz = std::max(5, (int)std::lround(31.f * rel)) | 1;
    cv::Mat k = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ksz, ksz));
    cv::morphologyEx(bin, bin, cv::MORPH_CLOSE, k);
    int osz = std::max(3, (int)std::lround(5.f * rel)) | 1;
    cv::morphologyEx(bin, bin, cv::MORPH_OPEN,
                     cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(osz, osz)));
    cv::Mat labels, stats, cent;
    int n = cv::connectedComponentsWithStats(bin, labels, stats, cent, 8, CV_32S);
    int best = 0, best_area = 0;
    int min_area = std::max(800, gray.rows * gray.cols / 80);
    for (int i = 1; i < n; i++) {
        int a = stats.at<int>(i, cv::CC_STAT_AREA);
        int w = stats.at<int>(i, cv::CC_STAT_WIDTH);
        int h = stats.at<int>(i, cv::CC_STAT_HEIGHT);
        if (a < min_area || w < 30) continue;
        /* A lamp in the frame is a thin bright bar. The object is the wide blob. */
        if (w * 5 < h && w < (int)(90 * rel + 8)) continue;
        if (a > best_area) {
            best_area = a;
            best = i;
        }
    }
    cv::Mat mask(gray.size(), CV_8U, cv::Scalar(0));
    if (!best) return mask;
    for (int y = 0; y < gray.rows; y++) {
        const int *lab = labels.ptr<int>(y);
        uint8_t *m = mask.ptr<uint8_t>(y);
        for (int x = 0; x < gray.cols; x++)
            if (lab[x] == best) m[x] = 255;
    }
    return mask;
}

static void render_body(const std::vector<ShadeVert> &grid, int gw, int gh, cv::Mat &out,
                        float yaw_deg, float pitch_deg, float bump_mm, float *dist, int user_zoom) {
    out.setTo(cv::Scalar(12, 14, 18));
    cv::Vec3f c(0, 0, 0);
    int cnt = 0;
    cv::Vec3f lo(1e9f, 1e9f, 1e9f), hi(-1e9f, -1e9f, -1e9f);
    for (const ShadeVert &v : grid) {
        if (!v.ok) continue;
        cv::Vec3f p(v.x, v.y, v.z - bump_mm * v.bump);
        c += p;
        cnt++;
        for (int k = 0; k < 3; k++) {
            lo[k] = std::min(lo[k], p[k]);
            hi[k] = std::max(hi[k], p[k]);
        }
    }
    if (cnt < 40) {
        cv::putText(out, "no object in frame", cv::Point(16, out.rows / 2),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(170, 170, 170), 1, cv::LINE_AA);
        return;
    }
    c *= 1.f / cnt;
    float ext_x = hi[0] - lo[0];
    float ext_y = hi[1] - lo[1];
    float extent = std::max(ext_x, ext_y);
    int w = out.cols, h = out.rows;
    float auto_dist = std::max(extent * 1.55f, 30.f);
    /* Keep the camera where it is. Refitting every frame hides the distance
     * slider, because the model and the zoom grow by the same amount. */
    float use = (dist && *dist > 1.f) ? *dist : auto_dist;
    if (dist && *dist <= 1.f) *dist = use;
    float yaw = yaw_deg * 3.14159265f / 180.f;
    float pitch = pitch_deg * 3.14159265f / 180.f;
    /* Orthographic. A perspective camera makes the nearer centre bulge and
     * pinches the outline, so the texture no longer sits on the camera pixels. */
    float scale = 1.05f * (float)h / std::max(use, 1.f);
    float cyaw = std::cos(yaw), syaw = std::sin(yaw);
    float cp = std::cos(pitch), sp = std::sin(pitch);
    (void)extent;
    (void)user_zoom;
    std::vector<float> zbuf((size_t)w * h, 1e9f);
    auto project_point = [&](const cv::Vec3f &p, float &sx, float &sy, float &zc) {
        cv::Vec3f d = p - c;
        float x1 = cyaw * d[0] + syaw * d[2];
        float z1 = -syaw * d[0] + cyaw * d[2];
        float y2 = cp * d[1] - sp * z1;
        zc = sp * d[1] + cp * z1;
        sx = w * 0.50f + x1 * scale;
        sy = h * 0.50f - y2 * scale;
    };
    auto proj = [&](const ShadeVert &v, float &sx, float &sy, float &zc) {
        project_point(cv::Vec3f(v.x, v.y, v.z - bump_mm * v.bump), sx, sy, zc);
    };
    for (int gy = 0; gy < gh - 1; gy++) {
        for (int gx = 0; gx < gw - 1; gx++) {
            const ShadeVert &a = grid[(size_t)gy * gw + gx];
            const ShadeVert &b = grid[(size_t)gy * gw + gx + 1];
            const ShadeVert &c0 = grid[(size_t)(gy + 1) * gw + gx];
            const ShadeVert &d = grid[(size_t)(gy + 1) * gw + gx + 1];
            const ShadeVert *tris[2][3] = {{&a, &b, &c0}, {&b, &d, &c0}};
            for (auto &t : tris) {
                if (!t[0]->ok || !t[1]->ok || !t[2]->ok) continue;
                cv::Vec3f pa(t[0]->x, t[0]->y, t[0]->z - bump_mm * t[0]->bump);
                cv::Vec3f pb(t[1]->x, t[1]->y, t[1]->z - bump_mm * t[1]->bump);
                cv::Vec3f pc(t[2]->x, t[2]->y, t[2]->z - bump_mm * t[2]->bump);
                if (cv::norm(pa - pb) > 28.f || cv::norm(pa - pc) > 28.f || cv::norm(pb - pc) > 28.f)
                    continue;
                cv::Vec3f nrm = (pb - pa).cross(pc - pa);
                float ln = std::sqrt(nrm.dot(nrm));
                if (ln < 1e-6f) continue;
                nrm /= ln;
                cv::Vec3f light = cv::normalize(cv::Vec3f(-syaw, 0.25f, -cyaw));
                float lam = nrm.dot(light) * 0.35f + 0.65f;
                if (lam < 0.45f) lam = 0.45f;
                float alb = t[0]->albedo;
                float shade = alb * (0.72f + 0.28f * lam);
                int g = (int)std::min(255.f, 255.f * shade);
                cv::Vec3b col((uint8_t)g, (uint8_t)g, (uint8_t)g);
                float ax, ay, az, bx, by, bz, cxp, cyp, cz;
                proj(*t[0], ax, ay, az);
                proj(*t[1], bx, by, bz);
                proj(*t[2], cxp, cyp, cz);
                int minx = std::max(0, (int)std::floor(std::min(ax, std::min(bx, cxp))) - 1);
                int maxx = std::min(w - 1, (int)std::ceil(std::max(ax, std::max(bx, cxp))) + 1);
                int miny = std::max(0, (int)std::floor(std::min(ay, std::min(by, cyp))) - 1);
                int maxy = std::min(h - 1, (int)std::ceil(std::max(ay, std::max(by, cyp))) + 1);
                float area = (bx - ax) * (cyp - ay) - (by - ay) * (cxp - ax);
                if (std::fabs(area) < 0.4f) continue;
                for (int py = miny; py <= maxy; py++) {
                    for (int px = minx; px <= maxx; px++) {
                        float w0 = (bx - px) * (cyp - py) - (by - py) * (cxp - px);
                        float w1 = (cxp - px) * (ay - py) - (cyp - py) * (ax - px);
                        float w2 = (ax - px) * (by - py) - (ay - py) * (bx - px);
                        if (!((w0 >= 0 && w1 >= 0 && w2 >= 0) || (w0 <= 0 && w1 <= 0 && w2 <= 0)))
                            continue;
                        float zc = (w0 * az + w1 * bz + w2 * cz) / area;
                        float &slot = zbuf[(size_t)py * w + px];
                        if (zc < slot) {
                            slot = zc;
                            out.at<cv::Vec3b>(py, px) = col;
                        }
                    }
                }
            }
        }
    }
}

static BodyView make_body(const cv::Mat &sensor, const cv::Mat &pattern, const fox_calib *cal,
                          float fx, float fy, float cx, float cy, float yaw_deg, float pitch_deg,
                          float &dist, int user_zoom, int shade_live, float z_mm, int shape) {
    BodyView body;
    if (sensor.empty() || fx < 50.f) return body;
    cv::Mat gray;
    cv::rotate(sensor, gray, cv::ROTATE_180);
    float cxr = (gray.cols - 1) - cx;
    float cyr = (gray.rows - 1) - cy;
    float rel = gray.cols / 1280.f;
    cv::Mat mask = foreground_mask(gray);
    std::vector<float> left, right;
    longest_runs(mask, left, right);
    fill_edge_gaps(left, std::max(24, (int)std::lround(80.f * rel)));
    fill_edge_gaps(right, std::max(24, (int)std::lround(80.f * rel)));
    repair_clipped_sides(left, right, gray.cols);
    gaussian_valid(left, std::max(8.f, 18.f * rel));
    gaussian_valid(right, std::max(8.f, 18.f * rel));

    body.gray = gray;
    const float z0 = z_mm > 80.f ? z_mm : 220.f;
    int nrow = 0;
    double sx = 0, sy = 0, sr = 0;
    for (int y = 0; y < gray.rows; y++) {
        if (left[y] < 0.f || right[y] < 0.f) continue;
        sx += 0.5 * (left[y] + right[y]);
        sy += y;
        sr += 0.5 * (right[y] - left[y]);
        nrow++;
    }
    if (nrow > 0) {
        body.center = cv::Point2f((float)(sx / nrow), (float)(sy / nrow));
        body.radius_px = (float)(sr / nrow);
        body.axis = cv::Vec3f((body.center.x - cxr) * z0 / fx,
                              -((body.center.y - cyr) * z0 / fy), z0);
    }

    cv::cvtColor(gray, body.camera, cv::COLOR_GRAY2BGR);
    for (int y = 0; y < gray.rows; y++) {
        if (left[y] < 0.f || right[y] < 0.f) continue;
        int x0 = (int)std::lround(left[y]);
        int x1 = (int)std::lround(right[y]);
        if ((unsigned)x0 < (unsigned)body.camera.cols)
            body.camera.at<cv::Vec3b>(y, x0) = cv::Vec3b(70, 220, 90);
        if ((unsigned)x1 < (unsigned)body.camera.cols)
            body.camera.at<cv::Vec3b>(y, x1) = cv::Vec3b(70, 220, 90);
    }

    int gw = 2;
    int gh = 2;
    std::vector<ShadeVert> grid;
    if (shape != FOX_SHAPE_MEASURED) {
        cv::Mat mid, wide;
        cv::GaussianBlur(gray, mid, cv::Size(0, 0), std::max(2.f, 7.f * rel));
        cv::GaussianBlur(gray, wide, cv::Size(0, 0), std::max(6.f, 26.f * rel));
        int ystep = std::max(2, (int)std::lround(3.f * rel));
        gh = (gray.rows + ystep - 1) / ystep;
        gw = 112;
        grid.assign((size_t)gw * gh, ShadeVert{});
        const float th_max = 1.40f;
        const float sin_max = std::sin(th_max);
        for (int iy = 0; iy < gh; iy++) {
            int y = std::min(gray.rows - 1, iy * ystep);
            float half = 0.5f * (right[y] - left[y]);
            float mid_x = 0.5f * (right[y] + left[y]);
            bool row = left[y] >= 0.f && right[y] >= 0.f && half > 8.f;
            for (int ix = 0; ix < gw; ix++) {
                ShadeVert v{0, 0, 0, 0, 0, false};
                if (row) {
                    float u = -1.f + 2.f * (float)ix / (float)(gw - 1);
                    float th = u * th_max;
                    float x_geom = mid_x + half * (std::sin(th) / sin_max);
                    int x = (int)std::lround(x_geom);
                    x = std::max(0, std::min(gray.cols - 1, x));
                    float hf = (float)gray.at<uint8_t>(y, x) - (float)mid.at<uint8_t>(y, x);
                    float br = (float)mid.at<uint8_t>(y, x) - (float)wide.at<uint8_t>(y, x);
                    float bump = (hf * 0.55f + br * 0.35f) / 22.f;
                    bump = std::max(-1.f, std::min(1.f, bump));
                    float radius = half * z0 / fx;
                    v.x = (x_geom - cxr) * z0 / fx;
                    v.y = -((y - cyr) * z0 / fy);
                    v.z = z0 - radius * std::cos(th);
                    v.bump = bump;
                    v.albedo = gray.at<uint8_t>(y, x) / 255.f;
                    v.ok = true;
                }
                grid[(size_t)iy * gw + ix] = v;
            }
        }
        std::vector<float> sm(grid.size(), 0.f);
        for (int y = 0; y < gh; y++) {
            for (int x = 0; x < gw; x++) {
                float s = 0, w = 0;
                for (int dy = -2; dy <= 2; dy++) {
                    for (int dx = -2; dx <= 2; dx++) {
                        int yy = y + dy, xx = x + dx;
                        if (yy < 0 || xx < 0 || yy >= gh || xx >= gw) continue;
                        const ShadeVert &nb = grid[(size_t)yy * gw + xx];
                        if (!nb.ok) continue;
                        float wt = std::exp(-(dx * dx + dy * dy) / 3.f);
                        s += wt * nb.bump;
                        w += wt;
                    }
                }
                sm[(size_t)y * gw + x] = w > 0.f ? s / w : 0.f;
            }
        }
        for (size_t i = 0; i < grid.size(); i++)
            if (grid[i].ok) grid[i].bump = sm[i];
        std::vector<float> zcopy(grid.size());
        for (size_t i = 0; i < grid.size(); i++) zcopy[i] = grid[i].z;
        for (int iy = 0; iy < gh; iy++) {
            for (int ix = 0; ix < gw; ix++) {
                ShadeVert &v = grid[(size_t)iy * gw + ix];
                if (!v.ok) continue;
                float s = 0, w = 0;
                for (int dy = -5; dy <= 5; dy++) {
                    int yy = iy + dy;
                    if (yy < 0 || yy >= gh) continue;
                    const ShadeVert &nb = grid[(size_t)yy * gw + ix];
                    if (!nb.ok) continue;
                    float wt = std::exp(-(float)(dy * dy) / 10.f);
                    s += wt * zcopy[(size_t)yy * gw + ix];
                    w += wt;
                }
                if (w > 0.f) v.z = s / w;
            }
        }
    } else {
    cv::Mat depth(gray.rows, gray.cols, CV_32F, cv::Scalar(0));
    cv::Mat depth_n(gray.rows, gray.cols, CV_32F, cv::Scalar(0));
    if (cal && cal->nplanes >= 8 && !pattern.empty() && pattern.type() == CV_8UC1 &&
        pattern.cols == gray.cols && pattern.rows == gray.rows) {
        cv::Mat pat = pattern.isContinuous() ? pattern : pattern.clone();
        std::vector<fox_shot> shots(24000);
        int nshot = fox_oneshot_points(pat.ptr<uint8_t>(), pat.cols, pat.rows, cal, z0, shots.data(),
                                       (int)shots.size());
        if (nshot > 0 && cal->cam[1].fx > 100.0) {
            cv::Mat R;
            cv::Rodrigues(cv::Vec3d(cal->cam[1].rvec[0], cal->cam[1].rvec[1], cal->cam[1].rvec[2]), R);
            cv::Matx33d Rb(R);
            cv::Vec3d tb(cal->cam[1].tvec[0], cal->cam[1].tvec[1], cal->cam[1].tvec[2]);
            cv::Mat allow;
            cv::dilate(mask, allow,
                       cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(11, 11)));
            for (int i = 0; i < nshot; i++) {
                const fox_shot &s = shots[(size_t)i];
                cv::Vec3d Xb = Rb * cv::Vec3d(s.x, s.y, s.z) + tb;
                if (Xb[2] < 40.0 || Xb[2] > 900.0) continue;
                float u = (float)(fx * Xb[0] / Xb[2] + cx);
                float v = (float)(fy * Xb[1] / Xb[2] + cy);
                int xu = (int)std::lround((gray.cols - 1) - u);
                int yu = (int)std::lround((gray.rows - 1) - v);
                if (xu < 0 || yu < 0 || xu >= gray.cols || yu >= gray.rows) continue;
                if (!allow.at<uint8_t>(yu, xu)) continue;
                depth.at<float>(yu, xu) += (float)Xb[2];
                depth_n.at<float>(yu, xu) += 1.f;
            }
        }
    }
    for (int y = 0; y < gray.rows; y++) {
        float *d = depth.ptr<float>(y);
        const float *c = depth_n.ptr<float>(y);
        for (int x = 0; x < gray.cols; x++)
            if (c[x] > 0.f) d[x] /= c[x];
    }
    /* Stripes are a few pixels apart. Fill that gap inside the object so
     * the mesh is one surface. A second pass closes the corners. */
    const int gap_px = 28;
    const float gap_mm = 16.f;
    for (int pass = 0; pass < 2; pass++) {
    for (int y = 0; y < gray.rows; y++) {
        float *d = depth.ptr<float>(y);
        const uint8_t *m = mask.ptr<uint8_t>(y);
        int x = 0;
        while (x < gray.cols) {
            if (d[x] <= 0.f) {
                x++;
                continue;
            }
            int a = x;
            int b = a + 1;
            while (b < gray.cols && d[b] <= 0.f) b++;
            if (b < gray.cols && b > a + 1 && b - a - 1 <= gap_px && std::fabs(d[a] - d[b]) <= gap_mm) {
                bool inside = true;
                for (int k = a + 1; k < b; k++)
                    if (!m[k]) inside = false;
                if (inside) {
                    for (int k = a + 1; k < b; k++) {
                        float t = (float)(k - a) / (float)(b - a);
                        d[k] = (1.f - t) * d[a] + t * d[b];
                    }
                }
            }
            x = b < gray.cols ? b : gray.cols;
        }
    }
    for (int x = 0; x < gray.cols; x++) {
        int y = 0;
        while (y < gray.rows) {
            if (depth.at<float>(y, x) <= 0.f) {
                y++;
                continue;
            }
            int a = y;
            int b = a + 1;
            while (b < gray.rows && depth.at<float>(b, x) <= 0.f) b++;
            if (b < gray.rows && b > a + 1 && b - a - 1 <= gap_px &&
                std::fabs(depth.at<float>(a, x) - depth.at<float>(b, x)) <= gap_mm) {
                bool inside = true;
                for (int k = a + 1; k < b; k++)
                    if (!mask.at<uint8_t>(k, x)) inside = false;
                if (inside) {
                    float za = depth.at<float>(a, x);
                    float zb = depth.at<float>(b, x);
                    for (int k = a + 1; k < b; k++) {
                        if (depth.at<float>(k, x) > 0.f) continue;
                        float t = (float)(k - a) / (float)(b - a);
                        depth.at<float>(k, x) = (1.f - t) * za + t * zb;
                    }
                }
            }
            y = b < gray.rows ? b : gray.rows;
        }
    }
    }

    int xstep = std::max(3, (int)std::lround(4.f * rel));
    int ystep = std::max(3, (int)std::lround(4.f * rel));
    gw = std::max(2, (gray.cols + xstep - 1) / xstep);
    gh = std::max(2, (gray.rows + ystep - 1) / ystep);
    grid.assign((size_t)gw * gh, ShadeVert{});
    double sx3 = 0, sy3 = 0;
    float z_far = 0.f;
    int nok = 0;
    /* Read the filled depth. A stripe that did not land on this pixel
     * borrows the nearest filled pixel inside a few millimetres. */
    auto sample_z = [&](int x, int y, float &z) -> bool {
        if (x < 0 || y < 0 || x >= depth.cols || y >= depth.rows) return false;
        if (!mask.at<uint8_t>(y, x)) return false;
        float here = depth.at<float>(y, x);
        if (here > 40.f) {
            z = here;
            return true;
        }
        float acc = 0.f;
        int n = 0;
        float lo = 1e9f, hi = -1e9f;
        for (int dy = -6; dy <= 6; dy += 2) {
            int yy = y + dy;
            if (yy < 0 || yy >= depth.rows) continue;
            const float *row = depth.ptr<float>(yy);
            for (int dx = -6; dx <= 6; dx += 2) {
                int xx = x + dx;
                if (xx < 0 || xx >= depth.cols || row[xx] <= 40.f) continue;
                acc += row[xx];
                n++;
                lo = std::min(lo, row[xx]);
                hi = std::max(hi, row[xx]);
            }
        }
        if (n < 1) return false;
        z = acc / (float)n;
        return true;
    };
    for (int iy = 0; iy < gh; iy++) {
        int y = std::min(gray.rows - 1, iy * ystep);
        for (int ix = 0; ix < gw; ix++) {
            int x = std::min(gray.cols - 1, ix * xstep);
            float z = 0.f;
            ShadeVert v{0, 0, 0, 0, 0, false};
            if (sample_z(x, y, z)) {
                v.x = (x - cxr) * z / fx;
                v.y = -((y - cyr) * z / fy);
                v.z = z;
                v.bump = 0.f;
                v.albedo = gray.at<uint8_t>(y, x) / 255.f;
                v.ok = true;
                sx3 += v.x;
                sy3 += v.y;
                if (v.z > z_far) z_far = v.z;
                nok++;
            }
            grid[(size_t)iy * gw + ix] = v;
        }
    }
    for (int pass = 0; pass < 8; pass++) {
        std::vector<ShadeVert> prev = grid;
        for (int iy = 1; iy < gh - 1; iy++) {
            int y = std::min(gray.rows - 1, iy * ystep);
            for (int ix = 1; ix < gw - 1; ix++) {
                size_t id = (size_t)iy * gw + ix;
                if (prev[id].ok) continue;
                int x = std::min(gray.cols - 1, ix * xstep);
                if (!mask.at<uint8_t>(y, x)) continue;
                float acc = 0.f, lo = 1e9f, hi = -1e9f;
                int n = 0;
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        const ShadeVert &nb = prev[(size_t)(iy + dy) * gw + (ix + dx)];
                        if (!nb.ok) continue;
                        acc += nb.z;
                        lo = std::min(lo, nb.z);
                        hi = std::max(hi, nb.z);
                        n++;
                    }
                }
                if (n < 3 || hi - lo > 30.f) continue;
                float z = acc / (float)n;
                ShadeVert v{0, 0, 0, 0, 0, false};
                v.x = (x - cxr) * z / fx;
                v.y = -((y - cyr) * z / fy);
                v.z = z;
                v.albedo = gray.at<uint8_t>(y, x) / 255.f;
                v.ok = true;
                grid[id] = v;
                if (z > z_far) z_far = z;
                nok++;
            }
        }
    }
    /* Stripe noise becomes spikes when the surface is seen from the side.
     * Smooth depth, and keep each point on its camera ray. */
    {
        std::vector<float> zcopy(grid.size());
        for (size_t i = 0; i < grid.size(); i++) zcopy[i] = grid[i].z;
        for (int iy = 0; iy < gh; iy++) {
            for (int ix = 0; ix < gw; ix++) {
                ShadeVert &v = grid[(size_t)iy * gw + ix];
                if (!v.ok || v.z < 40.f) continue;
                float s = 0.f, w = 0.f;
                for (int dy = -2; dy <= 2; dy++) {
                    for (int dx = -2; dx <= 2; dx++) {
                        int yy = iy + dy, xx = ix + dx;
                        if (yy < 0 || xx < 0 || yy >= gh || xx >= gw) continue;
                        const ShadeVert &nb = grid[(size_t)yy * gw + xx];
                        if (!nb.ok) continue;
                        float wt = std::exp(-(dx * dx + dy * dy) / 2.5f);
                        s += wt * zcopy[(size_t)yy * gw + xx];
                        w += wt;
                    }
                }
                if (w <= 0.f) continue;
                float z = s / w;
                float scale = z / v.z;
                v.x *= scale;
                v.y *= scale;
                v.z = z;
            }
        }
    }
    if (nok > 40) {
        /* The turn axis sits just behind the measured shell. Putting it in
         * the surface makes the front and back of a few millimetres of relief
         * count as opposite sides of the object. */
        body.axis = cv::Vec3f((float)(sx3 / nok), (float)(sy3 / nok), z_far + 8.f);
    }
    }

    const float bump_mm = 1.8f;
    if (shade_live) {
        body.solid.create(gray.rows, gray.cols, CV_8UC3);
        body.side.create(gray.rows, gray.cols, CV_8UC3);
        render_body(grid, gw, gh, body.solid, yaw_deg, pitch_deg, bump_mm, &dist, user_zoom);
        float ignored = 0;
        float side_yaw = shape == FOX_SHAPE_MEASURED ? 32.f : 78.f;
        render_body(grid, gw, gh, body.side, side_yaw, -8.f, bump_mm, &ignored, 0);
    } else {
        (void)yaw_deg;
        (void)pitch_deg;
        (void)dist;
        (void)user_zoom;
    }

    std::vector<float> zs;
    zs.reserve(grid.size() / 2);
    body.P.reserve(grid.size());
    body.N.reserve(grid.size());
    auto displaced = [&](int iy, int ix, cv::Vec3f &p) -> bool {
        if (iy < 0 || ix < 0 || iy >= gh || ix >= gw) return false;
        const ShadeVert &v = grid[(size_t)iy * gw + ix];
        if (!v.ok) return false;
        p = cv::Vec3f(v.x, v.y, v.z - bump_mm * v.bump);
        return true;
    };
    for (int iy = 0; iy < gh; iy++) {
        for (int ix = 0; ix < gw; ix++) {
            cv::Vec3f p, pr, pd;
            if (!displaced(iy, ix, p)) continue;
            cv::Vec3f nrm(0, 0, -1);
            if (displaced(iy, ix + 1, pr) && displaced(iy + 1, ix, pd)) {
                nrm = (pr - p).cross(pd - p);
                float ln = std::sqrt(nrm.dot(nrm));
                if (ln > 1e-6f) nrm /= ln;
                else nrm = cv::Vec3f(0, 0, -1);
                if (nrm[2] > 0.f) nrm = -nrm;
            }
            body.P.push_back(p);
            body.N.push_back(nrm);
            if (((iy * gw + ix) & 3) == 0) zs.push_back(p[2]);
        }
    }
    if (!zs.empty()) {
        size_t mid = zs.size() / 2;
        std::nth_element(zs.begin(), zs.begin() + (ptrdiff_t)mid, zs.end());
        body.median_mm = zs[mid];
    }
    body.tris.reserve((size_t)gw * gh * 2);
    for (int gy = 0; gy < gh - 1; gy++) {
        for (int gx = 0; gx < gw - 1; gx++) {
            cv::Vec3f p00, p10, p01, p11;
            bool a = displaced(gy, gx, p00);
            bool b = displaced(gy, gx + 1, p10);
            bool c0 = displaced(gy + 1, gx, p01);
            bool d = displaced(gy + 1, gx + 1, p11);
            auto tone3 = [&](int y0, int x0, int y1, int x1, int y2, int x2) {
                const int ys[3] = {y0, y1, y2};
                const int xs[3] = {x0, x1, x2};
                float s = 0;
                int n = 0;
                for (int i = 0; i < 3; i++) {
                    const ShadeVert &v = grid[(size_t)ys[i] * gw + xs[i]];
                    if (!v.ok) continue;
                    s += v.albedo;
                    n++;
                }
                return n ? s / (float)n : 0.55f;
            };
            if (a && b && c0 && cv::norm(p00 - p10) < 14.f && cv::norm(p00 - p01) < 14.f &&
                cv::norm(p10 - p01) < 14.f)
                body.tris.push_back(Tri{p00, p10, p01, tone3(gy, gx, gy, gx + 1, gy + 1, gx)});
            if (b && d && c0 && cv::norm(p10 - p11) < 14.f && cv::norm(p10 - p01) < 14.f &&
                cv::norm(p11 - p01) < 14.f)
                body.tris.push_back(Tri{p10, p11, p01, tone3(gy, gx + 1, gy + 1, gx + 1, gy + 1, gx)});
        }
    }
    return body;
}

static const int kBins = 72;

static cv::Matx33f rot_y(float a) {
    float c = std::cos(a), s = std::sin(a);
    return cv::Matx33f(c, 0.f, s, 0.f, 1.f, 0.f, -s, 0.f, c);
}

/* 0 is the side that faced the camera on the first frame. */
static int bin_of(float x, float z) {
    float a = std::atan2(x, -z);
    float u = (a + 3.14159265f) / (6.2831853f);
    int b = (int)std::floor(u * kBins);
    if (b < 0) b = 0;
    if (b >= kBins) b = kBins - 1;
    return b;
}

static float bin_angle(int b) {
    return -3.14159265f + (b + 0.5f) * (6.2831853f / kBins);
}

/* Fixed window on the object, high-pass so the outline does not pin the
 * shift at zero. The window size does not follow the silhouette, so a turn
 * that makes one side look wider still measures both directions.
 * phaseCorrelate(prev, curr) shift.x is positive when the texture moves right. */
static cv::Mat front_crop(const BodyView &body, float cx, float cy) {
    const int cw = 360;
    const int ch = 220;
    cv::Mat crop(ch, cw, CV_8U, cv::Scalar(0));
    int x0 = (int)std::lround(cx) - cw / 2;
    int y0 = (int)std::lround(cy) - ch / 2;
    int src_x = std::max(0, x0);
    int src_y = std::max(0, y0);
    int dst_x = src_x - x0;
    int dst_y = src_y - y0;
    int w = std::min(body.gray.cols - src_x, cw - dst_x);
    int h = std::min(body.gray.rows - src_y, ch - dst_y);
    if (w < 48 || h < 48) return cv::Mat();
    body.gray(cv::Rect(src_x, src_y, w, h)).copyTo(crop(cv::Rect(dst_x, dst_y, w, h)));
    cv::Mat small, f, blur;
    cv::resize(crop, small, cv::Size(180, 110), 0, 0, cv::INTER_AREA);
    small.convertTo(f, CV_32F, 1.0 / 255.0);
    cv::GaussianBlur(f, blur, cv::Size(0, 0), 4.0);
    f -= blur;
    return f;
}

/* Rotation of prev camera points that lands them on curr. 0 if it cannot tell. */
static float yaw_from_points(const std::vector<cv::Vec3f> &prev, const std::vector<cv::Vec3f> &curr,
                             const cv::Vec3f &axis, float &inlier_frac) {
    inlier_frac = 0.f;
    if (prev.size() < 40 || curr.size() < 40) return 0.f;
    const float cell = 4.f;
    std::unordered_set<uint64_t> occ;
    occ.reserve(curr.size() * 2);
    auto key = [&](const cv::Vec3f &p) {
        int ix = (int)std::lround(p[0] / cell);
        int iy = (int)std::lround(p[1] / cell);
        int iz = (int)std::lround(p[2] / cell);
        return voxel_key(ix, iy, iz);
    };
    for (size_t i = 0; i < curr.size(); i += 2) occ.insert(key(curr[i]));
    int stride = std::max(1, (int)prev.size() / 350);
    float best_th = 0.f;
    int best_n = 0;
    int tested = 0;
    for (int deg = -20; deg <= 20; deg++) {
        float th = deg * 3.14159265f / 180.f;
        cv::Matx33f R = rot_y(th);
        int n = 0;
        tested = 0;
        for (size_t i = 0; i < prev.size(); i += (size_t)stride) {
            tested++;
            cv::Vec3f q = axis + R * (prev[i] - axis);
            if (occ.count(key(q))) n++;
        }
        if (n > best_n) {
            best_n = n;
            best_th = th;
        }
    }
    if (tested < 20) return 0.f;
    inlier_frac = (float)best_n / (float)tested;
    if (inlier_frac < 0.28f) return 0.f;
    return best_th;
}

/* 1 = turn applied, 0 = lost (yaw stays put), -1 = no object.
 * The window stays on the object instead of chasing the silhouette.
 * A measured turn is taken from the depth itself when that agrees. */
static int track_spin(fox_live *L, const BodyView &body, float &response) {
    response = 0.f;
    if (body.gray.empty() || body.radius_px < 30.f || body.P.size() < 80) return -1;
    if (!L->track_have) {
        L->track_cx = body.center.x;
        L->track_cy = body.center.y;
    } else {
        L->track_cx = 0.8f * L->track_cx + 0.2f * body.center.x;
        L->track_cy = 0.8f * L->track_cy + 0.2f * body.center.y;
    }
    cv::Mat f = front_crop(body, L->track_cx, L->track_cy);
    if (f.empty()) return -1;
    if (!L->track_have || L->track_ref.size() != f.size()) {
        L->track_ref = f;
        L->track_have = 1;
        L->track_miss = 0;
        L->track_pts = body.P;
        L->track_axis = body.axis;
        response = 1.f;
        return 1;
    }
    /* Several tiles. One cheek leaving the window must not cancel the turn,
     * and one failed tile must not throw the frame away. */
    const int tiles_x = 4;
    const int tiles_y = 3;
    std::vector<float> xs, ys;
    xs.reserve(12);
    ys.reserve(12);
    float best_resp = 0.f;
    int tw = f.cols / tiles_x;
    int th = f.rows / tiles_y;
    for (int ty = 0; ty < tiles_y; ty++) {
        for (int tx = 0; tx < tiles_x; tx++) {
            cv::Rect r(tx * tw, ty * th, tw, th);
            if (r.width < 24 || r.height < 24) continue;
            cv::Mat window;
            cv::createHanningWindow(window, r.size(), CV_32F);
            double resp = 0;
            cv::Point2d shift = cv::phaseCorrelate(L->track_ref(r), f(r), window, &resp);
            if (!std::isfinite(resp) || resp < 0.02 || !std::isfinite(shift.x) || !std::isfinite(shift.y))
                continue;
            if (std::fabs(shift.y) > r.height * 0.45) continue;
            xs.push_back((float)shift.x);
            ys.push_back((float)shift.y);
            if (resp > best_resp) best_resp = (float)resp;
        }
    }
    response = best_resp;
    float sx = 0.f, sy = 0.f;
    bool have_tex = xs.size() >= 3;
    if (have_tex) {
        std::nth_element(xs.begin(), xs.begin() + (ptrdiff_t)xs.size() / 2, xs.end());
        std::nth_element(ys.begin(), ys.begin() + (ptrdiff_t)ys.size() / 2, ys.end());
        sx = xs[xs.size() / 2];
        sy = ys[ys.size() / 2];
    }
    float radius_s = body.radius_px * ((float)f.cols / 360.f);
    if (radius_s < 8.f) radius_s = 8.f;
    float dtheta = have_tex ? -(sx / radius_s) : 0.f;
    float frac = 0.f;
    float th3 = yaw_from_points(L->track_pts, body.P, L->track_axis, frac);
    /* Depth wins when it sees a real turn. A still surface must not cancel
     * the texture measurement of a slow turn. */
    if (frac >= 0.28f && (!have_tex || std::fabs(th3) >= 1.f * 3.14159265f / 180.f)) dtheta = th3;
    if (!have_tex && frac < 0.28f) {
        L->track_miss++;
        if (L->track_miss >= 3) {
            L->track_ref = f;
            L->track_pts = body.P;
            L->track_axis = body.axis;
            L->track_miss = 0;
        }
        return 0;
    }
    if (!std::isfinite(dtheta)) dtheta = 0.f;
    const float kMax = 15.f * 3.14159265f / 180.f;
    if (dtheta > kMax) dtheta = kMax;
    if (dtheta < -kMax) dtheta = -kMax;
    if (have_tex && std::fabs(sy) > f.rows * 0.45f && frac < 0.28f) {
        L->track_ref = f;
        return 0;
    }
    L->object_yaw += dtheta;
    L->track_ref = f;
    L->track_pts = body.P;
    L->track_axis = body.axis;
    L->track_miss = 0;
    return 1;
}

static uint64_t cell_key(int ia, int iy) {
    return ((uint64_t)(uint32_t)ia << 32) | (uint32_t)iy;
}

/* One observation per cell this frame, then blended into the running surface.
 * A frontal view counts for more than a grazing one. Repeat passes raise the
 * weight and pull the point toward the new measurement instead of replacing
 * the patch. */
static cv::Vec3f world_of(const cv::Vec3f &p, const cv::Matx33f &R, const cv::Vec3f &t, float scale) {
    return scale * (R * p) + t;
}

/* measured: align this depth surface onto the model, then merge.
 * Returns 0 when the view does not land on the model. */
static int fuse_shell(fox_live *L, const BodyView &body, int measured) {
    if (L->wedge.size() != (size_t)kBins) L->wedge.assign(kBins, fox_live::Wedge{});
    /* Positive object_yaw is a turn whose texture moved left. The shell that
     * is in front of the camera belonged on the other side, so it is swung
     * the opposite way around the first view's axis. Both signs are used. */
    cv::Matx33f R = rot_y(-L->object_yaw);
    cv::Vec3f axis = body.axis;
    if (!L->have_axis) {
        L->axis0 = axis;
        L->have_axis = 1;
        L->radius_mm = std::max(8.f, body.radius_px * body.axis[2] / 2742.f);
        L->span_y0 = 1e9f;
        L->span_y1 = -1e9f;
    }
    cv::Vec3f t = L->axis0 - R * axis;
    /* Measured stays on the turn axis. A free alignment walks the shell
     * sideways and the export becomes a trail of copies. Scale stays 1.
     * The turn itself is object_yaw from track_spin; this does not drop it. */
    const float scale = 1.f;
    L->pose_R = R;
    L->pose_t = t;
    L->pose_s = scale;
    const float cell = measured ? 1.4f : 0.8f;
    struct Acc {
        cv::Vec3f sum;
        int n = 0;
        float view = 1.f;
    };
    std::unordered_map<uint64_t, Acc> frame;
    frame.reserve(body.tris.size());
    auto key_of_point = [&](const cv::Vec3f &w) {
        int ix = (int)std::lround(w[0] / cell);
        int iy = (int)std::lround(w[1] / cell);
        int iz = (int)std::lround(w[2] / cell);
        return voxel_key(ix, iy, iz);
    };
    auto faces_camera = [&](const Tri &src) {
        if (!measured) return true;
        cv::Vec3f n = (src.b - src.a).cross(src.c - src.a);
        float ln = std::sqrt(n.dot(n));
        return ln > 1e-6f && n[2] / ln < 0.25f;
    };
    /* 0 = already on the model, blend into that cell.
     * 1 = a few millimetres off, ignore it or it becomes a second skin.
     * 2 = far from the model, this is new surface. */
    auto classify = [&](const cv::Vec3f &w, uint64_t &k) -> int {
        if (!measured) {
            k = key_of_point(w);
            return 2;
        }
        if (L->cloud.size() >= 80) {
            int id = 0;
            if (nearest_model(L, w, id)) {
                float d = cv::norm(L->cloud[(size_t)id] - w);
                if (d < 8.f) {
                    k = key_of_point(L->cloud[(size_t)id]);
                    return 0;
                }
            }
        }
        k = key_of_point(w);
        return 2;
    };
    for (const Tri &src : body.tris) {
        if (!faces_camera(src)) continue;
        cv::Vec3f corners[3] = {src.a, src.b, src.c};
        for (int i = 0; i < 3; i++) {
            cv::Vec3f w = world_of(corners[i], R, t, scale);
            uint64_t k = 0;
            if (classify(w, k) == 1) continue;
            Acc &a = frame[k];
            a.sum += w;
            a.n++;
            L->span_y0 = std::min(L->span_y0, w[1]);
            L->span_y1 = std::max(L->span_y1, w[1]);
        }
    }
    if (frame.size() < 30) return measured ? 0 : 1;
    std::unordered_map<uint64_t, cv::Vec3f> before;
    before.reserve(frame.size());
    for (const auto &kv : frame) {
        auto it = L->cells.find(kv.first);
        if (it != L->cells.end() && it->second.n > 0) before.emplace(kv.first, it->second.p);
    }
    for (const auto &kv : frame) {
        const Acc &a = kv.second;
        cv::Vec3f p = a.sum * (1.f / (float)a.n);
        fox_live::Sample &s = L->cells[kv.first];
        if (s.n <= 0) {
            s.p = p;
            s.w = a.view;
            s.n = 1;
        } else {
            float keep = s.w / (s.w + a.view);
            s.p = s.p * keep + p * (1.f - keep);
            s.w = std::min(s.w + a.view, 24.f);
            s.n++;
        }
    }
    for (fox_live::TriRef &ref : L->refs) {
        for (int i = 0; i < 3; i++) {
            auto old = before.find(ref.k[i]);
            auto cur = L->cells.find(ref.k[i]);
            if (old == before.end() || cur == L->cells.end()) continue;
            cv::Vec3f delta = cur->second.p - old->second;
            float len = cv::norm(delta);
            if (len > 0.4f) delta *= 0.4f / len;
            ref.p[i] += delta;
        }
    }
    /* The first time a spot is seen, keep that triangle. A later pass moves
     * the vertices above. A pass over a gap adds the new triangles. */
    std::unordered_set<uint64_t> fresh;
    for (const Tri &src : body.tris) {
        if (!faces_camera(src)) continue;
        cv::Vec3f corners[3] = {src.a, src.b, src.c};
        cv::Vec3f w[3];
        for (int i = 0; i < 3; i++) w[i] = world_of(corners[i], R, t, scale);
        cv::Vec3f mid = (w[0] + w[1] + w[2]) * (1.f / 3.f);
        uint64_t ck = 0;
        int kind = classify(mid, ck);
        if (L->seen_cells.count(ck)) continue;
        if (measured && kind != 2) continue;
        fresh.insert(ck);
        fox_live::TriRef ref;
        ref.bin = bin_of(mid[0] - L->axis0[0], mid[2] - L->axis0[2]);
        ref.tone = src.tone;
        for (int i = 0; i < 3; i++) {
            ref.k[i] = key_of_point(w[i]);
            ref.p[i] = w[i];
        }
        L->refs.push_back(ref);
        if (ref.bin >= 0 && ref.bin < kBins) L->wedge[(size_t)ref.bin].filled = 1;
    }
    L->seen_cells.insert(fresh.begin(), fresh.end());
    for (auto &wd : L->wedge) wd.tris.clear();
    for (const fox_live::TriRef &ref : L->refs) {
        if (ref.bin < 0 || ref.bin >= kBins) continue;
        if (cv::norm(ref.p[0] - ref.p[1]) > 28.f || cv::norm(ref.p[0] - ref.p[2]) > 28.f) continue;
        L->wedge[(size_t)ref.bin].tris.push_back(Tri{ref.p[0], ref.p[1], ref.p[2], ref.tone});
        L->wedge[(size_t)ref.bin].filled = 1;
    }
    L->cloud.clear();
    L->cloud_n.clear();
    L->vox.clear();
    cv::Vec3f centre(0, 0, 0);
    long passes = 0;
    int cells = 0;
    for (const auto &kv : L->cells) {
        centre += kv.second.p;
        passes += kv.second.n;
        cells++;
        if (L->cloud.size() >= 80000) continue;
        const cv::Vec3f &p = kv.second.p;
        int ix = (int)std::floor(p[0] / 2.5f);
        int iy = (int)std::floor(p[1] / 2.5f);
        int iz = (int)std::floor(p[2] / 2.5f);
        uint64_t k = voxel_key(ix, iy, iz);
        if (L->vox.find(k) != L->vox.end()) continue;
        L->vox.emplace(k, (uint32_t)L->cloud.size());
        L->cloud.push_back(p);
        L->cloud_n.push_back(cv::Vec3f(0, 0, -1));
    }
    if (cells > 0) centre *= 1.f / (float)cells;
    (void)centre;
    int nbin = 0;
    for (const auto &wd : L->wedge)
        if (!wd.tris.empty()) nbin++;
    L->scanned_bins = nbin;
    L->detail = cells ? std::max(1, (int)(passes / cells)) : 1;
    return 1;
}

static int model_tris(const fox_live *L) {
    int n = 0;
    for (const auto &w : L->wedge) n += (int)w.tris.size();
    return n;
}

static void draw_coverage_ring(cv::Mat &img, const fox_live *L) {
    int r = std::max(28, std::min(img.cols, img.rows) / 10);
    cv::Point c(img.cols - r - 16, img.rows - r - 18);
    cv::circle(img, c, r, cv::Scalar(28, 28, 32), -1, cv::LINE_AA);
    cv::circle(img, c, r, cv::Scalar(70, 70, 78), 2, cv::LINE_AA);
    for (int b = 0; b < kBins; b++) {
        if (b >= (int)L->wedge.size() || !L->wedge[(size_t)b].filled) continue;
        float a0 = bin_angle(b) - 0.5f * (6.2831853f / kBins);
        float a1 = a0 + (6.2831853f / kBins);
        /* 0° (first side) at the top of the dial. OpenCV angles grow clockwise. */
        float d0 = 90.f - a0 * 180.f / 3.14159265f;
        float d1 = 90.f - a1 * 180.f / 3.14159265f;
        cv::ellipse(img, c, cv::Size(r - 6, r - 6), 0, d1, d0, cv::Scalar(70, 200, 110), 8, cv::LINE_AA);
    }
    int deg = (int)std::lround(L->scanned_bins * (360.0 / kBins));
    char t[32];
    snprintf(t, sizeof t, "%d°", deg);
    cv::putText(img, t, cv::Point(c.x - 16, c.y + 5), cv::FONT_HERSHEY_SIMPLEX, 0.45,
                cv::Scalar(240, 240, 240), 1, cv::LINE_AA);
}

static void render_model(fox_live *L, cv::Mat &out) {
    out.setTo(cv::Scalar(12, 14, 18));
    cv::Vec3f c = L->axis0;
    c[1] = 0.5f * (L->span_y0 + L->span_y1);
    float height = std::max(10.f, L->span_y1 - L->span_y0);
    float extent = std::max(height, L->radius_mm * 2.2f);
    int w = out.cols, h = out.rows;
    if (L->dist <= 1.f) L->dist = std::max(extent * 1.65f, 40.f);
    float dist = L->dist;
    float yaw = L->yaw;
    float pitch = L->pitch;
    float scale = 1.05f * (float)h / std::max(dist, 1.f);
    float cyaw = std::cos(yaw), syaw = std::sin(yaw);
    float cp = std::cos(pitch), sp = std::sin(pitch);
    std::vector<float> zbuf((size_t)w * h, 1e9f);

    auto project = [&](const cv::Vec3f &p, float &sx, float &sy, float &zc) {
        cv::Vec3f d = p - c;
        float x1 = cyaw * d[0] - syaw * d[2];
        float z1 = syaw * d[0] + cyaw * d[2];
        float y2 = cp * d[1] - sp * z1;
        zc = sp * d[1] + cp * z1;
        sx = w * 0.50f + x1 * scale;
        sy = h * 0.50f - y2 * scale;
    };
    auto paint_tri = [&](const cv::Vec3f &a, const cv::Vec3f &b, const cv::Vec3f &c0, cv::Vec3b col) {
        float ax, ay, az, bx, by, bz, cx, cy, cz;
        project(a, ax, ay, az);
        project(b, bx, by, bz);
        project(c0, cx, cy, cz);
        int minx = std::max(0, (int)std::floor(std::min(ax, std::min(bx, cx))));
        int maxx = std::min(w - 1, (int)std::ceil(std::max(ax, std::max(bx, cx))));
        int miny = std::max(0, (int)std::floor(std::min(ay, std::min(by, cy))));
        int maxy = std::min(h - 1, (int)std::ceil(std::max(ay, std::max(by, cy))));
        float area = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
        if (std::fabs(area) < 0.3f) return;
        for (int py = miny; py <= maxy; py++) {
            for (int px = minx; px <= maxx; px++) {
                float w0 = (bx - px) * (cy - py) - (by - py) * (cx - px);
                float w1 = (cx - px) * (ay - py) - (cy - py) * (ax - px);
                float w2 = (ax - px) * (by - py) - (ay - py) * (bx - px);
                if (!((w0 >= 0 && w1 >= 0 && w2 >= 0) || (w0 <= 0 && w1 <= 0 && w2 <= 0))) continue;
                float zc = (w0 * az + w1 * bz + w2 * cz) / area;
                float &slot = zbuf[(size_t)py * w + px];
                if (zc < slot) {
                    slot = zc;
                    out.at<cv::Vec3b>(py, px) = col;
                }
            }
        }
    };

    for (const auto &wedge : L->wedge) {
        if (!wedge.filled) continue;
        for (const Tri &t : wedge.tris) {
            cv::Vec3f n = (t.b - t.a).cross(t.c - t.a);
            float ln = std::sqrt(n.dot(n));
            if (ln < 1e-6f) continue;
            n /= ln;
            cv::Vec3f light = cv::Vec3f(-syaw, 0.25f, -cyaw);
            float ll = std::sqrt(light.dot(light)) + 1e-6f;
            float lam = n.dot(light) / ll;
            lam = lam * 0.55f + 0.45f;
            if (lam < 0.18f) lam = 0.18f;
            float tone = t.tone > 0.02f ? t.tone : 0.55f;
            int g = (int)std::min(255.f, 255.f * tone * (0.55f + 0.45f * lam));
            paint_tri(t.a, t.b, t.c, cv::Vec3b((uint8_t)g, (uint8_t)g, (uint8_t)g));
        }
    }
    draw_coverage_ring(out, L);
    if (L->mode == FOX_MODE_SCAN && L->last_track == 0) {
        cv::putText(out, "tracking lost  —  stay on the object", cv::Point(16, out.rows - 18),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(40, 40, 220), 2, cv::LINE_AA);
    }
}

static void paint_preview(fox_live *L, const cv::Mat &cam_a, const cv::Mat &cam_b,
                          const BodyView &body) {
    /* Keep the sensor aspect. A 1280x720 frame drawn into a tall half-panel
     * is squeezed to half its width, and a round crown looks smallest. */
    const int view_w = std::max(320, L->size.width);
    const int view_h = std::max(180, L->size.height);
    const int hero_w = view_w * 2;
    const int hero_h = view_h;
    const int side_h = std::max(1, hero_h / 3);
    const int side_w = std::max(1, side_h * view_w / view_h);
    cv::Size side(side_w, side_h);
    cv::Size view(view_w, view_h);

    cv::Mat hero(hero_h, hero_w, CV_8UC3, cv::Scalar(12, 12, 12));
    cv::Mat solid = body.solid;
    cv::Mat model;
    if ((L->shape == FOX_SHAPE_MEASURED && L->fused > 0) ||
        (L->shape != FOX_SHAPE_MEASURED && L->scanned_bins > 0)) {
        model.create(view_h, view_w, CV_8UC3);
        render_model(L, model);
        solid = model;
    }
    if (!body.camera.empty() && !solid.empty()) {
        cv::Mat left, right;
        cv::resize(body.camera, left, view, 0, 0, cv::INTER_AREA);
        if (solid.size() == view) right = solid;
        else cv::resize(solid, right, view, 0, 0, cv::INTER_AREA);
        label_panel(left, "camera");
        int deg = (int)std::lround(L->scanned_bins * (360.0 / kBins));
        char cap[96];
        if (L->scanned_bins > 0)
            snprintf(cap, sizeof cap, "3D   scanned %d°   open %d°   detail x%d", deg, 360 - deg,
                     std::max(1, L->detail));
        else
            snprintf(cap, sizeof cap, "3D");
        label_panel(right, cap);
        left.copyTo(hero(cv::Rect(0, 0, hero_w / 2, hero_h)));
        right.copyTo(hero(cv::Rect(hero_w / 2, 0, hero_w / 2, hero_h)));
    } else {
        cv::putText(hero, "waiting for a frame", cv::Point(12, hero_h / 2),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(160, 160, 160), 1, cv::LINE_AA);
    }

    /* Side cameras are the rectified stereo pair when it is available: lens
     * distortion removed and the two pictures share horizontal rows. */
    cv::Mat show_a, show_b;
    const cv::Mat &src_a = L->stereo_a.empty() ? cam_a : L->stereo_a;
    const cv::Mat &src_b = L->stereo_b.empty() ? cam_b : L->stereo_b;
    if (!src_a.empty()) {
        if (L->stereo_a.empty()) cv::rotate(src_a, show_a, cv::ROTATE_180);
        else show_a = src_a;
    }
    if (!src_b.empty()) {
        if (L->stereo_b.empty()) cv::rotate(src_b, show_b, cv::ROTATE_180);
        else show_b = src_b;
    }
    cv::Mat a = to_bgr(show_a, side);
    cv::Mat b = to_bgr(show_b, side);
    cv::Mat d = body.side.empty() ? cv::Mat(side, CV_8UC3, cv::Scalar(12, 14, 18))
                                  : to_bgr(body.side, side);
    if (body.side.empty() && L->scanned_bins > 0) {
        int deg = (int)std::lround(L->scanned_bins * (360.0 / kBins));
        char line[64];
        snprintf(line, sizeof line, "%d° scanned", deg);
        cv::putText(d, line, cv::Point(12, d.rows / 2 - 8), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                    cv::Scalar(90, 210, 120), 1, cv::LINE_AA);
        snprintf(line, sizeof line, "%d° not yet", 360 - deg);
        cv::putText(d, line, cv::Point(12, d.rows / 2 + 22), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                    cv::Scalar(160, 160, 170), 1, cv::LINE_AA);
    }
    label_panel(a, L->stereo_a.empty() ? "camera A" : "stereo A");
    label_panel(b, L->stereo_b.empty() ? "camera B" : "stereo B");
    label_panel(d, body.side.empty() ? "coverage" : "side");

    const int bar = 28;
    L->preview.create(bar + hero_h, hero_w + side_w, CV_8UC3);
    L->preview.setTo(cv::Scalar(12, 12, 12));
    hero.copyTo(L->preview(cv::Rect(0, bar, hero_w, hero_h)));
    a.copyTo(L->preview(cv::Rect(hero_w, bar, side_w, side_h)));
    b.copyTo(L->preview(cv::Rect(hero_w, bar + side_h, side_w, side_h)));
    d.copyTo(L->preview(cv::Rect(hero_w, bar + 2 * side_h, side_w, side_h)));
    L->view_x = 0;
    L->view_y = bar;
    L->view_w = hero_w;
    L->view_h = hero_h;

    cv::Scalar col = L->mode == FOX_MODE_SCAN ? cv::Scalar(80, 220, 80)
                    : L->mode == FOX_MODE_PAUSE ? cv::Scalar(80, 210, 230)
                                                : cv::Scalar(180, 180, 180);
    cv::putText(L->preview, L->line1, cv::Point(8, 18), cv::FONT_HERSHEY_SIMPLEX, 0.45, col, 1, cv::LINE_AA);
    L->buttons.clear();
    L->preview_w = L->preview.cols;
    L->preview_h = L->preview.rows;
}

static int ensure_kinfu(fox_live *L) {
    if (L->kf) return 0;
    try {
        cv::Ptr<cv::kinfu::Params> p = cv::kinfu::Params::coarseParams();
        p->frameSize = L->size;
        p->intr = cv::Matx33f(L->geom.fx, 0, L->geom.cx,
                              0, L->geom.fy, L->geom.cy,
                              0, 0, 1);
        p->depthFactor = 1.f;
        p->bilateral_sigma_depth = 0.008f;
        p->bilateral_sigma_spatial = 2.5f;
        p->bilateral_kernel_size = 3;
        p->icpDistThresh = 0.04f;
        p->icpAngleThresh = (float)(30.0 * CV_PI / 180.0);
        p->icpIterations = {10, 5, 3};
        p->pyramidLevels = (int)p->icpIterations.size();
        float z_near = 0.10f;
        float side = 0.40f;
        int dim = 128;
        p->volumeDims = cv::Vec3i::all(dim);
        p->voxelSize = side / (float)dim;
        p->tsdf_trunc_dist = 3.f * p->voxelSize;
        p->tsdf_max_weight = 48;
        p->tsdf_min_camera_movement = 0.f;
        p->truncateThreshold = 0.f;
        p->raycast_step_factor = 0.6f;
        cv::Matx44f pose = cv::Matx44f::eye();
        pose(0, 3) = -side * 0.5f;
        pose(1, 3) = -side * 0.5f;
        pose(2, 3) = z_near;
        p->setInitialVolumePose(pose);
        L->kf = cv::kinfu::KinFu::create(p);
    } catch (const cv::Exception &e) {
        fprintf(stderr, "fusion init failed: %s\n", e.what());
        L->kf.release();
        return -1;
    }
    if (!L->kf) {
        fprintf(stderr, "fusion init failed\n");
        return -1;
    }
    fprintf(stderr, "fusion volume %.0f mm cube, voxel %.1f mm, near plane at %.0f mm\n",
            L->kf->getParams().voxelSize * L->kf->getParams().volumeDims[0] * 1000.f,
            L->kf->getParams().voxelSize * 1000.f,
            L->kf->getParams().volumePose(2, 3) * 1000.f);
    return 0;
}

int fox_live_push(fox_live *live, const uint8_t *ya, const uint8_t *yb,
                  int width, int height, fox_live_status *status) {
    fox_live_status st{};
    st.tracking = -1;
    if (!live || width != live->calib.width || height != live->calib.height) return -1;

    auto t0 = std::chrono::steady_clock::now();
    cv::Mat A(height, width, CV_8UC1, const_cast<uint8_t *>(ya));
    cv::Mat B(height, width, CV_8UC1, const_cast<uint8_t *>(yb));
    cv::Mat Au, Bu;
    const cv::Mat *use_a = &A;
    const cv::Mat *use_b = &B;
    float bfx = (float)live->calib.cam[1].fx;
    float bfy = (float)live->calib.cam[1].fy;
    float bcx = (float)live->calib.cam[1].cx;
    float bcy = (float)live->calib.cam[1].cy;
    if (!live->und_bx.empty() && live->und_bx.size() == B.size()) {
        cv::remap(A, Au, live->und_ax, live->und_ay, cv::INTER_LINEAR);
        cv::remap(B, Bu, live->und_bx, live->und_by, cv::INTER_LINEAR);
        use_a = &Au;
        use_b = &Bu;
        bfx = (float)live->view_fx;
        bfy = (float)live->view_fy;
        bcx = (float)live->view_cx;
        bcy = (float)live->view_cy;
    }
    cv::Mat As, Bs;
    cv::resize(A, As, live->size, 0, 0, cv::INTER_AREA);
    cv::resize(B, Bs, live->size, 0, 0, cv::INTER_AREA);
    if (live->geom.map1x.size() == As.size() && live->geom.map2x.size() == Bs.size()) {
        cv::Mat ra, rb, ua, ub;
        cv::remap(As, ra, live->geom.map1x, live->geom.map1y, cv::INTER_LINEAR);
        cv::remap(Bs, rb, live->geom.map2x, live->geom.map2y, cv::INTER_LINEAR);
        cv::rotate(ra, ua, cv::ROTATE_180);
        cv::rotate(rb, ub, cv::ROTATE_180);
        live->stereo_a = ua;
        live->stereo_b = ub;
    }

    /* Camera B is the clean view (no projector dots). The picture is
     * undistorted above and turned upright inside make_body. */
    float yaw_deg = live->yaw * (180.f / 3.14159265f);
    float pitch_deg = live->pitch * (180.f / 3.14159265f);
    int measured = live->shape == FOX_SHAPE_MEASURED;
    int shade_live = 1;
    BodyView body = make_body(*use_b, A, &live->calib, bfx, bfy, bcx, bcy, yaw_deg, pitch_deg,
                              live->dist, live->user_zoom, shade_live, live->distance_mm, live->shape);
    live->locked = 1;

    auto t1 = std::chrono::steady_clock::now();
    st.match_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    st.valid_pixels = (int)body.P.size();
    st.median_mm = body.median_mm;

    if (live->mode == FOX_MODE_SCAN || live->mode == FOX_MODE_PAUSE) {
        float response = 0.f;
        int trk = track_spin(live, body, response);
        live->last_track = trk;
        if (trk < 0) {
            st.tracking = -1;
        } else if (trk == 0) {
            if (live->mode == FOX_MODE_SCAN) live->lost++;
            st.tracking = 0;
            /* Mold still keeps the solid of this frame. Measured waits until
             * the turn is known, so a lost frame is not pasted on the wrong side. */
            if (!measured && live->mode == FOX_MODE_SCAN) {
                fuse_shell(live, body, 0);
                live->fused++;
            }
        } else if (live->mode == FOX_MODE_SCAN) {
            /* Measured aligns this depth surface to the model and adds the
             * new part. A view that does not land is dropped. Mold always
             * keeps the turned solid. */
            int added = fuse_shell(live, body, measured);
            if (added || !measured) {
                live->fused++;
                live->tracked_frames++;
                st.tracking = 1;
            } else {
                live->lost++;
                st.tracking = 0;
            }
        } else {
            st.tracking = 1;
        }
    } else {
        live->last_track = -1;
        st.tracking = body.P.size() >= 80 ? 1 : -1;
        if (measured && live->fused == 0) live->shell = body.tris;
    }

    st.fused = live->fused;
    st.lost = live->lost;
    st.mode = live->mode;
    st.points = (measured && live->fused == 0) ? (int)live->shell.size() : model_tris(live);
    st.scanned_deg = (int)std::lround(live->scanned_bins * (360.0 / kBins));
    st.detail = std::max(1, live->detail);
    const char *mode_name = live->mode == FOX_MODE_SCAN ? "SCANNING" :
                            live->mode == FOX_MODE_PAUSE ? "PAUSED" : "STOPPED";
    const char *shape_name = measured ? "MEASURED" : "MOLD";
    const char *lost = (live->mode == FOX_MODE_SCAN && st.tracking == 0) ? "   TRACKING LOST" : "";
    float height_mm = (live->span_y1 > live->span_y0 && live->span_y0 < 1e8f)
                          ? live->span_y1 - live->span_y0 : 0.f;
    float width_mm = live->have_axis ? live->radius_mm * 2.f : 0.f;
    if (width_mm > 1.f && height_mm > 1.f)
        snprintf(live->line1, sizeof live->line1,
                 "%s   %s   %.0f mm away   model %.0f x %.0f mm   scanned %d°   detail x%d   %d tris%s",
                 shape_name, mode_name, live->distance_mm, width_mm, height_mm, st.scanned_deg,
                 st.detail, st.points, lost);
    else if (measured)
        snprintf(live->line1, sizeof live->line1,
                 "%s   %s   %.0f mm away   visible surface   %d tris",
                 shape_name, mode_name, live->distance_mm, st.points);
    else
        snprintf(live->line1, sizeof live->line1,
                 "%s   %s   %.0f mm away   scanned %d°   not scanned %d°   detail x%d   %d tris%s",
                 shape_name, mode_name, live->distance_mm, st.scanned_deg, 360 - st.scanned_deg,
                 st.detail, st.points, lost);
    paint_preview(live, *use_a, *use_b, body);
    if (status) *status = st;
    return 0;
}

const uint8_t *fox_live_preview_bgr(const fox_live *live, int *width, int *height) {
    if (!live || live->preview.empty()) return NULL;
    if (width) *width = live->preview_w;
    if (height) *height = live->preview_h;
    return live->preview.data;
}

int fox_live_write(fox_live *live, const char *stl_path, int *triangles_out) {
    std::vector<Tri> tris;
    if (live) {
        for (const auto &w : live->wedge)
            tris.insert(tris.end(), w.tris.begin(), w.tris.end());
    }
    if (tris.size() >= 80) {
        fprintf(stderr, "writing %zu scanned triangles, %d° covered\n", tris.size(),
                live->scanned_bins * 5);
    } else if (live && live->shell.size() >= 80) {
        fprintf(stderr, "writing %zu shell triangles\n", live->shell.size());
        tris = live->shell;
    } else if (live && live->cloud.size() >= 80) {
        fprintf(stderr, "meshing %zu model points\n", live->cloud.size());
        mesh_points(live->cloud, live->cloud_n, tris);
    } else if (live && live->kf) {
        cv::Mat pts, nrm;
        try {
            live->kf->getCloud(pts, nrm);
        } catch (const cv::Exception &e) {
            fprintf(stderr, "read model: %s\n", e.what());
        }
        std::vector<cv::Vec3f> P, N;
        if (read_cloud(pts, nrm, P, N)) {
            for (cv::Vec3f &p : P) p *= 1000.f;
            cv::Vec3f lo = P[0], hi = P[0];
            for (const cv::Vec3f &p : P) {
                lo[0] = std::min(lo[0], p[0]); lo[1] = std::min(lo[1], p[1]); lo[2] = std::min(lo[2], p[2]);
                hi[0] = std::max(hi[0], p[0]); hi[1] = std::max(hi[1], p[1]); hi[2] = std::max(hi[2], p[2]);
            }
            fprintf(stderr,
                    "fused cloud %zu points  bbox x %.0f..%.0f  y %.0f..%.0f  z %.0f..%.0f mm\n",
                    P.size(), lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]);
            mesh_points(P, N, tris);
        } else {
            fprintf(stderr, "fused cloud is empty (mat %d x %d type %d)\n",
                    pts.rows, pts.cols, pts.type());
        }
    }
    if (write_mesh(stl_path, tris) < 0) return -1;
    if (triangles_out) *triangles_out = (int)tris.size();
    return 0;
}

int fox_mesh_self_test(void) {
    const int n = 48;
    const float voxel = 1.f;
    const float radius = 12.f;
    const float ox = -24.f, oy = -24.f, oz = -24.f;
    std::vector<float> sdf((size_t)n * n * n);
    for (int z = 0; z < n; z++) {
        for (int y = 0; y < n; y++) {
            for (int x = 0; x < n; x++) {
                float px = ox + (x + 0.5f) * voxel;
                float py = oy + (y + 0.5f) * voxel;
                float pz = oz + (z + 0.5f) * voxel;
                float dist = std::sqrt(px * px + py * py + pz * pz);
                sdf[(size_t)((z * n + y) * n + x)] = dist - radius;
            }
        }
    }
    std::vector<Tri> tris;
    marching_cubes(sdf, NULL, n, n, n, ox, oy, oz, voxel, tris);
    if (tris.size() < 400) {
        fprintf(stderr, "mesh self-test: only %zu triangles\n", tris.size());
        return 1;
    }
    double worst = 0;
    for (const Tri &t : tris) {
        for (const cv::Vec3f &v : {t.a, t.b, t.c}) {
            double r = std::sqrt((double)v.dot(v));
            worst = std::max(worst, std::fabs(r - radius));
        }
    }
    fprintf(stderr, "mesh self-test: %zu triangles, max radius error %.3f mm\n", tris.size(), worst);
    if (worst >= 1.2) return 1;

    Tri one{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
    std::vector<Tri> one_tri{one};
    const char *outs[] = {"/tmp/fox3d-export.stl", "/tmp/fox3d-export.obj", "/tmp/fox3d-export.ply"};
    for (const char *path : outs) {
        if (write_mesh(path, one_tri) != 0) {
            fprintf(stderr, "mesh self-test: could not write %s\n", path);
            return 1;
        }
        FILE *fp = fopen(path, "rb");
        if (!fp) return 1;
        char buf[4096];
        size_t n = fread(buf, 1, sizeof buf - 1, fp);
        fclose(fp);
        remove(path);
        buf[n] = 0;
        if (strstr(buf, "JMM") || strstr(buf, "DevID") || strstr(buf, "serial")) {
            fprintf(stderr, "mesh self-test: %s contains a device id\n", path);
            return 1;
        }
    }
    fprintf(stderr, "mesh self-test: stl, obj, and ply written with geometry only\n");
    return 0;
}
