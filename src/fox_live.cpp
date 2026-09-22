#include "fox_live.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/rgbd/kinfu.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <unordered_map>
#include <vector>

namespace {

#include "mc_tables.inc"

struct Tri {
    cv::Vec3f a, b, c;
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

static int write_stl(const char *path, const std::vector<Tri> &tris) {
    FILE *fp = fopen(path, "wb");
    if (!fp) return -1;
    char header[80];
    memset(header, 0, sizeof header);
    snprintf(header, sizeof header, "fox3d fused %zu triangles", tris.size());
    fwrite(header, 1, 80, fp);
    uint32_t n = (uint32_t)tris.size();
    fwrite(&n, 4, 1, fp);
    for (const Tri &t : tris) {
        cv::Vec3f u = t.b - t.a;
        cv::Vec3f v = t.c - t.a;
        cv::Vec3f nn = u.cross(v);
        float len = std::sqrt(nn.dot(nn));
        if (len < 1e-12f) nn = cv::Vec3f(0, 0, 1);
        else nn /= len;
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
    cv::stereoRectify(K1, D1, K2, D2, size, R, T, R1, R2, P1, P2, g.Q,
                      cv::STEREO_ZERO_DISPARITY, -1, size);
    cv::initUndistortRectifyMap(K1, D1, R1, P1, size, CV_32FC1, g.map1x, g.map1y);
    cv::initUndistortRectifyMap(K2, D2, R2, P2, size, CV_32FC1, g.map2x, g.map2y);
    g.fx = (float)P1.at<double>(0, 0);
    g.fy = (float)P1.at<double>(1, 1);
    g.cx = (float)P1.at<double>(0, 2);
    g.cy = (float)P1.at<double>(1, 2);

    double baseline = std::fabs(1.0 / g.Q.at<double>(3, 2));
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
    cv::GaussianBlur(f.rect, left_b, cv::Size(0, 0), 3.2);
    cv::GaussianBlur(recR, right_b, cv::Size(0, 0), 3.2);
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
            float d = drow[x];
            float z = row[x][2];
            /* stereoRectify's Q puts the scene at negative Z when the baseline
             * Tx is positive. Depth is the distance in front of the camera. */
            if (std::isfinite(z) && z < 0.f) z = -z;
            if (std::fabs(d) <= 1.f || !std::isfinite(z)) continue;
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
    float yaw = 0.9f;
    float pitch = -0.35f;
    float dist = 420.f;
    int spin = 1;
    int dragging = 0;
    int drag_x = 0;
    int drag_y = 0;
    int view_x = 0, view_y = 0, view_w = 0, view_h = 0;
    std::vector<FoxButton> buttons;
};

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
    snprintf(L->line1, sizeof L->line1, "stopped");
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
}

void fox_live_set_mode(fox_live *live, int mode) {
    if (!live) return;
    if (mode != FOX_MODE_SCAN && mode != FOX_MODE_PAUSE) mode = FOX_MODE_STOP;
    live->mode = mode;
}

int fox_live_mode(const fox_live *live) { return live ? live->mode : FOX_MODE_STOP; }
int fox_live_points(const fox_live *live) { return live ? (int)live->cloud.size() : 0; }
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
        live->dist *= delta > 0 ? 0.9f : 1.1f;
        if (live->dist < 80.f) live->dist = 80.f;
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

/* Depth is metres, Y down, Z forward. Stored points are millimetres, Y up. */
static void depth_samples(const cv::Mat &depth, float fx, float fy, float cx, float cy,
                          const cv::Affine3f *world_from_cam,
                          std::vector<cv::Vec3f> &P, std::vector<cv::Vec3f> &N) {
    P.clear();
    N.clear();
    if (depth.empty()) return;
    for (int y = 1; y < depth.rows - 1; y += 2) {
        const float *row = depth.ptr<float>(y);
        const float *dn = depth.ptr<float>(y + 1);
        for (int x = 1; x < depth.cols - 1; x += 2) {
            float z = row[x];
            float zx = row[x + 1];
            float zy = dn[x];
            if (z <= 0.f || zx <= 0.f || zy <= 0.f) continue;
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
    float dist = L->dist;
    cv::Vec3f eye = c + cv::Vec3f(dist * std::cos(L->pitch) * std::sin(L->yaw),
                                  dist * std::sin(L->pitch),
                                  dist * std::cos(L->pitch) * std::cos(L->yaw));
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
    const int rad = 2;
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

static void paint_preview(fox_live *L, const cv::Mat &cam_a, const cv::Mat &cam_b,
                          const cv::Mat &depth_m, int tracking) {
    const int hero_w = L->size.width * 2;
    const int hero_h = L->size.height * 2;
    const int side_w = hero_w / 3;
    const int side_h = hero_h / 3;
    cv::Size side(side_w, side_h);

    std::vector<cv::Vec3f> frame_p, frame_n;
    const std::vector<cv::Vec3f> *draw_p = &L->cloud;
    const std::vector<cv::Vec3f> *draw_n = &L->cloud_n;
    char title[96];
    if (L->cloud.size() >= 30) {
        snprintf(title, sizeof title, "3D model   %zu points   drag to orbit", L->cloud.size());
    } else {
        depth_samples(depth_m, L->geom.fx, L->geom.fy, L->geom.cx, L->geom.cy, NULL, frame_p, frame_n);
        draw_p = &frame_p;
        draw_n = &frame_n;
        snprintf(title, sizeof title, "3D preview   %zu points   press Start to keep them", frame_p.size());
    }
    cv::Mat hero = render_orbit(L, *draw_p, *draw_n, hero_w, hero_h, title);
    (void)tracking;

    cv::Mat a = to_bgr(cam_a, side);
    cv::Mat b = to_bgr(cam_b, side);
    cv::Mat d = depth_color(depth_m, L->opt.min_mm, L->opt.max_mm, side);
    label_panel(a, "camera A");
    label_panel(b, "camera B");
    label_panel(d, "depth");

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
    cv::Mat As, Bs;
    cv::resize(A, As, live->size, 0, 0, cv::INTER_AREA);
    cv::resize(B, Bs, live->size, 0, 0, cv::INTER_AREA);

    DepthFrame chosen;
    if (!live->locked) {
        DepthFrame fa = match_depth(live->geom, As, Bs, live->opt.min_mm, live->opt.max_mm);
        DepthFrame fb = match_depth(live->geom, Bs, As, live->opt.min_mm, live->opt.max_mm);
        int prefer_a = fa.valid >= fb.valid;
        int best = prefer_a ? fa.valid : fb.valid;
        int other = prefer_a ? fb.valid : fa.valid;
        if (best >= 1500 && (best > other * 1.12 || best > 6000)) {
            live->locked = prefer_a ? 1 : 2;
            live->hist_n = 0;
            fprintf(stderr, "cameras locked: %s is calib camera 1 (%d vs %d in-range pixels)\n",
                    prefer_a ? "A" : "B", best, other);
            chosen = prefer_a ? fa : fb;
        } else {
            chosen = prefer_a ? fa : fb;
            snprintf(live->line1, sizeof live->line1,
                     "searching  A-as-left %d px   B-as-left %d px   need a surface at %.0f-%.0f mm",
                     fa.valid, fb.valid, live->opt.min_mm, live->opt.max_mm);
        }
    } else {
        if (live->locked == 1) chosen = match_depth(live->geom, As, Bs, live->opt.min_mm, live->opt.max_mm);
        else chosen = match_depth(live->geom, Bs, As, live->opt.min_mm, live->opt.max_mm);
    }

    if (live->locked) {
        live->depth_hist[live->hist_i] = chosen.depth_m.clone();
        live->hist_i = (live->hist_i + 1) % 3;
        if (live->hist_n < 3) live->hist_n++;
        if (live->hist_n == 3) {
            cv::Mat steady = cv::Mat::zeros(chosen.depth_m.size(), CV_32F);
            int valid = 0;
            std::vector<float> samples;
            for (int y = 0; y < steady.rows; y++) {
                const float *a = live->depth_hist[0].ptr<float>(y);
                const float *b = live->depth_hist[1].ptr<float>(y);
                const float *c = live->depth_hist[2].ptr<float>(y);
                float *o = steady.ptr<float>(y);
                for (int x = 0; x < steady.cols; x++) {
                    float v0 = a[x], v1 = b[x], v2 = c[x];
                    if (v0 <= 0.f || v1 <= 0.f || v2 <= 0.f) continue;
                    float lo = std::min(v0, std::min(v1, v2));
                    float hi = std::max(v0, std::max(v1, v2));
                    if (hi - lo > 0.008f) continue;
                    o[x] = v0 + v1 + v2 - lo - hi;
                    valid++;
                    if (((y * steady.cols + x) & 15) == 0) samples.push_back(o[x] * 1000.f);
                }
            }
            chosen.depth_m = steady;
            chosen.valid = valid;
            if (!samples.empty()) {
                size_t mid = samples.size() / 2;
                std::nth_element(samples.begin(), samples.begin() + (ptrdiff_t)mid, samples.end());
                chosen.median_mm = samples[mid];
            }
        }
    }

    auto t1 = std::chrono::steady_clock::now();
    st.match_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    st.valid_pixels = chosen.valid;
    st.median_mm = chosen.median_mm;

    if (live->mode == FOX_MODE_SCAN && live->locked && chosen.valid < 800) {
        snprintf(live->line1, sizeof live->line1,
                 "SCANNING   model %zu   surface left the window (%d px)",
                 live->cloud.size(), chosen.valid);
        snprintf(live->line2, sizeof live->line2,
                 "move the object back into range, then keep rotating");
    } else if (live->mode == FOX_MODE_SCAN && live->locked && chosen.valid >= 800) {
        if (ensure_kinfu(live) == 0) {
            bool ok = false;
            try {
                ok = live->kf->update(chosen.depth_m);
            } catch (const cv::Exception &e) {
                fprintf(stderr, "fusion: %s\n", e.what());
                if (live->fused == 0) live->kf.release();
            }
            if (ok) {
                live->fused++;
                st.tracking = 1;
                try {
                    cv::Affine3f cam_from_world(live->kf->getPose());
                    cv::Affine3f world_from_cam = cam_from_world.inv();
                    std::vector<cv::Vec3f> P, N;
                    depth_samples(chosen.depth_m, live->geom.fx, live->geom.fy, live->geom.cx, live->geom.cy,
                                  &world_from_cam, P, N);
                    absorb_points(live, P, N);
                } catch (const cv::Exception &e) {
                    fprintf(stderr, "model points: %s\n", e.what());
                }
            } else {
                live->lost++;
                st.tracking = 0;
            }
        }
    }

    st.fused = live->fused;
    st.lost = live->lost;
    st.mode = live->mode;
    st.points = (int)live->cloud.size();
    const char *mode_name = live->mode == FOX_MODE_SCAN ? "SCANNING" :
                            live->mode == FOX_MODE_PAUSE ? "PAUSED" : "STOPPED";
    snprintf(live->line1, sizeof live->line1,
             "%s   model %d pts   depth %d   z %.0f mm   %.0f ms",
             mode_name, st.points, st.valid_pixels, st.median_mm, st.match_ms);
    paint_preview(live, As, Bs, chosen.depth_m, st.tracking);
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
    if (live && live->cloud.size() >= 80) {
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
    if (write_stl(stl_path, tris) < 0) return -1;
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
    return worst < 1.2 ? 0 : 1;
}
