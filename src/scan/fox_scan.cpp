#include "fox/fox_scan.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

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

struct cloud_stats {
    int triangles = 0;
    int in_range = 0;
};

static cloud_stats match_and_write(const cv::Mat &left, const cv::Mat &right,
                                   const fox_pinhole &c1, const fox_pinhole &c2,
                                   double scale, const fox_scan_opts &opt,
                                   const char *stl_path, const char *preview) {
    cloud_stats st;
    cv::Size size(left.cols, left.rows);
    cv::Mat K1 = pinhole_k(c1, scale);
    cv::Mat K2 = pinhole_k(c2, scale);
    cv::Mat D1 = pinhole_d(c1);
    cv::Mat D2 = pinhole_d(c2);
    cv::Mat rvec(3, 1, CV_64F);
    rvec.at<double>(0) = c2.rvec[0];
    rvec.at<double>(1) = c2.rvec[1];
    rvec.at<double>(2) = c2.rvec[2];
    cv::Mat R, T(3, 1, CV_64F);
    T.at<double>(0) = c2.tvec[0];
    T.at<double>(1) = c2.tvec[1];
    T.at<double>(2) = c2.tvec[2];
    cv::Rodrigues(rvec, R);

    cv::Mat R1, R2, P1, P2, Q;
    cv::stereoRectify(K1, D1, K2, D2, size, R, T, R1, R2, P1, P2, Q,
                      cv::CALIB_ZERO_DISPARITY, 0, size);

    cv::Mat map1x, map1y, map2x, map2y;
    cv::initUndistortRectifyMap(K1, D1, R1, P1, size, CV_32FC1, map1x, map1y);
    cv::initUndistortRectifyMap(K2, D2, R2, P2, size, CV_32FC1, map2x, map2y);
    cv::Mat recL, recR;
    cv::remap(left, recL, map1x, map1y, cv::INTER_LINEAR);
    cv::remap(right, recR, map2x, map2y, cv::INTER_LINEAR);
    if (preview) cv::imwrite(preview, recL);

    /* Full-res disparity at 150 mm is about fx*baseline/150 ≈ 580 px.
     * Scale it with the image. Must be a multiple of 16. */
    int max_disp = (int)std::lround((c1.fx * scale) * std::fabs(c2.tvec[0]) / opt.min_mm);
    max_disp = (max_disp + 15) & ~15;
    if (max_disp < 64) max_disp = 64;
    if (max_disp > 640) max_disp = 640;

    int block = 7;
    auto sgbm = cv::StereoSGBM::create(0, max_disp, block);
    sgbm->setP1(8 * block * block);
    sgbm->setP2(32 * block * block);
    sgbm->setUniquenessRatio(8);
    sgbm->setSpeckleWindowSize(80);
    sgbm->setSpeckleRange(2);
    sgbm->setDisp12MaxDiff(2);
    sgbm->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);

    cv::Mat disp;
    sgbm->compute(recL, recR, disp);
    cv::Mat disp_f;
    disp.convertTo(disp_f, CV_32F, 1.0 / 16.0);

    double dmin = 0, dmax = 0;
    cv::minMaxLoc(disp_f, &dmin, &dmax);
    int disp_pos = cv::countNonZero(disp_f > 1.0f);
    fprintf(stderr, "disparity min %.1f max %.1f  pixels>1: %d / %d\n",
            dmin, dmax, disp_pos, disp_f.rows * disp_f.cols);

    cv::Mat xyz;
    cv::reprojectImageTo3D(disp_f, xyz, Q, true);
    double zmin = 1e300, zmax = -1e300;
    int finite = 0;
    for (int y = 0; y < xyz.rows; y += 8) {
        for (int x = 0; x < xyz.cols; x += 8) {
            float z = xyz.at<cv::Vec3f>(y, x)[2];
            if (!std::isfinite(z)) continue;
            finite++;
            if (z < zmin) zmin = z;
            if (z > zmax) zmax = z;
        }
    }
    fprintf(stderr, "sampled Z finite %d  range %.1f .. %.1f mm\n", finite,
            finite ? zmin : 0, finite ? zmax : 0);

    const int w = xyz.cols, h = xyz.rows;
    auto at = [&](int x, int y) -> cv::Vec3f { return xyz.at<cv::Vec3f>(y, x); };
    auto ok = [&](const cv::Vec3f &p) {
        if (!std::isfinite(p[2])) return false;
        double z = p[2];
        return z >= opt.min_mm && z <= opt.max_mm;
    };
    auto near_enough = [&](const cv::Vec3f &a, const cv::Vec3f &b) {
        double dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
        return (dx * dx + dy * dy + dz * dz) <= opt.edge_mm * opt.edge_mm;
    };

    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++)
            if (ok(at(x, y))) st.in_range++;

    struct tri { cv::Vec3f a, b, c; };
    std::vector<tri> tris;
    tris.reserve((size_t)st.in_range);
    for (int y = 0; y < h - 1; y++) {
        for (int x = 0; x < w - 1; x++) {
            cv::Vec3f p00 = at(x, y), p10 = at(x + 1, y), p01 = at(x, y + 1), p11 = at(x + 1, y + 1);
            bool a = ok(p00), b = ok(p10), c = ok(p01), d = ok(p11);
            if (a && b && c && near_enough(p00, p10) && near_enough(p00, p01) && near_enough(p10, p01))
                tris.push_back({p00, p10, p01});
            if (b && c && d && near_enough(p10, p01) && near_enough(p10, p11) && near_enough(p01, p11))
                tris.push_back({p10, p11, p01});
        }
    }
    st.triangles = (int)tris.size();

    FILE *fp = fopen(stl_path, "wb");
    if (!fp) return st;
    char header[80];
    memset(header, 0, sizeof header);
    snprintf(header, sizeof header, "fox3d stereo %d triangles", st.triangles);
    fwrite(header, 1, 80, fp);
    uint32_t n = (uint32_t)tris.size();
    fwrite(&n, 4, 1, fp);
    for (const tri &t : tris) {
        float nx, ny, nz;
        float ax = t.b[0] - t.a[0], ay = t.b[1] - t.a[1], az = t.b[2] - t.a[2];
        float bx = t.c[0] - t.a[0], by = t.c[1] - t.a[1], bz = t.c[2] - t.a[2];
        nx = ay * bz - az * by;
        ny = az * bx - ax * bz;
        nz = ax * by - ay * bx;
        float len = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (len > 1e-8f) { nx /= len; ny /= len; nz /= len; }
        else { nx = ny = 0; nz = 1; }
        float v[12] = {nx, ny, nz,
                       t.a[0], t.a[1], t.a[2],
                       t.b[0], t.b[1], t.b[2],
                       t.c[0], t.c[1], t.c[2]};
        fwrite(v, 4, 12, fp);
        uint16_t attr = 0;
        fwrite(&attr, 2, 1, fp);
    }
    fclose(fp);
    return st;
}

int fox_scan_to_stl(const uint8_t *ya, const uint8_t *yb, int width, int height,
                    const fox_calib *calib, const fox_scan_opts *opt,
                    const char *stl_path, int *triangles_out) {
    fox_scan_opts o = *opt;
    if (o.scale < 0.2) o.scale = 0.2;
    if (o.scale > 1.0) o.scale = 1.0;
    cv::Mat A(height, width, CV_8UC1, const_cast<uint8_t *>(ya));
    cv::Mat B(height, width, CV_8UC1, const_cast<uint8_t *>(yb));
    cv::Mat As, Bs;
    if (o.scale != 1.0) {
        cv::resize(A, As, cv::Size(), o.scale, o.scale, cv::INTER_AREA);
        cv::resize(B, Bs, cv::Size(), o.scale, o.scale, cv::INTER_AREA);
    } else {
        As = A;
        Bs = B;
    }

    /* Calib camera 1 is the reference. The USB names are "Camera A" (636a)
     * and "Camera B" (636b). Try both assignments; the wrong one puts the
     * cloud behind the cameras. */
    char tmp_a[512], tmp_b[512];
    snprintf(tmp_a, sizeof tmp_a, "%s.tryA", stl_path);
    snprintf(tmp_b, sizeof tmp_b, "%s.tryB", stl_path);
    cloud_stats sa = match_and_write(As, Bs, calib->cam[0], calib->cam[1], o.scale, o, tmp_a,
                                     o.preview_png);
    cloud_stats sb = match_and_write(Bs, As, calib->cam[0], calib->cam[1], o.scale, o, tmp_b, NULL);
    fprintf(stderr, "assignment A-as-left: %d points in range, %d triangles\n", sa.in_range, sa.triangles);
    fprintf(stderr, "assignment B-as-left: %d points in range, %d triangles\n", sb.in_range, sb.triangles);

    int use_a = sa.in_range >= sb.in_range;
    const char *keep = use_a ? tmp_a : tmp_b;
    const char *drop = use_a ? tmp_b : tmp_a;
    if (rename(keep, stl_path) != 0) {
        remove(tmp_a);
        remove(tmp_b);
        return -1;
    }
    remove(drop);
    if (triangles_out) *triangles_out = use_a ? sa.triangles : sb.triangles;
    return 0;
}
