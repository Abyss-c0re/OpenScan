#include "lizard_color.h"

#include <math.h>

static void rodriques(const double r[3], double R[9]) {
    double th = sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
    double kx, ky, kz, c, s, C;
    if (th < 1e-12) {
        R[0] = 1; R[1] = 0; R[2] = 0;
        R[3] = 0; R[4] = 1; R[5] = 0;
        R[6] = 0; R[7] = 0; R[8] = 1;
        return;
    }
    kx = r[0] / th;
    ky = r[1] / th;
    kz = r[2] / th;
    c = cos(th);
    s = sin(th);
    C = 1.0 - c;
    R[0] = c + kx * kx * C;
    R[1] = kx * ky * C - kz * s;
    R[2] = kx * kz * C + ky * s;
    R[3] = ky * kx * C + kz * s;
    R[4] = c + ky * ky * C;
    R[5] = ky * kz * C - kx * s;
    R[6] = kz * kx * C - ky * s;
    R[7] = kz * ky * C + kx * s;
    R[8] = c + kz * kz * C;
}

static void distort_project(const openscan_pinhole *cam, double X, double Y, double Z,
                            double *u, double *v) {
    double xn = X / Z, yn = Y / Z;
    double r2 = xn * xn + yn * yn;
    double r4 = r2 * r2;
    double rad = 1.0 + cam->k1 * r2 + cam->k2 * r4 + cam->k3 * r4 * r2;
    double xd = xn * rad + 2.0 * cam->p1 * xn * yn + cam->p2 * (r2 + 2.0 * xn * xn);
    double yd = yn * rad + cam->p1 * (r2 + 2.0 * yn * yn) + 2.0 * cam->p2 * xn * yn;
    *u = cam->fx * xd + cam->cx;
    *v = cam->fy * yd + cam->cy;
}

int lz_project_b(const openscan_pinhole *cam, float x_mm, float y_up_mm, float z_mm,
                 float *u, float *v) {
    double R[9], Xc[3];
    double Xp = x_mm;
    double Yp = -(double)y_up_mm; /* camera B intrinsics are Y down, same as os_stripe */
    double Zp = z_mm;
    if (!cam || !(cam->fx > 1.0)) return 0;
    rodriques(cam->rvec, R);
    Xc[0] = R[0] * Xp + R[1] * Yp + R[2] * Zp + cam->tvec[0];
    Xc[1] = R[3] * Xp + R[4] * Yp + R[5] * Zp + cam->tvec[1];
    Xc[2] = R[6] * Xp + R[7] * Yp + R[8] * Zp + cam->tvec[2];
    if (!(Xc[2] > 1.0)) return 0;
    {
        double uu, vv;
        distort_project(cam, Xc[0], Xc[1], Xc[2], &uu, &vv);
        *u = (float)uu;
        *v = (float)vv;
    }
    return 1;
}

static float cl01(float x) {
    if (x < 0.f) return 0.f;
    if (x > 1.f) return 1.f;
    return x;
}

static int sample_plane(const uint8_t *p, int w, int h, int stride, int pix, int x, int y) {
    if (!p || w < 1 || h < 1) return 128;
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x >= w) x = w - 1;
    if (y >= h) y = h - 1;
    if (pix < 1) pix = 1;
    return p[(size_t)y * (size_t)stride + (size_t)x * (size_t)pix];
}

void lz_sample_yuv(const uint8_t *y, int w, int h, int y_stride,
                   const uint8_t *u, const uint8_t *v,
                   int uv_w, int uv_h, int u_stride, int v_stride, int uv_pixel_stride,
                   float pu, float pv, float *r, float *g, float *b) {
    int x, yy, cx, cy;
    float Y, U, V, C, D, E;
    if (!y || w < 2 || h < 2) {
        *r = *g = *b = 0.6f;
        return;
    }
    x = (int)(pu >= 0.f ? pu + 0.5f : pu - 0.5f);
    yy = (int)(pv >= 0.f ? pv + 0.5f : pv - 0.5f);
    if (y_stride < w) y_stride = w;
    Y = (float)sample_plane(y, w, h, y_stride, 1, x, yy);
    cx = x / 2;
    cy = yy / 2;
    if (uv_w < 1) uv_w = w / 2;
    if (uv_h < 1) uv_h = h / 2;
    U = (float)sample_plane(u, uv_w, uv_h, u_stride, uv_pixel_stride, cx, cy);
    V = (float)sample_plane(v, uv_w, uv_h, v_stride, uv_pixel_stride, cx, cy);
    C = Y - 16.f;
    D = U - 128.f;
    E = V - 128.f;
    *r = cl01(((298.f * C + 409.f * E + 128.f) / 256.f) / 255.f);
    *g = cl01(((298.f * C - 100.f * D - 208.f * E + 128.f) / 256.f) / 255.f);
    *b = cl01(((298.f * C + 516.f * D + 128.f) / 256.f) / 255.f);
}
