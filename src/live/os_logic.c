#include "os_logic.h"

#include <stdint.h>
#include <stdio.h>

static const float OS_PI = 3.14159265f;
static const float OS_HPI = 1.57079633f;

float os_fabs(float x) { return x < 0.f ? -x : x; }
float os_min(float a, float b) { return a < b ? a : b; }
float os_max(float a, float b) { return a > b ? a : b; }

int os_lround(float x) {
    return (int)(x >= 0.f ? x + 0.5f : x - 0.5f);
}

static float wrap_pi(float a) {
    const float tw = 6.2831853f;
    while (a > OS_PI) a -= tw;
    while (a < -OS_PI) a += tw;
    return a;
}

float os_sin(float a) {
    a = wrap_pi(a);
    float s = 1.f;
    if (a < 0.f) {
        s = -1.f;
        a = -a;
    }
    if (a > OS_HPI) a = OS_PI - a;
    float x2 = a * a;
    float t = a;
    float sum = t;
    t *= -x2 / 6.f;
    sum += t;
    t *= -x2 / 20.f;
    sum += t;
    t *= -x2 / 42.f;
    sum += t;
    t *= -x2 / 72.f;
    sum += t;
    t *= -x2 / 110.f;
    sum += t;
    return s * sum;
}

float os_cos(float a) { return os_sin(a + OS_HPI); }

float os_sqrt(float x) {
    if (x <= 0.f) return 0.f;
    union {
        float f;
        uint32_t u;
    } v;
    v.f = x;
    v.u = (v.u >> 1) + 0x1fbb4000u;
    float g = v.f;
    g = 0.5f * (g + x / g);
    g = 0.5f * (g + x / g);
    g = 0.5f * (g + x / g);
    g = 0.5f * (g + x / g);
    return g;
}

static float atan_unit(float a) {
    float s = a * a;
    return ((0.079339f * s - 0.288679f) * s + 0.995354f) * a;
}

float os_atan2(float y, float x) {
    if (x == 0.f && y == 0.f) return 0.f;
    float ax = os_fabs(x), ay = os_fabs(y);
    float r = ax > ay ? atan_unit(ay / ax) : OS_HPI - atan_unit(ax / ay);
    if (x < 0.f) r = OS_PI - r;
    if (y < 0.f) r = -r;
    return r;
}

float os_turn_step(os_turn *t, float dx, float radius, int scanning, int *fuse) {
    if (fuse) *fuse = 0;
    if (!t) return 0.f;
    if (!scanning) {
        t->carry = 0.f;
        t->agree = 0;
        t->sign = 0;
        return 0.f;
    }
    if (radius < 20.f) radius = 20.f;
    float adx = os_fabs(dx);
    if (adx < 0.8f) {
        t->carry = 0.f;
        t->agree = 0;
        t->sign = 0;
        return 0.f;
    }
    int sgn = dx > 0.f ? 1 : -1;
    float raw = -dx / radius;
    if (raw > 0.35f) raw = 0.35f;
    if (raw < -0.35f) raw = -0.35f;
    if (sgn == t->sign) t->agree++;
    else {
        t->agree = 1;
        t->carry = 0.f;
        t->sign = sgn;
    }
    t->carry += raw;
    /* A couple of pixels is camera noise. That must not lay another shell. */
    float gate = 2.f * OS_PI / 180.f;
    int big = adx >= 6.f;
    if (!big && (t->agree < 3 || os_fabs(t->carry) < gate)) return 0.f;
    float applied = t->carry;
    if (applied > 0.35f) applied = 0.35f;
    if (applied < -0.35f) applied = -0.35f;
    t->yaw += applied;
    t->carry = 0.f;
    t->agree = 0;
    if (fuse) *fuse = 1;
    return applied;
}

int os_scanned_deg(int sweep_deg, float yaw, int bins, int nmodel) {
    int sweep = sweep_deg < 60 ? 60 : sweep_deg;
    int front = sweep * 2;
    int turned = (int)(os_fabs(yaw) * (180.f / OS_PI) + 0.5f);
    int covered = bins * 5;
    if (covered < front && nmodel > 80) covered = front;
    if (turned > covered) covered = turned;
    return covered;
}

static int near(float a, float b, float tol) { return os_fabs(a - b) <= tol; }

int os_logic_self_test(void) {
    if (!near(os_sin(0.f), 0.f, 1e-6f)) return 1;
    if (!near(os_sin(OS_PI / 6.f), 0.5f, 1e-5f)) return 2;
    if (!near(os_sin(OS_HPI), 1.f, 1e-5f)) return 3;
    if (!near(os_cos(0.f), 1.f, 1e-5f)) return 4;
    if (!near(os_sin(80.f * OS_PI / 180.f), 0.98480775f, 1e-5f)) return 5;
    if (!near(os_sqrt(3848.f), 62.03f, 0.05f)) return 6;
    if (!near(os_atan2(0.f, 1.f), 0.f, 0.02f)) return 7;
    if (!near(os_atan2(1.f, 1.f), OS_PI / 4.f, 0.02f)) return 8;
    if (!near(os_atan2(1.f, 0.f), OS_HPI, 0.02f)) return 9;
    if (!near(os_atan2(0.f, -1.f), OS_PI, 0.02f)) return 10;
    if (!near(os_atan2(-1.f, 0.f), -OS_HPI, 0.02f)) return 11;

    os_turn t;
    t.yaw = t.carry = 0.f;
    t.sign = t.agree = 0;
    int fuse = 1;
    if (os_turn_step(&t, 0.f, 124.f, 1, &fuse) != 0.f || fuse || t.yaw != 0.f) return 12;
    float step = os_turn_step(&t, 20.f, 124.f, 1, &fuse);
    if (!fuse || !near(step, -20.f / 124.f, 1e-4f) || !near(t.yaw, step, 1e-5f)) return 13;
    float held = t.yaw;
    if (os_turn_step(&t, 0.2f, 124.f, 1, &fuse) != 0.f || fuse || t.yaw != held) return 14;

    os_turn slow;
    slow.yaw = slow.carry = 0.f;
    slow.sign = slow.agree = 0;
    int fused = 0;
    for (int i = 0; i < 3; i++) {
        os_turn_step(&slow, 1.2f, 124.f, 1, &fuse);
        fused += fuse;
    }
    if (fused != 0 || slow.yaw != 0.f) return 15;
    for (int i = 0; i < 3; i++) {
        os_turn_step(&slow, 4.f, 124.f, 1, &fuse);
        fused += fuse;
    }
    if (fused != 1 || os_fabs(slow.yaw) < 2.f * OS_PI / 180.f) return 15;

    if (os_scanned_deg(80, 0.f, 0, 100) != 160) return 16;
    if (os_scanned_deg(80, 4.f, 0, 100) < 220) return 17;
    if (os_scanned_deg(40, 0.f, 0, 100) < 120) return 18;
    return 0;
}
