#include "lizard_pose.h"

#include <math.h>

void lz_mount_default(LzMount *m) {
    /* Fox bar on the Quest 3 shell: above the eyes, just in front of the foam.
     * Pitch stays 0 here so the host identity checks do not depend on the
     * headset tilt. The Quest loader fills the measured downward pitch. */
    m->ox_m = 0.f;
    m->oy_m = 0.055f;
    m->of_m = 0.040f;
    m->pitch_rad = 0.f;
}

void lz_quat_rotate(float qx, float qy, float qz, float qw,
                    float x, float y, float z,
                    float *ox, float *oy, float *oz) {
    float tx = 2.f * (qy * z - qz * y);
    float ty = 2.f * (qz * x - qx * z);
    float tz = 2.f * (qx * y - qy * x);
    *ox = x + qw * tx + (qy * tz - qz * ty);
    *oy = y + qw * ty + (qz * tx - qx * tz);
    *oz = z + qw * tz + (qx * ty - qy * tx);
}

void lz_cam_to_world(const LzPose *head, const LzMount *mount,
                     float x_mm, float y_mm, float z_mm,
                     float *ox, float *oy, float *oz) {
    float hx, hy, hz, wx, wy, wz;
    LzMount d;
    LzPose id;
    if (!mount) {
        lz_mount_default(&d);
        mount = &d;
    }
    if (!head) {
        id.px = id.py = id.pz = 0.f;
        id.qx = id.qy = id.qz = 0.f;
        id.qw = 1.f;
        head = &id;
    }
    /* Camera +Z is forward. OpenXR forward is -Z.
     * A positive pitch dips that axis below the eye forward. */
    {
        float x = x_mm * 0.001f;
        float y = y_mm * 0.001f;
        float z = z_mm * 0.001f;
        float c = cosf(mount->pitch_rad);
        float s = sinf(mount->pitch_rad);
        hx = x + mount->ox_m;
        hy = y * c - z * s + mount->oy_m;
        hz = -y * s - z * c - mount->of_m;
    }
    lz_quat_rotate(head->qx, head->qy, head->qz, head->qw, hx, hy, hz, &wx, &wy, &wz);
    *ox = head->px + wx;
    *oy = head->py + wy;
    *oz = head->pz + wz;
}

void lz_cam_origin_world(const LzPose *head, const LzMount *mount,
                         float *ox, float *oy, float *oz) {
    lz_cam_to_world(head, mount, 0.f, 0.f, 0.f, ox, oy, oz);
}

void lz_world_to_cam(const LzPose *head, const LzMount *mount,
                     float x, float y, float z,
                     float *ox, float *oy, float *oz) {
    float cx, cy, cz, lx, ly, lz;
    LzMount d;
    LzPose id;
    if (!mount) {
        lz_mount_default(&d);
        mount = &d;
    }
    if (!head) {
        id.px = id.py = id.pz = 0.f;
        id.qx = id.qy = id.qz = 0.f;
        id.qw = 1.f;
        head = &id;
    }
    lz_cam_origin_world(head, mount, &cx, &cy, &cz);
    /* Inverse of the head rotation, then the mount pitch. */
    lz_quat_rotate(-head->qx, -head->qy, -head->qz, head->qw, x - cx, y - cy, z - cz, &lx, &ly, &lz);
    {
        float c = cosf(mount->pitch_rad);
        float s = sinf(mount->pitch_rad);
        *ox = lx;
        *oy = ly * c - lz * s;
        *oz = -ly * s - lz * c;
    }
}
