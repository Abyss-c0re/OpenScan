#ifndef LIZARD_POSE_H
#define LIZARD_POSE_H

/* Scanner bolted to the top of the headset.
 * OpenXR view space: metres, +X right, +Y up, -Z forward, origin between the eyes.
 * Camera A: millimetres, +X right, +Y up, +Z forward, the factory calibration frame.
 * ox_m / oy_m are right and up from the eyes. of_m is forward and is positive.
 * pitch_rad tilts the camera down about +X. Zero keeps the optical axis parallel
 * to the eyes. The Quest bar looks down relative to that axis. */

typedef struct {
    float px, py, pz;
    float qx, qy, qz, qw;
} LzPose;

typedef struct {
    float ox_m, oy_m, of_m;
    float pitch_rad;
} LzMount;

void lz_mount_default(LzMount *m);

/* OpenXR quaternion, same product the view matrix uses. */
void lz_quat_rotate(float qx, float qy, float qz, float qw,
                    float x, float y, float z,
                    float *ox, float *oy, float *oz);

/* Identity pose, camera-frame millimetres to OpenXR metres. */
void lz_cam_to_world(const LzPose *head, const LzMount *mount,
                     float x_mm, float y_mm, float z_mm,
                     float *ox, float *oy, float *oz);

void lz_cam_origin_world(const LzPose *head, const LzMount *mount,
                         float *ox, float *oy, float *oz);

/* World metres to the scanner frame: +X right, +Y up, +Z forward, metres. */
void lz_world_to_cam(const LzPose *head, const LzMount *mount,
                     float x, float y, float z,
                     float *ox, float *oy, float *oz);

#endif
