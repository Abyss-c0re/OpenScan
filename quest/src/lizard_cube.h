#ifndef LIZARD_CUBE_H
#define LIZARD_CUBE_H

#include "lizard_mold.h"
#include "lizard_pose.h"

/* Object frame. The cube is axis-aligned in OpenXR local space (Y up, gravity).
 * Its centre is the origin of the scan. Controllers place it. Hand joints are only
 * a mask and a readout. Half-edge is metres. Beams and samples are cube-local. */

#define LZ_CUBE_BEAM_VERTS 288
#define LZ_CUBE_CORNER_VERTS 8

typedef struct {
    float cx, cy, cz;
    float half;
    int ready;
    int placed;
    int grabbed;
    int grab_hand;
    int pinch_latched[2];
    float grab_ox, grab_oy, grab_oz;
    int had_aim;
    /* Head-still lock when no controller pose is driving the cube. */
    float still_px, still_py, still_pz;
    float still_qx, still_qy, still_qz, still_qw;
    int still_n;
    int settle_cool;
} LzCube;

/* Frames the head must hold still before a controller-less cube locks and scans. */
#define LZ_CUBE_SETTLE 80

void lz_cube_init(LzCube *c, float half_m);

/* Head-follow, pinch-grab, release-to-lock. Kept for the host test.
 * The headset build does not call this. 1 means the mold must be cleared. */
int lz_cube_step(LzCube *c, const LzPose *head, int head_ok,
                 const LzJoint *hands, int nhands);

typedef struct {
    int aim_ok;
    float ax, ay, az;
    float dx, dy, dz;
    int grip;
    int lock_toggle;
    int bigger;
    int smaller;
    int reset_scan;
} LzCubeDrive;

/* Point dist metres along the aim ray. */
void lz_cube_ray(float ax, float ay, float az, float dx, float dy, float dz, float dist, float *x, float *y,
                 float *z);

/* Grip slides the cube along the aim. Lock, resize, and reset come from the menu.
 * 1 means the mold must be cleared. */
int lz_cube_drive(LzCube *c, const LzPose *head, int head_ok, const LzCubeDrive *d);

int lz_cube_scanning(const LzCube *c);

void lz_cube_to_local(const LzCube *c, float x, float y, float z,
                      float *ox, float *oy, float *oz);
void lz_cube_to_world(const LzCube *c, float x, float y, float z,
                      float *ox, float *oy, float *oz);

/* World point. margin grows the box, metres. The face counts as inside. */
int lz_cube_contains(const LzCube *c, float x, float y, float z, float margin);

void lz_joints_into_cube(const LzCube *c, const LzJoint *in, int n, LzJoint *out);

/* Column-major translation. Cube-local (0,0,0) lands on the centre. */
void lz_cube_model(const LzCube *c, float m[16]);

/* Cube-local xyzrgb. Beams are 12 edges. Corners are the re-grab handles. */
int lz_cube_beams(const LzCube *c, float *out);
int lz_cube_corners(const LzCube *c, float *out);

void lz_cube_status(const LzCube *c, const LzJoint *hands, int nhands, char *buf, int cap);

#endif
