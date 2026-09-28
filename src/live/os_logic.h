#ifndef OS_LOGIC_H
#define OS_LOGIC_H

/* Host physics for one scan step. CubalC does not run in this loop:
 * its law leaves cameras, the sweep, and the mesh in the host program. */

typedef struct {
    float yaw;
    float carry;
    int sign;
    int agree;
} os_turn;

float os_sin(float a);
float os_cos(float a);
float os_sqrt(float x);
float os_atan2(float y, float x);
float os_fabs(float x);
int os_lround(float x);
float os_min(float a, float b);
float os_max(float a, float b);

/* dx and radius are in pixels. scanning 0 holds the yaw.
 * *fuse is 1 when the outline should be added to the solid.
 * The return is the yaw step just applied, in radians. */
float os_turn_step(os_turn *t, float dx, float radius, int scanning, int *fuse);

/* Front shell covers twice the sweep (at least 120°). A real turn
 * is |yaw| in degrees and replaces that number once it is larger. */
int os_scanned_deg(int sweep_deg, float yaw, int bins, int nmodel);

int os_logic_self_test(void);

#endif
