#ifndef OS_CLOUD_H
#define OS_CLOUD_H

#include "os_stripe.h"

/* Stored cloud for one object.
 * Shapes are grouped from the points and written into a 3D matrix.
 * That matrix locates the next view. The body uses a cell only after a
 * second view has landed on it.
 * Points live in object space: X right, Y up, Z forward, millimetres. */

typedef struct os_cloud os_cloud;

typedef struct {
    float a[3], b[3], c[3];
} os_cloud_tri;

os_cloud *os_cloud_create(void);
void os_cloud_free(os_cloud *c);
void os_cloud_reset(os_cloud *c);

/* Fold one measured view into the stored cloud.
 * pts are camera-frame samples. *yaw_io is a hint in radians and comes back
 * as the yaw the shape matrix locked. axis_x/axis_z is the turn centre.
 * Returns points added, or -1 when the stored shapes reject the view. */
int os_cloud_integrate(os_cloud *c, const os_stripe_pt *pts, int n,
                       float *yaw_io, float axis_x, float axis_z);

int os_cloud_points(const os_cloud *c);
int os_cloud_shape_count(const os_cloud *c);
int os_cloud_coverage(const os_cloud *c); /* 0..100, how solid the patches are */
int os_cloud_degrees(const os_cloud *c);

/* One triangle skin around the turn, from confirmed cells only. */
int os_cloud_body(os_cloud *c, os_cloud_tri *out, int cap);
/* Patch mesh kept for tests. The window uses os_cloud_body. */
int os_cloud_mesh(const os_cloud *c, os_cloud_tri *out, int cap);
/* One speck per point, when the solid mesh is off. */
int os_cloud_specks(const os_cloud *c, os_cloud_tri *out, int cap);

#endif
