#ifndef LIZARD_MOLD_H
#define LIZARD_MOLD_H

/* Cloud, then a solid.
 * Points are metres, Y up. The scanner passes the object cube: OpenXR local
 * translated so the cube centre is the origin.
 * A cell is kept only after a second view from a moved scanner.
 * A cell is meshed only when a confirmed neighbour holds it up.
 * Hands are capsules. A point inside one is not stored. */

typedef struct {
    float x, y, z;
    float r, g, b;
} LzSample;

typedef struct {
    float x, y, z;
    float radius;
} LzJoint;

typedef struct LzMold LzMold;

typedef struct {
    float *vert;     /* xyz rgb, 6 floats each */
    int nvert;
    unsigned *idx;
    int nidx;
} LzMesh;

LzMold *lz_mold_create(void);
void lz_mold_free(LzMold *m);
void lz_mold_reset(LzMold *m);

int lz_hand_hits(const LzJoint *joints, int njoints, float x, float y, float z);

/* Returns how many cells became confirmed on this call. */
int lz_mold_integrate(LzMold *m, const LzSample *pts, int n,
                      float camx, float camy, float camz,
                      const LzJoint *hands, int nhands);

int lz_mold_confirmed(const LzMold *m);
int lz_mold_stored(const LzMold *m);
int lz_mold_rejected(const LzMold *m);

/* Solid skin of confirmed cells. 0 on success. out owns the buffers.
 * One glance stays empty: a cell is meshed only after the scanner moves. */
int lz_mold_mesh(const LzMold *m, LzMesh *out);

/* Skin of every stored cell, including the first glance.
 * This is the model drawn while the scan is still building.
 * It does not mark those cells confirmed. */
int lz_mold_shell(const LzMold *m, LzMesh *out);

/* Stored cell centres, xyz rgb. Returns how many were written, at most cap. */
int lz_mold_cloud(const LzMold *m, float *xyzrgb, int cap);
void lz_mesh_free(LzMesh *out);

#endif
