/* Host checks for the Quest mount, camera-B colour, and the solid.
 * A hand inside the volume is dropped. One glance does not mold. */
#include "lizard_color.h"
#include "lizard_cube.h"
#include "lizard_menu.h"
#include "lizard_mold.h"
#include "lizard_pose.h"
#include "lizard_settings.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int g_fail;

static void expect(int cond, const char *msg) {
    if (cond) return;
    fprintf(stderr, "fail: %s\n", msg);
    g_fail++;
}

static void test_pose(void) {
    LzMount m;
    LzPose head;
    float x, y, z;
    lz_mount_default(&m);
    memset(&head, 0, sizeof head);
    head.qw = 1.f;
    lz_cam_to_world(&head, &m, 0.f, 0.f, 220.f, &x, &y, &z);
    expect(fabsf(x) < 1e-5f, "forward point stays on x");
    expect(fabsf(y - 0.055f) < 1e-4f, "mount lifts the camera");
    expect(fabsf(z - (-0.260f)) < 1e-4f, "camera +Z is OpenXR -Z plus the forward mount");
    /* +90° about Y. */
    head.qy = 0.70710678f;
    head.qw = 0.70710678f;
    lz_cam_to_world(&head, &m, 0.f, 0.f, 220.f, &x, &y, &z);
    expect(fabsf(x - (-0.260f)) < 0.01f, "yaw carries forward onto -X");
    expect(fabsf(y - 0.055f) < 0.01f, "yaw keeps height");
    expect(fabsf(z) < 0.02f, "yaw leaves little Z");

    memset(&head, 0, sizeof head);
    head.qw = 1.f;
    lz_cam_origin_world(&head, &m, &x, &y, &z);
    lz_world_to_cam(&head, &m, x + 0.01f, y + 0.02f, z - 0.03f, &x, &y, &z);
    expect(fabsf(x - 0.01f) < 1e-4f && fabsf(y - 0.02f) < 1e-4f && fabsf(z - 0.03f) < 1e-4f,
           "a point in front of the scanner is +Z in the camera");
    lz_cam_to_world(&head, &m, 10.f, 20.f, 30.f, &x, &y, &z);
    lz_world_to_cam(&head, &m, x, y, z, &x, &y, &z);
    expect(fabsf(x - 0.01f) < 1e-4f && fabsf(y - 0.02f) < 1e-4f && fabsf(z - 0.03f) < 1e-4f,
           "world and camera frames round trip");
    head.qy = 0.70710678f;
    head.qw = 0.70710678f;
    lz_cam_to_world(&head, &m, 0.f, 0.f, 220.f, &x, &y, &z);
    lz_world_to_cam(&head, &m, x, y, z, &x, &y, &z);
    expect(fabsf(x) < 1e-3f && fabsf(y) < 1e-3f && fabsf(z - 0.220f) < 1e-3f,
           "yaw round trip stays on the camera axis");

    /* 10° down. A point on the optical axis drops, and still round-trips. */
    memset(&head, 0, sizeof head);
    head.qw = 1.f;
    m.pitch_rad = 10.f * 0.017453292f;
    lz_cam_to_world(&head, &m, 0.f, 0.f, 220.f, &x, &y, &z);
    expect(y < 0.03f, "downward pitch lowers the forward point");
    expect(z > -0.260f && z < -0.20f, "downward pitch keeps the point in front");
    lz_world_to_cam(&head, &m, x, y, z, &x, &y, &z);
    expect(fabsf(x) < 1e-3f && fabsf(y) < 1e-3f && fabsf(z - 0.220f) < 1e-3f,
           "pitched point round-trips to the camera axis");
}

static void test_color(void) {
    openscan_pinhole cam;
    float u, v;
    uint8_t y[16], uu[4], vv[4];
    float r, g, b;
    int i;
    memset(&cam, 0, sizeof cam);
    cam.fx = 2742.509033;
    cam.fy = 2743.267090;
    cam.cx = 616.671021;
    cam.cy = 357.606995;
    cam.k1 = -0.591162;
    cam.k2 = 0.242073;
    cam.p1 = 0.000849;
    cam.p2 = -0.000573;
    cam.k3 = 1.334951;
    cam.rvec[0] = 0.004269;
    cam.rvec[1] = -0.117442;
    cam.rvec[2] = 0.000712;
    cam.tvec[0] = 32.207294;
    cam.tvec[1] = -0.052929;
    cam.tvec[2] = -3.550440;
    expect(lz_project_b(&cam, 0.f, 0.f, 220.f, &u, &v) == 1, "axis point projects");
    expect(u > 680.f && u < 720.f, "baseline and yaw land near u=700");
    expect(v > 340.f && v < 380.f, "v stays near the principal point");
    expect(lz_project_b(&cam, 0.f, 0.f, -10.f, &u, &v) == 0, "behind the camera is dropped");
    for (i = 0; i < 16; i++) y[i] = 200;
    for (i = 0; i < 4; i++) {
        uu[i] = 128;
        vv[i] = 128;
    }
    lz_sample_yuv(y, 4, 4, 4, uu, vv, 2, 2, 2, 2, 1, 1.f, 1.f, &r, &g, &b);
    expect(r > 0.6f && g > 0.6f && b > 0.6f, "neutral chroma stays bright");
    expect(fabsf(r - g) < 0.08f && fabsf(g - b) < 0.08f, "mono lattice camera stays gray");
}

static void test_mold(void) {
    LzMold *m = lz_mold_create();
    LzSample a[2], b[2];
    LzJoint hand;
    LzMesh mesh;
    LzSample blocked;
    expect(m != NULL, "mold allocates");
    a[0].x = 0.f; a[0].y = 1.f; a[0].z = -0.4f; a[0].r = 1.f; a[0].g = 0.f; a[0].b = 0.f;
    a[1].x = 0.003f; a[1].y = 1.f; a[1].z = -0.4f; a[1].r = 0.f; a[1].g = 1.f; a[1].b = 0.f;
    b[0] = a[0];
    b[1] = a[1];
    expect(lz_mold_integrate(m, a, 2, 0.f, 1.5f, 0.f, NULL, 0) == 0, "first glance does not confirm");
    expect(lz_mold_mesh(m, &mesh) == 0, "mesh call");
    expect(mesh.nidx == 0, "one glance has no solid");
    lz_mesh_free(&mesh);
    expect(lz_mold_shell(m, &mesh) == 0, "shell call");
    expect(mesh.nidx >= 6, "stored neighbours are drawn before the second view");
    lz_mesh_free(&mesh);
    {
        float xyz[12];
        expect(lz_mold_cloud(m, xyz, 2) == 2, "stored cells stay as a cloud");
    }
    expect(lz_mold_integrate(m, b, 2, 0.05f, 1.5f, 0.f, NULL, 0) == 2, "a moved scanner confirms both cells");
    expect(lz_mold_mesh(m, &mesh) == 0, "solid mesh");
    expect(mesh.nidx >= 6, "neighbours become a skin");
    lz_mesh_free(&mesh);
    hand.x = 0.2f;
    hand.y = 1.f;
    hand.z = -0.4f;
    hand.radius = 0.03f;
    blocked.x = 0.20f;
    blocked.y = 1.f;
    blocked.z = -0.40f;
    blocked.r = blocked.g = blocked.b = 1.f;
    {
        int before = lz_mold_stored(m);
        lz_mold_integrate(m, &blocked, 1, 0.f, 1.5f, -0.2f, &hand, 1);
        lz_mold_integrate(m, &blocked, 1, 0.08f, 1.5f, -0.2f, &hand, 1);
        expect(lz_mold_stored(m) == before, "a point inside the hand is not stored");
        expect(lz_mold_rejected(m) >= 2, "hand hits are counted");
    }
    expect(lz_hand_hits(&hand, 1, 0.2f, 1.f, -0.4f) == 1, "joint sphere hits");
    expect(lz_hand_hits(&hand, 1, 0.2f, 1.f, -0.2f) == 0, "a point clear of the hand stays");
    lz_mold_free(m);
}

static void tip(LzJoint *j, float x, float y, float z) {
    j->x = x;
    j->y = y;
    j->z = z;
    j->radius = 0.012f;
}

static void test_cube(void) {
    LzCube c;
    LzPose head;
    LzJoint hands[52], local;
    float x, y, z, ox, oy, oz, m[16];
    float beams[LZ_CUBE_BEAM_VERTS * 6];
    float corners[LZ_CUBE_CORNER_VERTS * 6];
    int i, n, reset;
    char status[96];
    memset(&head, 0, sizeof head);
    memset(hands, 0, sizeof hands);
    head.qw = 1.f;
    lz_cube_init(&c, 0.12f);
    expect(fabsf(c.half - 0.12f) < 1e-6f, "half edge stays 0.12 m");
    expect(lz_cube_step(&c, &head, 1, hands, 52) == 0, "follow does not clear the mold");
    expect(c.ready && !c.placed && !c.grabbed, "cube follows before a pinch");
    expect(lz_cube_scanning(&c) == 0, "a following cube does not scan");
    /* Near face is 0.20 m ahead: centre = 0.20 + half, 0.03 m below the eyes. */
    expect(fabsf(c.cx) < 1e-4f, "identity follow stays on x");
    expect(fabsf(c.cy - (-0.03f)) < 1e-4f, "identity follow drops 3 cm");
    expect(fabsf(c.cz - (-0.32f)) < 1e-4f, "identity follow is 0.32 m along -Z");
    expect(lz_cube_contains(&c, c.cx, c.cy, c.cz, 0.f) == 1, "centre is inside");
    expect(lz_cube_contains(&c, c.cx + 0.12f, c.cy, c.cz, 0.f) == 1, "the face counts as inside");
    expect(lz_cube_contains(&c, c.cx + 0.13f, c.cy, c.cz, 0.f) == 0, "past the face is outside");
    lz_cube_to_local(&c, c.cx + 0.05f, c.cy - 0.02f, c.cz + 0.01f, &x, &y, &z);
    expect(fabsf(x - 0.05f) < 1e-5f && fabsf(y - (-0.02f)) < 1e-5f && fabsf(z - 0.01f) < 1e-5f,
           "world point becomes cube-local");
    lz_cube_to_world(&c, x, y, z, &ox, &oy, &oz);
    expect(fabsf(ox - (c.cx + 0.05f)) < 1e-5f && fabsf(oy - (c.cy - 0.02f)) < 1e-5f &&
               fabsf(oz - (c.cz + 0.01f)) < 1e-5f,
           "cube-local point returns to world");

    head.px = 0.5f;
    head.qy = 0.70710678f;
    head.qw = 0.70710678f;
    lz_cube_step(&c, &head, 1, hands, 52);
    expect(fabsf(c.cx - 0.18f) < 0.01f, "yaw carries the cube onto -X");
    expect(fabsf(c.cy - (-0.03f)) < 0.01f, "yaw keeps the drop");
    expect(fabsf(c.cz) < 0.02f, "yaw leaves little Z");

    memset(&head, 0, sizeof head);
    head.qw = 1.f;
    lz_cube_init(&c, 0.12f);
    lz_cube_step(&c, &head, 1, hands, 52);
    tip(&hands[31], c.cx - 0.005f, c.cy, c.cz);
    tip(&hands[36], c.cx + 0.005f, c.cy, c.cz);
    reset = lz_cube_step(&c, &head, 1, hands, 52);
    expect(reset == 1 && c.grabbed && !c.placed, "a pinch inside the cube grabs it");
    expect(lz_cube_scanning(&c) == 0, "a grabbed cube does not scan");
    ox = c.cx;
    oy = c.cy;
    oz = c.cz;
    hands[31].x += 0.05f;
    hands[36].x += 0.05f;
    expect(lz_cube_step(&c, &head, 1, hands, 52) == 0, "holding the pinch is not a new lock");
    expect(fabsf(c.cx - (ox + 0.05f)) < 1e-4f && fabsf(c.cy - oy) < 1e-4f && fabsf(c.cz - oz) < 1e-4f,
           "the cube tracks the pinch");
    /* 30 mm stays inside the release gap. */
    tip(&hands[31], c.cx - 0.10f, c.cy, c.cz);
    tip(&hands[36], c.cx - 0.07f, c.cy, c.cz);
    expect(c.grabbed, "precondition");
    expect(lz_cube_step(&c, &head, 1, hands, 52) == 0 && c.grabbed, "30 mm still counts as a pinch");
    tip(&hands[31], c.cx - 0.08f, c.cy, c.cz);
    tip(&hands[36], c.cx + 0.08f, c.cy, c.cz);
    expect(lz_cube_step(&c, &head, 1, hands, 52) == 1 && c.placed && !c.grabbed, "opening the pinch locks the cube");
    expect(lz_cube_scanning(&c) == 1, "a locked cube is the scan frame");
    ox = c.cx;
    oy = c.cy;
    oz = c.cz;
    head.px = 1.f;
    lz_cube_step(&c, &head, 1, hands, 52);
    expect(fabsf(c.cx - ox) < 1e-5f && fabsf(c.cy - oy) < 1e-5f && fabsf(c.cz - oz) < 1e-5f,
           "a locked cube stays in the room");

    tip(&hands[31], c.cx, c.cy, c.cz);
    tip(&hands[36], c.cx + 0.01f, c.cy, c.cz);
    expect(lz_cube_step(&c, &head, 1, hands, 52) == 0 && c.placed && !c.grabbed,
           "a pinch in the middle does not steal a locked cube");
    tip(&hands[5], c.cx + c.half, c.cy + c.half, c.cz + c.half - 0.005f);
    tip(&hands[10], c.cx + c.half, c.cy + c.half, c.cz + c.half + 0.005f);
    ox = c.cx;
    oy = c.cy;
    oz = c.cz;
    expect(lz_cube_step(&c, &head, 1, hands, 52) == 1 && c.grabbed, "a corner pinch picks the cube up again");
    expect(fabsf(c.cx - ox) < 1e-3f && fabsf(c.cy - oy) < 1e-3f && fabsf(c.cz - oz) < 1e-3f,
           "a corner grab does not jump");

    lz_cube_init(&c, 0.12f);
    c.ready = 1;
    c.placed = 1;
    c.cx = 1.f;
    c.cy = 2.f;
    c.cz = -3.f;
    memset(&local, 0, sizeof local);
    local.x = 1.03f;
    local.y = 1.98f;
    local.z = -2.99f;
    local.radius = 0.012f;
    lz_joints_into_cube(&c, &local, 1, hands);
    expect(fabsf(hands[0].x - 0.03f) < 1e-5f && fabsf(hands[0].y - (-0.02f)) < 1e-5f &&
               fabsf(hands[0].z - 0.01f) < 1e-5f,
           "hand joint is stored relative to the object");
    expect(fabsf(hands[0].radius - 0.012f) < 1e-6f, "hand radius is unchanged by the cube frame");
    expect(lz_hand_hits(hands, 1, 0.03f, -0.02f, 0.01f) == 1, "the hand still covers its joint in object space");
    expect(lz_hand_hits(hands, 1, 0.20f, 0.f, 0.f) == 0, "object space away from the hand is kept");
    lz_cube_model(&c, m);
    x = m[0] * 0.10f + m[4] * 0.f + m[8] * 0.f + m[12];
    y = m[1] * 0.10f + m[5] * 0.f + m[9] * 0.f + m[13];
    z = m[2] * 0.10f + m[6] * 0.f + m[10] * 0.f + m[14];
    expect(fabsf(x - 1.10f) < 1e-5f && fabsf(y - 2.f) < 1e-5f && fabsf(z - (-3.f)) < 1e-5f,
           "model matrix puts cube-local +X on the world centre");
    c.grabbed = 0;
    c.placed = 1;
    n = lz_cube_beams(&c, beams);
    expect(n == LZ_CUBE_BEAM_VERTS, "twelve edges become beams");
    for (i = 0; i < n; i++) {
        float px = beams[i * 6], py = beams[i * 6 + 1], pz = beams[i * 6 + 2];
        expect(fabsf(px) <= 0.12f + 0.006f && fabsf(py) <= 0.12f + 0.006f && fabsf(pz) <= 0.12f + 0.006f,
               "beams stay cube-local when the cube is away from the origin");
    }
    expect(beams[3] < 0.4f && beams[5] > 0.9f, "a locked cube is the blue frame");
    expect(lz_cube_corners(&c, corners) == 8, "eight corner handles");
    expect(fabsf(fabsf(corners[0]) - 0.12f) < 1e-5f, "a corner sits on the half edge");
    lz_cube_status(&c, &local, 1, status, (int)sizeof status);
    expect(strncmp(status, "cube L 1.00,2.00,-3.00 hL 0.03,-0.02,0.01", 40) == 0,
           "status names the lock and the hand in the object frame");
}

static void test_drive(void) {
    LzCube c;
    LzCubeDrive d;
    LzPose head;
    float x, y, z, ox, oy, oz, half;
    memset(&head, 0, sizeof head);
    head.qw = 1.f;
    lz_cube_init(&c, 0.12f);
    memset(&d, 0, sizeof d);
    expect(lz_cube_drive(&c, &head, 1, &d) == 0, "head follow does not clear the mold");
    expect(c.ready && !c.placed && !c.had_aim, "the cube follows until a controller is seen");
    expect(fabsf(c.cx) < 1e-4f && fabsf(c.cy + 0.03f) < 1e-4f && fabsf(c.cz + 0.32f) < 1e-4f,
           "controller-less follow matches the head pose");

    d.aim_ok = 1;
    d.dz = -1.f;
    expect(lz_cube_drive(&c, &head, 1, &d) == 0, "aiming places without locking");
    expect(c.had_aim && !c.placed && !c.grabbed, "an aim is remembered");
    expect(fabsf(c.cx) < 1e-4f && fabsf(c.cy) < 1e-4f && fabsf(c.cz + 0.40f) < 1e-4f,
           "the cube sits 0.40 m along the aim");
    d.ax = 0.10f;
    expect(lz_cube_drive(&c, &head, 1, &d) == 0, "aim follow keeps moving");
    expect(fabsf(c.cx - 0.10f) < 1e-4f, "the cube tracks the aim point");
    ox = c.cx;
    oy = c.cy;
    oz = c.cz;
    d.grip = 1;
    expect(lz_cube_drive(&c, &head, 1, &d) == 0 && c.grabbed && !c.placed, "grip picks the cube up");
    expect(fabsf(c.cx - ox) < 1e-4f && fabsf(c.cy - oy) < 1e-4f && fabsf(c.cz - oz) < 1e-4f,
           "grip does not jump");
    d.ax = 0.15f;
    expect(lz_cube_drive(&c, &head, 1, &d) == 0, "holding grip is not a lock");
    expect(fabsf(c.cx - (ox + 0.05f)) < 1e-4f, "grip keeps the grab offset");
    d.grip = 0;
    expect(lz_cube_drive(&c, &head, 1, &d) == 1 && c.placed && !c.grabbed, "releasing grip locks and clears");
    expect(lz_cube_scanning(&c) == 1, "a controller lock scans");
    ox = c.cx;
    d.ax = 0.40f;
    expect(lz_cube_drive(&c, &head, 1, &d) == 0, "a locked cube ignores the aim");
    expect(fabsf(c.cx - ox) < 1e-5f, "a locked cube stays put");
    half = c.half;
    d.bigger = 1;
    expect(lz_cube_drive(&c, &head, 1, &d) == 0, "bigger does not clear the scan");
    expect(fabsf(c.half - (half + 0.02f)) < 1e-5f, "bigger grows the edge");
    d.bigger = 0;
    d.smaller = 1;
    lz_cube_drive(&c, &head, 1, &d);
    expect(fabsf(c.half - half) < 1e-5f, "smaller shrinks the edge");
    d.smaller = 0;
    d.lock_toggle = 1;
    expect(lz_cube_drive(&c, &head, 1, &d) == 0 && !c.placed, "unlock returns to the aim");
    expect(fabsf(c.cx - 0.40f) < 1e-4f, "unlock snaps onto the aim");
    d.lock_toggle = 0;
    d.lock_toggle = 1;
    expect(lz_cube_drive(&c, &head, 1, &d) == 1 && c.placed, "lock from the menu clears the mold");
    d.lock_toggle = 0;
    ox = c.cx;
    d.reset_scan = 1;
    expect(lz_cube_drive(&c, &head, 1, &d) == 1 && fabsf(c.cx - ox) < 1e-6f, "reset clears without moving");
    d.reset_scan = 0;
    d.grip = 1;
    expect(lz_cube_drive(&c, &head, 1, &d) == 0 && c.grabbed, "grip lifts a locked cube");
    d.aim_ok = 0;
    expect(lz_cube_drive(&c, &head, 1, &d) == 1 && c.placed && !c.grabbed, "losing the aim locks the grab");

    lz_cube_init(&c, 0.12f);
    memset(&d, 0, sizeof d);
    d.aim_ok = 1;
    d.dz = -1.f;
    lz_cube_drive(&c, &head, 1, &d);
    d.aim_ok = 0;
    lz_cube_drive(&c, &head, 1, &d);
    expect(c.had_aim && !c.placed, "a dropped controller does not place the cube");
    expect(fabsf(c.cz + 0.40f) < 1e-3f, "a dropped controller does not snap back to the head");
    lz_cube_ray(0.f, 0.f, 0.f, 0.f, 0.f, -2.f, 0.40f, &x, &y, &z);
    expect(fabsf(x) < 1e-5f && fabsf(y) < 1e-5f && fabsf(z + 0.40f) < 1e-5f, "the aim distance is normalised");
    c.half = 0.24f;
    memset(&d, 0, sizeof d);
    d.bigger = 1;
    lz_cube_drive(&c, &head, 0, &d);
    lz_cube_drive(&c, &head, 0, &d);
    expect(fabsf(c.half - 0.25f) < 1e-5f, "the edge stops at 0.50 m");

    lz_cube_init(&c, 0.12f);
    memset(&d, 0, sizeof d);
    memset(&head, 0, sizeof head);
    head.qw = 1.f;
    {
        int i, early = 0;
        for (i = 0; i < LZ_CUBE_SETTLE - 1; i++)
            if (lz_cube_drive(&c, &head, 1, &d) != 0 || c.placed) early = 1;
        expect(!early, "a still head waits out the settle before it locks");
    }
    expect(lz_cube_drive(&c, &head, 1, &d) == 1 && c.placed && lz_cube_scanning(&c),
           "a still head locks the cube and starts the scan");
    d.lock_toggle = 1;
    expect(lz_cube_drive(&c, &head, 1, &d) == 0 && !c.placed, "unlock leaves the cube free");
    d.lock_toggle = 0;
    expect(lz_cube_drive(&c, &head, 1, &d) == 0 && !c.placed, "unlock does not lock again on the next frame");
}

static void test_menu(void) {
    LzMenuView v;
    LzPanel p;
    LzPose head;
    float t = -1.f;
    float y, q[30], ray[LZ_MENU_RAY_VERTS * 6];
    int i, n, ink = 0, near0 = 0, near1 = 0;
    uint8_t *rgba;
    memset(&head, 0, sizeof head);
    head.qw = 1.f;
    memset(&v, 0, sizeof v);
    lz_menu_view(&v, 0, 0, 0, 240, 0, 0, 0, 0, 0, 0.f, 0.f, 0.f);
    expect(strcmp(v.line[0], "WAIT") == 0, "no controller waits");
    expect(strstr(v.line[2], "NO PATTERN") != NULL, "no lattice is said in words");
    expect(strstr(v.line[1], "EDGE") != NULL && strstr(v.line[1], "240") != NULL, "the cube edge is on the panel");
    expect(strcmp(v.line[6], "BIGGER") == 0 && strcmp(v.line[7], "SMALLER") == 0, "the cube tab resizes");
    expect(strstr(v.line[4], "[CUBE]") != NULL && strstr(v.line[4], "SET") != NULL, "cube and settings tabs are shown");
    expect(strstr(v.line[8], "RESET") != NULL && strstr(v.line[8], "EXPORT") != NULL, "reset and export share the last row");
    lz_menu_view(&v, 1, 1, 0, 240, 3, 9, 0, 0, 2, 0.01f, -0.02f, 0.03f);
    expect(strcmp(v.line[0], "LOCKED") == 0, "a placed cube reads as locked");
    expect(strstr(v.line[3], "HR") != NULL, "the right hand is named");
    expect(strstr(v.line[3], "+1") != NULL && strstr(v.line[3], "-2") != NULL && strstr(v.line[3], "+3") != NULL,
           "the hand is centimetres in the camera frame");
    expect(strstr(v.line[5], "UNLOCK") != NULL, "a locked cube offers unlock");
    lz_menu_view(&v, 1, 0, 1, 200, 0, 0, 4, 0, 1, 0.f, 0.f, 0.f);
    expect(strcmp(v.line[0], "GRAB") == 0, "grip reads as grab");
    expect(strstr(v.line[2], "COLS 4") != NULL, "a lattice count is shown");
    expect(strstr(v.line[5], "LOCK CUBE") != NULL, "an open cube offers lock");

    lz_menu_panel_at(&p, 0.f, 0.f, 0.f, &head);
    expect(fabsf(p.cx) < 1e-4f && fabsf(p.cy) < 1e-4f && fabsf(p.cz + 0.42f) < 1e-3f,
           "a head anchor puts the menu 0.42 m forward");
    expect(fabsf(p.rx - LZ_MENU_W * 0.5f) < 1e-4f && fabsf(p.ry) < 1e-4f, "panel right is +X");
    expect(fabsf(p.uy - LZ_MENU_H * 0.5f) < 1e-4f, "panel up is +Y");
    lz_menu_panel_at(&p, 0.f, 0.f, -0.50f, &head);
    y = lz_menu_line_y(5);
    expect(lz_menu_hit(&p, 0.f, y, 0.20f, 0.f, 0.f, -1.f, &t) == LZ_BTN_LOCK, "the aim hits lock");
    expect(fabsf(t - 0.70f) < 0.02f, "the hit distance reaches the panel");
    expect(lz_menu_hit(&p, 0.f, lz_menu_line_y(6), 0.20f, 0.f, 0.f, -1.f, &t) == LZ_BTN_BIGGER,
           "bigger is the next row");
    expect(lz_menu_hit(&p, 0.f, y, -1.f, 0.f, 0.f, 1.f, &t) == LZ_BTN_NONE, "the back of the menu misses");
    expect(lz_menu_hit(&p, 0.f, lz_menu_line_y(0), 0.20f, 0.f, 0.f, -1.f, &t) == LZ_BTN_NONE,
           "the title is not a button");
    lz_menu_quad(&p, q);
    expect(fabsf(q[0] + LZ_MENU_W * 0.5f) < 1e-4f && fabsf(q[1] + LZ_MENU_H * 0.5f) < 1e-4f &&
               fabsf(q[2] + 0.50f) < 1e-4f && q[3] == 0.f && q[4] == 0.f,
           "the menu quad starts at the bottom left");
    n = lz_menu_ray(0.f, 0.f, 0.f, 0.f, 0.f, -1.f, 0.40f, 1, ray);
    expect(n > 0 && n <= LZ_MENU_RAY_VERTS, "the ray fits the buffer");
    for (i = 0; i < n; i++) {
        float px = ray[i * 6], py = ray[i * 6 + 1], pz = ray[i * 6 + 2];
        if (px * px + py * py + pz * pz < 0.0004f) near0 = 1;
        px -= 0.f;
        py -= 0.f;
        pz += 0.40f;
        if (px * px + py * py + pz * pz < 0.0004f) near1 = 1;
    }
    expect(near0 && near1, "the ray runs from the aim to its end");
    expect(ray[3] > 0.9f && ray[5] < 0.4f, "a hot ray is yellow");

    v.hot = LZ_BTN_LOCK;
    rgba = (uint8_t *)malloc((size_t)LZ_MENU_TEX_W * LZ_MENU_TEX_H * 4);
    expect(rgba != NULL, "menu raster buffer");
    if (rgba) {
        lz_menu_raster(&v, rgba);
        for (i = 0; i < LZ_MENU_TEX_W * 18 * 4; i += 4)
            if (rgba[i] > 200) ink++;
        expect(ink > 20, "the menu raster draws letters");
        free(rgba);
    }
}

static void test_settings(void) {
    LzSettings s;
    LzMenuView v;
    LzPanel p;
    LzPose head;
    float t = -1.f;
    int side = 0, line = -1, page = 0;
    char path[] = "/tmp/lizard-settings-selftest.txt";
    lz_settings_default(&s);
    expect(s.ea == 22 && s.ga == 6 && s.eb == 16 && s.gb == 4, "exposure defaults match the desktop");
    expect(s.distance == 220 && s.near_mm == 80 && s.far_mm == 550, "distance near and far match the desktop");
    expect(s.stride == 1 && s.smooth == 5 && s.sweep == 80 && s.relief == 40, "stride smooth sweep and relief match");
    expect(s.solid == 1 && s.flip == 1, "solid and flip start on");

    s.ea = 0;
    s.eb = 900;
    s.ga = -4;
    s.gb = 140;
    s.distance = 40;
    s.near_mm = 10;
    s.far_mm = 50;
    s.stride = 0;
    s.smooth = 12;
    s.sweep = 20;
    s.relief = 150;
    lz_settings_clamp(&s);
    expect(s.ea == 1 && s.eb == 200 && s.ga == 0 && s.gb == 100, "exposure and gain stay on the sliders");
    expect(s.distance == 100, "distance stays on 100..500");
    expect(s.near_mm == 60 && s.far_mm == 100, "far stays 40 mm past near");
    expect(s.stride == 1 && s.smooth == 8 && s.relief == 100, "stride smooth and relief clamp");
    expect(s.sweep == 60, "sweep stays on the engine floor of 60");

    s.near_mm = 500;
    s.far_mm = 900;
    lz_settings_clamp(&s);
    expect(s.near_mm == 450 && s.far_mm == 800, "near stops at 450 and far at 800");
    s.near_mm = 200;
    s.far_mm = 230;
    expect(lz_settings_nudge(&s, LZ_SET_FAR, -1) == 1, "far accepts a step");
    expect(s.far_mm == 240, "far cannot enter near");
    s.ea = 22;
    expect(lz_settings_nudge(&s, LZ_SET_EA, 1) == 1 && s.ea == 23, "exposure steps by one");
    expect(lz_settings_nudge(&s, LZ_SET_EA, 0) == 0 && s.ea == 23, "a zero step does nothing");
    lz_settings_default(&s);
    expect(s.ea == 22 && s.far_mm == 550 && s.stride == 1 && s.sweep == 80, "defaults restore the desktop table");

    s.ea = 30;
    s.sweep = 10;
    s.flip = 0;
    expect(lz_settings_save(path, &s) == 0, "settings save");
    lz_settings_default(&s);
    expect(lz_settings_load(path, &s) == 0, "settings load");
    expect(s.ea == 30 && s.flip == 0 && s.sweep == 60, "a saved sweep below 60 becomes the engine floor");
    s.ea = 41;
    expect(lz_settings_load("/tmp/lizard-settings-missing.txt", &s) != 0 && s.ea == 41, "a missing file keeps the value");
    remove(path);

    memset(&head, 0, sizeof head);
    head.qw = 1.f;
    lz_menu_panel_at(&p, 0.f, 0.f, -0.50f, &head);
    expect(lz_menu_pick(&p, 0, -0.10f, lz_menu_line_y(4), 0.20f, 0.f, 0.f, -1.f, &side, &line, &t) == LZ_BTN_TAB_CUBE,
           "the left of the tab row is the cube");
    expect(line == 4, "the cube tab reports its row");
    expect(lz_menu_pick(&p, 0, 0.06f, lz_menu_line_y(4), 0.20f, 0.f, 0.f, -1.f, &side, &line, &t) == LZ_BTN_TAB_SET,
           "the right of the tab row opens settings");
    expect(lz_menu_pick(&p, 0, 0.f, lz_menu_line_y(6), 0.20f, 0.f, 0.f, -1.f, &side, &line, &t) == LZ_BTN_BIGGER,
           "bigger is still the resize row");
    expect(lz_menu_pick(&p, 0, 0.f, lz_menu_line_y(7), 0.20f, 0.f, 0.f, -1.f, &side, &line, &t) == LZ_BTN_SMALLER,
           "smaller is still the next resize row");
    expect(lz_menu_pick(&p, 0, -0.06f, lz_menu_line_y(8), 0.20f, 0.f, 0.f, -1.f, &side, &line, &t) == LZ_BTN_RESET,
           "the left of the last row resets the scan");
    expect(lz_menu_pick(&p, 0, 0.06f, lz_menu_line_y(8), 0.20f, 0.f, 0.f, -1.f, &side, &line, &t) == LZ_BTN_EXPORT,
           "the right of the last row exports the model");
    expect(lz_menu_pick(&p, 0, 0.f, lz_menu_line_y(6), -1.f, 0.f, 0.f, 1.f, &side, &line, &t) == LZ_BTN_NONE,
           "the back of a settings pick misses");

    lz_settings_default(&s);
    memset(&v, 0, sizeof v);
    lz_menu_settings(&v, 1, &s);
    expect(v.line[1][2] == '-' && v.line[1][22] == '+', "minus and plus sit in the gutters");
    expect(strstr(v.line[1], "A EXP 22") != NULL, "camera A exposure is listed");
    expect(strstr(v.line[2], "A GAIN 6") != NULL, "camera A gain is listed");
    expect(strstr(v.line[3], "B EXP 16") != NULL, "camera B exposure is listed");
    expect(strstr(v.line[4], "B GAIN 4") != NULL, "camera B gain is listed");
    expect(strstr(v.line[5], "DIST 220") != NULL, "distance is listed");
    expect(strstr(v.line[6], "NEAR 80") != NULL, "near is listed");
    expect(strstr(v.line[7], "FAR 550") != NULL, "far is listed");
    expect(strcmp(v.line[8], "NEXT") == 0, "the first settings page continues");
    expect(lz_menu_pick(&p, 1, -0.102f, lz_menu_line_y(1), 0.20f, 0.f, 0.f, -1.f, &side, &line, &t) ==
                   LZ_BTN_FIELD + LZ_SET_EA &&
               side == -1,
           "the minus glyph lowers exposure");
    expect(lz_menu_pick(&p, 1, 0.098f, lz_menu_line_y(1), 0.20f, 0.f, 0.f, -1.f, &side, &line, &t) ==
                   LZ_BTN_FIELD + LZ_SET_EA &&
               side == 1,
           "the plus glyph raises exposure");
    expect(lz_menu_pick(&p, 1, 0.f, lz_menu_line_y(1), 0.20f, 0.f, 0.f, -1.f, &side, &line, &t) ==
                   LZ_BTN_FIELD + LZ_SET_EA &&
               side == 0,
           "the middle of a value does not pick a side");
    expect(lz_menu_pick(&p, 1, 0.f, lz_menu_line_y(8), 0.20f, 0.f, 0.f, -1.f, &side, &line, &t) == LZ_BTN_NEXT,
           "next is the last row of the first page");

    lz_menu_settings(&v, 2, &s);
    expect(strstr(v.line[1], "STRIDE 1") != NULL, "stride is listed");
    expect(strstr(v.line[2], "SMOOTH 5") != NULL, "smooth is listed");
    expect(strstr(v.line[3], "SWEEP 80") != NULL, "mold sweep is listed");
    expect(strstr(v.line[4], "RELIEF 40") != NULL, "mold relief is listed");
    expect(strcmp(v.line[5], "SOLID ON") == 0, "solid mesh is listed");
    expect(strcmp(v.line[6], "FLIP ON") == 0, "flip scanner is listed");
    expect(strcmp(v.line[7], "DEFAULTS") == 0, "defaults is listed");
    expect(strcmp(v.line[8], "BACK") == 0, "the second settings page goes back");
    expect(lz_menu_pick(&p, 2, 0.f, lz_menu_line_y(5), 0.20f, 0.f, 0.f, -1.f, &side, &line, &t) == LZ_BTN_SOLID,
           "solid is its own row");
    expect(lz_menu_pick(&p, 2, 0.f, lz_menu_line_y(6), 0.20f, 0.f, 0.f, -1.f, &side, &line, &t) == LZ_BTN_FLIP,
           "flip is its own row");
    expect(lz_menu_pick(&p, 2, 0.f, lz_menu_line_y(7), 0.20f, 0.f, 0.f, -1.f, &side, &line, &t) == LZ_BTN_DEFAULTS,
           "defaults is its own row");
    expect(lz_menu_pick(&p, 2, 0.098f, lz_menu_line_y(3), 0.20f, 0.f, 0.f, -1.f, &side, &line, &t) ==
                   LZ_BTN_FIELD + LZ_SET_SWEEP &&
               side == 1,
           "sweep has a plus");

    page = 0;
    expect(lz_menu_click(&page, LZ_BTN_TAB_SET, 0, &s) == 2 && page == 1, "set opens the first settings page");
    expect(lz_menu_click(&page, LZ_BTN_NEXT, 0, &s) == 2 && page == 2, "next opens the rest of the settings");
    expect(lz_menu_click(&page, LZ_BTN_FIELD + LZ_SET_STRIDE, 1, &s) == 1 && s.stride == 2, "plus raises stride");
    expect(lz_menu_click(&page, LZ_BTN_FIELD + LZ_SET_STRIDE, 0, &s) == 0 && s.stride == 2,
           "the middle of stride does not change it");
    expect(lz_menu_click(&page, LZ_BTN_SOLID, 0, &s) == 1 && s.solid == 0, "solid toggles off");
    expect(lz_menu_click(&page, LZ_BTN_FLIP, 0, &s) == 1 && s.flip == 0, "flip toggles off");
    s.ea = 9;
    expect(lz_menu_click(&page, LZ_BTN_DEFAULTS, 0, &s) == 1 && s.ea == 22 && s.stride == 1 && s.solid == 1 &&
               s.flip == 1 && s.far_mm == 550,
           "defaults restores every scanner setting");
    expect(lz_menu_click(&page, LZ_BTN_BACK, 0, &s) == 2 && page == 1, "back returns to the first settings page");
    expect(lz_menu_click(&page, LZ_BTN_TAB_CUBE, 0, &s) == 2 && page == 0, "cube returns to resize and lock");
    expect(lz_menu_click(&page, LZ_BTN_BIGGER, 0, &s) == 0, "resize is not a scanner setting");
    expect(strcmp(lz_menu_button_name(LZ_BTN_BIGGER), "bigger") == 0, "bigger keeps its name");
    expect(strcmp(lz_menu_button_name(LZ_BTN_FIELD + LZ_SET_EA), "ea") == 0, "exposure keeps the desktop key");
    expect(strcmp(lz_menu_button_name(LZ_BTN_EXPORT), "export") == 0, "export keeps its name");
}

int main(void) {
    test_pose();
    test_color();
    test_mold();
    test_cube();
    test_drive();
    test_menu();
    test_settings();
    if (g_fail) {
        fprintf(stderr, "%d checks failed\n", g_fail);
        return 1;
    }
    printf("lizard self-test ok\n");
    return 0;
}
