#ifndef LIZARD_MENU_H
#define LIZARD_MENU_H

#include <stdint.h>

#include "lizard_pose.h"
#include "lizard_settings.h"

/* Controller menu. The panel faces the headset. Buttons are in panel metres,
 * +X right, +Y up, origin at the centre. Hand tracking does not press them.
 * Page 0 is the cube. Pages 1 and 2 are the scanner settings. */

#define LZ_MENU_W 0.252f
#define LZ_MENU_H 0.162f
#define LZ_MENU_LINES 9
#define LZ_MENU_TEX_W 252
#define LZ_MENU_TEX_H 162
#define LZ_MENU_RAY_VERTS 48

enum {
    LZ_BTN_NONE = 0,
    LZ_BTN_LOCK = 1,
    LZ_BTN_BIGGER = 2,
    LZ_BTN_SMALLER = 3,
    LZ_BTN_RESET = 4,
    LZ_BTN_TAB_CUBE = 5,
    LZ_BTN_TAB_SET = 6,
    LZ_BTN_NEXT = 7,
    LZ_BTN_BACK = 8,
    LZ_BTN_DEFAULTS = 9,
    LZ_BTN_SOLID = 10,
    LZ_BTN_FLIP = 11,
    LZ_BTN_EXPORT = 12,
    LZ_BTN_FIELD = 32
};

typedef struct {
    float cx, cy, cz;
    float rx, ry, rz;
    float ux, uy, uz;
} LzPanel;

typedef struct {
    char line[LZ_MENU_LINES][28];
    int hot;
    int hot_line; /* -1 when the aim is not on a row. 0 is the top row. */
    int placed;
} LzMenuView;

float lz_menu_line_y(int line);

void lz_menu_panel_at(LzPanel *p, float ax, float ay, float az, const LzPose *head);

/* Button id when the ray hits the front of a cube-page button, otherwise LZ_BTN_NONE.
 * Lines 5..8 are lock, bigger, smaller, and reset. *t_out is the ray distance.
 * The back of the panel does not hit. */
int lz_menu_hit(const LzPanel *p, float ox, float oy, float oz, float dx, float dy, float dz, float *t_out);

/* Page 0 is the cube. Pages 1 and 2 are settings. *side is -1, 0, or +1 for a
 * numeric row (left gutter, middle, right gutter). *line_out is the row, or -1. */
int lz_menu_pick(const LzPanel *p, int page, float ox, float oy, float oz, float dx, float dy, float dz, int *side,
                 int *line_out, float *t_out);

/* hand_ok: 0 hidden, 1 left, 2 right. Camera offsets are metres; the panel shows centimetres.
 * The cube page keeps lock, bigger, smaller, reset, and export. tris is the solid triangle count. */
void lz_menu_view(LzMenuView *v, int controller, int placed, int grabbed, int edge_mm, int pts, int stored,
                  int cols, int tris, int hand_ok, float camx_m, float camy_m, float camz_m);

/* page 1 is exposure, gain, distance, near, and far. page 2 is the rest, solid, flip, and defaults. */
void lz_menu_settings(LzMenuView *v, int page, const LzSettings *s);

/* One trigger click. *page_io is 0, 1, or 2.
 * Returns 1 when scanner settings change, 2 when the page changes, else 0.
 * Cube lock, resize, and reset are left for the caller. */
int lz_menu_click(int *page_io, int id, int side, LzSettings *s);

const char *lz_menu_button_name(int id);

/* Top-down RGBA. Row 0 is the top of the menu. */
void lz_menu_raster(const LzMenuView *v, uint8_t *rgba);

/* 6 vertices, xyzuv, column of 5 floats. */
void lz_menu_quad(const LzPanel *p, float *out);

/* World xyzrgb beam along the ray. */
int lz_menu_ray(float ox, float oy, float oz, float dx, float dy, float dz, float len, int hot, float *out);

#endif
