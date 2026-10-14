#include "lizard_menu.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/* 5x7 glyphs. Bit 0x10 is the left pixel. Unknown characters are not drawn. */
static const char *k_chars = " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789+-./[]";
static const uint8_t k_font[][7] = {
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, /* space */
    {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}, /* A */
    {0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E}, /* B */
    {0x0F, 0x10, 0x10, 0x10, 0x10, 0x10, 0x0F}, /* C */
    {0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E}, /* D */
    {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F}, /* E */
    {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10}, /* F */
    {0x0F, 0x10, 0x10, 0x13, 0x11, 0x11, 0x0F}, /* G */
    {0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}, /* H */
    {0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E}, /* I */
    {0x07, 0x02, 0x02, 0x02, 0x02, 0x12, 0x0C}, /* J */
    {0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11}, /* K */
    {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F}, /* L */
    {0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11}, /* M */
    {0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11}, /* N */
    {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}, /* O */
    {0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10}, /* P */
    {0x0E, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0D}, /* Q */
    {0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11}, /* R */
    {0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E}, /* S */
    {0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04}, /* T */
    {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}, /* U */
    {0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04}, /* V */
    {0x11, 0x11, 0x11, 0x15, 0x15, 0x15, 0x0A}, /* W */
    {0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11}, /* X */
    {0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04}, /* Y */
    {0x1F, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1F}, /* Z */
    {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E}, /* 0 */
    {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E}, /* 1 */
    {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F}, /* 2 */
    {0x1E, 0x01, 0x01, 0x0E, 0x01, 0x01, 0x1E}, /* 3 */
    {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02}, /* 4 */
    {0x1F, 0x10, 0x10, 0x1E, 0x01, 0x01, 0x1E}, /* 5 */
    {0x0E, 0x10, 0x10, 0x1E, 0x11, 0x11, 0x0E}, /* 6 */
    {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08}, /* 7 */
    {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E}, /* 8 */
    {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x01, 0x0E}, /* 9 */
    {0x00, 0x04, 0x04, 0x1F, 0x04, 0x04, 0x00}, /* + */
    {0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00}, /* - */
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C}, /* . */
    {0x01, 0x02, 0x02, 0x04, 0x08, 0x08, 0x10}, /* / */
    {0x1E, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1E}, /* [ */
    {0x0F, 0x01, 0x01, 0x01, 0x01, 0x01, 0x0F}, /* ] */
};

static const uint8_t *glyph(char ch) {
    const char *hit = strchr(k_chars, ch);
    if (!hit) return NULL;
    return k_font[(int)(hit - k_chars)];
}

static int norm3(float *x, float *y, float *z) {
    float len = sqrtf((*x) * (*x) + (*y) * (*y) + (*z) * (*z));
    if (len < 1e-6f) return 0;
    *x /= len;
    *y /= len;
    *z /= len;
    return 1;
}

static int cm_of(float metres) {
    float cm = metres * 100.f;
    if (cm >= 0.f) return (int)(cm + 0.5f);
    return (int)(cm - 0.5f);
}

float lz_menu_line_y(int line) {
    if (line < 0) line = 0;
    if (line >= LZ_MENU_LINES) line = LZ_MENU_LINES - 1;
    return LZ_MENU_H * 0.5f - ((float)line + 0.5f) * (LZ_MENU_H / (float)LZ_MENU_LINES);
}

void lz_menu_panel_at(LzPanel *p, float ax, float ay, float az, const LzPose *head) {
    LzPose id;
    float fx, fy, fz, rx, ry, rz, ux, uy, uz;
    float dx, dy, dz;
    if (!p) return;
    if (!head) {
        memset(&id, 0, sizeof id);
        id.qw = 1.f;
        head = &id;
    }
    lz_quat_rotate(head->qx, head->qy, head->qz, head->qw, 0.f, 0.f, -1.f, &fx, &fy, &fz);
    if (!norm3(&fx, &fy, &fz)) {
        fx = 0.f;
        fy = 0.f;
        fz = -1.f;
    }
    /* Right is forward × world up. Straight up falls back to +X. */
    rx = fy * 0.f - fz * 1.f;
    ry = fz * 0.f - fx * 0.f;
    rz = fx * 1.f - fy * 0.f;
    if (!norm3(&rx, &ry, &rz)) {
        rx = 1.f;
        ry = 0.f;
        rz = 0.f;
    }
    ux = ry * fz - rz * fy;
    uy = rz * fx - rx * fz;
    uz = rx * fy - ry * fx;
    if (!norm3(&ux, &uy, &uz)) {
        ux = 0.f;
        uy = 1.f;
        uz = 0.f;
    }
    dx = ax - head->px;
    dy = ay - head->py;
    dz = az - head->pz;
    if (dx * dx + dy * dy + dz * dz < 0.0004f) {
        ax = head->px + fx * 0.42f;
        ay = head->py + fy * 0.42f;
        az = head->pz + fz * 0.42f;
    }
    p->cx = ax;
    p->cy = ay;
    p->cz = az;
    p->rx = rx * (LZ_MENU_W * 0.5f);
    p->ry = ry * (LZ_MENU_W * 0.5f);
    p->rz = rz * (LZ_MENU_W * 0.5f);
    p->ux = ux * (LZ_MENU_H * 0.5f);
    p->uy = uy * (LZ_MENU_H * 0.5f);
    p->uz = uz * (LZ_MENU_H * 0.5f);
}

/* 1 when the ray hits the front of the panel. *a_out is -1..1 across the width.
 * *line_out is the row. The back of the panel does not hit. */
static int aim_panel(const LzPanel *p, float ox, float oy, float oz, float dx, float dy, float dz, float *a_out,
                     float *t_out, int *line_out) {
    float nx, ny, nz, nl, denom, t, hx, hy, hz, inv;
    float relx, rely, relz, rr, uu, a, b, ym;
    int i;
    if (line_out) *line_out = -1;
    if (!p) return 0;
    inv = sqrtf(dx * dx + dy * dy + dz * dz);
    if (inv < 1e-8f) return 0;
    dx /= inv;
    dy /= inv;
    dz /= inv;
    nx = p->ry * p->uz - p->rz * p->uy;
    ny = p->rz * p->ux - p->rx * p->uz;
    nz = p->rx * p->uy - p->ry * p->ux;
    nl = sqrtf(nx * nx + ny * ny + nz * nz);
    rr = p->rx * p->rx + p->ry * p->ry + p->rz * p->rz;
    uu = p->ux * p->ux + p->uy * p->uy + p->uz * p->uz;
    if (nl < 1e-8f || rr < 1e-8f || uu < 1e-8f) return 0;
    /* The normal points toward the headset. A back-face ray is rejected. */
    if (dx * nx + dy * ny + dz * nz >= 0.f) return 0;
    denom = dx * nx + dy * ny + dz * nz;
    t = ((p->cx - ox) * nx + (p->cy - oy) * ny + (p->cz - oz) * nz) / denom;
    if (!(t > 0.001f)) return 0;
    hx = ox + t * dx;
    hy = oy + t * dy;
    hz = oz + t * dz;
    relx = hx - p->cx;
    rely = hy - p->cy;
    relz = hz - p->cz;
    a = (relx * p->rx + rely * p->ry + relz * p->rz) / rr;
    b = (relx * p->ux + rely * p->uy + relz * p->uz) / uu;
    if (fabsf(a) > 1.f || fabsf(b) > 1.f) return 0;
    ym = b * (LZ_MENU_H * 0.5f);
    for (i = 0; i < LZ_MENU_LINES; i++) {
        float ly = lz_menu_line_y(i);
        float hh = (LZ_MENU_H / (float)LZ_MENU_LINES) * 0.45f;
        if (fabsf(ym - ly) <= hh) {
            if (a_out) *a_out = a;
            if (t_out) *t_out = t;
            if (line_out) *line_out = i;
            return 1;
        }
    }
    return 0;
}

static int side_of(float a) {
    if (a < -0.45f) return -1;
    if (a > 0.45f) return 1;
    return 0;
}

static int tab_id(float a) {
    if (a < -0.08f) return LZ_BTN_TAB_CUBE;
    if (a > 0.08f) return LZ_BTN_TAB_SET;
    return LZ_BTN_NONE;
}

int lz_menu_hit(const LzPanel *p, float ox, float oy, float oz, float dx, float dy, float dz, float *t_out) {
    float a, t;
    int line = -1;
    if (!aim_panel(p, ox, oy, oz, dx, dy, dz, &a, &t, &line)) return LZ_BTN_NONE;
    (void)a;
    if (line >= 5 && line <= 8) {
        if (t_out) *t_out = t;
        return line - 4;
    }
    return LZ_BTN_NONE;
}

int lz_menu_pick(const LzPanel *p, int page, float ox, float oy, float oz, float dx, float dy, float dz, int *side,
                 int *line_out, float *t_out) {
    float a, t;
    int line = -1;
    int id = LZ_BTN_NONE;
    int sd = 0;
    if (line_out) *line_out = -1;
    if (side) *side = 0;
    if (!aim_panel(p, ox, oy, oz, dx, dy, dz, &a, &t, &line)) return LZ_BTN_NONE;
    if (page < 0 || page > 2) page = 0;
    if (page == 0) {
        if (line == 4) id = tab_id(a);
        else if (line == 8) id = a < 0.f ? LZ_BTN_RESET : LZ_BTN_EXPORT;
        else if (line >= 5 && line <= 7) id = line - 4;
    } else if (line == 0) {
        id = tab_id(a);
    } else if (page == 1) {
        if (line >= 1 && line <= 7) {
            id = LZ_BTN_FIELD + (line - 1);
            sd = side_of(a);
        } else if (line == 8) {
            id = LZ_BTN_NEXT;
        }
    } else {
        if (line >= 1 && line <= 4) {
            id = LZ_BTN_FIELD + LZ_SET_STRIDE + (line - 1);
            sd = side_of(a);
        } else if (line == 5) {
            id = LZ_BTN_SOLID;
        } else if (line == 6) {
            id = LZ_BTN_FLIP;
        } else if (line == 7) {
            id = LZ_BTN_DEFAULTS;
        } else if (line == 8) {
            id = LZ_BTN_BACK;
        }
    }
    if (id == LZ_BTN_NONE) return LZ_BTN_NONE;
    if (side) *side = sd;
    if (line_out) *line_out = line;
    if (t_out) *t_out = t;
    return id;
}

static void tab_line(char *dst, int settings) {
    if (settings)
        snprintf(dst, 28, "%-16s[SET]", "CUBE");
    else
        snprintf(dst, 28, "%-16sSET", "[CUBE]");
}

/* '-' at column 2 and '+' at column 22, both inside the panel. */
static void row_pm(char *dst, const char *label, int value) {
    char body[32];
    snprintf(body, sizeof body, "%s %d", label, value);
    snprintf(dst, 28, "  - %-17.17s +", body);
}

void lz_menu_view(LzMenuView *v, int controller, int placed, int grabbed, int edge_mm, int pts, int stored,
                  int cols, int tris, int hand_ok, float camx_m, float camy_m, float camz_m) {
    const char *st = "WAIT";
    int hot;
    if (!v) return;
    hot = v->hot;
    memset(v->line, 0, sizeof v->line);
    v->hot = hot;
    v->hot_line = -1;
    v->placed = placed ? 1 : 0;
    if (grabbed) st = "GRAB";
    else if (placed) st = "LOCKED";
    else if (controller) st = "FOLLOW";
    snprintf(v->line[0], 28, "%s", st);
    snprintf(v->line[1], 28, "EDGE %d MM %d/%d", edge_mm, pts, stored);
    if (cols > 0) snprintf(v->line[2], 28, "COLS %d SEE %d T %d", cols, pts, tris);
    else if (pts > 0 || tris > 0) snprintf(v->line[2], 28, "SEE %d T %d", pts, tris);
    else snprintf(v->line[2], 28, "NO PATTERN");
    if (hand_ok)
        snprintf(v->line[3], 28, "%s %+d %+d %+d CM", hand_ok == 2 ? "HR" : "HL", cm_of(camx_m), cm_of(camy_m),
                 cm_of(camz_m));
    else
        snprintf(v->line[3], 28, "HAND NOT SEEN");
    tab_line(v->line[4], 0);
    snprintf(v->line[5], 28, "%s", placed ? "UNLOCK" : "LOCK CUBE");
    snprintf(v->line[6], 28, "BIGGER");
    snprintf(v->line[7], 28, "SMALLER");
    snprintf(v->line[8], 28, "%-11s%s", "RESET", "EXPORT");
}

void lz_menu_settings(LzMenuView *v, int page, const LzSettings *s) {
    LzSettings local;
    int hot;
    if (!v) return;
    hot = v->hot;
    memset(v->line, 0, sizeof v->line);
    v->hot = hot;
    v->hot_line = -1;
    if (!s) {
        lz_settings_default(&local);
        s = &local;
    }
    if (page < 1) page = 1;
    if (page > 2) page = 2;
    tab_line(v->line[0], 1);
    if (page == 1) {
        row_pm(v->line[1], "A EXP", s->ea);
        row_pm(v->line[2], "A GAIN", s->ga);
        row_pm(v->line[3], "B EXP", s->eb);
        row_pm(v->line[4], "B GAIN", s->gb);
        row_pm(v->line[5], "DIST", s->distance);
        row_pm(v->line[6], "NEAR", s->near_mm);
        row_pm(v->line[7], "FAR", s->far_mm);
        snprintf(v->line[8], 28, "NEXT");
    } else {
        row_pm(v->line[1], "STRIDE", s->stride);
        row_pm(v->line[2], "SMOOTH", s->smooth);
        row_pm(v->line[3], "SWEEP", s->sweep);
        row_pm(v->line[4], "RELIEF", s->relief);
        snprintf(v->line[5], 28, "SOLID %s", s->solid ? "ON" : "OFF");
        snprintf(v->line[6], 28, "FLIP %s", s->flip ? "ON" : "OFF");
        snprintf(v->line[7], 28, "DEFAULTS");
        snprintf(v->line[8], 28, "BACK");
    }
}

int lz_menu_click(int *page_io, int id, int side, LzSettings *s) {
    int page;
    if (!page_io || !s || id == LZ_BTN_NONE) return 0;
    page = *page_io;
    if (page < 0 || page > 2) page = 0;
    if (id == LZ_BTN_TAB_CUBE) {
        *page_io = 0;
        return page == 0 ? 0 : 2;
    }
    if (id == LZ_BTN_TAB_SET) {
        *page_io = 1;
        return page == 1 ? 0 : 2;
    }
    if (page == 0) return 0;
    if (id == LZ_BTN_NEXT && page == 1) {
        *page_io = 2;
        return 2;
    }
    if (id == LZ_BTN_BACK && page == 2) {
        *page_io = 1;
        return 2;
    }
    if (id == LZ_BTN_DEFAULTS) {
        lz_settings_default(s);
        return 1;
    }
    if (id == LZ_BTN_SOLID) {
        s->solid = s->solid ? 0 : 1;
        return 1;
    }
    if (id == LZ_BTN_FLIP) {
        s->flip = s->flip ? 0 : 1;
        return 1;
    }
    if (id >= LZ_BTN_FIELD && id < LZ_BTN_FIELD + LZ_SET_COUNT) {
        if (side == 0) return 0;
        lz_settings_nudge(s, id - LZ_BTN_FIELD, side < 0 ? -1 : 1);
        return 1;
    }
    return 0;
}

const char *lz_menu_button_name(int id) {
    switch (id) {
    case LZ_BTN_LOCK: return "lock";
    case LZ_BTN_BIGGER: return "bigger";
    case LZ_BTN_SMALLER: return "smaller";
    case LZ_BTN_RESET: return "reset";
    case LZ_BTN_EXPORT: return "export";
    case LZ_BTN_TAB_CUBE: return "cube";
    case LZ_BTN_TAB_SET: return "set";
    case LZ_BTN_NEXT: return "next";
    case LZ_BTN_BACK: return "back";
    case LZ_BTN_DEFAULTS: return "defaults";
    case LZ_BTN_SOLID: return "solid";
    case LZ_BTN_FLIP: return "flip";
    default: break;
    }
    if (id >= LZ_BTN_FIELD && id < LZ_BTN_FIELD + LZ_SET_COUNT) return lz_settings_key(id - LZ_BTN_FIELD);
    return "button";
}

static void stamp(uint8_t *rgba, int x, int y, const uint8_t *g, int hot) {
    int row, col, sy, sx;
    uint8_t cr = hot ? 255 : 236;
    uint8_t cg = hot ? 214 : 242;
    uint8_t cb = hot ? 70 : 230;
    for (row = 0; row < 7; row++) {
        for (col = 0; col < 5; col++) {
            if ((g[row] & (0x10 >> col)) == 0) continue;
            for (sy = 0; sy < 2; sy++) {
                for (sx = 0; sx < 2; sx++) {
                    int px = x + col * 2 + sx;
                    int py = y + row * 2 + sy;
                    uint8_t *pix;
                    if (px < 0 || py < 0 || px >= LZ_MENU_TEX_W || py >= LZ_MENU_TEX_H) continue;
                    pix = rgba + (py * LZ_MENU_TEX_W + px) * 4;
                    pix[0] = cr;
                    pix[1] = cg;
                    pix[2] = cb;
                    pix[3] = 255;
                }
            }
        }
    }
}

static int hot_line_of(const LzMenuView *v) {
    if (!v) return -1;
    if (v->hot_line >= 0 && v->hot_line < LZ_MENU_LINES) return v->hot_line;
    if (v->hot >= LZ_BTN_LOCK && v->hot <= LZ_BTN_RESET) return v->hot + 4;
    return -1;
}

static void draw_line(uint8_t *rgba, int line, const char *text, int hot) {
    int x = 4;
    int y = line * 18 + 2;
    if (!text) return;
    for (; *text; text++) {
        const uint8_t *g = glyph(*text);
        if (!g) continue;
        if (*text != ' ') stamp(rgba, x, y, g, hot);
        x += 10;
        if (x > LZ_MENU_TEX_W - 10) break;
    }
}

void lz_menu_raster(const LzMenuView *v, uint8_t *rgba) {
    int i, x, y;
    if (!rgba) return;
    for (i = 0; i < LZ_MENU_TEX_W * LZ_MENU_TEX_H; i++) {
        uint8_t *p = rgba + i * 4;
        p[0] = 10;
        p[1] = 16;
        p[2] = 22;
        p[3] = 210;
    }
    if (!v) return;
    {
        int line = hot_line_of(v);
        if (line >= 0) {
            int y0 = line * 18;
            for (y = y0; y < y0 + 18 && y < LZ_MENU_TEX_H; y++) {
                for (x = 0; x < LZ_MENU_TEX_W; x++) {
                    uint8_t *p = rgba + (y * LZ_MENU_TEX_W + x) * 4;
                    p[0] = 28;
                    p[1] = 72;
                    p[2] = 48;
                    p[3] = 230;
                }
            }
        }
    }
    for (i = 0; i < LZ_MENU_LINES; i++) draw_line(rgba, i, v->line[i], hot_line_of(v) == i);
}

static void vert5(float *o, float x, float y, float z, float u, float v) {
    o[0] = x;
    o[1] = y;
    o[2] = z;
    o[3] = u;
    o[4] = v;
}

void lz_menu_quad(const LzPanel *p, float *out) {
    float blx, bly, blz, brx, bry, brz, trx, try_, trz, tlx, tly, tlz;
    if (!p || !out) return;
    blx = p->cx - p->rx - p->ux;
    bly = p->cy - p->ry - p->uy;
    blz = p->cz - p->rz - p->uz;
    brx = p->cx + p->rx - p->ux;
    bry = p->cy + p->ry - p->uy;
    brz = p->cz + p->rz - p->uz;
    trx = p->cx + p->rx + p->ux;
    try_ = p->cy + p->ry + p->uy;
    trz = p->cz + p->rz + p->uz;
    tlx = p->cx - p->rx + p->ux;
    tly = p->cy - p->ry + p->uy;
    tlz = p->cz - p->rz + p->uz;
    vert5(out + 0, blx, bly, blz, 0.f, 0.f);
    vert5(out + 5, brx, bry, brz, 1.f, 0.f);
    vert5(out + 10, trx, try_, trz, 1.f, 1.f);
    vert5(out + 15, blx, bly, blz, 0.f, 0.f);
    vert5(out + 20, trx, try_, trz, 1.f, 1.f);
    vert5(out + 25, tlx, tly, tlz, 0.f, 1.f);
}

static void emit6(float *o, int *n, float x, float y, float z, float r, float g, float b) {
    float *p;
    if (*n >= LZ_MENU_RAY_VERTS) return;
    p = o + (*n) * 6;
    p[0] = x;
    p[1] = y;
    p[2] = z;
    p[3] = r;
    p[4] = g;
    p[5] = b;
    (*n)++;
}

int lz_menu_ray(float ox, float oy, float oz, float dx, float dy, float dz, float len, int hot, float *out) {
    float sl, sx, sy, sz, ux, uy, uz;
    float r, g, b, ex, ey, ez;
    float off[4][3];
    float va[4][3], vb[4][3];
    int i, n = 0;
    const float half = 0.002f;
    if (!out || len < 1e-4f) return 0;
    sl = sqrtf(dx * dx + dy * dy + dz * dz);
    if (sl < 1e-6f) return 0;
    dx /= sl;
    dy /= sl;
    dz /= sl;
    ex = ox + dx * len;
    ey = oy + dy * len;
    ez = oz + dz * len;
    if (fabsf(dy) < 0.9f) {
        sx = -dz;
        sy = 0.f;
        sz = dx;
    } else {
        sx = 0.f;
        sy = dz;
        sz = -dy;
    }
    sl = sqrtf(sx * sx + sy * sy + sz * sz);
    if (sl < 1e-6f) return 0;
    sx /= sl;
    sy /= sl;
    sz /= sl;
    ux = sy * dz - sz * dy;
    uy = sz * dx - sx * dz;
    uz = sx * dy - sy * dx;
    r = hot ? 1.f : 0.92f;
    g = hot ? 0.84f : 0.95f;
    b = hot ? 0.15f : 1.f;
    off[0][0] = (sx + ux) * half;
    off[0][1] = (sy + uy) * half;
    off[0][2] = (sz + uz) * half;
    off[1][0] = (sx - ux) * half;
    off[1][1] = (sy - uy) * half;
    off[1][2] = (sz - uz) * half;
    off[2][0] = (-sx - ux) * half;
    off[2][1] = (-sy - uy) * half;
    off[2][2] = (-sz - uz) * half;
    off[3][0] = (-sx + ux) * half;
    off[3][1] = (-sy + uy) * half;
    off[3][2] = (-sz + uz) * half;
    for (i = 0; i < 4; i++) {
        va[i][0] = ox + off[i][0];
        va[i][1] = oy + off[i][1];
        va[i][2] = oz + off[i][2];
        vb[i][0] = ex + off[i][0];
        vb[i][1] = ey + off[i][1];
        vb[i][2] = ez + off[i][2];
    }
    for (i = 0; i < 4; i++) {
        int k = (i + 1) & 3;
        emit6(out, &n, va[i][0], va[i][1], va[i][2], r, g, b);
        emit6(out, &n, va[k][0], va[k][1], va[k][2], r, g, b);
        emit6(out, &n, vb[i][0], vb[i][1], vb[i][2], r, g, b);
        emit6(out, &n, va[k][0], va[k][1], va[k][2], r, g, b);
        emit6(out, &n, vb[k][0], vb[k][1], vb[k][2], r, g, b);
        emit6(out, &n, vb[i][0], vb[i][1], vb[i][2], r, g, b);
    }
    return n;
}
