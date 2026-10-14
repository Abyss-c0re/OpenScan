#include "os_surf.h"

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/keysym.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

struct os_surf {
    Display *dpy;
    Window win;
    GC gc;
    int screen;
    int w, h;
};

os_surf *os_surf_open(int w, int h) {
    Display *dpy = XOpenDisplay(NULL);
    if (!dpy) return NULL;
    os_surf *s = calloc(1, sizeof *s);
    if (!s) {
        XCloseDisplay(dpy);
        return NULL;
    }
    s->dpy = dpy;
    s->w = w;
    s->h = h;
    s->screen = DefaultScreen(dpy);
    s->win = XCreateSimpleWindow(dpy, RootWindow(dpy, s->screen), 40, 40, (unsigned)w, (unsigned)h, 0,
                                 BlackPixel(dpy, s->screen), 0x12141a);
    XStoreName(dpy, s->win, "OpenScan");
    XSelectInput(dpy, s->win, ExposureMask | ButtonPressMask | ButtonReleaseMask |
                 ButtonMotionMask | KeyPressMask | StructureNotifyMask);
    Atom wm = XInternAtom(dpy, "WM_DELETE_WINDOW", False);
    XSetWMProtocols(dpy, s->win, &wm, 1);
    XMapWindow(dpy, s->win);
    s->gc = XCreateGC(dpy, s->win, 0, NULL);
    XFontStruct *font = XLoadQueryFont(dpy, "fixed");
    if (font) XSetFont(dpy, s->gc, font->fid);
    return s;
}

void os_surf_close(os_surf *s) {
    if (!s) return;
    XFreeGC(s->dpy, s->gc);
    XCloseDisplay(s->dpy);
    free(s);
}

int os_surf_pump(os_surf *s, int *mx, int *my, int *key, int *quit, int *held) {
    if (mx) *mx = -1;
    if (my) *my = -1;
    if (key) *key = 0;
    if (quit) *quit = 0;
    if (held) *held = 0;
    if (!s || !XPending(s->dpy)) return 0;
    XEvent ev;
    XNextEvent(s->dpy, &ev);
    if (ev.type == ClientMessage) {
        if (quit) *quit = 1;
        return 1;
    }
    if (ev.type == ButtonPress || ev.type == MotionNotify) {
        if (mx) *mx = ev.type == ButtonPress ? ev.xbutton.x : ev.xmotion.x;
        if (my) *my = ev.type == ButtonPress ? ev.xbutton.y : ev.xmotion.y;
        if (held) *held = 1;
        return 1;
    }
    if (ev.type == ButtonRelease) {
        if (mx) *mx = ev.xbutton.x;
        if (my) *my = ev.xbutton.y;
        if (held) *held = 0;
        return 1;
    }
    if (ev.type == KeyPress) {
        char buf[8];
        KeySym ks = 0;
        int n = XLookupString(&ev.xkey, buf, (int)sizeof buf, &ks, NULL);
        int ch = 0;
        if (ks == XK_Escape) ch = 27;
        else if (ks == XK_BackSpace) ch = 8;
        else if (ks == XK_Return || ks == XK_KP_Enter) ch = '\n';
        else if (n == 1 && (unsigned char)buf[0] >= 32 && (unsigned char)buf[0] < 127) ch = (unsigned char)buf[0];
        if (key) *key = ch;
        return 1;
    }
    return 1;
}

void os_surf_blit_bgr(os_surf *s, const uint8_t *bgr, int w, int h, int x, int y) {
    if (!s || !bgr || w < 1 || h < 1) return;
    XImage *img = XCreateImage(s->dpy, DefaultVisual(s->dpy, s->screen),
                               (unsigned)DefaultDepth(s->dpy, s->screen), ZPixmap, 0, NULL,
                               (unsigned)w, (unsigned)h, 32, 0);
    if (!img) return;
    img->data = malloc((size_t)w * (size_t)h * 4);
    if (!img->data) {
        XDestroyImage(img);
        return;
    }
    int msb = ImageByteOrder(s->dpy) == MSBFirst;
    for (int i = 0; i < w * h; i++) {
        uint8_t b = bgr[i * 3], g = bgr[i * 3 + 1], r = bgr[i * 3 + 2];
        if (msb) {
            img->data[i * 4 + 0] = 0;
            img->data[i * 4 + 1] = r;
            img->data[i * 4 + 2] = g;
            img->data[i * 4 + 3] = b;
        } else {
            img->data[i * 4 + 0] = b;
            img->data[i * 4 + 1] = g;
            img->data[i * 4 + 2] = r;
            img->data[i * 4 + 3] = 0;
        }
    }
    XPutImage(s->dpy, s->win, s->gc, img, 0, 0, x, y, (unsigned)w, (unsigned)h);
    free(img->data);
    img->data = NULL;
    XDestroyImage(img);
}

void os_surf_bar(os_surf *s, int x, int y, int w, int h, unsigned rgb) {
    if (!s) return;
    XSetForeground(s->dpy, s->gc, rgb);
    XFillRectangle(s->dpy, s->win, s->gc, x, y, (unsigned)w, (unsigned)h);
}

void os_surf_text(os_surf *s, int x, int y, const char *text, unsigned rgb) {
    if (!s || !text) return;
    XSetForeground(s->dpy, s->gc, rgb);
    XDrawString(s->dpy, s->win, s->gc, x, y, text, (int)strlen(text));
}

void os_surf_flush(os_surf *s) {
    if (s) XFlush(s->dpy);
}

/* 0 confirmed, -1 cancelled or failed, -2 program missing. */
static int dialog_exec(char *const argv[], char *out, size_t n) {
    int fd[2];
    pid_t pid;
    if (pipe(fd) != 0) return -1;
    pid = fork();
    if (pid < 0) {
        close(fd[0]);
        close(fd[1]);
        return -1;
    }
    if (pid == 0) {
        dup2(fd[1], STDOUT_FILENO);
        close(fd[0]);
        close(fd[1]);
        execvp(argv[0], argv);
        _exit(127);
    }
    close(fd[1]);
    size_t got = 0;
    while (got + 1 < n) {
        ssize_t r = read(fd[0], out + got, n - 1 - got);
        if (r <= 0) break;
        got += (size_t)r;
    }
    out[got] = 0;
    close(fd[0]);
    int st = 0;
    if (waitpid(pid, &st, 0) < 0 || !WIFEXITED(st)) return -1;
    if (WEXITSTATUS(st) == 127) return -2;
    if (WEXITSTATUS(st) != 0) return -1;
    while (got > 0 && (out[got - 1] == '\n' || out[got - 1] == '\r')) out[--got] = 0;
    return got > 0 ? 0 : -1;
}

static int pick_format(char *ext, size_t n) {
    char got[64];
    char *kargv[] = {"kdialog", "--title", "Export format", "--radiolist", "Format",
                     "stl", "STL", "on", "obj", "OBJ", "off", "ply", "PLY", "off", NULL};
    char *zargv[] = {"zenity", "--list", "--radiolist", "--title=Export format",
                     "--column=pick", "--column=Format", "--print-column=2",
                     "TRUE", "STL", "FALSE", "OBJ", "FALSE", "PLY", NULL};
    int rc = dialog_exec(kargv, got, sizeof got);
    if (rc == 0) {
        if (!strcmp(got, "stl") || !strcmp(got, "obj") || !strcmp(got, "ply")) {
            ext[0] = got[0];
            ext[1] = got[1];
            ext[2] = got[2];
            ext[3] = 0;
            (void)n;
            return 0;
        }
        return -1;
    }
    if (rc != -2) return -1;
    rc = dialog_exec(zargv, got, sizeof got);
    if (rc == -2) return -2;
    if (rc != 0) return -1;
    if (!strcmp(got, "OBJ") || !strcmp(got, "obj")) snprintf(ext, n, "obj");
    else if (!strcmp(got, "PLY") || !strcmp(got, "ply")) snprintf(ext, n, "ply");
    else snprintf(ext, n, "stl");
    return 0;
}

static void force_ext(char *path, size_t n, const char *ext) {
    size_t len = strlen(path);
    int match = 0;
    if (!ext || !ext[0] || !ext[1] || !ext[2] || ext[3]) return;
    if (len >= 4 && path[len - 4] == '.') {
        const char *e = path + len - 3;
        if (!strcmp(e, "stl") || !strcmp(e, "STL") || !strcmp(e, "obj") || !strcmp(e, "OBJ") ||
            !strcmp(e, "ply") || !strcmp(e, "PLY")) {
            match = 1;
            if (strcmp(e, ext) != 0) {
                path[len - 3] = 0;
                if (strlen(path) + 3 < n) memcpy(path + strlen(path), ext, 4);
            }
        }
    }
    if (!match && len + 4 < n) {
        path[len] = '.';
        path[len + 1] = ext[0];
        path[len + 2] = ext[1];
        path[len + 3] = ext[2];
        path[len + 4] = 0;
    }
}

int os_surf_save_dialog(os_surf *s, const char *suggested, char *out, size_t n) {
    char cwd[360], suggest[512], name[256], ext[8];
    char *kargv[] = {
        "kdialog", "--title", "Export", "--getsavefilename", suggest,
        "STL (*.stl)|OBJ (*.obj)|PLY (*.ply)", NULL};
    char *zargv[] = {
        "zenity", "--file-selection", "--save", "--title=Export", "--filename", suggest,
        "--file-filter=STL | *.stl", "--file-filter=OBJ | *.obj", "--file-filter=PLY | *.ply", NULL};
    const char *base;
    int rc, fmt;
    (void)s;
    if (!out || n < 2) return -1;
    out[0] = 0;
    if (!suggested || !suggested[0]) suggested = "openscan-last.stl";
    fmt = pick_format(ext, sizeof ext);
    if (fmt == -1) return -1;
    if (fmt == -2) snprintf(ext, sizeof ext, "stl");
    base = strrchr(suggested, '/');
    base = base ? base + 1 : suggested;
    snprintf(name, sizeof name, "%s", base);
    {
        size_t len = strlen(name);
        if (len >= 4 && name[len - 4] == '.') name[len - 4] = 0;
    }
    if (!getcwd(cwd, sizeof cwd)) snprintf(cwd, sizeof cwd, ".");
    snprintf(suggest, sizeof suggest, "%s/%s.%s", cwd, name, ext);
    rc = dialog_exec(kargv, out, n);
    if (rc == -2) rc = dialog_exec(zargv, out, n);
    if (rc != 0) return -1;
    force_ext(out, n, ext);
    return out[0] ? 0 : -1;
}
