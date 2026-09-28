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

int os_surf_save_dialog(os_surf *s, const char *suggested, char *out, size_t n) {
    char cwd[360], suggest[512];
    char *kargv[] = {
        "kdialog", "--title", "Export", "--getsavefilename", suggest,
        "STL (*.stl)|OBJ (*.obj)|PLY (*.ply)", NULL};
    char *zargv[] = {
        "zenity", "--file-selection", "--save", "--title=Export", "--filename", suggest,
        "--file-filter=STL | *.stl", "--file-filter=OBJ | *.obj", "--file-filter=PLY | *.ply", NULL};
    int rc;
    (void)s;
    if (!out || n < 2) return -1;
    out[0] = 0;
    if (!suggested || !suggested[0]) suggested = "openscan-last.stl";
    if (!getcwd(cwd, sizeof cwd)) snprintf(cwd, sizeof cwd, ".");
    if (suggested[0] == '/') snprintf(suggest, sizeof suggest, "%s", suggested);
    else snprintf(suggest, sizeof suggest, "%s/%s", cwd, suggested);
    rc = dialog_exec(kargv, out, n);
    if (rc != -2) return rc;
    rc = dialog_exec(zargv, out, n);
    return rc == 0 ? 0 : -1;
}
