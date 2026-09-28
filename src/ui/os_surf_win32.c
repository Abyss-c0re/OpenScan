#include "os_surf.h"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <stdlib.h>
#include <string.h>

struct os_surf {
    HWND hwnd;
    int w, h;
    int quit;
    int hit, mx, my, key;
};

static LRESULT CALLBACK os_wnd_proc(HWND hwnd, UINT msg, WPARAM wp, LPARAM lp) {
    os_surf *s = (os_surf *)GetWindowLongPtrA(hwnd, GWLP_USERDATA);
    if (msg == WM_NCCREATE) {
        CREATESTRUCTA *cs = (CREATESTRUCTA *)lp;
        SetWindowLongPtrA(hwnd, GWLP_USERDATA, (LONG_PTR)cs->lpCreateParams);
        return DefWindowProcA(hwnd, msg, wp, lp);
    }
    if (!s) return DefWindowProcA(hwnd, msg, wp, lp);
    if (msg == WM_CLOSE) {
        s->quit = 1;
        return 0;
    }
    if (msg == WM_LBUTTONDOWN) {
        s->mx = (short)LOWORD(lp);
        s->my = (short)HIWORD(lp);
        s->hit = 1;
        return 0;
    }
    if (msg == WM_KEYDOWN) {
        int ch = 0;
        if (wp == VK_ESCAPE) ch = 27;
        else if (wp == 'Q') ch = 'q';
        else if (wp == 'S') ch = 's';
        else if (wp == VK_SPACE) ch = ' ';
        if (ch) s->key = ch;
        return 0;
    }
    if (msg == WM_ERASEBKGND) return 1;
    return DefWindowProcA(hwnd, msg, wp, lp);
}

os_surf *os_surf_open(int w, int h) {
    HINSTANCE inst = GetModuleHandleA(NULL);
    WNDCLASSA wc;
    memset(&wc, 0, sizeof wc);
    wc.lpfnWndProc = os_wnd_proc;
    wc.hInstance = inst;
    wc.lpszClassName = "OpenScan";
    wc.hCursor = LoadCursor(NULL, IDC_ARROW);
    wc.hbrBackground = (HBRUSH)GetStockObject(BLACK_BRUSH);
    RegisterClassA(&wc);
    os_surf *s = calloc(1, sizeof *s);
    if (!s) return NULL;
    s->w = w;
    s->h = h;
    RECT rc = {0, 0, w, h};
    DWORD style = WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX;
    AdjustWindowRect(&rc, style, FALSE);
    s->hwnd = CreateWindowExA(0, wc.lpszClassName, "OpenScan", style, CW_USEDEFAULT, CW_USEDEFAULT,
                              rc.right - rc.left, rc.bottom - rc.top, NULL, NULL, inst, s);
    if (!s->hwnd) {
        free(s);
        return NULL;
    }
    ShowWindow(s->hwnd, SW_SHOW);
    return s;
}

void os_surf_close(os_surf *s) {
    if (!s) return;
    if (s->hwnd) DestroyWindow(s->hwnd);
    free(s);
}

int os_surf_pump(os_surf *s, int *mx, int *my, int *key, int *quit) {
    if (mx) *mx = -1;
    if (my) *my = -1;
    if (key) *key = 0;
    if (quit) *quit = 0;
    if (!s) return 0;
    MSG msg;
    if (!PeekMessageA(&msg, NULL, 0, 0, PM_REMOVE)) {
        if (s->quit && quit) *quit = 1;
        return 0;
    }
    TranslateMessage(&msg);
    DispatchMessageA(&msg);
    if (s->quit && quit) *quit = 1;
    if (s->hit) {
        if (mx) *mx = s->mx;
        if (my) *my = s->my;
        s->hit = 0;
    }
    if (s->key) {
        if (key) *key = s->key;
        s->key = 0;
    }
    return 1;
}

void os_surf_blit_bgr(os_surf *s, const uint8_t *bgr, int w, int h, int x, int y) {
    if (!s || !s->hwnd || !bgr || w < 1 || h < 1) return;
    int stride = (w * 3 + 3) & ~3;
    uint8_t *row = malloc((size_t)stride * (size_t)h);
    if (!row) return;
    for (int yy = 0; yy < h; yy++) {
        memcpy(row + (size_t)yy * (size_t)stride, bgr + (size_t)yy * (size_t)w * 3, (size_t)w * 3);
    }
    BITMAPINFO bi;
    memset(&bi, 0, sizeof bi);
    bi.bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    bi.bmiHeader.biWidth = w;
    bi.bmiHeader.biHeight = -h;
    bi.bmiHeader.biPlanes = 1;
    bi.bmiHeader.biBitCount = 24;
    bi.bmiHeader.biCompression = BI_RGB;
    HDC dc = GetDC(s->hwnd);
    SetDIBitsToDevice(dc, x, y, (DWORD)w, (DWORD)h, 0, 0, 0, (UINT)h, row, &bi, DIB_RGB_COLORS);
    ReleaseDC(s->hwnd, dc);
    free(row);
}

void os_surf_bar(os_surf *s, int x, int y, int w, int h, unsigned rgb) {
    if (!s || !s->hwnd) return;
    RECT rc = {x, y, x + w, y + h};
    HBRUSH br = CreateSolidBrush(RGB((rgb >> 16) & 255, (rgb >> 8) & 255, rgb & 255));
    HDC dc = GetDC(s->hwnd);
    FillRect(dc, &rc, br);
    ReleaseDC(s->hwnd, dc);
    DeleteObject(br);
}

void os_surf_text(os_surf *s, int x, int y, const char *text, unsigned rgb) {
    if (!s || !s->hwnd || !text) return;
    HDC dc = GetDC(s->hwnd);
    SetBkMode(dc, TRANSPARENT);
    SetTextColor(dc, RGB((rgb >> 16) & 255, (rgb >> 8) & 255, rgb & 255));
    TextOutA(dc, x, y - 14, text, (int)strlen(text));
    ReleaseDC(s->hwnd, dc);
}

void os_surf_flush(os_surf *s) {
    if (s && s->hwnd) GdiFlush();
}
