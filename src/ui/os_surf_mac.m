#import <Cocoa/Cocoa.h>

#include "os_surf.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct os_surf {
    NSWindow *win;
    NSView *view;
    uint8_t *fb;
    int w, h;
    int quit, hit, mx, my, key;
    int nlab;
    struct {
        int x, y;
        unsigned rgb;
        char text[160];
    } lab[40];
};

@interface OSView : NSView
@property(nonatomic, assign) os_surf *surf;
@end

@interface OSWin : NSObject <NSWindowDelegate>
@property(nonatomic, assign) os_surf *surf;
@end

@implementation OSView
- (BOOL)isFlipped {
    return YES;
}
- (BOOL)acceptsFirstResponder {
    return YES;
}
- (void)drawRect:(NSRect)dirty {
    (void)dirty;
    os_surf *s = self.surf;
    if (!s || !s->fb) return;
    NSBitmapImageRep *rep = [[NSBitmapImageRep alloc]
        initWithBitmapDataPlanes:NULL
                      pixelsWide:s->w
                      pixelsHigh:s->h
                   bitsPerSample:8
                 samplesPerPixel:3
                        hasAlpha:NO
                        isPlanar:NO
                  colorSpaceName:NSCalibratedRGBColorSpace
                     bytesPerRow:s->w * 3
                    bitsPerPixel:24];
    if (rep.bitmapData) memcpy(rep.bitmapData, s->fb, (size_t)s->w * (size_t)s->h * 3);
    [rep drawInRect:self.bounds];
    for (int i = 0; i < s->nlab; i++) {
        float r = ((s->lab[i].rgb >> 16) & 255) / 255.f;
        float g = ((s->lab[i].rgb >> 8) & 255) / 255.f;
        float b = (s->lab[i].rgb & 255) / 255.f;
        NSDictionary *attr = @{
            NSForegroundColorAttributeName : [NSColor colorWithCalibratedRed:r green:g blue:b alpha:1],
            NSFontAttributeName : [NSFont userFixedPitchFontOfSize:13]
        };
        NSString *str = [NSString stringWithUTF8String:s->lab[i].text];
        [str drawAtPoint:NSMakePoint(s->lab[i].x, s->lab[i].y - 14) withAttributes:attr];
    }
}
@end

@implementation OSWin
- (BOOL)windowShouldClose:(NSWindow *)sender {
    (void)sender;
    if (self.surf) self.surf->quit = 1;
    return NO;
}
@end

os_surf *os_surf_open(int w, int h) {
    @autoreleasepool {
        [NSApplication sharedApplication];
        [NSApp setActivationPolicy:NSApplicationActivationPolicyRegular];
        os_surf *s = calloc(1, sizeof *s);
        if (!s) return NULL;
        s->w = w;
        s->h = h;
        s->fb = calloc((size_t)w * (size_t)h * 3, 1);
        if (!s->fb) {
            free(s);
            return NULL;
        }
        NSRect frame = NSMakeRect(40, 40, w, h);
        NSWindow *win = [[NSWindow alloc]
            initWithContentRect:frame
                      styleMask:NSWindowStyleMaskTitled | NSWindowStyleMaskClosable | NSWindowStyleMaskMiniaturizable
                        backing:NSBackingStoreBuffered
                          defer:NO];
        win.title = @"OpenScan";
        OSView *view = [[OSView alloc] initWithFrame:frame];
        view.surf = s;
        win.contentView = view;
        OSWin *delegate = [OSWin new];
        delegate.surf = s;
        win.delegate = delegate;
        s->win = win;
        s->view = view;
        [view release];
        [win makeKeyAndOrderFront:nil];
        [NSApp activateIgnoringOtherApps:YES];
        [win makeFirstResponder:view];
        return s;
    }
}

void os_surf_close(os_surf *s) {
    if (!s) return;
    id del = [s->win delegate];
    [s->win setDelegate:nil];
    [del release];
    [s->win close];
    [s->win release];
    s->win = nil;
    s->view = nil;
    free(s->fb);
    free(s);
}

int os_surf_pump(os_surf *s, int *mx, int *my, int *key, int *quit, int *held) {
    if (mx) *mx = -1;
    if (my) *my = -1;
    if (key) *key = 0;
    if (quit) *quit = 0;
    if (held) *held = 0;
    if (!s) return 0;
    @autoreleasepool {
        NSEvent *ev = [NSApp nextEventMatchingMask:NSEventMaskAny
                                         untilDate:[NSDate date]
                                            inMode:NSDefaultRunLoopMode
                                           dequeue:YES];
        if (!ev) {
            if (s->quit && quit) *quit = 1;
            return 0;
        }
        if (ev.type == NSEventTypeLeftMouseDown || ev.type == NSEventTypeLeftMouseDragged ||
            ev.type == NSEventTypeLeftMouseUp) {
            NSPoint p = [s->view convertPoint:ev.locationInWindow fromView:nil];
            if (mx) *mx = (int)p.x;
            if (my) *my = (int)p.y;
            if (held) *held = ev.type == NSEventTypeLeftMouseUp ? 0 : 1;
        } else if (ev.type == NSEventTypeKeyDown) {
            int ch = 0;
            if (ev.keyCode == 53) ch = 27;
            else if (ev.charactersIgnoringModifiers.length > 0)
                ch = [ev.charactersIgnoringModifiers characterAtIndex:0];
            if (ch == 'Q') ch = 'q';
            if (ch == 'S') ch = 's';
            if (key) *key = ch;
        }
        [NSApp sendEvent:ev];
        if (s->quit && quit) *quit = 1;
        return 1;
    }
}

void os_surf_blit_bgr(os_surf *s, const uint8_t *bgr, int w, int h, int x, int y) {
    if (!s || !s->fb || !bgr || w < 1 || h < 1) return;
    for (int yy = 0; yy < h; yy++) {
        int dy = y + yy;
        if (dy < 0 || dy >= s->h) continue;
        for (int xx = 0; xx < w; xx++) {
            int dx = x + xx;
            if (dx < 0 || dx >= s->w) continue;
            const uint8_t *p = bgr + ((size_t)yy * (size_t)w + (size_t)xx) * 3;
            uint8_t *d = s->fb + ((size_t)dy * (size_t)s->w + (size_t)dx) * 3;
            d[0] = p[2];
            d[1] = p[1];
            d[2] = p[0];
        }
    }
}

void os_surf_bar(os_surf *s, int x, int y, int w, int h, unsigned rgb) {
    if (!s || !s->fb) return;
    uint8_t r = (rgb >> 16) & 255, g = (rgb >> 8) & 255, b = rgb & 255;
    for (int yy = 0; yy < h; yy++) {
        int dy = y + yy;
        if (dy < 0 || dy >= s->h) continue;
        for (int xx = 0; xx < w; xx++) {
            int dx = x + xx;
            if (dx < 0 || dx >= s->w) continue;
            uint8_t *d = s->fb + ((size_t)dy * (size_t)s->w + (size_t)dx) * 3;
            d[0] = r;
            d[1] = g;
            d[2] = b;
        }
    }
}

void os_surf_text(os_surf *s, int x, int y, const char *text, unsigned rgb) {
    if (!s || !text || s->nlab >= 40) return;
    s->lab[s->nlab].x = x;
    s->lab[s->nlab].y = y;
    s->lab[s->nlab].rgb = rgb;
    snprintf(s->lab[s->nlab].text, sizeof s->lab[s->nlab].text, "%s", text);
    s->nlab++;
}

void os_surf_flush(os_surf *s) {
    if (!s || !s->view) return;
    [s->view setNeedsDisplay:YES];
    [s->view displayIfNeeded];
    s->nlab = 0;
}
