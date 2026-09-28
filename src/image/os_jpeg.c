#include <stdint.h>
#include <stdlib.h>
#include <string.h>

static void scale_gray(const uint8_t *full, int sw, int sh, uint8_t *dst, int w, int h) {
    if (sw < 1 || sh < 1) return;
    for (int yy = 0; yy < h; yy++) {
        int sy = yy * sh / h;
        if (sy >= sh) sy = sh - 1;
        for (int xx = 0; xx < w; xx++) {
            int sx = xx * sw / w;
            if (sx >= sw) sx = sw - 1;
            dst[yy * w + xx] = full[sy * sw + sx];
        }
    }
}

#if defined(_WIN32)

#define COBJMACROS
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <wincodec.h>

int os_jpeg_gray(const uint8_t *jpg, size_t n, int w, int h, uint8_t *dst) {
    if (!jpg || !dst || n < 16 || w < 1 || h < 1) return -1;
    static int com = 0;
    if (!com) {
        HRESULT hr = CoInitializeEx(NULL, COINIT_MULTITHREADED);
        if (FAILED(hr) && hr != RPC_E_CHANGED_MODE) return -1;
        com = 1;
    }
    IWICImagingFactory *fac = NULL;
    if (FAILED(CoCreateInstance(&CLSID_WICImagingFactory, NULL, CLSCTX_INPROC_SERVER,
                                &IID_IWICImagingFactory, (void **)&fac)))
        return -1;
    IWICStream *stream = NULL;
    IWICBitmapDecoder *dec = NULL;
    IWICBitmapFrameDecode *frame = NULL;
    IWICFormatConverter *conv = NULL;
    int rc = -1;
    if (FAILED(IWICImagingFactory_CreateStream(fac, &stream))) goto done;
    if (FAILED(IWICStream_InitializeFromMemory(stream, (BYTE *)jpg, (DWORD)n))) goto done;
    if (FAILED(IWICImagingFactory_CreateDecoderFromStream(fac, (IStream *)stream, NULL,
                                                         WICDecodeMetadataCacheOnLoad, &dec)))
        goto done;
    if (FAILED(IWICBitmapDecoder_GetFrame(dec, 0, &frame))) goto done;
    if (FAILED(IWICImagingFactory_CreateFormatConverter(fac, &conv))) goto done;
    if (FAILED(IWICFormatConverter_Initialize(conv, (IWICBitmapSource *)frame, &GUID_WICPixelFormat8bppGray,
                                             WICBitmapDitherTypeNone, NULL, 0.0, WICBitmapPaletteTypeCustom)))
        goto done;
    UINT sw = 0, sh = 0;
    if (FAILED(IWICFormatConverter_GetSize(conv, &sw, &sh)) || sw < 1 || sh < 1) goto done;
    uint8_t *full = malloc((size_t)sw * sh);
    if (!full) goto done;
    if (SUCCEEDED(IWICFormatConverter_CopyPixels(conv, NULL, sw, sw * sh, full))) {
        scale_gray(full, (int)sw, (int)sh, dst, w, h);
        rc = 0;
    }
    free(full);
done:
    if (conv) IWICFormatConverter_Release(conv);
    if (frame) IWICBitmapFrameDecode_Release(frame);
    if (dec) IWICBitmapDecoder_Release(dec);
    if (stream) IWICStream_Release(stream);
    if (fac) IWICImagingFactory_Release(fac);
    return rc;
}

#elif defined(__APPLE__)

#include <CoreGraphics/CoreGraphics.h>
#include <ImageIO/ImageIO.h>

int os_jpeg_gray(const uint8_t *jpg, size_t n, int w, int h, uint8_t *dst) {
    if (!jpg || !dst || n < 16 || w < 1 || h < 1) return -1;
    CFDataRef data = CFDataCreate(NULL, jpg, (CFIndex)n);
    if (!data) return -1;
    CGImageSourceRef src = CGImageSourceCreateWithData(data, NULL);
    CGImageRef image = src ? CGImageSourceCreateImageAtIndex(src, 0, NULL) : NULL;
    int rc = -1;
    if (image) {
        size_t sw = CGImageGetWidth(image), sh = CGImageGetHeight(image);
        CGColorSpaceRef gray = CGColorSpaceCreateDeviceGray();
        uint8_t *full = calloc(sw * sh, 1);
        CGContextRef ctx = full && gray ? CGBitmapContextCreate(full, sw, sh, 8, sw, gray, kCGImageAlphaNone) : NULL;
        if (ctx) {
            CGContextDrawImage(ctx, CGRectMake(0, 0, (CGFloat)sw, (CGFloat)sh), image);
            scale_gray(full, (int)sw, (int)sh, dst, w, h);
            rc = 0;
            CGContextRelease(ctx);
        }
        free(full);
        if (gray) CGColorSpaceRelease(gray);
        CGImageRelease(image);
    }
    if (src) CFRelease(src);
    CFRelease(data);
    return rc;
}

#else

#include <stdio.h>
#include <jpeglib.h>

int os_jpeg_gray(const uint8_t *jpg, size_t n, int w, int h, uint8_t *dst) {
    if (!jpg || !dst || n < 16 || w < 1 || h < 1) return -1;
    struct jpeg_decompress_struct cinfo;
    struct jpeg_error_mgr jerr;
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_decompress(&cinfo);
    jpeg_mem_src(&cinfo, jpg, n);
    if (jpeg_read_header(&cinfo, TRUE) != JPEG_HEADER_OK) {
        jpeg_destroy_decompress(&cinfo);
        return -1;
    }
    cinfo.out_color_space = JCS_GRAYSCALE;
    jpeg_start_decompress(&cinfo);
    int sw = (int)cinfo.output_width, sh = (int)cinfo.output_height;
    uint8_t *row = malloc((size_t)sw);
    uint8_t *full = malloc((size_t)sw * (size_t)sh);
    if (!row || !full || sw < 1 || sh < 1) {
        free(row);
        free(full);
        jpeg_destroy_decompress(&cinfo);
        return -1;
    }
    int y = 0;
    while (cinfo.output_scanline < cinfo.output_height) {
        jpeg_read_scanlines(&cinfo, &row, 1);
        memcpy(full + y * sw, row, (size_t)sw);
        y++;
    }
    jpeg_finish_decompress(&cinfo);
    jpeg_destroy_decompress(&cinfo);
    scale_gray(full, sw, sh, dst, w, h);
    free(full);
    free(row);
    return 0;
}

#endif
