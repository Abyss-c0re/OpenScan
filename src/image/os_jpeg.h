#ifndef OS_JPEG_H
#define OS_JPEG_H
#include <stddef.h>
#include <stdint.h>
int os_jpeg_gray(const uint8_t *jpg, size_t n, int w, int h, uint8_t *dst);
#endif
