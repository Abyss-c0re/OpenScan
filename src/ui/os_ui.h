#ifndef OS_UI_H
#define OS_UI_H
#include "openscan/openscan_calib.h"
#include "openscan/openscan_v4l2.h"
#include <stddef.h>
#include <stdint.h>
int os_ui_main(const openscan_calib *cal, int exp_a, int gain_a, int exp_b, int gain_b);
int os_grab_gray(openscan_cam *cam, uint8_t *storage, size_t cap, uint8_t *gray);
#endif
