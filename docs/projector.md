# Projector

Streaming camera A is the projector control. `openscan_cam_start` sends `VIDIOC_STREAMON` to the pattern camera. The camera runs its own sensor script, and that script pulses the NIR lamp. OpenScan does not send a projector packet.

Stopping camera A is the off switch. `openscan_cam_stop` sends `VIDIOC_STREAMOFF`. The lattice leaves with that stream.

## Cameras

The scanner is two Sonix cameras behind one USB 2 hub. Linux uses the `uvcvideo` driver. OpenScan does not install a kernel module.

- Camera A, USB `0c45:636a`, is the pattern camera. Its name is `KYT Camera A: <serial>_A`. The udev rule calls the capture node `/dev/openscan-a`. This sensor sees the NIR lattice.
- Camera B, USB `0c45:636b`, is the clean camera. Its name is `KYT Camera B: <serial>_B`. The node is `/dev/openscan-b`. An IR cut filter hides the lattice.

Both cameras carry the same serial. On the unit this was checked with, the serial starts with `JMM8`. A mixed pair is not opened.

The lamp is a fixed NIR pattern. The calibration file models it as a third pinhole, `cam[2]` in `openscan_calib`. Depth is computed on the host. On that unit the A-to-B baseline is about 32 mm, and the projector sits about 57 mm from camera A along X.

## What the host writes

The scan path is `openscan/src/device/openscan_v4l2.c`.

1. `openscan_find_cameras` finds the two capture nodes.
2. `openscan_cam_open` opens each one. Both streams are MJPEG at 1280×720. Two uncompressed 720p streams do not fit the hub. The lattice is still in the MJPEG luminance.
3. Exposure and gain are set with ordinary V4L2 controls. `exposure_time_absolute` is in steps of 100 microseconds. Gain is the 0..100 value written to the camera. These are the only camera controls the scan path writes.
4. `openscan_cam_start` queues the buffers and calls `VIDIOC_STREAMON`.
5. `openscan_cam_stop` calls `VIDIOC_STREAMOFF`.

Camera B can stream on its own. The lamp stays off, because B's sensor is not wired to it. Backlight compensation does not change the lamp.

The desktop defaults match the sliders and the Android app: camera A exposure 22 and gain 6, camera B exposure 16 and gain 4. On a bright plush, exposure 22 clips the dots to white. Exposure 8 and gain 20 left the columns mid-gray. The defaults are unchanged. Pass `--exposure-a 8 --gain-a 20` when the lattice is clipped.

## Why the lamp follows camera A

The bridge chip on this unit is an SN9C285. Camera A's image sensor is an OV9281. The factory init script lives in the camera's serial flash and runs when that sensor starts. The end of the script makes the sensor STROBE pin an output and then starts the stream (`0x0100 = 1`). The NIR lamp is tied to that frame pulse.

The script is already in the camera. OpenScan does not write sensor registers, extension-unit registers, or flash to turn the lamp on.

A capture program that only opens camera A shows the same dots. The vendor desktop path for this camera family sends no projector packet. Its texture-light call is an empty return. Serial lamp commands used by a different module have nowhere to go on this unit: there is no UART, and there is no HID interface `0c45:672e`.

## From the dots to a surface

Measured and Cloud both call `os_stripe_make` on camera A's luminance (`os_live.c`). Each calibration plane is one line in the projector image, `u = slope * v + offset`. A bright column in camera A is one of those planes.

On this scanner the columns repeat about every 13 px. `lattice_hp` in `os_stripe.c` subtracts a 13 px mean from a 3 px mean, so each column is one peak. A wider average splits one column into two.

On a full frame the plane index is the column's place on one period, measured from the center column. The period is that pixel gap divided by the calibrated plane pitch at the center. The distance slider is the depth of the center column only. Each peak keeps that plane and is intersected with its own ray, so a bend in the column stays as relief. The stored plane list is extended with the same slope and offset step, because a close snout falls past the last factory line. The slider is the scale: the same picture keeps its shape, and the relief grows with the distance.

The synthetic wall inside `openscan turn-test` is a small frame and has no lattice. Each painted stripe is one plane at the hint, and the test still reports 179.1 mm. On a stored plush frame with the slider at 220 mm, the cheeks sit on that depth and the snout is closer.

Points are millimetres in camera A: X right, Y up, Z forward.

The live picture has to be a matte object. A monitor fills camera A to the white clip and shows no columns.
