# Using OpenScan

OpenScan scans one object. You can turn the object, or walk the scanner around it, move closer, and come in from above. The window starts idle so you can frame the object before anything is kept.

Tested on the 3DMakerpro Fox. Other 3DMakerpro scanners that use the same USB cameras may work. A missing calibration file is downloaded from 3DMakerpro's proprietary servers.

## A scan

1. Plug in the scanner. `./build/openscan devices` should list camera A and camera B and the serial.
2. Start `./build/openscan`.
3. Set the object inside the green outline. The right-hand view is the 3D model, shaded with the same gray as the camera. The side panels are the rectified stereo pair: lens distortion is removed, and a feature is on the same row in both.
4. Press **Start scan**. Move so each new view still sees part of the object you already have.
5. Press **Stop** when that pass is done. The model stays.
6. Press **Export…** and choose STL, OBJ, or PLY.

**Reset** throws the model away and returns to idle. Use it when the next Start should be a different object or a clean pass. **Pause** keeps the model and keeps following the turn, so Start can continue the same scan.

## What the 3D view means

- The surface in the 3D view is the part that has been scanned.
- The ring and the status line count how much of a full turn that surface covers: `scanned N°` and `not scanned M°`.
- **detail xN** is how many times the scanned surface has been observed. The first pass is x1. Turning back over the same side raises it. The new view is blended into those points.

Only the segmented object is kept. A frame whose overlap does not fit the model is dropped, so the table and the room are not scanned. The status line says **TRACKING LOST**. Come back to the object more slowly. The model already built is left as it is. A turn that still shows part of the scanned surface is kept, including the side that has just come into view.

Drag the 3D view to orbit it. The wheel zooms.

## Settings

The camera and size controls are open under the toolbar when the app starts. **Settings** hides or shows that row. The values are saved and used the next time the app starts.

| Setting | What to do |
| --- | --- |
| Camera A exposure | Raise it when A is too dark to see the object. It is in steps of 0.1 ms. |
| Camera A gain | Leave this low. High gain washes out the projector dots. |
| Camera B exposure | Lower it when the clean camera is white. |
| Camera B gain | Usually a little lower than camera A. |
| Mold / Measured | Mold is the solid body swept from the camera outline. Measured is the projector-stripe surface of the face. Switching clears the model. |
| Distance | How far the scanner is from the object, 100–500 mm. Mold uses it as the size of the solid. Measured uses it to identify the stripes. |
| Near / Far | Drops 3D points outside this depth band, the same cut Kinect uses on a cloud. |
| Stride | Sample step of the mesh. 1 is the full grid. Higher numbers build a coarser model. |
| Smooth | Depth blur before triangles are built. 0 keeps the stripes. 8 clays the surface. |
| Mold sweep | How far the outline is wrapped into the solid, 20° to 140°. |
| Mold relief | How hard the camera shading cuts the solid. 0 is smooth. 100 keeps eyes and nose. |
| Solid mesh | On writes triangles. Off keeps the point cloud and exports those points. |
| Defaults | Camera A 22 / gain 6, camera B 16 / gain 4, distance 220 mm. |

A starting point that has shown the object clearly is A 22 / gain 6 and B 16 / gain 4, at about 220 mm. Keep the whole object in the camera, including the top. A part that leaves the frame is missing from the model.

## Export

Export writes a file on this computer. Nothing is uploaded.

| Extension | Format |
| --- | --- |
| `.stl` | Binary STL, millimetres |
| `.obj` | Wavefront OBJ with normals |
| `.ply` | ASCII PLY with normals |

The file contains vertex positions, normals, and faces. It does not contain the scanner serial or the calibration file.

From a terminal:

```bash
./build/openscan scan --no-window -o model.obj --seconds 20 --no-ae
```

`--no-ae` keeps the exposure you set. Without it, scan can change exposure while running.

## Calibration

The factory file for a unit is `calib/<serial>.txt` next to the sources, or a file you pass with `--calib`. If OpenScan cannot find one, it downloads that unit's file from 3DMakerpro's proprietary servers into `~/.local/share/openscan/calib/`.

The lookup is `https://sw.3dyunzhan.com/v2/swift/calibration/find`, then `https://swcn.3dyunzhan.com/v2/swift/calibration/find`. That download is the only network use. The calibration file is never sent anywhere. A mesh export does not contact the network. Tested on the Fox. Other 3DMakerpro scanners may use the same service.

## Lights and exposure

The Fox is a near-infrared stereo scanner. Camera A shows the projector dots. Camera B is the cleaner picture used to follow the object.

Both cameras share one USB 2 hub, so capture is MJPEG at 1280×720, 10 frames per second. Two uncompressed 720p streams do not fit.

Exposure is in units of 100 microseconds, after manual exposure mode is selected. Gain runs from 0 to 100. On the Fox used while writing this program, camera A at exposure 22 and gain 6, and camera B at exposure 16 and gain 4, showed the object without clipping it white.

## Building a package

The commands that produce `build/openscan`, an AppImage, and an Android APK are in the [README](../README.md#build). The AppImage command is `packaging/appimage.sh`. `build/` and `dist/` are generated and are not committed.

## Permissions

The user running openscan must be able to read and write the camera devices. Group `video` is enough for capture. The rule in `udev/99-openscan.rules` also creates `/dev/openscan-a` and `/dev/openscan-b`. Install steps are in the [README](../README.md).
