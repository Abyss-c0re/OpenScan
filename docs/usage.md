# Using fox3d

fox3d scans one object. You can turn the object, or walk the scanner around it, move closer, and come in from above. The window starts idle so you can frame the object before anything is kept.

## A scan

1. Plug in the scanner. `./build/fox3d devices` should list camera A and camera B and the serial.
2. Start `./build/fox3d`.
3. Set the object inside the green outline. The right-hand view is the 3D model, shaded with the same gray as the camera. The side panels are the rectified stereo pair: lens distortion is removed, and a feature is on the same row in both.
4. Press **Start scan**. Move so each new view still sees part of the object you already have.
5. Press **Stop** when that pass is done. The model stays.
6. Press **Export…** and choose STL, OBJ, or PLY.

**Reset** throws the model away and returns to idle. Use it when the next Start should be a different object or a clean pass. **Pause** keeps the model and keeps following the turn, so Start can continue the same scan.

## What the 3D view means

- The faceted surface is the part that has been scanned.
- The smooth dark shell is the part that has not been scanned yet.
- The ring and the status line count both: `scanned N°` and `not scanned M°`. The first view is the front of the object, about 120°.
- **detail xN** is how many times the scanned surface has been observed. The first pass is x1. Turning back over the same side raises it. The new view is blended into those points, and gaps in that area gain triangles.

Only the segmented object is kept. A frame that does not match that point cloud is dropped, so the table and the room are not scanned. The status line says **TRACKING LOST**. Come back to the object more slowly. The model already built is left as it is.

Drag the 3D view to orbit it. The wheel zooms.

## Settings

The camera and size controls are open under the toolbar when the app starts. **Settings** hides or shows that row. The values are saved and used the next time the app starts.

| Setting | What to do |
| --- | --- |
| Camera A exposure | Raise it when A is too dark to see the object. It is in steps of 0.1 ms. |
| Camera A gain | Leave this low. High gain washes out the projector dots. |
| Camera B exposure | Lower it when the clean camera is white. |
| Camera B gain | Usually a little lower than camera A. |
| Mold / Measured | Mold is the solid body, and the mode that opens. Measured is the stripe surface facing the camera. Switching clears the model. |
| Distance | How far the scanner is from the object, 100–500 mm. Mold uses it as the size of the solid. Measured uses it to identify the stripes. Reset and scan again after changing it. The Fox is meant to work around 200–400 mm. |
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
./build/fox3d scan --no-window -o model.obj --seconds 20 --no-ae
```

`--no-ae` keeps the exposure you set. Without it, scan can change exposure while running.

## Calibration

The factory file for a unit is `calib/<serial>.txt` next to the sources, or a file you pass with `--calib`. If fox3d cannot find one, it downloads that unit's file into `~/.local/share/fox3d/calib/`.

That download is the only network use. The calibration file is never sent anywhere. A mesh export does not contact the network.

## Lights and exposure

The Fox is a near-infrared stereo scanner. Camera A shows the projector dots. Camera B is the cleaner picture used to follow the object.

Both cameras share one USB 2 hub, so capture is MJPEG at 1280×720, 10 frames per second. Two uncompressed 720p streams do not fit.

Exposure is in units of 100 microseconds, after manual exposure mode is selected. Gain runs from 0 to 100. On the Fox used while writing this program, camera A at exposure 22 and gain 6, and camera B at exposure 16 and gain 4, showed the object without clipping it white.

## Permissions

The user running fox3d must be able to read and write the camera devices. Group `video` is enough for capture. The rule in `udev/99-fox3d.rules` also creates `/dev/fox3d-a` and `/dev/fox3d-b`. Install steps are in the [README](../README.md).
