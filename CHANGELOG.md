# Changelog

Versions follow [semantic versioning](https://semver.org). The number in the window title, `fox3d --version`, and the git tag `vX.Y.Z` are the same version. Nothing before 0.5.0 was tagged.

## 0.6.1

- Mold and Measured are separate scan modes, chosen on the toolbar. Mold is the solid body and is the default. Measured is the stripe surface facing the camera.
- Measured no longer spins each new frame around the object. That pile of sheets was the mess when the model was rotated. Mold still turns as one solid.

## 0.6.0

- The live surface is the projector's light planes from the factory calibration. Each stripe is intersected with its plane. The outline is no longer revolved into a cylinder.
- Distance is the range used to identify the stripes. The mesh is the measured surface, in millimetres. Reset and scan again after changing it.

## 0.5.0

- The pictures are undistorted with the factory calibration. The side panels are the rectified stereo pair, so a feature sits on the same row in both cameras.
- The 3D view uses the camera's gray instead of a tinted clay color, so the texture can be compared with the picture.
- Distance scales the model on screen and in the exported file. Settings for exposure, gain, and distance stay open and are saved.
- Turn tracking follows the face both ways. A frame that does not match is dropped.

## Earlier development

The commits before this version added the USB cameras, the live window, silhouette meshing, turn tracking, factory calibration download, STL/OBJ/PLY export, and the CMake build. See `git log` for that history.
