# Recording the side-by-side clips

The comparison clips put the simulator's own top-down camera next to RViz drawing the same moment.
This document holds the recording setup that used to sit in the README.

## Why the camera half has to be rotated

The two halves have to be turned to the same angle or the comparison is worthless, and the top
camera's image axes are not the world axes the map, the path and RViz use. A 1.2 m marker was placed
at a known world point, one at a time, and located by differencing the camera frame against the frame
before it, so nothing is inferred from colour or from what looks plausible:

| marker, 3 m from the camera in | offset from image centre |
| :--- | :--- |
| world **+X** | ( −0.5, −82.1) — image up |
| world **−X** | ( −9.0, +81.5) — image down |
| world **+Y** | (−82.5, +8.0) — image left |

The camera hangs over the arena centre, which is image (240, 240). 82 px for 3 m is 27.4 px/m against
27.2 predicted from the camera height and field of view, so the measurement is sound, and the three
points agree with each other.

World +X is therefore image up and world +Y is image left, which is a quarter turn away from the
orientation the map and RViz are drawn in. Rotating the camera half 90° clockwise puts +X right and
+Y up. Checked a second way: the car sits at the known pose (8.0727, 7.5312) during the test, which
that orientation places at (361, 61), and it is measured at (359.5, 62.2) — about 2 px out.

`tools/record_run_sidebyside.py` applies the rotation; `tools/assemble_sidebyside.py` checks the
result, including the car's pose in each half.

## Environment notes

Recording Gazebo needs `render_engine:=ogre` on this machine, and the `GZ_SIM_SYSTEM_PLUGIN_PATH`
export. Both, and the top-camera spawn parameters, are recorded in
[`ALGORITHM_PLUGINS.md`](ALGORITHM_PLUGINS.md) §10.
