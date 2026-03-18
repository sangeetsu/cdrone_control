# Calibration Files

Place the stereo calibration file for the two IMX219 cameras here:

- `stereo_calibration.npz`

The new in-repo VIO launch defaults to:

- `/home/jetson/cdrone_control/calibration/stereo_calibration.npz`

Expected arrays inside the `.npz` file:

- `P1`
- `P2`
- `map1_x`
- `map1_y`
- `map2_x`
- `map2_y`

Until this file exists, `stereo_vio_node` will refuse to start.
