# Intel RealSense D435 vs D435i vs D455

Source references:
- D435 product page: https://www.intelrealsense.com/depth-camera-d435/
- D435 Intel specs page: https://www.intel.com/content/www/us/en/products/sku/128255/intel-realsense-depth-camera-d435/specifications.html
- D435i product page: https://www.realsenseai.com/products/depth-camera-d435i/
- D435i Intel specs page: https://www.intel.com/content/www/us/en/products/sku/190004/intel-realsense-depth-camera-d435i/specifications.html
- D455 product page: https://www.intelrealsense.com/depth-camera-d455/
- D455 Intel specs page: https://www.intel.com/content/www/us/en/products/sku/205847/intel-realsense-depth-camera-d455/specifications.html
- Intel field-of-view note for D400 family: https://www.intel.com/content/www/us/en/support/articles/000030385/emerging-technologies/intel-realsense-technology.html

## Short Bottom Line

- `D435` is the simplest of the three and does not include an IMU.
- `D435i` is basically the `D435` plus an onboard IMU.
- `D455` is the more navigation-oriented model with an IMU, longer-range depth behavior, and better RGB-depth overlap.

For drones, VIO, and GPS-denied navigation:
- `D435` is the weakest fit
- `D435i` is a workable visual-inertial camera
- `D455` is usually the strongest fit of the three

## Why These Three Get Confused

These cameras belong to the same D400 family and share much of the same software ecosystem:
- `librealsense`
- `realsense-viewer`
- `realsense-ros`
- RealSense firmware tools

Because of that, they often look similar at first glance. But for robotics development, they differ in an important way:

- `D435` is mainly a depth camera
- `D435i` is a depth camera plus IMU
- `D455` is also a depth camera plus IMU, but with hardware tuned more toward navigation and longer-range perception

## Hardware Differences

### 1. IMU

This is the biggest dividing line.

`D435`
- no onboard IMU

`D435i`
- onboard IMU

`D455`
- onboard IMU

Practical effect:
- `D435` cannot serve as a self-contained visual-inertial sensor
- `D435i` and `D455` can be used in VIO / visual-inertial SLAM pipelines directly

For drone work, this is a major difference.

### 2. Depth Range Behavior

`D435`
- shorter-range depth camera
- Intel positions it around roughly `0.3 m to 3 m`

`D435i`
- same general depth-camera behavior as `D435`
- adding the IMU does not fundamentally change its stereo depth geometry

`D455`
- longer-range depth camera
- Intel positions it around roughly `0.6 m to 6 m`
- Intel states depth error below `2%` at `4 m`

Practical effect:
- `D435` and `D435i` are better when closer objects matter more
- `D455` is better when you need more stable depth farther away

For room-scale mapping and drone navigation, that usually favors `D455`.

### 3. Minimum Distance

`D435`
- better for closer work

`D435i`
- similar close-range behavior to `D435`

`D455`
- worse near-field minimum distance than the D435 family

Practical effect:
- the D435 family is nicer for close-up perception or manipulation
- the D455 is better for moderate-distance navigation tasks

### 4. Stereo Baseline and Geometry

`D435` and `D435i`
- shorter stereo baseline

`D455`
- larger stereo baseline

Practical effect:
- `D455` generally has better long-range depth precision
- `D435` / `D435i` generally do better for closer-range scenes

This is one of the central reasons developers often choose `D455` for mobile robotics and drones.

### 5. RGB Camera and FOV Match

`D435`
- color FOV is narrower than the depth FOV

`D435i`
- broadly the same RGB-depth relationship as the D435

`D455`
- RGB FOV is much better aligned with the stereo/depth FOV
- Intel also notes a global-shutter RGB sensor

Practical effect:
- `D455` is often easier when you want color and depth to correspond well
- `D435` / `D435i` can require more care when building RGB+depth perception pipelines

### 6. Physical Size

`D435`
- compact
- about `90 x 25 x 25 mm`

`D435i`
- physically similar to `D435`

`D455`
- larger
- about `124 x 26 x 29 mm`

Practical effect:
- `D435` / `D435i` are easier to fit on compact airframes
- `D455` is bulkier, but often worth it if navigation performance matters more than compactness

## Software Stack Differences

### 1. Base SDK Support

All three cameras use the same core software ecosystem:
- `librealsense`
- `realsense-viewer`
- `realsense-ros`
- firmware update tools

So basic device bring-up looks similar across all three:
- detect USB device
- verify permissions / udev rules
- check streams in `realsense-viewer` or `rs-enumerate-devices`

### 2. Available Data Streams

`D435`
- depth
- infrared
- color

`D435i`
- depth
- infrared
- color
- accelerometer
- gyroscope

`D455`
- depth
- infrared
- color
- accelerometer
- gyroscope

Practical effect:
- software written for `D435i` or `D455` may assume IMU topics exist
- that same code will not work unchanged on `D435`

### 3. Visual-Inertial Readiness

`D435`
- depth/perception camera only
- needs an external IMU if you want VIO

`D435i`
- visual-inertial camera
- much more directly usable for SLAM / VIO stacks

`D455`
- visual-inertial camera
- generally better suited than D435i for navigation-oriented perception because of longer-range geometry and better RGB-depth overlap

Practical effect:
- `D435` usually means more system integration work
- `D435i` and `D455` are much more natural fits for Isaac ROS / VIO / SLAM style stacks

### 4. Firmware and Version Sensitivity

All three can be sensitive to:
- firmware version
- `librealsense` version
- ROS wrapper version
- Jetson kernel / USB setup

But in practice:
- `D435` integrations are often simpler if you only use video/depth
- `D435i` and `D455` integrations are more likely to depend on correct IMU timing, calibration, and version compatibility

That means advanced robotics projects tend to feel more “version-sensitive” on `D435i` and `D455` than on a plain `D435`.

## Development Differences

### 1. Basic Bring-Up

For all three:
- verify USB enumeration
- install `librealsense`
- verify stream access
- optionally test through `realsense-viewer`, ROS, or GStreamer

So the early setup work is very similar.

### 2. SLAM / VIO Development

`D435`
- requires a separate IMU if you want real VIO
- requires additional camera-to-IMU integration and calibration work
- often becomes a more custom multi-sensor architecture

`D435i`
- much easier to use for VIO because the IMU is already in the camera
- often the minimum acceptable D400-family choice for drone VIO work

`D455`
- also easy to use for VIO
- tends to be a better fit for navigation and mapping because of its longer-range stereo behavior

Practical effect:
- `D435` is the hardest of the three for no-GPS drone development
- `D435i` is viable and popular
- `D455` is often the strongest choice when the vehicle size can tolerate it

### 3. Calibration Burden

`D435`
- simplest if you only want depth and RGB
- hardest if you want accurate visual-inertial state estimation, because the IMU must come from elsewhere

`D435i`
- more calibration-sensitive than D435 because you will likely use the IMU
- but easier than a D435-plus-separate-IMU setup because the camera and IMU are integrated

`D455`
- similar visual-inertial calibration concerns to D435i
- often used in pipelines that care more about extrinsics, timestamps, and IMU quality because the target application is more navigation-heavy

### 4. PX4 / Drone Integration Impact

`D435`
- poor match if your goal is an external-vision / VIO pipeline without adding more hardware

`D435i`
- strong match for PX4 external vision workflows
- gives stereo + IMU from one unit

`D455`
- also a strong match for PX4 external vision workflows
- often preferred when flight happens in larger indoor spaces or when stronger mid-range perception is useful

## Practical Fit By Use Case

### D435 Is Best When

- you want a compact depth camera
- you care about shorter-range scenes
- you do not need IMU data from the camera
- you are building a simpler perception stack

### D435i Is Best When

- you want the D435 form factor plus IMU
- you need a practical visual-inertial camera
- you are building VIO / SLAM but do not specifically need the D455's longer-range geometry

### D455 Is Best When

- you want better long-range depth behavior
- you want IMU data
- you are doing drone navigation, SLAM, or GPS-denied robotics
- you want RGB and depth FOVs to match more closely

## Comparison Table

| Area | D435 | D435i | D455 |
|---|---|---|---|
| IMU | No | Yes | Yes |
| Visual-inertial ready | No | Yes | Yes |
| Ideal depth range | Shorter | Shorter | Longer |
| Near-field performance | Better | Better | Worse |
| Long-range depth | Weaker | Weaker | Better |
| RGB-depth FOV match | Less matched | Less matched | Better matched |
| Physical size | Smaller | Smaller | Larger |
| SLAM / VIO suitability | Weak without extra IMU | Good | Very good |
| Drone / no-GPS fit | Weakest | Good | Strongest |

## Recommendation For This Project

For your Jetson + drone + no-GPS direction:

- `D435` is the least attractive of the three
- `D435i` is a real candidate
- `D455` is usually the best fit if you can accept the larger size

Why the `D455` usually wins here:
- onboard IMU
- better fit for visual-inertial pipelines
- longer useful range for navigation
- better RGB-depth alignment

## Final Takeaway

The important progression is:

- `D435`: depth camera
- `D435i`: depth camera plus IMU
- `D455`: more navigation-oriented depth camera plus IMU

All three live in the same RealSense software family, but they lead to different development paths. For simple depth perception, the `D435` can be enough. For SLAM, VIO, and GPS-denied drones, the real decision is usually between `D435i` and `D455`, and the `D455` is generally the more capable choice.
