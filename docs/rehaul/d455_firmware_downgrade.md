# D455 Firmware Downgrade Handoff

This document is for the person who will physically take the Intel RealSense D455 and downgrade its firmware for this project.

The goal is to move the camera from the currently observed firmware version `5.16.0.1` to firmware version `5.13.0.50`, because that is the version explicitly required by the pinned Isaac ROS `release-3.2` RealSense stack used in this repo.

## Why This Downgrade Is Being Requested

This repo is trying to bring up a RealSense-based VIO path on a Jetson Orin Nano for PX4 indoor flight.

The Jetson-side investigation already confirmed all of the following:

- the D455 now enumerates on USB 3 at `5000M`
- the camera is visible inside the Isaac ROS RealSense container
- `rs-enumerate-devices` inside that container reports the camera and firmware `5.16.0.1`
- the RealSense ROS node starts, but then fails with repeated low-level control/XU errors and no usable live IR or IMU samples
- host-side kernel and V4L2 tests on the Jetson also fail at a lower level, not only inside ROS

The remaining mismatch with the known-good Isaac ROS stack is the firmware version.

## Target Version

Downgrade the camera to:

```text
5.13.0.50
```

Do not choose a random older D400 firmware. The specific target for this repo is `5.13.0.50`.

## Official Resources

Please use these sources, in this order:

1. Intel RealSense D400 firmware releases
   https://dev.realsenseai.com/docs/firmware-releases-d400

2. Intel RealSense firmware update tool documentation
   https://dev.realsenseai.com/docs/firmware-update-tool

3. NVIDIA Isaac ROS `release-3.2` RealSense setup
   https://nvidia-isaac-ros.github.io/v/release-3.2/getting_started/hardware_setup/sensors/realsense_setup.html

4. The working Jetson reference that this repo is following
   https://www.hackster.io/bandofpv/gps-denied-drone-with-nvidia-jetson-orin-nano-9f3417

What those sources say:

- Intel's D400 firmware release page lists `5.13.0.50` as a D455-supported D400 firmware release.
- Intel's firmware update tool doc says to use `rs-fw-update` with a signed firmware image file named like `Signed_Image_UVC_<firmware_version>.bin`.
- NVIDIA's Isaac ROS `release-3.2` RealSense page explicitly requires:
  - firmware `5.13.0.50`
  - `librealsense` `2.55.1`
  - `realsense-ros` `4.51.1-isaac`
- The Hackster implementation also explicitly warns that firmware `5.13.0.50` is required for the RealSense camera in that flow.

## Recommended Host Machine

Recommended by this repo team:

- use a known-good x86 Ubuntu or Windows machine for the firmware flash
- do not use this Jetson for the flash unless you already have a working RealSense firmware tool setup on it

Reason:

- this Jetson currently shows low-level RealSense/UVC issues during streaming
- the host Jetson also does not currently have `rs-fw-update` installed as a host command

That does not prove the Jetson cannot flash firmware, but using a stable host reduces risk and variables.

## Preferred Method: `rs-fw-update`

This is the cleanest method because Intel documents the command flow directly.

### 1. Download the correct firmware package

Open:

```text
https://dev.realsenseai.com/docs/firmware-releases-d400
```

Find the D400 release entry for:

```text
Version-5_13_0_50
FW Version 5.13.0.50
Supported SKU includes D455
```

Download the firmware archive for that release and extract it.

You should end up with a signed firmware image file whose name follows Intel's documented naming convention:

```text
Signed_Image_UVC_5_13_0_50.bin
```

### 2. Make sure no other app is using the camera

Intel's firmware tool documentation explicitly says not to run other RealSense applications at the same time.

So before flashing:

- close `realsense-viewer`
- close ROS nodes that use the camera
- close any custom app using the camera

### 3. Connect only the D455 you want to flash

This is a strong recommendation from this repo for safety and simplicity.

If only one RealSense camera is connected, the command is simpler and there is less risk of flashing the wrong unit.

### 4. List devices

Run:

```bash
rs-fw-update -l
```

Intel documents this as the way to list connected devices before flashing.

Record:

- model
- serial number
- current firmware version

### 5. Flash the firmware

If only one RealSense camera is connected:

```bash
rs-fw-update -f Signed_Image_UVC_5_13_0_50.bin
```

If more than one RealSense camera is connected, or if you want to be explicit:

```bash
rs-fw-update -s <serial_number> -f Signed_Image_UVC_5_13_0_50.bin
```

Intel's documentation shows both patterns.

### 6. Wait for the tool to finish

Intel's documented success flow is:

- firmware update starts
- progress reaches `100[%]`
- the device reconnects
- the tool reports that the device was successfully updated

Do not unplug the camera during the update.

### 7. Re-list the device

Run again:

```bash
rs-fw-update -l
```

Confirm the D455 now reports firmware:

```text
5.13.0.50
```

## If The Camera Enters Recovery Mode

Intel's firmware update tool documentation says recovery-mode devices may appear as a D4xx recovery device.

If that happens, Intel documents the recovery flow as:

```bash
rs-fw-update -r -f Signed_Image_UVC_5_13_0_50.bin
```

Only do this if the tool clearly shows the device in recovery mode.

## Acceptable Alternative: RealSense Viewer

Intel documents that firmware updates can also be done using the Viewer.

Resource:

```text
https://dev.realsenseai.com/docs/firmware-updates
```

If you already have a working RealSense Viewer environment, this is acceptable.

For this repo, the CLI method is still preferred because:

- Intel documents the exact commands
- it produces clearer success or failure evidence
- it makes it easier to verify the exact image file used

## What To Hand Back After The Downgrade

Please send back:

1. A photo or screenshot of the camera/tool output showing the device model and firmware `5.13.0.50`.
2. The exact host OS used for the downgrade.
3. Whether `rs-fw-update -l` succeeded before and after the flash.
4. Whether the device ever entered recovery mode.
5. The exact filename of the firmware image used.

The most useful text output would be:

```bash
rs-fw-update -l
```

after the downgrade.

## What Happens After You Return The Camera

Once the camera comes back, this repo will re-run the Jetson-side validation:

1. check USB 3 negotiation with `lsusb -t`
2. bring up the cached Isaac ROS RealSense container
3. launch `realsense2_camera`
4. verify whether live IR and IMU samples are now flowing
5. if successful, move on to Isaac VSLAM and then the PX4 bridge

## Repo Contact Notes

Current Jetson-side observations before the downgrade:

- Jetson Linux: `R36.4.4`
- Kernel: `5.15.148-tegra`
- Camera link: USB 3 / `5000M`
- RealSense host packages on the Jetson:
  - `ros-humble-librealsense2 2.56.4`
  - `ros-humble-realsense2-camera 4.56.4`

That is not the pinned Isaac ROS RealSense combination, which is why the firmware downgrade is the next controlled experiment.
