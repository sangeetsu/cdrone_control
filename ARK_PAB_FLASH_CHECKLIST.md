# ARK PAB Jetson Flash Checklist

This file is a literal, follow-along checklist for **your exact case**:

- Board: **ARK PAB** carrier
- Module: **Jetson Orin Nano 8GB**
- Target: **super**
- Storage target: **NVMe**
- Goal: **flash using the current `ark_jetson_kernel` source-build flow so the ARK device-tree and bootloader pinmux/BCT are applied**

This checklist was cross-checked against the current ARK repo clone at commit `1086634` on **2026-03-25**:

- `README.md`
- `setup.sh`
- `build_kernel.sh`
- `flash.sh`
- `scripts/configure_user.sh`
- `scripts/add_wifi_network.sh`
- `scripts/share_wifi.sh`
- `scripts/stop_share_wifi.sh`
- `packaging/README.md`

Important:

- This checklist follows the **current ARK repo** flow, which downloads **L4T / JetPack `36.4.4`**.
- Your other working 4GB ARK PAB came with **`36.4.3`**. That is a different software baseline.
- If you later decide you want to match that 4GB unit exactly, make a separate `36.4.3` checklist. Do **not** silently mix the two.

---

## 0. What You Need Before You Start

Check these boxes before you do anything else.

- [ ] You have a **separate Ubuntu 22.04 host PC**. Do not try to flash the Jetson from itself.
- [ ] You have a **good micro-USB data cable**.
- [ ] You know which **micro-USB port** to use: the one **next to mini HDMI** on the ARK PAB.
- [ ] You have already backed up your current Jetson data.
- [ ] You understand that the default flash path will **wipe the Jetson NVMe install**.
- [ ] You have **sudo** on the host PC.
- [ ] The host PC has a decent amount of free space for Jetson BSP, sources, and build output.
- [ ] The Jetson is powered off.

Stop here if any box above is not true.

---

## 1. On The Host PC, Make A Fresh Workspace

Open a terminal on the **host PC** and run:

```bash
mkdir -p ~/ark_flash
cd ~/ark_flash
git clone https://github.com/ARK-Electronics/ark_jetson_kernel.git
cd ark_jetson_kernel
git rev-parse --short HEAD
```

Expected:

- [ ] `git clone` finishes without errors.
- [ ] `git rev-parse --short HEAD` prints a short commit hash.

For the repo I checked on **2026-03-25**, the short hash was:

```text
1086634
```

If you get a different hash, that is okay, but understand you are no longer following this checklist against the exact same repo revision.

---

## 2. Confirm The Host PC Is Ubuntu 22.04

Run:

```bash
lsb_release -a
```

Expected:

- [ ] `Distributor ID: Ubuntu`
- [ ] `Release: 22.04`

If it is not Ubuntu 22.04, stop and switch hosts. The ARK repo README says this flow has only been tested on Ubuntu 22.04.

---

## 3. Run ARK Setup

From the repo root on the host PC:

```bash
cd ~/ark_flash/ark_jetson_kernel
./setup.sh
```

What this script does:

- downloads Jetson Linux BSP and sample rootfs
- downloads public sources
- downloads the Bootlin toolchain
- applies NVIDIA binaries
- creates default Jetson user credentials
- creates `prebuilt/` and `source_build/`

Expected output during the run:

- [ ] `Downloading prebuilt BSP and root filesystem`
- [ ] `Untarring files, this may take some time`
- [ ] `Satisfying prerequisites`
- [ ] `Applying binaries`
- [ ] `Setting up login credentials for the Jetson`
- [ ] `Downloading Jetson sources`
- [ ] `Extracting kernel source`
- [ ] `Downloading Jetson bootlin toolchain`
- [ ] `Setup complete in ...`
- [ ] `You can now build the kernel with ./build_kernel.sh`

Expected side effects:

- [ ] `prebuilt/Linux_for_Tegra/` now exists
- [ ] `source_build/Linux_for_Tegra/` now exists

What the script bakes in:

- default username: `jetson`
- default password: `jetson`
- default hostname: `jetson`

Stop immediately if you see either of these:

- [ ] `Failed to download ...`
- [ ] `Error` that causes the script to exit

Important watch-out:

- The current `setup.sh` downloads **`36.4.4`**, but `build_kernel.sh` still references a patch file with `36.4.3` in its name. That is not automatically wrong, but if the later patch step fails, stop there instead of pushing through.

---

## 4. Build The Kernel And ARK Device Tree

From the repo root on the host PC:

```bash
cd ~/ark_flash/ark_jetson_kernel
./build_kernel.sh
```

You will get a menu.

When prompted:

```text
Please select the target platform:
Note: PAB Rev3 is not the same as PAB_V3. PAB_V3 is a separate product.
1) PAB
2) JAJ
3) PAB_V3
Enter your choice (1, 2, or 3):
```

Type:

```text
1
```

For your board, the correct choice is **`1) PAB`**.

Do **not** pick `3) PAB_V3` just because your physical board may say `Rev3`. The script explicitly says those are not the same thing.

Expected output during the build:

- [ ] `Copying ARK device tree files for PAB`
- [ ] either `Patch is already applied.` or `Patch applied successfully.`
- [ ] `Building the kernel for PAB platform`
- [ ] `Kernel build successful. Installing in-tree modules and dtbs...`
- [ ] `In-tree modules and dtbs installation successful. Installing out-of-tree modules...`
- [ ] `Fixing kernel module symlinks in rootfs`
- [ ] `Copying kernel Image to prebuilt`
- [ ] `Build complete in ...`
- [ ] `Target platform: PAB`
- [ ] `You can now flash the device with ./flash.sh`

Expected side effects:

- [ ] `source_build/LAST_BUILT_TARGET` now exists and contains `PAB`
- [ ] fresh DTBs are copied into `prebuilt/Linux_for_Tegra/rootfs/boot/`
- [ ] fresh DTBs are copied into `prebuilt/Linux_for_Tegra/kernel/dtb/`

Double-check the target file right now:

```bash
cat source_build/LAST_BUILT_TARGET
```

Expected:

```text
PAB
```

Stop immediately if you see:

- [ ] `Invalid choice. Exiting.`
- [ ] `Error: Patch cannot be applied cleanly.`
- [ ] `Kernel build failed. Exiting.`
- [ ] `Failed to install ...`

---

## 5. Optional: Preload Wi-Fi Before Flashing

If you want the Jetson to join Wi-Fi on first boot, run this **after** the build and **before** the flash.

```bash
cd ~/ark_flash/ark_jetson_kernel
./scripts/add_wifi_network.sh "YOUR_WIFI_NAME" "YOUR_WIFI_PASSWORD"
```

Expected:

- [ ] `Network configuration for SSID 'YOUR_WIFI_NAME' has been created at ...`

This modifies the prebuilt rootfs so the Jetson can connect to Wi-Fi after first boot.

If you skip this, that is okay. You can add Wi-Fi later using `nmcli` on the Jetson.

---

## 6. Optional: Generate A Reusable Flash Package

If you want a reusable package for future boards, do this now.

Generate the package:

```bash
cd ~/ark_flash/ark_jetson_kernel
./packaging/generate_flash_package.sh
```

Expected:

- [ ] an `ark-*.tar.gz` appears in the repo root
- [ ] or, if it is large, an `ark-*_split/` directory appears

Optional variants:

```bash
# SD card image package
./packaging/generate_flash_package.sh --sdcard

# Non-super package
./packaging/generate_flash_package.sh --no-super
```

You do **not** need this for the actual flash you are about to perform. This is just for later reuse.

---

## 7. Put The Jetson In Force Recovery Mode

Do this on the **Jetson hardware**.

1. [ ] Make sure the Jetson is fully powered off.
2. [ ] Connect the **micro-USB port next to mini HDMI** to the **host PC** with a data cable.
3. [ ] Hold the **Force Recovery** button.
4. [ ] While holding Force Recovery, apply power.
5. [ ] Release the button after power-on.

Now verify on the **host PC**:

```bash
lsusb | grep 0955:
```

Expected:

- [ ] You see one NVIDIA recovery device.
- [ ] Product ID is one of the values ARK waits for: `7323`, `7423`, `7523`, or `7623`.

Example:

```text
Bus 001 Device 012: ID 0955:7523 NVIDIA Corp. APX
```

If you do not see an NVIDIA APX device:

- [ ] check the cable
- [ ] check the port
- [ ] power-cycle and retry recovery mode
- [ ] do not start flashing yet

---

## 8. Flash The Jetson

This is the exact command for **your case**:

```bash
cd ~/ark_flash/ark_jetson_kernel
./flash.sh
```

Do **not** add `--no-super`.

Do **not** add `--sdcard`.

Do **not** add `--usb`.

Those are for different targets than your exact case.

You should see:

```text
=========================================
  Built target: PAB
=========================================
Flash this target? (y/N):
```

Type:

```text
y
```

Then expected output includes:

- [ ] `Waiting for device...`
- [ ] NVIDIA flash logs from `l4t_initrd_flash.sh`
- [ ] a long stream of flash progress text

Treat the flash as successful only if:

- [ ] the command completes without error
- [ ] you see `Flash complete (successful)` near the end

If the flash stops with an error, do **not** guess. Save the terminal output and stop there.

Important:

- This flash path uses `--erase-all`.
- It also flashes QSPI through `flash_t234_qspi.xml`.
- That is what you want for a proper ARK baseline, especially if the current unit may have a mismatched flash.

---

## 9. First Boot After Flash

When the flash is done:

1. [ ] Let the Jetson reboot.
2. [ ] Wait patiently. First boot can take longer than a normal boot.
3. [ ] Try one of these access methods:

### Option A: SSH over mDNS

```bash
ssh jetson@jetson.local
```

### Option B: SSH over micro-USB RNDIS

```bash
ssh jetson@192.168.55.1
```

Login credentials:

- username: `jetson`
- password: `jetson`

Expected:

- [ ] login works
- [ ] hostname is `jetson`

Verify the flash baseline:

```bash
cat /etc/nv_tegra_release
tr '\0' '\n' </proc/device-tree/model
```

Expected:

- [ ] `36.4.4` appears in the NVIDIA release string
- [ ] model mentions `ARK PAB`

---

## 10. Optional: Give The Jetson Internet Over Micro-USB

If the Jetson is connected over micro-USB and your host PC is on Wi-Fi, you can share the host internet to the Jetson.

On the **host PC**:

```bash
cd ~/ark_flash/ark_jetson_kernel
./scripts/share_wifi.sh
```

This sets up NAT from the host Wi-Fi interface to the `192.168.55.0/24` Jetson USB link.

When you are done, stop it with:

```bash
cd ~/ark_flash/ark_jetson_kernel
./scripts/stop_share_wifi.sh
```

If you used the preloaded Wi-Fi step earlier, you may not need this.

---

## 11. Optional: Add Wi-Fi After Flash Instead

If you did **not** preload Wi-Fi before flashing, do this on the Jetson after login:

```bash
sudo nmcli dev wifi connect "YOUR_WIFI_NAME" password "YOUR_WIFI_PASSWORD"
```

Expected:

- [ ] `successfully activated` or equivalent NetworkManager success output

---

## 12. Optional: Enable Super Mode And Max Clocks

On the Jetson:

```bash
sudo nvpmodel -m 2
sudo jetson_clocks
```

Use this only if you want the higher-performance mode.

Quick checks:

```bash
sudo jetson_clocks --show
sudo nvpmodel -q
```

Expected:

- [ ] `nvpmodel` shows the selected mode
- [ ] `jetson_clocks --show` prints the current clock policy

---

## 13. Very Important: Test The FC USB Correctly

Because the ARK PAB micro-USB path is muxed with the flight-controller USB path:

1. [ ] Finish any micro-USB flashing or SSH session.
2. [ ] Unplug the micro-USB cable from the Jetson.
3. [ ] Reboot the Jetson.
4. [ ] Only after that, test for the FC USB.

On the Jetson, after reboot:

```bash
cat /sys/class/usb_role/usb2-0-role-switch/role
ls /dev/ttyACM0
ls /dev/serial/by-id
dmesg | rg -i 'ttyACM|cdc_acm|ARK|PX4'
```

Expected:

- [ ] USB role shows `host`
- [ ] `/dev/ttyACM0` exists, or
- [ ] an ARK/PX4 entry exists under `/dev/serial/by-id`

If micro-USB is still plugged in, this test is invalid.

---

## 14. Restore Your Backup And VIO Stack

This is outside the ARK repo itself, but it matters for your workflow.

After the Jetson boots and you can log in:

1. [ ] Reattach your external SSD that contains `cdrone_backup`.
2. [ ] Let it auto-mount.
3. [ ] Find the mount point:

```bash
lsblk -f
```

4. [ ] Load the Docker image backup:

If `docker` is not installed yet on the freshly flashed Jetson, stop here and follow the host-restore section in your saved `recovery.md` first. Do not guess at the Docker / NVIDIA runtime setup.

```bash
sudo apt-get update
sudo apt-get install -y zstd
zstd -dc /media/jetson/backup/cdrone_backup/recovery_exports/isaac_ros_dev-aarch64_latest_20260325.tar.zst | docker load
```

5. [ ] Verify the image is present:

```bash
docker images | grep isaac_ros_dev-aarch64
```

6. [ ] Restore the repo worktree quickly:

```bash
mkdir -p ~/restore_tmp
tar -xzf /media/jetson/backup/cdrone_backup/cdrone_control_worktree_20260325.tar.gz -C ~/restore_tmp
```

7. [ ] Or, if you want git history too, restore from the bundle:

```bash
mkdir -p ~/code
cd ~/code
git clone /media/jetson/backup/cdrone_backup/recovery_exports/cdrone_control_20260325.bundle cdrone_control
```

8. [ ] Read your saved recovery guide:

```bash
less /media/jetson/backup/cdrone_backup/recovery.md
```

9. [ ] Use that recovery guide for the RealSense / VIO stack rebuild details.

If the SSD mounts somewhere other than `/media/jetson/backup`, adjust the paths above.

---

## 15. Optional: QSPI-Only Flash

Use this only if you intentionally want to flash just QSPI and then use a separately prepared NVMe.

From the repo root on the host PC:

```bash
cd prebuilt/Linux_for_Tegra/
sudo ./flash.sh --no-systemimg -c bootloader/generic/cfg/flash_t234_qspi.xml jetson-orin-nano-devkit-super nvme0n1p1
```

If that fails, the README gives this alternative:

```bash
sudo ./tools/kernel_flash/l4t_initrd_flash.sh -p "--no-systemimg -c bootloader/generic/cfg/flash_t234_qspi.xml" --network usb0 jetson-orin-nano-devkit-super nvme0n1p1
```

Do **not** use this QSPI-only path for your first recovery attempt unless you know exactly why you want it.

---

## 16. Optional: Flash To Other Targets

These are real ARK repo options, but they are **not** your exact case.

```bash
# SD card target
./flash.sh --sdcard

# USB storage target
./flash.sh --usb

# Non-super module target
./flash.sh --no-super

# Both SD card and non-super
./flash.sh --sdcard --no-super
```

Only use these if you intentionally changed hardware or module target.

---

## 17. Optional: Use ARK's Prebuilt Release Instead Of Building

If you later decide you do not want to build from source, ARK also supports a prebuilt package path.

From an Ubuntu 22.04 host:

```bash
curl -LO https://github.com/ARK-Electronics/ark_jetson_kernel/releases/latest/download/flash_from_package.sh
chmod +x flash_from_package.sh
./flash_from_package.sh
```

That is **not** the path this checklist is using. This checklist is for the source-build path because you care about the device-tree and bootloader content.

---

## 18. Optional: Publish Your Own Reusable Package

After a successful build and test, you can publish your own package.

Generate:

```bash
./packaging/generate_flash_package.sh
```

Publish:

```bash
sudo apt-get install -y gh
gh auth login
./packaging/publish_release.sh v1.0.0
```

Do this only after you have verified the package on real hardware.

---

## 19. Optional: Camera Overlay Build Flow

The ARK README includes a camera overlay workflow. It is not required for the FC USB fix, but here are the exact core commands from the README.

Build overlays:

```bash
export CROSS_COMPILE=$HOME/l4t-gcc/aarch64--glibc--stable-2022.08-1/bin/aarch64-buildroot-linux-gnu-
export KERNEL_HEADERS=$PWD/source_build/Linux_for_Tegra/source/kernel/kernel-jammy-src
cd source_build/Linux_for_Tegra/source/
make dtbs
```

Copy an overlay to the Jetson:

```bash
DTB_PATH="$PWD/source_build/Linux_for_Tegra/source/kernel-devicetree/generic-dts/dtbs/"
OVERLAY_DTB=<your_overlay>
scp $DTB_PATH/$OVERLAY_DTB jetson@192.168.55.1:~
```

Install it on the Jetson:

```bash
ssh jetson@192.168.55.1
sudo mv <your_overlay> /boot
sudo /opt/nvidia/jetson-io/config-by-hardware.py -l
sudo /opt/nvidia/jetson-io/config-by-hardware.py -n 2="Camera ARK IMX477 Single"
sudo reboot
```

Check camera sensor visibility:

```bash
nvargus_nvraw --lps
```

---

## 20. Optional: ARK Software Packages

The README points to ARK-OS as an optional next step:

```text
https://github.com/ARK-Electronics/ARK-OS
```

I did not inline ARK-OS install commands here because they live in a different repo and can drift independently from `ark_jetson_kernel`.

---

## 21. Final Sanity Checklist

Before you call this done, all of these should be true:

- [ ] `setup.sh` finished cleanly
- [ ] `build_kernel.sh` finished cleanly
- [ ] you selected `1) PAB`
- [ ] `cat source_build/LAST_BUILT_TARGET` prints `PAB`
- [ ] the Jetson entered recovery mode and showed up in `lsusb`
- [ ] `./flash.sh` completed successfully
- [ ] you logged in as `jetson`
- [ ] `cat /etc/nv_tegra_release` shows `36.4.4`
- [ ] `/proc/device-tree/model` mentions `ARK PAB`
- [ ] after unplugging micro-USB and rebooting, the FC USB test was done again

If any box above is false, stop and fix that box before moving on.
