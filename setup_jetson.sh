#!/bin/bash
# Host bootstrap for the active VIO-first cdrone_control stack.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ARCH="$(dpkg --print-architecture)"

if [[ -r /etc/os-release ]]; then
  # shellcheck disable=SC1091
  source /etc/os-release
else
  echo "Unable to read /etc/os-release."
  exit 1
fi

if [[ "${VERSION_CODENAME:-}" != "jammy" ]]; then
  echo "This script expects Ubuntu 22.04 (jammy). Found '${VERSION_CODENAME:-unknown}'."
  exit 1
fi

require_sudo() {
  if ! sudo -n true 2>/dev/null; then
    cat <<'EOF'
This script needs sudo access for apt, rosdep init, udev rules, and GeographicLib setup.
Run it from an interactive shell where you can enter your sudo password:

  ./setup_jetson.sh

If you prefer to review before running, see:
  docs/guides/setup/host_setup.md
EOF
    exit 1
  fi
}

ensure_ros_apt_source() {
  if grep -Rqs "packages.ros.org/ros2/ubuntu" /etc/apt/sources.list /etc/apt/sources.list.d/* 2>/dev/null; then
    return
  fi

  sudo apt-get update
  sudo apt-get install -y curl gnupg2 ca-certificates lsb-release software-properties-common
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=${ARCH} signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${VERSION_CODENAME} main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
}

install_system_packages() {
  sudo apt-get update
  sudo apt-get install -y \
    git-lfs \
    python3-pip \
    python3-opencv \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-yaml \
    usbutils \
    v4l-utils \
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-mavros \
    ros-${ROS_DISTRO}-mavros-extras \
    ros-${ROS_DISTRO}-geographic-msgs \
    ros-${ROS_DISTRO}-realsense2-camera \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-imu-filter-madgwick
}

install_geographiclib() {
  if [[ -d /usr/share/GeographicLib ]]; then
    return
  fi

  local tmp_script
  tmp_script="$(mktemp)"
  curl -fsSL https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh \
    -o "${tmp_script}"
  chmod +x "${tmp_script}"
  sudo "${tmp_script}"
  rm -f "${tmp_script}"
}

install_realsense_rules() {
  local rules_dir="${SCRIPT_DIR}/librealsense/config"
  local scripts_dir="${SCRIPT_DIR}/librealsense/scripts"

  if [[ ! -d "${rules_dir}" ]]; then
    return
  fi

  sudo install -m 0644 \
    "${rules_dir}/99-realsense-libusb.rules" \
    /etc/udev/rules.d/99-realsense-libusb.rules

  local v4l2_util
  v4l2_util="$(command -v v4l2-ctl || true)"
  if [[ -n "${v4l2_util}" ]]; then
    local is_tegra=""
    local is_ipu6=""
    is_tegra="$("${v4l2_util}" --list-devices 2>/dev/null | grep tegra || true)"
    is_ipu6="$("${v4l2_util}" --list-devices 2>/dev/null | grep ipu6 || true)"

    if [[ -n "${is_tegra}" || -n "${is_ipu6}" ]]; then
      sudo install -m 0644 \
        "${rules_dir}/99-realsense-d4xx-mipi-dfu.rules" \
        /etc/udev/rules.d/99-realsense-d4xx-mipi-dfu.rules

      if [[ -f "${scripts_dir}/rs-enum.sh" ]]; then
        sudo install -m 0755 "${scripts_dir}/rs-enum.sh" /usr/local/bin/rs-enum.sh
      fi
      if [[ -f "${scripts_dir}/rs_ipu6_d457_bind.sh" ]]; then
        sudo install -m 0755 \
          "${scripts_dir}/rs_ipu6_d457_bind.sh" \
          /usr/local/bin/rs_ipu6_d457_bind.sh
      fi
    fi
  fi

  sudo udevadm control --reload-rules
  sudo udevadm trigger
}

init_rosdep() {
  if [[ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
  fi
  rosdep update
}

install_python_requirements() {
  if [[ -f "${SCRIPT_DIR}/requirements-jetson.txt" ]]; then
    python3 -m pip install --user -r "${SCRIPT_DIR}/requirements-jetson.txt"
  fi
}

install_ultralytics_runtime() {
  local torchvision_wheel="${HOME}/vision-dist/torchvision-0.20.0-cp310-cp310-linux_aarch64.whl"
  local need_ultralytics="true"
  local need_torchvision="true"

  if python3 - <<'PY' >/dev/null 2>&1
import ultralytics
import torch
import torchvision

if ultralytics.__version__ != "8.4.33":
    raise SystemExit(1)
if not torchvision.__version__.startswith("0.20.0"):
    raise SystemExit(1)
if not hasattr(torch.ops, "torchvision") or not hasattr(torch.ops.torchvision, "nms"):
    raise SystemExit(1)
PY
  then
    return
  fi

  if ! python3 - <<'PY' >/dev/null 2>&1
import torch
import torchvision
PY
  then
    cat <<'EOF'
Missing a Jetson-compatible torch/torchvision install.

This repo intentionally does not let pip resolve torch for ultralytics because
that can replace the NVIDIA CUDA-enabled build with a generic CPU wheel.
Mirror the working cdrone3 torch/torchvision environment first, then rerun:

  python3 -m pip install --user --no-deps ultralytics==8.4.33 ultralytics-thop

EOF
    exit 1
  fi

  if python3 - <<'PY' >/dev/null 2>&1
import torch
import torchvision
if torchvision.__version__.startswith("0.20.0"):
    if hasattr(torch.ops, "torchvision") and hasattr(torch.ops.torchvision, "nms"):
        raise SystemExit(0)
raise SystemExit(1)
PY
  then
    need_torchvision="false"
  fi

  if python3 - <<'PY' >/dev/null 2>&1
import ultralytics
if ultralytics.__version__ == "8.4.33":
    raise SystemExit(0)
raise SystemExit(1)
PY
  then
    need_ultralytics="false"
  fi

  if [[ "${need_torchvision}" == "true" ]]; then
    if [[ ! -f "${torchvision_wheel}" ]]; then
      cat <<EOF
Missing the Jetson-compatible torchvision wheel:

  ${torchvision_wheel}

The generic PyPI torchvision 0.20.0 wheel does not provide working custom ops
on this host. Copy the verified wheel from cdrone3, then rerun setup:

  scp jetson@192.168.0.194:${torchvision_wheel} ${HOME}/vision-dist/

EOF
      exit 1
    fi

    python3 -m pip install --user --force-reinstall --no-deps "${torchvision_wheel}"
  fi

  if [[ "${need_ultralytics}" == "true" ]]; then
    python3 -m pip install --user --no-deps ultralytics==8.4.33 ultralytics-thop
  fi
}

build_workspace() {
  set +u
  # shellcheck disable=SC1091
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
  cd "${SCRIPT_DIR}/ros2"
  rosdep install --from-paths src --ignore-src -r -y --rosdistro "${ROS_DISTRO}"
  rm -rf build install log
  colcon build
}

verify_workspace() {
  set +u
  # shellcheck disable=SC1091
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  # shellcheck disable=SC1091
  source "${SCRIPT_DIR}/ros2/install/setup.bash"
  set -u

  local missing=0
  for pkg in drone_bringup drone_control_pkg ros2_poselib; do
    if ros2 pkg list | grep -q "^${pkg}$"; then
      echo "  ✓ ${pkg}"
    else
      echo "  ✗ ${pkg}"
      missing=$((missing + 1))
    fi
  done

  if [[ ${missing} -ne 0 ]]; then
    echo "Verification failed: ${missing} package(s) missing."
    exit 1
  fi
}

verify_tracker_runtime() {
  python3 - <<'PY'
import importlib
import importlib.metadata
import sys

required_modules = {
    "cv2": "python3-opencv",
    "norfair": "norfair",
    "ultralytics": "ultralytics",
    "torchvision": "torchvision==0.20.0",
    "yaml": "PyYAML",
    "pyrealsense2": "pyrealsense2==2.57.7.10387",
}
expected_distributions = {
    "norfair": "2.3.0",
    "opencv-python": "4.13.0.92",
    "pyrealsense2": "2.57.7.10387",
    "torchvision": "0.20.0",
    "ultralytics": "8.4.33",
}

missing = []
for module_name, install_hint in required_modules.items():
    try:
        importlib.import_module(module_name)
        print(f"  ✓ {module_name}")
    except Exception as exc:
        print(
            f"  ✗ {module_name} ({type(exc).__name__}: {exc})"
            f" -> install {install_hint}"
        )
        missing.append(module_name)

version_mismatches = []
for distribution_name, expected_version in expected_distributions.items():
    try:
        installed_version = importlib.metadata.version(distribution_name)
    except importlib.metadata.PackageNotFoundError:
        version_mismatches.append(
            f"{distribution_name}=missing (expected {expected_version})"
        )
        continue
    if installed_version != expected_version:
        version_mismatches.append(
            f"{distribution_name}={installed_version} (expected {expected_version})"
        )

if missing or version_mismatches:
    print(
        "Verification failed: tracker runtime does not match the verified cdrone3 "
        "baseline. milestone3_demo.launch.py uses tracking_source_mode:=direct by "
        "default, so the host must have the expected tracker packages installed."
    )
    for mismatch in version_mismatches:
        print(f"  ! {mismatch}")
    sys.exit(1)

import torch
import torchvision

if not hasattr(torch.ops, "torchvision") or not hasattr(torch.ops.torchvision, "nms"):
    print(
        "Verification failed: torchvision is installed but its custom ops are not "
        "available. Install the Jetson-compatible wheel from ~/vision-dist."
    )
    sys.exit(1)
PY
}

main() {
  echo "=========================================="
  echo "cdrone_control Host Setup"
  echo "=========================================="
  echo "Target: ROS 2 ${ROS_DISTRO}, MAVROS, RealSense D455 bringup"
  echo ""

  require_sudo
  ensure_ros_apt_source
  install_system_packages
  install_geographiclib
  install_realsense_rules
  init_rosdep
  install_python_requirements
  install_ultralytics_runtime
  build_workspace
  verify_workspace
  verify_tracker_runtime

  cat <<'EOF'

Setup complete.

Next commands:
  source /opt/ros/humble/setup.bash
  source /home/jetson/cdrone_control/ros2/install/setup.bash
  ./scripts/check_vio_host_status.sh
  ros2 launch drone_bringup realsense_d455.launch.py
  ./scripts/check_d455_topics.sh

EOF
}

main "$@"
