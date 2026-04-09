#!/usr/bin/env python3
"""
Diagnose mocap frame alignment.

Usage:
  1. Launch the position_hover_demo (or just the external_pose_px4_bridge)
  2. Place the drone in the OptiTrack FOV, oriented with nose pointing
     in a known direction.
  3. Run this script:
       python3 scripts/diagnose_mocap_frame.py
  4. Follow the prompts — the script will sample pose at rest, then ask
     you to move the drone in specific directions so we can see which
     axis responds.
"""

import sys
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
import math


def quat_to_euler(q):
    """Convert quaternion (x,y,z,w) to roll,pitch,yaw in degrees."""
    x, y, z, w = q
    # Roll
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    # Pitch
    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1, min(1, sinp)))
    # Yaw
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class FrameDiagNode(Node):
    def __init__(self):
        super().__init__("frame_diag")
        self.vrpn_pose = None
        self.mavros_pose = None
        self.lock = threading.Lock()

        be_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Raw VRPN output (before any conversion)
        self.create_subscription(
            PoseStamped,
            "/vrpn_mocap/RigidBody3/pose",
            self.vrpn_cb,
            be_qos,
        )
        # What MAVROS receives (after adapter+bridge)
        self.create_subscription(
            PoseStamped,
            "/mavros/vision_pose/pose",
            self.mavros_cb,
            10,
        )

    def vrpn_cb(self, msg):
        with self.lock:
            self.vrpn_pose = msg

    def mavros_cb(self, msg):
        with self.lock:
            self.mavros_pose = msg

    def snapshot(self):
        with self.lock:
            return self.vrpn_pose, self.mavros_pose


def fmt_pose(msg):
    if msg is None:
        return "  (no data)"
    p = msg.pose.position
    q = msg.pose.orientation
    r, pi, y = quat_to_euler((q.x, q.y, q.z, q.w))
    return (
        f"  pos=({p.x:+8.3f}, {p.y:+8.3f}, {p.z:+8.3f})  "
        f"rpy=({r:+6.1f}, {pi:+6.1f}, {y:+7.1f})°"
    )


def sample(node, label, n=20, rate=0.05):
    """Collect n samples and return the average VRPN and MAVROS poses."""
    vrpn_samples = []
    mavros_samples = []
    for _ in range(n):
        rclpy.spin_once(node, timeout_sec=0.1)
        v, m = node.snapshot()
        if v is not None:
            p = v.pose.position
            q = v.pose.orientation
            vrpn_samples.append((p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        if m is not None:
            p = m.pose.position
            q = m.pose.orientation
            mavros_samples.append((p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        time.sleep(rate)

    if not vrpn_samples:
        print(f"\n[{label}] WARNING: No VRPN data received!")
        return None, None
    if not mavros_samples:
        print(f"\n[{label}] WARNING: No MAVROS vision_pose data received!")

    import numpy as np
    va = np.mean(vrpn_samples, axis=0)
    ma = np.mean(mavros_samples, axis=0) if mavros_samples else None

    print(f"\n[{label}]")
    print(f"  VRPN raw  : pos=({va[0]:+8.3f}, {va[1]:+8.3f}, {va[2]:+8.3f})  "
          f"rpy=({quat_to_euler(va[3:7])[0]:+6.1f}, {quat_to_euler(va[3:7])[1]:+6.1f}, {quat_to_euler(va[3:7])[2]:+7.1f})°")
    if ma is not None:
        print(f"  MAVROS vis: pos=({ma[0]:+8.3f}, {ma[1]:+8.3f}, {ma[2]:+8.3f})  "
              f"rpy=({quat_to_euler(ma[3:7])[0]:+6.1f}, {quat_to_euler(ma[3:7])[1]:+6.1f}, {quat_to_euler(ma[3:7])[2]:+7.1f})°")
    return va, ma


def main():
    rclpy.init()
    node = FrameDiagNode()

    # Wait for data
    print("Waiting for VRPN data on /vrpn_mocap/RigidBody3/pose ...")
    for _ in range(100):
        rclpy.spin_once(node, timeout_sec=0.1)
        v, _ = node.snapshot()
        if v is not None:
            break
    else:
        print("ERROR: No VRPN data after 10s. Is the mocap stack running?")
        return

    print("\n" + "=" * 65)
    print("MOCAP FRAME DIAGNOSTIC")
    print("=" * 65)

    print("\n--- STEP 1: REST POSITION ---")
    print("Place the drone flat on the ground in the OptiTrack FOV.")
    print("Note which direction the NOSE is pointing.")
    input("Press Enter when ready...")
    rest_v, rest_m = sample(node, "REST")

    print("\n--- STEP 2: LIFT UP ---")
    print("Lift the drone straight UP by ~30cm (keep it level).")
    input("Press Enter when holding it up...")
    up_v, up_m = sample(node, "LIFTED UP")

    print("\n--- STEP 3: BACK TO REST ---")
    print("Put it back down in the same spot.")
    input("Press Enter when ready...")
    sample(node, "BACK TO REST")

    print("\n--- STEP 4: SLIDE FORWARD ---")
    print("Slide the drone ~50cm in the NOSE direction (forward).")
    input("Press Enter when in position...")
    fwd_v, fwd_m = sample(node, "SLID FORWARD (nose direction)")

    print("\n--- STEP 5: BACK TO REST ---")
    print("Put it back in the original spot.")
    input("Press Enter when ready...")
    sample(node, "BACK TO REST")

    print("\n--- STEP 6: SLIDE RIGHT ---")
    print("Slide the drone ~50cm to the RIGHT (starboard, 90° from nose).")
    input("Press Enter when in position...")
    right_v, right_m = sample(node, "SLID RIGHT")

    print("\n--- STEP 7: YAW ---")
    print("Put it back in the original spot, then rotate it 90° CLOCKWISE")
    print("(nose now points to what was the drone's right).")
    input("Press Enter when rotated...")
    yaw_v, yaw_m = sample(node, "ROTATED 90° CW")

    # Analysis
    import numpy as np
    print("\n" + "=" * 65)
    print("ANALYSIS (VRPN raw frame)")
    print("=" * 65)

    if rest_v is not None and up_v is not None:
        delta = np.array(up_v[:3]) - np.array(rest_v[:3])
        dominant = ["X", "Y", "Z"][np.argmax(np.abs(delta))]
        sign = "+" if delta[np.argmax(np.abs(delta))] > 0 else "-"
        print(f"\n  UP movement:  delta = ({delta[0]:+.3f}, {delta[1]:+.3f}, {delta[2]:+.3f})")
        print(f"    => VRPN '{sign}{dominant}' axis = physical UP")

    if rest_v is not None and fwd_v is not None:
        delta = np.array(fwd_v[:3]) - np.array(rest_v[:3])
        dominant = ["X", "Y", "Z"][np.argmax(np.abs(delta))]
        sign = "+" if delta[np.argmax(np.abs(delta))] > 0 else "-"
        print(f"\n  FWD movement: delta = ({delta[0]:+.3f}, {delta[1]:+.3f}, {delta[2]:+.3f})")
        print(f"    => VRPN '{sign}{dominant}' axis = physical FORWARD (nose)")

    if rest_v is not None and right_v is not None:
        delta = np.array(right_v[:3]) - np.array(rest_v[:3])
        dominant = ["X", "Y", "Z"][np.argmax(np.abs(delta))]
        sign = "+" if delta[np.argmax(np.abs(delta))] > 0 else "-"
        print(f"\n  RIGHT movement: delta = ({delta[0]:+.3f}, {delta[1]:+.3f}, {delta[2]:+.3f})")
        print(f"    => VRPN '{sign}{dominant}' axis = physical RIGHT")

    if rest_v is not None and yaw_v is not None:
        rest_yaw = quat_to_euler(rest_v[3:7])[2]
        yaw_yaw = quat_to_euler(yaw_v[3:7])[2]
        delta_yaw = yaw_yaw - rest_yaw
        # Normalize
        while delta_yaw > 180: delta_yaw -= 360
        while delta_yaw < -180: delta_yaw += 360
        print(f"\n  YAW 90°CW:  yaw went from {rest_yaw:+.1f}° to {yaw_yaw:+.1f}° "
              f"(delta = {delta_yaw:+.1f}°)")
        if delta_yaw < -45:
            print("    => VRPN yaw: positive = CCW (standard ROS/ENU convention)")
        elif delta_yaw > 45:
            print("    => VRPN yaw: positive = CW (NED convention, needs negation for ROS)")
        else:
            print("    => WARNING: yaw change too small, check rotation was ~90°")

    # Required transform
    print("\n" + "=" * 65)
    print("REQUIRED TRANSFORM")
    print("=" * 65)
    print("""
For MAVROS vision_pose (ROS ENU convention), we need:
  position.x = East      (some horizontal axis)
  position.y = North     (some horizontal axis)
  position.z = Up        (vertical axis, positive = up)
  yaw: positive = CCW when viewed from above

Compare the VRPN axis assignments above to determine the correct
mapping in Motive or in the adapter node.
""")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
