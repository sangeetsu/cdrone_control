#!/usr/bin/env python3
"""Plot an onboard-hover ULog against its initial-climb limits.

Usage: python3 scripts/plot_initial_climb_report.py FLIGHT.ulg OUTPUT.png
"""

import argparse

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from pyulog import ULog


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ulog", help="PX4 ULog from the flight")
    parser.add_argument("output", help="PNG to create")
    args = parser.parse_args()

    log = ULog(args.ulog)
    start = log.start_timestamp

    def topic(name):
        return log.get_dataset(name).data

    def seconds(data):
        return (data["timestamp"].astype(float) - start) / 1e6

    position = topic("vehicle_local_position")
    flow = topic("vehicle_optical_flow")
    attitude = topic("vehicle_attitude_setpoint")
    commands = topic("vehicle_command")
    landed = topic("vehicle_land_detected")

    flow_t = seconds(flow)
    position_t = seconds(position)
    command_t = seconds(commands)
    landed_t = seconds(landed)
    takeoff = float(landed_t[np.flatnonzero(landed["landed"] == 0)[0]])
    touchdown = float(landed_t[np.flatnonzero(
        (landed_t > takeoff) & (landed["landed"] == 1))[0]])
    land_request = float(command_t[np.flatnonzero(
        (commands["command"] == 176) & (commands["param1"] == 129))[0]])

    ground_range = float(np.median(flow["distance_m"][flow_t < takeoff]))
    local_z_start = float(np.median(position["z"][position_t < takeoff]))
    measured_climb = flow["distance_m"] - ground_range
    local_altitude = local_z_start - position["z"]
    ekf_hagl = position["dist_bottom"]

    fig, axes = plt.subplots(4, 1, figsize=(11, 11), constrained_layout=True)
    fig.suptitle("September 27 onboard hover: initial-climb abort and landing estimate", fontsize=15)

    axes[0].plot(flow_t, measured_climb, "o-", markersize=3, color="#178b89",
                 label="Downward range above launch surface (ULog 2 Hz)")
    axes[0].plot(position_t, local_altitude, color="#555a7c", linewidth=1.3,
                 label="PX4 local altitude change")
    axes[0].axhline(0.25, color="#60893c", linestyle=":", label="Initial target: 0.25 m")
    axes[0].axhline(0.40, color="#bc3347", linestyle="--", label="Old height abort: 0.40 m")
    axes[0].axhline(1.25, color="#b5791c", linestyle="--", label="New height abort: 1.25 m")
    axes[0].set_xlim(0, 6)
    axes[0].set_ylim(-0.1, 1.4)
    axes[0].set_ylabel("Height (m)")
    axes[0].legend(loc="upper left", fontsize=8)

    axes[1].plot(position_t, -position["vz"], color="#555a7c",
                 label="PX4 estimated upward speed")
    axes[1].axhline(0.6, color="#bc3347", linestyle="--", label="Old speed abort: 0.6 m/s")
    axes[1].axhline(1.0, color="#b5791c", linestyle="--", label="New speed abort: 1.0 m/s")
    axes[1].set_xlim(0, 6)
    axes[1].set_ylim(-1, 1.25)
    axes[1].set_ylabel("Upward speed (m/s)")
    axes[1].legend(loc="upper left", fontsize=8)

    axes[2].plot(seconds(attitude), -attitude["thrust_body[2]"], color="#8c5c9f",
                 label="PX4 attitude thrust target")
    axes[2].set_xlim(0, 6)
    axes[2].set_ylim(0, 0.8)
    axes[2].set_ylabel("Normalized thrust")
    axes[2].set_xlabel("Seconds after ULog start")
    axes[2].legend(loc="upper left", fontsize=8)

    axes[3].plot(flow_t, flow["distance_m"], "o-", markersize=2,
                 color="#178b89", label="Downward range")
    axes[3].plot(position_t, ekf_hagl, color="#a27530", linewidth=1.3,
                 label="PX4 estimated height above ground")
    axes[3].plot(position_t, local_altitude, color="#555a7c", linewidth=1.2,
                 label="PX4 local altitude change")
    axes[3].set_xlim(0, (log.last_timestamp - start) / 1e6)
    axes[3].set_ylim(-0.3, 7)
    axes[3].set_ylabel("Height (m)")
    axes[3].set_xlabel("Seconds after ULog start")
    axes[3].legend(loc="upper left", fontsize=8)

    for axis in axes:
        axis.axvline(land_request, color="#bc3347", linewidth=1.4)
        axis.grid(alpha=0.2)
    axes[0].annotate("AUTO.LAND requested", (land_request, 0.49),
                     xytext=(2.7, 1.05), fontsize=9,
                     arrowprops={"arrowstyle": "->", "color": "#bc3347"})
    axes[3].axvline(touchdown, color="#3d6430", linestyle=":", linewidth=1.4)
    axes[3].text(touchdown - 0.25, 6.65, "PX4 landed", ha="right", fontsize=9)
    fig.savefig(args.output, dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    main()
