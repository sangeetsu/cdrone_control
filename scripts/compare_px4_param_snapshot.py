#!/usr/bin/env python3
"""Compare a QGroundControl .params snapshot with PX4 ULog initial parameters.

Usage: python3 scripts/compare_px4_param_snapshot.py SNAPSHOT.params FLIGHT.ulg OUT.csv
"""

import argparse
import csv
import math
from pathlib import Path

from pyulog import ULog


def snapshot_parameters(path):
    values = {}
    for line in Path(path).read_text().splitlines():
        fields = line.split("\t")
        if len(fields) >= 5 and not line.startswith("#"):
            values[fields[2]] = float(fields[3])
    return values


def category(name):
    if name.startswith("BAT"):
        return "Battery"
    if name.startswith("CAL_") or name.startswith("SENS_BOARD"):
        return "Calibration"
    if name.startswith(("EKF2_", "SENS_FLOW", "UAVCAN_")):
        return "Estimator / sensors"
    if name.startswith(("MPC_", "MIS_")):
        return "Motion"
    if name.startswith(("COM_", "NAV_", "GF_", "CP_", "GPS_", "SER_GPS")):
        return "Commander / navigation"
    return "Other"


def equal(a, b):
    return a == b or (math.isnan(a) and math.isnan(b))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("snapshot")
    parser.add_argument("ulog")
    parser.add_argument("output")
    args = parser.parse_args()

    snapshot = snapshot_parameters(args.snapshot)
    flight = ULog(args.ulog).initial_parameters
    rows = []
    for name in sorted(snapshot.keys() | flight.keys()):
        reference = snapshot.get(name)
        recorded = float(flight[name]) if name in flight else None
        if reference is not None and recorded is not None:
            if equal(reference, recorded):
                continue
            status = ("precision-only" if math.isclose(reference, recorded,
                                                       rel_tol=1e-6, abs_tol=1e-6)
                      else "value changed")
        else:
            status = "snapshot only" if reference is not None else "flight only"
        rows.append((category(name), name,
                     "" if reference is None else format(reference, ".17g"),
                     "" if recorded is None else format(recorded, ".17g"), status))
    rows.sort(key=lambda row: (row[0], row[1]))

    with Path(args.output).open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(("Category", "Parameter", "Snapshot", "Flight ULog", "Comparison"))
        writer.writerows(rows)
    print(f"snapshot={len(snapshot)} flight={len(flight)} common={len(snapshot.keys() & flight.keys())}")
    for status in ("value changed", "precision-only", "snapshot only", "flight only"):
        print(f"{status}: {sum(row[-1] == status for row in rows)}")


if __name__ == "__main__":
    main()
