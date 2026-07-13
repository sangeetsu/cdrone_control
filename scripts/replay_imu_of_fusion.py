#!/usr/bin/env python3
from __future__ import annotations

from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
PKG_SRC = REPO_ROOT / "ros2" / "src" / "drone_control_pkg"
if str(PKG_SRC) not in sys.path:
    sys.path.insert(0, str(PKG_SRC))

from drone_control_pkg.imu_of_replay import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main())
