# Copyright 2026 cdrone_control contributors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_vision_pkg.realsense_tracker_node import (
    normalize_zed_enum_name,
    normalize_zed_flip_mode,
    normalize_zed_image_view,
    zed_timestamp_key,
)


def test_normalize_zed_enum_name_accepts_case_insensitive_values() -> None:
    assert (
        normalize_zed_enum_name(
            "hd720",
            parameter_name="zed_camera_resolution",
            allowed={"HD720", "HD1080"},
        )
        == "HD720"
    )


def test_normalize_zed_enum_name_rejects_unknown_values() -> None:
    with pytest.raises(ValueError, match="zed_depth_mode"):
        normalize_zed_enum_name(
            "fastish",
            parameter_name="zed_depth_mode",
            allowed={"NONE", "PERFORMANCE"},
        )


def test_normalize_zed_flip_mode_accepts_bool_and_aliases() -> None:
    assert normalize_zed_flip_mode(True) == "ON"
    assert normalize_zed_flip_mode(False) == "OFF"
    assert normalize_zed_flip_mode("auto") == "AUTO"
    assert normalize_zed_flip_mode("0") == "OFF"


def test_normalize_zed_image_view_maps_operator_friendly_names() -> None:
    assert normalize_zed_image_view("left") == "LEFT_BGR"
    assert normalize_zed_image_view("right_color") == "RIGHT_BGR"


def test_zed_timestamp_key_splits_nanoseconds() -> None:
    assert zed_timestamp_key(1_234_567_890) == (1, 234_567_890)
