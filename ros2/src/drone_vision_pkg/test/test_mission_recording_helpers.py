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

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_vision_pkg.realsense_tracker_node import (
    is_active_mission_state,
    led_bgr_for_color_name,
    led_color_name_for_engagement_state,
    make_depth_colormap,
)


def test_led_color_mapping_matches_milestone_states() -> None:
    assert led_color_name_for_engagement_state(state="IDLE") == "off"
    assert led_color_name_for_engagement_state(state="TAKEOFF") == "startup"
    assert led_color_name_for_engagement_state(state="SEARCH") == "search"
    assert led_color_name_for_engagement_state(state="FOLLOW") == "follow"
    assert led_color_name_for_engagement_state(state="DWELL") == "dwell"
    assert led_color_name_for_engagement_state(state="RETURN_TO_START") == "return"
    assert led_color_name_for_engagement_state(state="WAIT_TOUCHDOWN") == "landing"
    assert led_color_name_for_engagement_state(state="COMPLETE") == "complete"
    assert led_color_name_for_engagement_state(state="ABORT") == "abort"
    assert (
        led_color_name_for_engagement_state(
            state="FOLLOW",
            blocked_reason="STALE_LOCAL_POSE",
        )
        == "blocked"
    )
    assert led_color_name_for_engagement_state(
        state="SEARCH",
        estop_latched=True,
    ) == "abort"


def test_mission_state_activity_gate() -> None:
    assert not is_active_mission_state("IDLE")
    assert not is_active_mission_state("COMPLETE")
    assert not is_active_mission_state("ABORT")
    assert is_active_mission_state("SYNC_TAKEOFF_PARAM")
    assert is_active_mission_state("FOLLOW")


def test_led_bgr_for_color_name_returns_cv2_order() -> None:
    assert led_bgr_for_color_name("follow") == (0, 255, 0)
    assert led_bgr_for_color_name("abort") == (0, 0, 255)


def test_depth_colormap_has_requested_size_and_masks_invalid_depth() -> None:
    depth = np.array(
        [
            [0.0, 0.5],
            [1.0, 2.0],
        ],
        dtype=np.float32,
    )

    colored = make_depth_colormap(depth, max_depth_m=2.0, output_size=(8, 6))

    assert colored.shape == (6, 8, 3)
    assert colored.dtype == np.uint8
    assert np.all(colored[0, 0] == 0)
    assert np.any(colored[-1, -1] > 0)
