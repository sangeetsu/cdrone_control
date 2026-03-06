from drone_vision_pkg.stereo_utils import StereoDetection, match_detections_epipolar


def test_epipolar_matching_prefers_disparity_pairs():
    left = [
        StereoDetection((100, 100, 140, 140), 120.0, 120.0, 0.8, 1600.0),
        StereoDetection((200, 110, 240, 150), 220.0, 130.0, 0.7, 1600.0),
    ]
    right = [
        StereoDetection((80, 100, 120, 140), 100.0, 121.0, 0.9, 1600.0),
        StereoDetection((175, 110, 215, 150), 195.0, 129.0, 0.7, 1600.0),
    ]
    pairs = match_detections_epipolar(left, right, y_tolerance_px=5.0)
    assert pairs == [(0, 0), (1, 1)]
