from drone_behavior_pkg.math_utils import TrackSnapshot, compute_velocity_command, score_track


def test_score_prefers_inbound_for_equal_distance():
    inbound = TrackSnapshot(
        track_id=1,
        x_b_m=3.0,
        y_b_m=0.0,
        z_b_m=0.0,
        vx_b_mps=-1.0,
        vy_b_mps=0.0,
        vz_b_mps=0.0,
        distance_m=3.0,
        confidence=0.8,
        bbox_area_px=100.0,
        inbound=True,
        last_seen_s=0.0,
    )
    outbound = TrackSnapshot(
        track_id=2,
        x_b_m=3.0,
        y_b_m=0.0,
        z_b_m=0.0,
        vx_b_mps=1.0,
        vy_b_mps=0.0,
        vz_b_mps=0.0,
        distance_m=3.0,
        confidence=0.8,
        bbox_area_px=100.0,
        inbound=False,
        last_seen_s=0.0,
    )
    assert score_track(inbound) > score_track(outbound)


def test_min_safe_distance_blocks_forward_closing():
    track = TrackSnapshot(
        track_id=1,
        x_b_m=0.6,
        y_b_m=0.0,
        z_b_m=0.0,
        vx_b_mps=0.0,
        vy_b_mps=0.0,
        vz_b_mps=0.0,
        distance_m=0.6,
        confidence=1.0,
        bbox_area_px=0.0,
        inbound=True,
        last_seen_s=0.0,
    )
    vx, _, _, _ = compute_velocity_command(
        track=track,
        desired_distance_m=1.2,
        kp_xy=0.4,
        kp_z=0.3,
        kp_yaw=0.8,
        max_vel_xy_mps=1.5,
        max_vel_z_mps=0.8,
        max_yaw_rate_rps=0.6,
        min_safe_distance_m=0.8,
    )
    assert vx <= 0.0
