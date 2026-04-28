from drone_control_pkg.mavros_velocity_node import offboard_velocity_gate_open


def test_offboard_velocity_gate_requires_armed_offboard_when_enabled():
    assert not offboard_velocity_gate_open(
        require_guided_mode=True, armed=False, mode="OFFBOARD"
    )
    assert not offboard_velocity_gate_open(
        require_guided_mode=True, armed=True, mode="AUTO.LOITER"
    )
    assert not offboard_velocity_gate_open(
        require_guided_mode=True, armed=True, mode="POSCTL"
    )
    assert offboard_velocity_gate_open(
        require_guided_mode=True, armed=True, mode="OFFBOARD"
    )


def test_offboard_velocity_gate_can_be_disabled_for_bench_use():
    assert offboard_velocity_gate_open(
        require_guided_mode=False, armed=False, mode="AUTO.LOITER"
    )
