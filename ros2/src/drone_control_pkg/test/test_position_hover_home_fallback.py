from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from drone_control_pkg.drone_setup_node import DroneSetup  # noqa: E402
from drone_control_pkg.position_hover_demo_sequence_node import (  # noqa: E402
    PositionHoverDemoSequenceNode,
)


def make_hover_node_stub(**overrides):
    node = PositionHoverDemoSequenceNode.__new__(PositionHoverDemoSequenceNode)
    node.latest_home_position = None
    node.last_home_position_time_s = 0.0
    node.last_global_origin_time_s = 0.0
    node.last_fcu_connect_time_s = 1.0
    node.require_global_origin = True
    node.allow_synthetic_home_position = False
    node.synthetic_home_position_z_m = 0.0
    node.pose_fresh = lambda: True
    for name, value in overrides.items():
        setattr(node, name, value)
    return node


def test_home_position_requires_mavros_by_default() -> None:
    node = make_hover_node_stub()

    assert not node.home_position_ready()
    assert node.home_position_source() == "missing"
    assert node.home_position_z_m() is None


def test_synthetic_home_position_is_opt_in() -> None:
    node = make_hover_node_stub(
        allow_synthetic_home_position=True,
        synthetic_home_position_z_m=0.0,
    )

    assert node.home_position_ready()
    assert node.home_position_source() == "synthetic"
    assert node.home_position_z_m() == 0.0


def test_synthetic_home_position_does_not_depend_on_pose_freshness() -> None:
    node = make_hover_node_stub(
        allow_synthetic_home_position=True,
        pose_fresh=lambda: False,
    )

    assert node.home_position_ready()
    assert node.home_position_source() == "synthetic"


def test_global_origin_is_required_by_default() -> None:
    node = make_hover_node_stub()

    assert not node.global_origin_ready()


def test_global_origin_can_be_disabled_for_optical_flow_hover() -> None:
    node = make_hover_node_stub(require_global_origin=False)

    assert node.global_origin_ready()


def test_setup_node_can_assume_home_after_publish_when_opted_in() -> None:
    node = DroneSetup.__new__(DroneSetup)
    node.assume_home_position_after_publish = True
    node.last_connect_time_s = 10.0
    node.last_home_position_time_s = 0.0
    node.last_global_origin_time_s = 10.0
    node.connected = True
    node.publish_attempt_count = 0
    node.setup_complete_logged = False
    node.now_s = lambda: 11.0
    node.global_origin_ready = lambda: True
    node.home_position_ready = DroneSetup.home_position_ready.__get__(node)
    node.make_home_position_msg = lambda: object()
    node.set_home_pub = type(
        "Pub",
        (),
        {"publish": lambda self, msg: None},
    )()
    node.set_gp_pub = type(
        "Pub",
        (),
        {"publish": lambda self, msg: None},
    )()
    node.get_logger = lambda: type(
        "Logger",
        (),
        {"info": lambda self, msg: None},
    )()

    DroneSetup.timer_callback(node)

    assert node.home_position_ready()


def test_setup_node_can_assume_global_origin_after_publish_when_opted_in() -> None:
    node = DroneSetup.__new__(DroneSetup)
    node.assume_home_position_after_publish = False
    node.assume_global_origin_after_publish = True
    node.last_connect_time_s = 10.0
    node.last_home_position_time_s = 10.0
    node.last_global_origin_time_s = 0.0
    node.connected = True
    node.publish_attempt_count = 0
    node.setup_complete_logged = False
    node.now_s = lambda: 11.0
    node.global_origin_ready = DroneSetup.global_origin_ready.__get__(node)
    node.home_position_ready = lambda: True
    node.make_global_origin_msg = lambda: object()
    node.set_home_pub = type(
        "Pub",
        (),
        {"publish": lambda self, msg: None},
    )()
    node.set_gp_pub = type(
        "Pub",
        (),
        {"publish": lambda self, msg: None},
    )()
    node.get_logger = lambda: type(
        "Logger",
        (),
        {"info": lambda self, msg: None},
    )()

    DroneSetup.timer_callback(node)

    assert node.global_origin_ready()
