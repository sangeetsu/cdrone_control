"""MAVROS boundary tests with real message types and mocked command services."""

from concurrent.futures import Future
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import math

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from mavros_msgs.msg import AttitudeTarget, EstimatorStatus, ExtendedState, OpticalFlowRad, PositionTarget, State
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu, Range
from std_srvs.srv import Trigger

from drone_control_pkg.onboard_hover_demo_sequence_node import OnboardHoverDemoSequenceNode


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class Service:
    def __init__(self, field):
        self.field = field
        self.requests = []

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.requests.append(request)
        future = Future()
        future.set_result(SimpleNamespace(**{self.field: True}))
        return future


@pytest.fixture
def node_factory():
    rclpy.init()
    nodes = []

    def create(enable_flight=False, **overrides):
        node = OnboardHoverDemoSequenceNode(parameter_overrides=[
            Parameter("enable_flight", value=enable_flight),
            Parameter("request_telemetry_streams", value=False),
            Parameter("drone_id", value="onboard_test"),
            Parameter("mavros_namespace", value="/onboard_test/mavros"),
        ] + [Parameter(name, value=value) for name, value in overrides.items()])
        node.test_time = 100.0
        node.now_s = lambda: node.test_time
        node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=int(node.test_time * 1e9),
            to_msg=lambda: timestamp(node.test_time)))
        node.position_pub = Publisher()
        node.attitude_pub = Publisher()
        node.status_pub = Publisher()
        node.diagnostics_pub = Publisher()
        node.mode_client = Service("mode_sent")
        node.arm_client = Service("success")
        nodes.append(node)
        return node

    yield create
    for node in nodes:
        node.destroy_node()
    rclpy.shutdown()


def timestamp(value):
    from builtin_interfaces.msg import Time
    nanoseconds = round(value * 1e9)
    return Time(sec=nanoseconds // 1000000000, nanosec=nanoseconds % 1000000000)


def feed(node, *, z=17.5, landed=1, aiding=True, distance=0.3, minimum_range=0.08):
    t = node.controller.telemetry
    state = State(connected=True, armed=t.armed, mode=t.mode or "POSCTL")
    state.header.stamp = timestamp(node.test_time)
    node.state_callback(state)
    pose = PoseStamped()
    pose.header.stamp = timestamp(node.test_time)
    pose.header.frame_id = "map"
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = 123.0, -456.0, z
    pose.pose.orientation.w = 1.0
    node.pose_callback(pose)
    estimator = EstimatorStatus(attitude_status_flag=True, velocity_horiz_status_flag=aiding,
                                velocity_vert_status_flag=True, pos_horiz_rel_status_flag=aiding,
                                pos_vert_abs_status_flag=True,
                                pred_pos_horiz_rel_status_flag=aiding)
    estimator.header.stamp = timestamp(node.test_time)
    node.estimator_callback(estimator)
    flow = OpticalFlowRad(integration_time_us=10000, quality=100)
    flow.header.stamp = timestamp(node.test_time)
    node.flow_callback(flow)
    range_msg = Range(range=distance, min_range=minimum_range, max_range=30.0)
    range_msg.header.stamp = timestamp(node.test_time)
    node.range_callback(range_msg)
    extended = ExtendedState(landed_state=landed)
    extended.header.stamp = timestamp(node.test_time)
    node.extended_callback(extended)
    if node.controller.settings.initial_climb_enabled:
        imu = Imu()
        imu.header.stamp = timestamp(node.test_time)
        imu.orientation.w = 1.0
        node.imu_callback(imu)


def run(node, seconds, follow=False, ground=False):
    for _ in range(round(seconds / 0.05)):
        node.test_time += 0.05
        z = node.controller.target_z if follow else 17.5
        launch_range = node.controller.ground_distance_start
        base_range = 0.3 if launch_range is None else launch_range
        feed(node, z=z or 17.5, distance=base_range + (z - 17.5 if follow else 0.0),
             minimum_range=0.05 if node.controller.settings.initial_climb_enabled else 0.08,
             landed=1 if ground or not node.controller.telemetry.armed else 2)
        node.tick()
        if node.mode_client.requests:
            node.controller.telemetry.mode = node.mode_client.requests[-1].custom_mode
        if node.arm_client.requests:
            node.controller.telemetry.armed = node.arm_client.requests[-1].value


def test_diagnostic_node_has_no_flight_output(node_factory):
    node = node_factory()
    run(node, 3.0)
    assert node.check_callback(None, Trigger.Response()).success
    assert not node.start_callback(None, Trigger.Response()).success
    run(node, 3.0)
    assert not node.position_pub.messages
    assert not node.attitude_pub.messages
    assert not node.mode_client.requests
    assert not node.arm_client.requests
    assert node.status_pub.messages
    result = node.set_parameters([Parameter("enable_flight", value=True)])
    assert not result[0].successful
    assert not node.controller.settings.enable_flight


def test_check_service_does_not_report_ready_after_abort(node_factory):
    node = node_factory(True)
    run(node, 3.0)
    assert node.check_callback(None, Trigger.Response()).success
    node.controller.transition("ABORT", node.test_time, "no measured liftoff")
    response = node.check_callback(None, Trigger.Response())
    assert not response.success
    assert "restart node after abort run" in response.message
    assert not node.start_callback(None, Trigger.Response()).success


def test_full_sequence_through_mocked_mavros_services_without_home(node_factory):
    node = node_factory(True)
    run(node, 3.0)
    assert node.start_callback(None, Trigger.Response()).success
    run(node, 8.0, follow=True)
    assert node.controller.state == "HOVER"
    assert node.position_pub.messages[-1].pose.position.z == pytest.approx(18.5)
    run(node, 21.0, follow=True)
    assert node.controller.state == "LANDING"
    assert not any(not req.value for req in node.arm_client.requests)
    run(node, 2.0, ground=True)
    assert node.controller.state == "COMPLETE"
    assert node.arm_client.requests[-1].value is False


def test_repeated_delayed_and_invalid_sensor_samples_cannot_refresh_health(node_factory):
    node = node_factory()
    run(node, 3.0)
    before = node.controller.telemetry.received["pose"]
    pose = PoseStamped()
    pose.header.stamp = timestamp(node.test_time)
    node.test_time += 1.0
    node.pose_callback(pose)
    assert node.controller.telemetry.received["pose"] == before
    assert not node.check_callback(None, Trigger.Response()).success
    pose.header.stamp = timestamp(node.test_time)
    pose.pose.orientation.w = 0.0
    node.pose_callback(pose)  # zero quaternion
    assert node.controller.telemetry.pose is None


def test_insufficient_estimator_flags_are_not_mistaken_for_valid_pose(node_factory):
    node = node_factory()
    run(node, 3.0)
    node.test_time += 0.05
    estimator = EstimatorStatus(attitude_status_flag=True, pos_vert_abs_status_flag=True,
                                const_pos_mode_status_flag=True)
    estimator.header.stamp = timestamp(node.test_time)
    node.estimator_callback(estimator)
    result = node.check_callback(None, Trigger.Response())
    assert not result.success
    assert "horizontal velocity invalid" in result.message
    assert "constant-position" in result.message


def test_bad_flow_and_range_are_reported_independently(node_factory):
    node = node_factory()
    run(node, 3.0)
    node.test_time += 0.05
    flow = OpticalFlowRad(quality=0, integration_time_us=0)
    flow.header.stamp = timestamp(node.test_time)
    node.flow_callback(flow)
    distance = Range(range=float("nan"), min_range=0.08, max_range=30.0)
    distance.header.stamp = timestamp(node.test_time)
    node.range_callback(distance)
    result = node.check_callback(None, Trigger.Response())
    assert not result.success
    assert "flow:" in result.message and "range:" in result.message
    assert "integration_time_us=0" in result.message
    assert "quality=0 below minimum=1" in result.message
    assert "non-finite distance" in result.message
    assert node.controller.telemetry.estimator_ok


def test_ark_flow_optional_nan_gyros_do_not_invalidate_pixel_flow(node_factory):
    node = node_factory()
    run(node, 3.0)
    node.test_time += 0.05
    flow = OpticalFlowRad(integration_time_us=15872, quality=122,
                          integrated_x=0.0, integrated_y=0.0,
                          integrated_xgyro=float("nan"), integrated_ygyro=float("nan"),
                          integrated_zgyro=float("nan"))
    flow.header.stamp = timestamp(node.test_time)
    node.flow_callback(flow)
    assert node.controller.telemetry.flow_ok
    assert node.check_callback(None, Trigger.Response()).success
    node.test_time += 0.05
    flow.header.stamp = timestamp(node.test_time)
    flow.integrated_x = float("nan")
    node.flow_callback(flow)
    assert not node.controller.telemetry.flow_ok
    result = node.check_callback(None, Trigger.Response())
    assert "non-finite integrated pixel flow" in result.message


@pytest.mark.parametrize("constant_position", [False, True])
@pytest.mark.parametrize("aiding", [False, True])
def test_px4_rest_flag_requires_separate_horizontal_aiding_check(
        node_factory, constant_position, aiding):
    node = node_factory()
    run(node, 3.0)
    node.test_time += 0.05
    estimator = EstimatorStatus(attitude_status_flag=True, velocity_horiz_status_flag=True,
                                velocity_vert_status_flag=True, pos_horiz_rel_status_flag=True,
                                pos_vert_abs_status_flag=True,
                                const_pos_mode_status_flag=constant_position,
                                pred_pos_horiz_rel_status_flag=aiding)
    estimator.header.stamp = timestamp(node.test_time)
    node.estimator_callback(estimator)
    assert node.controller.telemetry.estimator_ok == aiding
    result = node.check_callback(None, Trigger.Response())
    assert result.success == aiding
    assert ("horizontal aiding inactive" in result.message) == (not aiding)
    assert "constant-position fallback active" not in result.message


@pytest.mark.parametrize("distance, reported_min, valid", [
    (0.049, 0.05, False), (0.05, 0.05, True), (0.075, 0.05, True),
    (0.072, 0.08, False), (0.08, 0.08, True), (0.2, 0.08, True), (0.2, 0.3, False),
])
def test_range_respects_configured_and_message_limits(node_factory, distance, reported_min, valid):
    node = node_factory()
    run(node, 3.0)
    node.test_time += 0.05
    msg = Range(range=distance, min_range=reported_min, max_range=30.0)
    msg.header.stamp = timestamp(node.test_time)
    node.range_callback(msg)
    assert node.controller.telemetry.range_ok == valid
    result = node.check_callback(None, Trigger.Response())
    assert result.success == valid
    if not valid:
        assert f"distance={distance:.3f} m outside usable [{reported_min:.3f}, 30.000] m" in (
            result.message)


def test_range_topic_default_and_override_and_missing_data_diagnostic(node_factory):
    node = node_factory()
    assert node.range_topic == "/onboard_test/mavros/px4flow/ground_distance"
    result = node.check_callback(None, Trigger.Response())
    assert "range: stale or missing" in result.message
    assert f"range_topic={node.range_topic}" in result.message
    assert "range: invalid" not in result.message
    alternate = node_factory(range_topic="/verified_downward_range")
    assert alternate.range_topic == "/verified_downward_range"
    topics = {subscription.topic_name for subscription in alternate.subscriptions}
    assert "/verified_downward_range" in topics


def launch_module():
    path = (Path(__file__).resolve().parents[2]
            / "drone_bringup/launch/position_hover_demo.launch.py")
    spec = importlib.util.spec_from_file_location("hover_launch_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize("source, executable, altitude", [
    ("onboard", "onboard_hover_demo_sequence_node", "1.0"),
    ("external_pose", "position_hover_demo_sequence_node", "0.7"),
])
def test_launch_selects_only_one_hover_stack(source, executable, altitude):
    module = launch_module()
    context = LaunchContext()
    context.launch_configurations["localization_source"] = source
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    nodes = module.launch_setup(context)
    assert len(nodes) == 2
    assert perform_substitutions(context, normalize_to_list_of_substitutions(
        nodes[1].node_executable)) == executable
    assert context.launch_configurations["takeoff_altitude_m"] == altitude
    assert context.launch_configurations["enable_flight"] == "false"
    nodes[0].launch_description_source.get_launch_description(context)
    location = str(nodes[0].launch_description_source.location)
    if source == "onboard":
        assert "external_pose" not in location
        assert "drone.launch.py" in location
        assert context.launch_configurations["range_topic"] == ""
        assert context.launch_configurations["initial_climb_enabled"] == "false"
        assert context.launch_configurations["initial_climb_hover_thrust"] == "0.0"
        assert context.launch_configurations["initial_climb_max_height_m"] == "1.25"
        assert context.launch_configurations["initial_climb_max_speed_m_s"] == "1.0"
    else:
        assert "external_pose_px4_bridge.launch.py" in location


def initial_feed(node, height=0.0, aiding=False):
    node.test_time += 0.05
    feed(node, z=17.5 + height, landed=2 if height > 0.03 else 1,
         aiding=aiding, distance=0.075 + height, minimum_range=0.05)


def initial_step(node, height=0.0, aiding=False):
    initial_feed(node, height, aiding)
    attitudes, positions = len(node.attitude_pub.messages), len(node.position_pub.messages)
    node.tick()
    assert not (len(node.attitude_pub.messages) > attitudes and
                len(node.position_pub.messages) > positions)
    if node.mode_client.requests:
        node.controller.telemetry.mode = node.mode_client.requests[-1].custom_mode
    if node.arm_client.requests:
        node.controller.telemetry.armed = node.arm_client.requests[-1].value


def initial_ready(node_factory, *, enabled=True):
    node = node_factory(enabled, initial_climb_enabled=True, initial_climb_hover_thrust=0.62)
    for _ in range(50):
        initial_step(node)
    assert node.check_callback(None, Trigger.Response()).success
    assert not node.controller.telemetry.estimator_ok
    return node


def test_initial_diagnostics_and_start_require_flight_gate(node_factory):
    node = initial_ready(node_factory, enabled=False)
    assert not node.start_callback(None, Trigger.Response()).success
    assert not node.attitude_pub.messages and not node.position_pub.messages
    assert not node.arm_client.requests and not node.mode_client.requests


def test_initial_sequence_switches_from_attitude_to_position_only_once(node_factory):
    node = initial_ready(node_factory)
    assert node.start_callback(None, Trigger.Response()).success
    height = 0.0
    while node.controller.state != "POSITION_HANDOVER":
        assert node.test_time < 115.0, node.controller.reason
        if node.controller.state == "INITIAL_CLIMB":
            height = min(0.25, height + 0.01)
        initial_step(node, height, aiding=height > 0.12)
    assert node.position_pub.messages
    assert node.attitude_pub.messages
    attitudes = len(node.attitude_pub.messages)
    for msg in node.attitude_pub.messages:
        assert msg.type_mask == (AttitudeTarget.IGNORE_ROLL_RATE |
                                 AttitudeTarget.IGNORE_PITCH_RATE | AttitudeTarget.IGNORE_YAW_RATE)
        assert msg.orientation.w == 1 and msg.orientation.x == msg.orientation.y == 0
        assert 0 <= msg.thrust <= 0.75
    assert node.attitude_pub.messages[0].thrust == 0
    assert node.attitude_pub.messages[-1].thrust > 0
    initial_feed(node, height, aiding=True)
    target = PositionTarget(coordinate_frame=PositionTarget.FRAME_LOCAL_NED)
    target.header.stamp = timestamp(node.test_time)
    target.position.x, target.position.y, target.position.z = (
        *node.controller.origin[:2], node.controller.target_z)
    target.yaw = node.controller.origin[3]
    node.position_target_callback(target)
    node.tick()
    assert node.controller.state == "TAKEOFF"
    run(node, 7, follow=True)
    assert node.controller.state == "HOVER"
    assert len(node.attitude_pub.messages) == attitudes
    assert node.position_pub.messages[-1].pose.position.z == pytest.approx(18.5, abs=0.04)
    # Operator abort still uses landing, never a thrust cut or forced disarm.
    assert node.abort_callback(None, Trigger.Response()).success
    run(node, 0.1, follow=True)
    assert node.mode_client.requests[-1].custom_mode == "AUTO.LAND"
    assert all(req.value for req in node.arm_client.requests)
    assert len(node.attitude_pub.messages) == attitudes


@pytest.mark.parametrize("invalid", ["nan", "zero", "unavailable", "tilt"])
def test_initial_imu_health_is_independent_of_pose_quaternion(node_factory, invalid):
    node = initial_ready(node_factory)
    initial_feed(node)
    msg = Imu()
    msg.header.stamp = timestamp(node.test_time + 0.01)
    msg.orientation.w = 1.0
    if invalid == "nan":
        msg.orientation.x = math.nan
    elif invalid == "zero":
        msg.orientation.w = 0.0
    elif invalid == "unavailable":
        msg.orientation_covariance[0] = -1.0
    else:
        msg.orientation.x, msg.orientation.w = math.sin(0.3 / 2), math.cos(0.3 / 2)
    node.test_time += 0.01
    node.imu_callback(msg)
    assert not node.check_callback(None, Trigger.Response()).success
    assert node.controller.telemetry.pose is not None


@pytest.mark.parametrize("mask, frame", [
    (PositionTarget.IGNORE_PX, PositionTarget.FRAME_LOCAL_NED),
    (PositionTarget.IGNORE_YAW, PositionTarget.FRAME_LOCAL_NED),
    (0, PositionTarget.FRAME_BODY_NED),
])
def test_handover_echo_rejects_ignored_axes_or_wrong_frame(node_factory, mask, frame):
    node = initial_ready(node_factory)
    msg = PositionTarget(type_mask=mask, coordinate_frame=frame)
    msg.header.stamp = timestamp(node.test_time)
    node.position_target_callback(msg)
    assert node.controller.telemetry.position_target is None


def test_launch_forwards_typed_initial_climb_options(monkeypatch):
    from launch_ros.utilities import evaluate_parameters
    module = launch_module()
    context = LaunchContext()
    context.launch_configurations.update({
        "localization_source": "onboard", "initial_climb_enabled": "true",
        "initial_climb_hover_thrust": "0.62", "initial_climb_height_m": "0.3",
        "initial_climb_max_height_m": "1.1", "initial_climb_max_speed_m_s": "1.2",
    })
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    captured = {}
    def capture_node(**kwargs):
        captured.update(kwargs)
        return kwargs
    monkeypatch.setattr(module, "Node", capture_node)
    module.launch_setup(context)
    # Normalize just as launch_ros does before evaluating node parameters.
    from launch_ros.utilities import normalize_parameters
    params = evaluate_parameters(context, normalize_parameters(captured["parameters"]))[0]
    assert params["initial_climb_enabled"] is True
    assert params["enable_flight"] is False
    assert params["initial_climb_hover_thrust"] == 0.62
    assert params["initial_climb_height_m"] == 0.3
    assert params["initial_climb_max_height_m"] == 1.1
    assert params["initial_climb_max_speed_m_s"] == 1.2


def test_zed_transport_reads_fresh_pose_without_flow_or_range(node_factory):
    node = node_factory(localization_source="zed")
    topics = {subscription.topic_name for subscription in node.subscriptions}
    assert node.vision_topic in topics
    assert node.flow_topic not in topics and node.range_topic not in topics
    for _ in range(51):
        node.test_time += 0.05
        feed(node)
        node.controller.telemetry.vision_profile_ok = True
        vision = PoseWithCovarianceStamped()
        vision.header.stamp = timestamp(node.test_time)
        vision.header.frame_id = "zed_odom"
        vision.pose.pose.orientation.w = 1.0
        node.vision_pose_callback(vision)
        node.tick()
    assert node.check_callback(None, Trigger.Response()).success
    assert node.controller.telemetry.vision_frame == "zed_odom"
    previous = node.controller.telemetry.received["vision"]
    node.test_time += 0.05
    node.vision_pose_callback(vision)
    assert node.controller.telemetry.received["vision"] == previous
    vision.header.stamp = timestamp(node.test_time)
    vision.pose.covariance[0] = math.nan
    node.vision_pose_callback(vision)
    assert node.controller.telemetry.vision_pose is None
    assert not node.check_callback(None, Trigger.Response()).success


def test_zed_estimator_keeps_valid_states_during_predictive_flag_dip(node_factory):
    node = node_factory(localization_source="zed")
    estimator = EstimatorStatus(
        attitude_status_flag=True, velocity_horiz_status_flag=True,
        velocity_vert_status_flag=True, pos_horiz_rel_status_flag=True,
        pos_vert_abs_status_flag=True, pred_pos_horiz_rel_status_flag=False)
    estimator.header.stamp = timestamp(node.test_time)
    node.estimator_callback(estimator)
    telemetry = node.controller.telemetry
    assert telemetry.estimator_ok
    assert not telemetry.relative_position_predicted
    assert telemetry.relative_position_prediction_lost_since == node.test_time


def test_zed_launch_selects_camera_and_vision_overlay():
    module = launch_module()
    context = LaunchContext()
    context.launch_configurations["localization_source"] = "zed"
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    nodes = module.launch_setup(context)
    assert len(nodes) == 3
    executables = [perform_substitutions(context, normalize_to_list_of_substitutions(
        node.node_executable)) for node in nodes[1:]]
    assert executables == ["zed_vio_node", "onboard_hover_demo_sequence_node"]
    assert context.launch_configurations["takeoff_altitude_m"] == "1.0"
    assert context.launch_configurations["zed_takeoff_setpoint_lead_m"] == "0.6"
    assert context.launch_configurations["zed_takeoff_no_lift_timeout_s"] == "8.0"
    assert "zed_mavros.yaml" in context.launch_configurations["zed_mavros_config"]


def test_zed_takeoff_limits_reach_controller_as_floats(monkeypatch):
    from launch_ros.utilities import evaluate_parameters, normalize_parameters
    module = launch_module()
    context = LaunchContext()
    context.launch_configurations.update({
        "localization_source": "zed",
        "zed_takeoff_setpoint_lead_m": "0.55",
        "zed_takeoff_no_lift_timeout_s": "9.0",
    })
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    captured = []
    monkeypatch.setattr(module, "Node", lambda **kwargs: captured.append(kwargs) or kwargs)
    module.launch_setup(context)
    params = evaluate_parameters(context, normalize_parameters(captured[0]["parameters"]))[0]
    assert params["zed_takeoff_setpoint_lead_m"] == 0.55
    assert params["zed_takeoff_no_lift_timeout_s"] == 9.0


def test_zed_readiness_requires_matching_fcu_vision_parameters(node_factory):
    node = node_factory(localization_source="zed")
    node.controller.telemetry.connected = True

    class ProfileService:
        def __init__(self, values):
            self.values = values

        def service_is_ready(self):
            return True

        def call_async(self, request):
            future = Future()
            values = []
            for name in request.names:
                value = self.values[name]
                values.append(SimpleNamespace(
                    type=3 if isinstance(value, float) else 2,
                    integer_value=int(value), double_value=float(value)))
            future.set_result(SimpleNamespace(values=values))
            return future

    node.profile_client = ProfileService(dict(node.ZED_PROFILE))
    node.service_vision_profile(node.test_time)
    node.service_vision_profile(node.test_time)
    assert node.controller.telemetry.vision_profile_ok
    node.controller.telemetry.vision_profile_ok = False
    node.profile_client = ProfileService(dict(node.ZED_PROFILE, EKF2_EV_CTRL=0))
    node.service_vision_profile(node.test_time)
    node.service_vision_profile(node.test_time)
    assert not node.controller.telemetry.vision_profile_ok
    assert "EKF2_EV_CTRL" in node.controller.telemetry.vision_profile_reason
