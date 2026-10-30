import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
from drone_msgs.msg import TargetTrack, TargetTrackArray
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.utilities import evaluate_parameters


LAUNCH_DIR = Path(__file__).resolve().parents[2] / "drone_bringup" / "launch"


def load_launch(name):
    spec = importlib.util.spec_from_file_location("milestone4_test_" + name, LAUNCH_DIR / name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def context_with_defaults(module):
    context = LaunchContext()
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument):
            entity.execute(context)
    return context


def executable(context, node):
    return perform_substitutions(
        context, normalize_to_list_of_substitutions(node.node_executable))


def launched_nodes(actions):
    for action in actions:
        if hasattr(action, "node_executable"):
            yield action
        yield from launched_nodes(getattr(action, "actions", []))


def test_pair_launch_isolated_identities_and_mission_arguments_reach_both_controllers():
    module = load_launch("milestone4_demo.launch.py")
    context = context_with_defaults(module)
    context.launch_configurations.update(
        launch_peer="true", drone_id="cdrone3", required_completion_count="3",
        companion_only="true", altitude_tolerance_m="0.10",
        peer_zed_mount_config="/home/jetson/cdrone4_mount.yaml", max_vel_xy_mps="0.6")
    actions = module.launch_setup(context)
    assert actions[0].__class__.__name__ == "SetEnvironmentVariable"
    actions[0].execute(context)
    assert context.environment["ROS_LOCALHOST_ONLY"] == "1"
    forwarded = dict(actions[1].launch_arguments)
    local_arguments = {key: perform_substitutions(context, normalize_to_list_of_substitutions(value))
                       for key, value in forwarded.items()}
    assert local_arguments["start_service"] == "/cdrone/cdrone3/demo/milestone3_start_local"
    dispatcher = actions[-1]
    assert executable(context, dispatcher) == "milestone4_start_dispatcher"
    parameters = evaluate_parameters(context, dispatcher._Node__parameters)[0]
    peer_arguments = json.loads(parameters["peer_arguments_json"])
    assert peer_arguments["launch_peer"] == "false"
    assert peer_arguments["start_service"] == ""
    assert peer_arguments["drone_namespace"] == "/cdrone/cdrone4"
    assert peer_arguments["mavros_namespace"] == "/cdrone/cdrone4/mavros"
    assert peer_arguments["engagement_state_topic"] == "/cdrone/cdrone4/engagement/state"
    assert peer_arguments["zed_mount_config"] == "/home/jetson/cdrone4_mount.yaml"
    assert not any("/cdrone/cdrone3/" in value for value in peer_arguments.values())
    m3 = load_launch("milestone3_demo.launch.py")
    for arguments in (local_arguments, peer_arguments):
        child = context_with_defaults(m3)
        child.launch_configurations.update(arguments)
        controller = next(node for node in launched_nodes(m3.launch_setup(child))
                          if executable(child, node) == "milestone3_demo_sequence_node")
        params = next(p for p in evaluate_parameters(child, controller._Node__parameters)
                      if isinstance(p, dict))
        assert params["required_completion_count"] == 3
        assert params["altitude_tolerance_m"] == 0.1
        assert params["companion_only"] is True
        assert params["max_vel_xy_mps"] == 0.6


@pytest.mark.parametrize("argument,value", [
    ("required_completion_count", "0"), ("required_completion_count", "-1"),
    ("required_completion_count", "1.5"), ("altitude_tolerance_m", "nan"),
    ("altitude_tolerance_m", "0"), ("altitude_tolerance_m", "2.0"),
])
def test_invalid_completion_count_and_altitude_tolerance_rejected(argument, value):
    module = load_launch("milestone4_demo.launch.py")
    context = context_with_defaults(module)
    context.launch_configurations[argument] = value
    with pytest.raises(ValueError):
        module.launch_setup(context)


def test_single_drone_keeps_existing_start_service_and_correct_led_topic():
    module = load_launch("milestone4_demo.launch.py")
    context = context_with_defaults(module)
    context.launch_configurations.update(drone_id="cdrone4", drone_namespace="/cdrone/cdrone4",
                                         mavros_namespace="/cdrone/cdrone4/mavros")
    actions = module.launch_setup(context)
    assert len(actions) == 2
    assert context.launch_configurations["start_service"] == ""
    assert context.launch_configurations["engagement_state_topic"] == "/cdrone/cdrone4/engagement/state"


def test_local_mission_requires_engine_and_defaults_to_best():
    module = load_launch("milestone4_demo.launch.py")
    path, device, task = module._tracker_backend("zed_local", "", "")
    assert path.endswith("/models/best.engine")
    assert (device, task) == ("0", "obb")
    for bad in ("models/best.pt", "models/best.onnx"):
        with pytest.raises(ValueError, match="requires a TensorRT"):
            module._tracker_backend("zed_local", bad, "")
    with pytest.raises(ValueError, match="CUDA"):
        module._tracker_backend("zed_local", path, "cpu")
    context = context_with_defaults(module)
    assert context.launch_configurations["pose_source"] == "zed_local"
    assert context.launch_configurations["enable_flight"] == "false"
    assert context.launch_configurations["tracking_metrics_write_video"] == "false"
    assert context.launch_configurations["tracker_record_mission_video"] == "false"
    assert context.launch_configurations["require_detector_target_on_start"] == "false"
    assert context.launch_configurations["tracker_enable_reid"] == "false"
    assert context.launch_configurations["tracker_enable_motion_estimator"] == "false"
    assert context.launch_configurations["zed_grab_compute_capping_fps"] == "20.0"
    assert context.launch_configurations["zed_start_delay_s"] == "15.0"
    actions = module.launch_setup(context)
    assert len(actions) == 2
    forwarded = dict(actions[0].launch_arguments)
    assert perform_substitutions(
        context, normalize_to_list_of_substitutions(
            forwarded["require_detector_target_on_start"])) == "false"
    context.launch_configurations["require_detector_target_on_start"] = "true"
    actions = module.launch_setup(context)
    forwarded = dict(actions[0].launch_arguments)
    assert perform_substitutions(
        context, normalize_to_list_of_substitutions(
            forwarded["require_detector_target_on_start"])) == "true"
    assert perform_substitutions(
        context, normalize_to_list_of_substitutions(
            forwarded["zed_grab_compute_capping_fps"])) == "20.0"
    assert perform_substitutions(
        context, normalize_to_list_of_substitutions(
            forwarded["zed_start_delay_s"])) == "15.0"
    for value in forwarded.values():
        # IncludeLaunchDescription must be able to normalize every value.
        normalize_to_list_of_substitutions(value)
    context.launch_configurations["model_path"] = "models/best.pt"
    with pytest.raises(ValueError, match="requires a TensorRT"):
        module.launch_setup(context)


def test_local_detector_scheduling_can_meet_ten_hz_3d_gate():
    import yaml

    module = load_launch("milestone4_demo.launch.py")
    context = context_with_defaults(module)
    image_rate_hz = float(context.launch_configurations["zed_image_rate_hz"])
    config = (Path(__file__).resolve().parents[2] /
              "drone_vision_pkg/config/zed2i_tracking.yaml")
    tracker_rate_hz = float(yaml.safe_load(config.read_text())[
        "realsense_tracker_node"]["ros__parameters"]["tracker_rate_hz"])
    assert image_rate_hz > 10.0
    assert tracker_rate_hz > 10.0


def test_one_terminal_diagnostics_runs_separate_processes_without_controller():
    module = load_launch("milestone4_diagnostics.launch.py")
    context = context_with_defaults(module)
    actions = module.launch_setup(context)
    names = [executable(context, action) for action in launched_nodes(actions)]
    assert names == ["zed_vio_node", "realsense_tracker_node"]
    assert [action.period for action in actions
            if action.__class__.__name__ == "TimerAction"] == [15.0, 30.0]
    assert len([action for action in actions if action.__class__.__name__ == "IncludeLaunchDescription"]) == 1
    context.launch_configurations["stage"] = "zed"
    actions = module.launch_setup(context)
    assert [executable(context, action) for action in launched_nodes(actions)] == ["zed_vio_node"]
    context.launch_configurations["stage"] = "mavros"
    actions = module.launch_setup(context)
    assert not list(launched_nodes(actions))


def test_cuda_device_zero_remains_a_string_in_both_launch_paths():
    vision_launch_dir = Path(__file__).resolve().parents[2] / "drone_vision_pkg" / "launch"
    spec = importlib.util.spec_from_file_location(
        "milestone4_test_vision_tracking", vision_launch_dir / "tracking_only.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = context_with_defaults(module)
    context.launch_configurations["inference_device"] = "0"
    nodes = [entity for entity in module.generate_launch_description().entities
             if hasattr(entity, "node_executable")]
    assert len(nodes) == 2
    for node in nodes:
        parameter_dicts = evaluate_parameters(context, node._Node__parameters)
        device = next(params["inference_device"] for params in parameter_dicts
                      if isinstance(params, dict) and "inference_device" in params)
        assert device == "0"
        assert isinstance(device, str)

    module = load_launch("milestone4_diagnostics.launch.py")
    context = context_with_defaults(module)
    tracker = next(action for action in launched_nodes(module.launch_setup(context))
                   if executable(context, action) == "realsense_tracker_node")
    parameter_dicts = evaluate_parameters(context, tracker._Node__parameters)
    device = next(params["inference_device"] for params in parameter_dicts
                  if isinstance(params, dict) and "inference_device" in params)
    assert device == "0"
    assert isinstance(device, str)


def test_zed_compute_cap_and_staged_start_flow_through_launches():
    vision_launch_dir = Path(__file__).resolve().parents[2] / "drone_vision_pkg" / "launch"
    spec = importlib.util.spec_from_file_location(
        "milestone4_test_vision_tracking_cap", vision_launch_dir / "tracking_only.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = context_with_defaults(module)
    context.launch_configurations["zed_grab_compute_capping_fps"] = "20.0"
    combined = next(entity for entity in module.generate_launch_description().entities
                    if getattr(entity, "node_executable", None) is not None
                    and executable(context, entity) == "zed_local_combined_node")
    parameter_dicts = evaluate_parameters(context, combined._Node__parameters)
    cap = next(params["grab_compute_capping_fps"] for params in parameter_dicts
               if isinstance(params, dict) and "grab_compute_capping_fps" in params)
    assert cap == 20.0
    assert any(params.get("depth_mode") == "PERFORMANCE"
               for params in parameter_dicts if isinstance(params, dict))
    assert any(params.get("zed_depth_mode") == "PERFORMANCE"
               for params in parameter_dicts if isinstance(params, dict))

    m3 = load_launch("milestone3_demo.launch.py")
    context = context_with_defaults(m3)
    context.launch_configurations["pose_source"] = "zed_local"
    context.launch_configurations["zed_start_delay_s"] = "15.0"
    actions = m3.launch_setup(context)
    assert actions[1].__class__.__name__ == "TimerAction"
    assert actions[1].period == 15.0


def test_engine_warmup_runs_inference_and_requires_obb_output():
    from drone_vision_pkg.zed_local_combined_node import warm_detector_engine

    calls = []
    model = SimpleNamespace(predict=lambda **kwargs:
                            calls.append(kwargs) or [SimpleNamespace(obb=object())])
    tracker = SimpleNamespace(
        model=model,
        model_data_config="/tmp/drone_detector.yaml",
        model_input_size=640,
        conf_threshold=0.4,
        inference_device="0",
        model_task="obb",
    )
    warm_detector_engine(tracker)
    assert calls[0]["source"].shape == (640, 640, 3)
    assert calls[0]["device"] == "0"
    assert calls[0]["data"] == "/tmp/drone_detector.yaml"

    tracker.model = SimpleNamespace(predict=lambda **kwargs:
                                    [SimpleNamespace(obb=None, boxes=object())])
    with pytest.raises(RuntimeError, match="no obb output"):
        warm_detector_engine(tracker)


def test_start_target_gate_records_only_fresh_detected_3d_tracks():
    from drone_control_pkg.milestone3_demo_sequence_node import Milestone3DemoSequenceNode

    node = SimpleNamespace(
        now_s=lambda: 100.0,
        tracks={},
        local_odom_mode=True,
        min_target_distance_m=0.8,
        max_target_distance_m=20.0,
        min_track_confidence=0.4,
        last_valid_detector_target_time_s=0.0,
        track_gc_s=5.0,
        track_timeout_s=1.0,
        active_track_id=None,
    )
    target = TargetTrack()
    target.track_id = 4
    target.source = 0
    target.x_b_m = 18.0
    target.distance_m = 18.0
    target.confidence = 0.9
    target.last_observed_age_s = 0.1
    message = TargetTrackArray()
    message.tracks = [target]
    Milestone3DemoSequenceNode.tracks_callback(node, message)
    assert node.last_valid_detector_target_time_s == 100.0
    node.last_valid_detector_target_time_s = 0.0
    target.source = 2  # predicted track is not detector validation
    Milestone3DemoSequenceNode.tracks_callback(node, message)
    assert node.last_valid_detector_target_time_s == 0.0
