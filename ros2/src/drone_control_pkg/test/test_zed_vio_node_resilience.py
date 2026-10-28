"""ZED failure handling without opening the camera or loading a detector."""

from types import SimpleNamespace
import time

from drone_control_pkg.zed_vio_node import ZedVioNode


def test_failed_grab_backs_off_instead_of_busy_looping():
    attempts = []
    camera = SimpleNamespace(grab=lambda: attempts.append(1) or object())
    node = SimpleNamespace(camera=camera, status="", next_grab_attempt_ns=0)

    ZedVioNode.capture(node)
    assert node.status == "camera grab failed"
    assert node.next_grab_attempt_ns > time.monotonic_ns()
    ZedVioNode.capture(node)
    assert len(attempts) == 1


def test_stale_ok_status_becomes_camera_timeout():
    published = []
    logger = SimpleNamespace(info=lambda _: None)
    node = SimpleNamespace(
        status="OK",
        last_status="",
        last_accepted_pose_monotonic_ns=time.monotonic_ns() - 800_000_000,
        status_pub=SimpleNamespace(publish=published.append),
        get_logger=lambda: logger,
    )

    ZedVioNode.publish_status(node)
    assert node.status == "camera frame timeout"
    assert published[0].data == "camera frame timeout"


def test_failed_image_retrieval_does_not_forward_stale_buffers():
    node = SimpleNamespace(
        camera=SimpleNamespace(
            retrieve_image=lambda *args: object(),
            retrieve_measure=lambda *args: object(),
        ),
        color_mat=object(),
        depth_mat=object(),
        status="OK",
    )

    assert ZedVioNode.deliver_camera_frame(node, 123) is False
    assert node.status.startswith("camera image/depth retrieval failed")
