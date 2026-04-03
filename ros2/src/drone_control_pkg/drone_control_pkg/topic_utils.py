from __future__ import annotations


def normalize_ns(namespace: str) -> str:
    namespace = str(namespace or "").strip()
    if not namespace:
        return ""
    if not namespace.startswith("/"):
        namespace = "/" + namespace
    return namespace.rstrip("/")


def join_topic(namespace: str, leaf: str) -> str:
    namespace = normalize_ns(namespace)
    leaf = "/" + str(leaf or "").strip().lstrip("/")
    return f"{namespace}{leaf}" if namespace else leaf


def cdrone_ns(drone_id: str) -> str:
    drone_id = str(drone_id or "").strip().strip("/")
    if not drone_id:
        return "/cdrone"
    return f"/cdrone/{drone_id}"


def cdrone_topic(drone_id: str, leaf: str) -> str:
    return join_topic(cdrone_ns(drone_id), leaf)


def external_pose_input_topic(drone_id: str) -> str:
    return cdrone_topic(drone_id, "external_pose/input_pose")


def legacy_vio_input_topic(drone_id: str) -> str:
    return cdrone_topic(drone_id, "vio/input_pose")
