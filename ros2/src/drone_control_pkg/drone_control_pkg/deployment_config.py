from __future__ import annotations

from functools import lru_cache
from pathlib import Path
from typing import Any

from ament_index_python.packages import get_package_share_directory
import yaml

_DRONE_BRINGUP_PACKAGE = "drone_bringup"
_SHARED_DEFAULTS_FILE = "optitrack_defaults.yaml"
_DRONE_ID_CONFIG_FILE = "droneid_config.yaml"
_CONFIG_SECTIONS_TO_FLATTEN = ("identity", "network", "mocap", "tracking")


def _share_config_path(filename: str) -> Path:
    return Path(get_package_share_directory(_DRONE_BRINGUP_PACKAGE)) / "config" / filename


def _load_yaml_mapping(path: Path, *, required: bool) -> dict[str, Any]:
    if not path.is_file():
        if required:
            raise FileNotFoundError(f"config file not found: {path}")
        return {}

    loaded = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(loaded, dict):
        raise ValueError(f"expected mapping in config file: {path}")
    return loaded


def _merge_mappings(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    merged = dict(base)
    for key, value in override.items():
        existing = merged.get(key)
        if isinstance(existing, dict) and isinstance(value, dict):
            merged[key] = _merge_mappings(existing, value)
        else:
            merged[key] = value
    return merged


def _flatten_config_sections(config: dict[str, Any]) -> dict[str, Any]:
    flattened: dict[str, Any] = {}
    for key, value in config.items():
        if key in _CONFIG_SECTIONS_TO_FLATTEN and isinstance(value, dict):
            flattened = _merge_mappings(flattened, value)
            continue
        flattened[key] = value
    return flattened


@lru_cache(maxsize=1)
def load_drone_launch_defaults() -> dict[str, Any]:
    shared_defaults = _load_yaml_mapping(
        _share_config_path(_SHARED_DEFAULTS_FILE), required=False
    )
    drone_defaults = _flatten_config_sections(
        _load_yaml_mapping(_share_config_path(_DRONE_ID_CONFIG_FILE), required=True)
    )
    defaults = _merge_mappings(shared_defaults, drone_defaults)

    drone_id = str(defaults.get("drone_id", "")).strip().strip("/")
    if drone_id and not str(defaults.get("mavros_namespace", "")).strip():
        defaults["mavros_namespace"] = f"/cdrone/{drone_id}/mavros"

    rigid_body_name = str(defaults.get("rigid_body_name", "")).strip()
    if rigid_body_name and not str(defaults.get("ownship_pose_topic", "")).strip():
        defaults["ownship_pose_topic"] = f"/vrpn_mocap/{rigid_body_name}/pose"

    return defaults


def get_default_value(key: str, fallback: Any) -> Any:
    defaults = load_drone_launch_defaults()
    value = defaults.get(key, fallback)
    if value is None:
        return fallback
    return value


def default_arg(defaults: dict[str, Any], key: str, fallback: Any) -> str:
    value = defaults.get(key, fallback)
    if value is None:
        value = fallback
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def configured_drone_id() -> str:
    return str(get_default_value("drone_id", "cdrone")).strip() or "cdrone"


def configured_hostname() -> str:
    return str(get_default_value("hostname", configured_drone_id())).strip()


def configured_local_ip() -> str:
    return str(get_default_value("local_ip", "")).strip()


def configured_mavros_namespace() -> str:
    namespace = str(get_default_value("mavros_namespace", "")).strip()
    if namespace:
        return namespace
    drone_id = configured_drone_id().strip().strip("/")
    return f"/cdrone/{drone_id}/mavros" if drone_id else "/mavros"


def configured_rigid_body_name() -> str:
    return str(get_default_value("rigid_body_name", "RigidBody")).strip() or "RigidBody"


def configured_ownship_pose_topic() -> str:
    ownship_pose_topic = str(get_default_value("ownship_pose_topic", "")).strip()
    if ownship_pose_topic:
        return ownship_pose_topic
    return f"/vrpn_mocap/{configured_rigid_body_name()}/pose"


def configured_compare_pose_topic() -> str:
    return (
        str(get_default_value("compare_pose_topic", "/vrpn_mocap/RigidBody2/pose")).strip()
        or "/vrpn_mocap/RigidBody2/pose"
    )
