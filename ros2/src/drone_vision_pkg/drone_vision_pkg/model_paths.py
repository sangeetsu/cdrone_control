from __future__ import annotations

import os
from pathlib import Path


DEFAULT_MODEL_CANDIDATES = (
    "best_large_640_100e_77k_fp16.engine",
    "best_large_640_100e_77k.pt",
)


def _candidate_roots() -> tuple[Path, ...]:
    repo_root = Path(__file__).resolve().parents[4]

    env_root = os.environ.get("CDRONE_YOLO_ROOT", "").strip()
    roots = [
        Path(env_root) if env_root else None,
        Path("/home/jetson/cdrone_yolo"),
        repo_root / "vendor" / "cdrone_yolo",
        repo_root / "third_party" / "cdrone_yolo",
    ]
    return tuple(root for root in roots if root is not None)


def resolve_model_path(requested_path: str) -> str:
    requested = str(requested_path or "").strip()
    if requested:
        path = Path(requested).expanduser()
        if path.exists():
            return str(path)
        raise FileNotFoundError(f"Requested model_path does not exist: {path}")

    env_model = os.environ.get("CDRONE_YOLO_MODEL_PATH", "").strip()
    if env_model:
        path = Path(env_model).expanduser()
        if path.exists():
            return str(path)
        raise FileNotFoundError(
            "CDRONE_YOLO_MODEL_PATH is set but the file does not exist: "
            f"{path}"
        )

    for root in _candidate_roots():
        for model_name in DEFAULT_MODEL_CANDIDATES:
            candidate = root / "models" / model_name
            if candidate.exists():
                return str(candidate)

    raise FileNotFoundError(
        "Could not auto-resolve a detector model. Set model_path explicitly or "
        "configure CDRONE_YOLO_MODEL_PATH/CDRONE_YOLO_ROOT."
    )
