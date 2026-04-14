from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass(frozen=True)
class Px4ParamProfile:
    name: str
    description: str
    parameters: dict[str, float]
    source_path: str

    @classmethod
    def load_from_yaml(cls, path: str) -> "Px4ParamProfile":
        config_path = Path(path).expanduser()
        if not config_path.is_file():
            raise FileNotFoundError(f"speed profile config not found: {config_path}")

        loaded = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
        if not isinstance(loaded, dict):
            raise ValueError(f"expected mapping in speed profile config: {config_path}")

        raw_parameters = loaded.get("parameters")
        if not isinstance(raw_parameters, dict) or not raw_parameters:
            raise ValueError("speed profile config must contain a non-empty 'parameters' mapping")

        parameters: dict[str, float] = {}
        for raw_name, raw_value in raw_parameters.items():
            param_name = str(raw_name or "").strip()
            if not param_name:
                raise ValueError("speed profile parameter names must be non-empty")
            parameters[param_name] = float(raw_value)

        profile_name = str(loaded.get("name", "")).strip() or config_path.stem
        description = str(loaded.get("description", "")).strip()

        return cls(
            name=profile_name,
            description=description,
            parameters=parameters,
            source_path=str(config_path),
        )
