from __future__ import annotations

from pathlib import Path
from typing import Any


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def resolve_path(path: str | Path) -> Path:
    value = Path(path).expanduser()
    if value.is_absolute() and value.exists():
        return value
    if value.exists():
        return value.resolve()

    candidates = [
        PACKAGE_ROOT / value,
        PACKAGE_ROOT / "config" / value.name,
        Path.cwd() / value,
    ]
    try:
        from ament_index_python.packages import get_package_share_directory

        share_dir = Path(get_package_share_directory("rm_massage"))
        candidates.extend([share_dir / value, share_dir / "config" / value.name])
    except Exception:
        pass
    for candidate in candidates:
        if candidate.exists():
            return candidate.resolve()
    return value


def load_yaml(path: str | Path) -> dict[str, Any]:
    import yaml

    resolved = resolve_path(path)
    if not resolved.exists():
        raise FileNotFoundError(f"config file not found: {path}")
    with resolved.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError(f"YAML root must be a map: {resolved}")
    return data


def save_yaml(path: str | Path, data: dict[str, Any]) -> None:
    import yaml

    resolved = Path(path).expanduser()
    if not resolved.is_absolute():
        resolved = (Path.cwd() / resolved).resolve()
    resolved.parent.mkdir(parents=True, exist_ok=True)
    with resolved.open("w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)


def deep_get(data: dict[str, Any], dotted_key: str, default: Any = None) -> Any:
    current: Any = data
    for part in dotted_key.split("."):
        if not isinstance(current, dict) or part not in current:
            return default
        current = current[part]
    return current


def merged_config(*configs: dict[str, Any]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for cfg in configs:
        _deep_merge(out, cfg)
    return out


def _deep_merge(dst: dict[str, Any], src: dict[str, Any]) -> None:
    for key, value in src.items():
        if isinstance(value, dict) and isinstance(dst.get(key), dict):
            _deep_merge(dst[key], value)
        else:
            dst[key] = value
