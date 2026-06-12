from __future__ import annotations


def normalize_motion_speed(
    control_backend: str,
    speed: float,
    *,
    ros_default: float = 0.3,
    json_min: float = 1.0,
) -> float:
    backend = str(control_backend).strip().lower()
    value = float(speed)
    if backend == "ros":
        if value <= 0.0:
            return float(ros_default)
        if value <= 1.0:
            return float(value)
        if value <= 10.0:
            return float(value / 10.0)
        return float(min(1.0, value / 100.0))
    return float(max(float(json_min), round(value)))
