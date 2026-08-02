#!/usr/bin/env python3
"""Validate the repository layout without importing hardware dependencies."""

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]

REQUIRED_PATHS = (
    "massage",
    "docs/PROJECT_STRUCTURE.md",
    "docs/DEVELOPMENT.md",
    "py-xiaozhi/src/mcp/tools/fairino_massage",
    "robots/fairino/ft.py",
    "robots/fairino/ft_agent_api.py",
    "robots/fairino/ft_agent_process.py",
    "robots/fairino/force_control.py",
    "robots/fairino/demo.py",
    "robots/fairino/lasttime.py",
    "robots/fairino/dianjing.py",
    "robots/fairino/rtmpose_detector.py",
    "robots/fairino/thigh_outerline_confirm.py",
    "robots/fairino/vendor/fairino_sdk/Robot.py",
    "shared/calibration",
    "shared/vision/yolo.py",
)

REMOVED_PATHS = (
    "robots/realman",
    "shared/third_party/co-tracker",
    "py-xiaozhi/src/mcp/tools/massage",
)

TEXT_REQUIREMENTS = {
    "massage": (
        'export XIAOZHI_MASSAGE_ONLY="${XIAOZHI_MASSAGE_ONLY:-1}"',
    ),
    "robots/fairino/run_lasttime_ros2.sh": (
        'LASTTIME_ROS2_SCRIPT="${LASTTIME_ROS2_SCRIPT:-ft.py}"',
    ),
    "py-xiaozhi/src/mcp/mcp_server.py": ("XIAOZHI_MASSAGE_ONLY",),
    "robots/fairino/fairino/__init__.py": (
        '_VENDORED_SDK_ROOT = os.path.join(_PROJECT_ROOT, "vendor", "fairino_sdk")',
    ),
}


def main() -> int:
    errors = []

    for relative_path in REQUIRED_PATHS:
        if not (ROOT / relative_path).exists():
            errors.append(f"missing required path: {relative_path}")

    for relative_path in REMOVED_PATHS:
        if (ROOT / relative_path).exists():
            errors.append(f"obsolete path returned: {relative_path}")

    for relative_path, expected_fragments in TEXT_REQUIREMENTS.items():
        path = ROOT / relative_path
        if not path.is_file():
            continue
        content = path.read_text(encoding="utf-8")
        for fragment in expected_fragments:
            if fragment not in content:
                errors.append(f"missing expected setting in {relative_path}: {fragment}")

    sdk_loader = ROOT / "robots/fairino/fairino/__init__.py"
    if sdk_loader.is_file() and "/home/franka" in sdk_loader.read_text(encoding="utf-8"):
        errors.append("FAIRINO SDK loader still depends on /home/franka")

    if errors:
        print("Project structure check failed:", file=sys.stderr)
        for error in errors:
            print(f"- {error}", file=sys.stderr)
        return 1

    print("Project structure check passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
