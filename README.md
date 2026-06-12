# massage workspace layout

This directory has been reorganized by robot vendor and shared assets.

## Main partitions

- `robots/realman/`: RealMan/RM robot code, ROS2 MVP package, RM JSON bridge, RM demos, RM outputs, RM vendor ROS message shims.
- `robots/fairino/`: Fairino robot SDK/ROS2 code, Fairino force/compliance demos, older bladder-meridian demos using the Fairino SDK.
- `shared/`: Vendor-neutral camera, vision, calibration, third-party, and utility assets.
- `docs/`: Patent and documentation material.
- `env/`: User-owned Python virtual environments that could be moved safely.
- `_meta/`: Workspace/editor/git metadata moved out of the source/data partitions.

## Important paths

RealMan MVP workspace:

```bash
cd /home/franka/massage/robots/realman/massage_ws
source install/setup.bash
```

Fairino scripts:

```bash
cd /home/franka/massage/robots/fairino
```

Shared RealSense/camera tools:

```bash
cd /home/franka/massage/shared/camera
```

Shared calibration files:

```bash
cd /home/franka/massage/shared/calibration
```

## Notes

- `.venv-noetic` is still at the root because it is root-owned and could not be moved by the current user.
- `robots/realman/yolo*.pt`, `robots/fairino/yolo*.pt`, and both vendor `camera_to_robot.json` files are symlinks to shared assets.
- Historical logs and JSON outputs may still contain old absolute paths as provenance records.
