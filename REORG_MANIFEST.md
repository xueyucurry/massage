# Reorganization Manifest

## RealMan

Moved into `robots/realman/`:

- `massage_ws/`
- `rm_demo/`, `rm_demo_output/`, `rm_demo_debug/`, `rm_demo_pkgs/`
- `ros_vendor/`
- `rm_*.py`
- RealMan run scripts: `run_bladder*`, `run_side_lying*`, `run_purple_20_point_press.py`, `run_live_top_outer_point_force.py`, `run_saved_bladder_plan.py`, `run_teach_normal_probe.py`
- RealMan calibration/control helpers: `calibrate_rm_camera_to_base_aruco.py`, `connect_jetson.sh`, `disconnect_jetson.sh`, `unlock_robot.py`

## Fairino

Moved into `robots/fairino/`:

- `fairino/`, `fairino_ros2/`, `fairino-python-sdk-master (1)/`
- `fairino_*`
- `force_control.py`, `force_sensor.py`, `test_force_sensor.py`
- `demo.py`, `lasttime*.py`, `run_lasttime*.sh`, `run_demo.sh`
- `robot_meridian_output/`
- Fairino compliance and spring configs/docs

## Shared

Moved into `shared/`:

- `camera/`: RealSense capture/view/rosbag tooling and bag data
- `vision/`: vendor-neutral vision scripts and YOLO weights
- `calibration/`: camera-to-robot calibration files and generic calibration scripts
- `third_party/`: CoTracker
- `tools/`: NoMachine installers
- `misc/`: miscellaneous standalone download utility

## Docs And Metadata

- Patent documents moved to `docs/patents/`
- `.venv` moved to `env/.venv`
- `.git`, `.claude`, `.codex`, `.ros2_cmd_server.log`, and root `__pycache__` moved to `_meta/`
- `.venv-noetic` was not moved because current user lacks permission
