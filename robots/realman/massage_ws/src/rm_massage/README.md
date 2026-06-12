# rm_massage MVP

This package implements the staged MVP from the massage robot cookbook:

1. Check arm, camera, TF, and force topics.
2. Confirm TCP force axis sign.
3. Teach a short fixed line in `robot_base`.
4. Preview the generated TCP trajectory in RViz.
5. Run dry-run, air-run, contact-test, then short force-guided massage.

The controller is dry-run by default. It publishes robot commands only when
`--execute` is provided.

## Build

```bash
cd /home/franka/massage/robots/realman/massage_ws
colcon build --symlink-install --packages-select rm_massage
source install/setup.bash
```

## First checks

```bash
bash src/rm_massage/scripts/00_start_external_d435i.sh
bash src/rm_massage/scripts/00_start_rm_json_bridge.sh
bash src/rm_massage/scripts/00_check_topics.sh
ros2 run rm_massage clear_force_json
ros2 run rm_massage check_force_axis --frames src/rm_massage/config/frames_json.yaml --save
```

On this workstation the installed RealSense launch file publishes color topics
under `/camera/camera/color/...` with its defaults. Pass explicit topic
arguments to `aruco_body_pose` if your launch produces a different namespace.

## Teach a fixed base-frame test line

```bash
ros2 run rm_massage teach_line \
  --target-frame robot_base \
  --source-frame massage_tcp \
  --name base_test_5cm \
  --output /home/franka/massage/robots/realman/massage_ws/src/rm_massage/config/massage_lines_base_test.yaml
```

## Preview and run

```bash
ros2 run rm_massage preview_line_rviz \
  --lines /home/franka/massage/robots/realman/massage_ws/src/rm_massage/config/massage_lines_base_test.yaml \
  --include-disabled

ros2 run rm_massage massage_controller \
  --lines /home/franka/massage/robots/realman/massage_ws/src/rm_massage/config/massage_lines_base_test.yaml \
  --line base_test_5cm
```

Add `--execute --air-run`, then `--execute --contact-test`, then plain
`--execute` only after the dry-run plan and RViz preview are correct.
