# RealMan Partition

This directory contains RealMan/RM-specific robot code and data.

Current MVP package:

```bash
cd /home/franka/massage/robots/realman/massage_ws
colcon build --symlink-install --packages-select rm_massage
source install/setup.bash
```

First checks:

```bash
bash src/rm_massage/scripts/00_start_external_d435i.sh
bash src/rm_massage/scripts/00_start_rm_json_bridge.sh
bash src/rm_massage/scripts/00_check_topics.sh
```

The JSON bridge uses RealMan controller `192.168.1.18:8080` and publishes standard ROS2 topics under `/rm_json/*`.
