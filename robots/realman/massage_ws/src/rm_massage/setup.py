from setuptools import find_packages, setup


package_name = "rm_massage"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", [
            "config/frames.yaml",
            "config/safety.yaml",
            "config/body_markers.yaml",
            "config/massage_lines.yaml",
            "config/massage_lines_base_test.yaml",
            "config/frames_json.yaml",
        ]),
        (f"share/{package_name}/launch", ["launch/massage_bringup.launch.py"]),
        (f"share/{package_name}/scripts", [
            "scripts/00_start_external_d435i.sh",
            "scripts/00_stop_external_d435i.sh",
            "scripts/00_start_rm_json_bridge.sh",
            "scripts/00_stop_rm_json_bridge.sh",
            "scripts/00_check_topics.sh",
            "scripts/01_clear_force_json.py",
            "scripts/01_check_force_axis.py",
            "scripts/02_calibrate_base_camera.py",
            "scripts/03_teach_line.py",
            "scripts/04_aruco_body_pose.py",
            "scripts/05_preview_line_rviz.py",
            "scripts/06_massage_controller.py",
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="franka",
    maintainer_email="franka@example.com",
    description="RealMan massage robot MVP nodes.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "check_force_axis = rm_massage.check_force_axis:main",
            "calibrate_base_camera = rm_massage.calibrate_base_camera:main",
            "teach_line = rm_massage.teach_line:main",
            "aruco_body_pose = rm_massage.aruco_body_pose:main",
            "preview_line_rviz = rm_massage.preview_line_rviz:main",
            "massage_controller = rm_massage.massage_controller:main",
            "safety_monitor = rm_massage.safety_monitor:main",
            "json_state_bridge = rm_massage.rm_json_bridge:main",
            "clear_force_json = rm_massage.clear_force_json:main",
        ],
    },
)
