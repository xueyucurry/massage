#!/usr/bin/env python3
"""
Temporary shun-jin only runner.

This script intentionally does not modify ft.py. It reuses ft.py's vision,
trajectory capture, ROS2 robot control, and force-control implementation, but
overrides the motion sequence to execute only shun-jin.
"""

import sys

import ft


class ShunJinOnlyDemo(ft.LastTimeRos2Demo):
    def execute_massage_sequence(self):
        print("\n开始执行顺筋单项测试...")

        try:
            print("移动到安全高度...")
            self.update_preview_status("移动到安全高度")
            safe_pose, should_move_to_safe = self._build_session_safe_pose()
            if not self._move_to_initial_safe_pose(safe_pose, should_move_to_safe):
                return False

            if ft.LASTTIME_ROS2_FORCE:
                print(f"初始化 {ft.FORCE_TARGET_N:.1f}N 恒力控制（请确认末端悬空无接触）...")
                self.init_force_controller()

            print("移动到顺筋起始悬空位...")
            first_frame = self.massage_frames[0]
            self.update_preview_status("移动到顺筋起点", first_frame.get("index", 0))
            first_pose = self._pose_from_frame_offset(first_frame, -self.hover_height_mm)
            if not self._move_to_work_pose(first_pose, "移动到顺筋起始位置", ft.MOVE_VEL_FAST):
                return False

            print("\n只执行顺筋动作...")
            if not self.execute_shun_jin():
                return False

            print("\n返回安全位置...")
            self.update_preview_status("返回安全位置")
            if not self._move_to_work_pose(safe_pose, "返回安全位置", ft.MOVE_VEL_FAST):
                return False

            print("\n顺筋单项测试完成！")
            return True

        except Exception as exc:
            print(f"\n错误：{exc}")
            return False
        finally:
            self.close_force_controller()


def main():
    print("=" * 60)
    print("run_shunjin_only.py - 顺筋单项测试（复用 ft.py）")
    print("=" * 60)
    print()

    massage_target = ft._select_massage_target()
    selected_hover_mm = ft.THIGH_HOVER_HEIGHT_MM if massage_target == "leg" else ft.BACK_HOVER_HEIGHT_MM
    selected_approach_max_mm = (
        ft.THIGH_FORCE_APPROACH_MAX_OFFSET_MM
        if massage_target == "leg"
        else ft.FORCE_APPROACH_MAX_OFFSET_MM
    )
    selected_normal_limit_n = max(abs(ft.FORCE_SOFTWARE_NORMAL_LIMIT_N), abs(ft.FORCE_TARGET_N) + 30.0)
    selected_tangent_limit_n = max(abs(ft.FORCE_SOFTWARE_TANGENTIAL_LIMIT_N), selected_normal_limit_n)

    print("配置参数：")
    print(f"  动作模式: 只执行顺筋")
    print(f"  按摩部位: {'腿部大腿外侧中线' if massage_target == 'leg' else '背部膀胱经'}")
    print(f"  机械臂IP: {ft.ROBOT_IP}")
    print(f"  悬空高度: {selected_hover_mm}mm")
    print(f"  工具端补偿: 法兰/传感器中心到按摩头={ft.TOOL_TIP_LENGTH_MM:.1f}mm")
    print(f"  采样点数: {ft.SAMPLE_POINTS}")
    print(f"  ROS2控制服务: {ft.ROS2_SERVICE_NAME}")
    print(f"  ROS2状态话题: {ft.ROS2_STATE_TOPIC}")
    print(f"  末端姿态: {'保持当前TCP姿态' if ft.ROS2_KEEP_CURRENT_ORIENTATION else '局部深度平面法向'}")
    print(f"  安全位策略: {'旧P24固定安全位' if ft.ROS2_USE_LEGACY_SAFE_POSE else '当前位置竖直抬升'} safe_z={ft.ROS2_LIFT_SAFE_Z_MM:.1f}mm")
    if massage_target == "leg":
        print(
            f"  腿部检测: side={ft.THIGH_SIDE} offset={ft.THIGH_OFFSET_MM:.1f}mm "
            f"direction={ft.THIGH_DIRECTION} samples={ft.THIGH_SAMPLE_POINTS}"
        )
    if ft.LASTTIME_ROS2_FORCE:
        print(
            f"  恒力控制: target={ft.FORCE_TARGET_N:.1f}N "
            f"normal_limit={selected_normal_limit_n:.1f}N "
            f"tangent_limit={selected_tangent_limit_n:.1f}N "
            f"approach_step={ft.FORCE_APPROACH_STEP_MM:.2f}mm "
            f"contact={ft.FORCE_APPROACH_CONTACT_N:.1f}N/{ft.FORCE_APPROACH_CONTACT_STEP_MM:.2f}mm "
            f"max_offset={selected_approach_max_mm:.1f}mm"
        )
    else:
        print("  恒力控制: 关闭")
    print()

    demo = ShunJinOnlyDemo(massage_target=massage_target)
    success = demo.run()
    if success:
        print("\n演示完成！")
        return 0

    print("\n演示失败")
    return 1


if __name__ == "__main__":
    sys.exit(main())
