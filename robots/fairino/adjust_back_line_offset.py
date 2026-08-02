#!/usr/bin/env python3
"""Interactively position the four detected back meridian lines."""

import argparse
import json
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

import ft
from ft_agent_api import AgentFTMassageDemo


WINDOW_NAME = "Back meridian line offset"


def _pixel(point, offset_x=0.0, offset_y=0.0):
    return (
        int(round(float(point[0]) + float(offset_x))),
        int(round(float(point[1]) + float(offset_y))),
    )


def _save_offset(path, offset_x, offset_y):
    path = Path(path).expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "offset_x_px": float(offset_x),
        "offset_y_px": float(offset_y),
        "updated_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
    }
    temp_path = path.with_suffix(path.suffix + ".tmp")
    temp_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temp_path.replace(path)
    return path


class OffsetEditor:
    def __init__(self, offset_x, offset_y, step_px):
        self.offset_x = float(offset_x)
        self.offset_y = float(offset_y)
        self.step_px = max(1.0, float(step_px))
        self.button_rects = {}
        self.pending_action = None
        self.finished = None

    def _layout(self, width, height):
        size = max(38, min(52, int(round(min(width, height) * 0.09))))
        gap = max(5, size // 8)
        margin = max(12, size // 3)
        x0 = width - margin - size * 3 - gap * 2
        y0 = height - margin - size * 3 - gap * 2

        def rect(col, row):
            left = x0 + col * (size + gap)
            top = y0 + row * (size + gap)
            return left, top, left + size, top + size

        self.button_rects = {
            "up": rect(1, 0),
            "left": rect(0, 1),
            "reset": rect(1, 1),
            "right": rect(2, 1),
            "cancel": rect(0, 2),
            "down": rect(1, 2),
            "save": rect(2, 2),
        }

    def on_mouse(self, event, x, y, _flags, _userdata):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        for action, (left, top, right, bottom) in self.button_rects.items():
            if left <= x <= right and top <= y <= bottom:
                self.pending_action = action
                return

    def apply_action(self, action):
        if action == "up":
            self.offset_y -= self.step_px
        elif action == "down":
            self.offset_y += self.step_px
        elif action == "left":
            self.offset_x -= self.step_px
        elif action == "right":
            self.offset_x += self.step_px
        elif action == "reset":
            self.offset_x = 0.0
            self.offset_y = 0.0
        elif action == "save":
            self.finished = "save"
        elif action == "cancel":
            self.finished = "cancel"

    def consume_mouse_action(self):
        action = self.pending_action
        self.pending_action = None
        if action is not None:
            self.apply_action(action)

    def handle_key(self, key):
        low_byte = key & 0xFF if key >= 0 else -1
        key_actions = {
            ord("w"): "up",
            ord("a"): "left",
            ord("s"): "down",
            ord("d"): "right",
            ord("r"): "reset",
        }
        arrow_actions = {
            65362: "up",
            65361: "left",
            65364: "down",
            65363: "right",
            2490368: "up",
            2424832: "left",
            2621440: "down",
            2555904: "right",
        }
        if key in arrow_actions:
            self.apply_action(arrow_actions[key])
        elif low_byte in key_actions:
            self.apply_action(key_actions[low_byte])
        elif low_byte in (10, 13):
            self.finished = "save"
        elif low_byte in (27, ord("q")):
            self.finished = "cancel"
        elif low_byte in (ord("+"), ord("=")):
            self.step_px = min(50.0, self.step_px + 1.0)
        elif low_byte in (ord("-"), ord("_")):
            self.step_px = max(1.0, self.step_px - 1.0)

    @staticmethod
    def _center(rect):
        left, top, right, bottom = rect
        return (left + right) // 2, (top + bottom) // 2

    def draw_controls(self, frame):
        height, width = frame.shape[:2]
        self._layout(width, height)
        overlay = frame.copy()
        for action, rect in self.button_rects.items():
            left, top, right, bottom = rect
            color = (35, 35, 35)
            if action == "save":
                color = (35, 115, 60)
            elif action == "cancel":
                color = (45, 45, 130)
            cv2.rectangle(overlay, (left, top), (right, bottom), color, -1, cv2.LINE_AA)
            cv2.rectangle(overlay, (left, top), (right, bottom), (220, 220, 220), 1, cv2.LINE_AA)
        cv2.addWeighted(overlay, 0.82, frame, 0.18, 0.0, frame)

        for action in ("up", "down", "left", "right"):
            cx, cy = self._center(self.button_rects[action])
            radius = max(9, (self.button_rects[action][2] - self.button_rects[action][0]) // 4)
            delta = {
                "up": (0, -radius),
                "down": (0, radius),
                "left": (-radius, 0),
                "right": (radius, 0),
            }[action]
            start = (cx - delta[0], cy - delta[1])
            end = (cx + delta[0], cy + delta[1])
            cv2.arrowedLine(frame, start, end, (245, 245, 245), 2, cv2.LINE_AA, tipLength=0.35)

        cx, cy = self._center(self.button_rects["reset"])
        cv2.circle(frame, (cx, cy), 9, (235, 235, 235), 2, cv2.LINE_AA)
        cv2.circle(frame, (cx, cy), 2, (235, 235, 235), -1, cv2.LINE_AA)

        cx, cy = self._center(self.button_rects["save"])
        cv2.line(frame, (cx - 10, cy), (cx - 3, cy + 8), (245, 245, 245), 3, cv2.LINE_AA)
        cv2.line(frame, (cx - 3, cy + 8), (cx + 12, cy - 9), (245, 245, 245), 3, cv2.LINE_AA)

        cx, cy = self._center(self.button_rects["cancel"])
        cv2.line(frame, (cx - 9, cy - 9), (cx + 9, cy + 9), (245, 245, 245), 3, cv2.LINE_AA)
        cv2.line(frame, (cx + 9, cy - 9), (cx - 9, cy + 9), (245, 245, 245), 3, cv2.LINE_AA)

        status = f"offset x={self.offset_x:+.0f}px  y={self.offset_y:+.0f}px  step={self.step_px:.0f}px"
        cv2.putText(frame, status, (16, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.66, (20, 20, 20), 4, cv2.LINE_AA)
        cv2.putText(frame, status, (16, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.66, (245, 245, 245), 1, cv2.LINE_AA)


def _draw_lines(frame, lines, offset_x, offset_y, color, thickness):
    if not lines:
        return
    for line in lines:
        cv2.line(
            frame,
            _pixel(line[0], offset_x, offset_y),
            _pixel(line[1], offset_x, offset_y),
            color,
            thickness,
            cv2.LINE_AA,
        )


def run(config_path, step_px):
    saved_x = float(ft.BACK_LINE_OFFSET_X_PX)
    saved_y = float(ft.BACK_LINE_OFFSET_Y_PX)
    editor = OffsetEditor(saved_x, saved_y, step_px)
    demo = AgentFTMassageDemo(massage_target="back")

    # Analyze the unshifted detector output; the editor applies the candidate
    # offset only to the preview until the user explicitly saves it.
    ft.BACK_LINE_OFFSET_X_PX = 0.0
    ft.BACK_LINE_OFFSET_Y_PX = 0.0

    print("启动背部膀胱经线偏移调节工具...")
    print("可点击画面中的方向箭头，也可使用方向键或 W/A/S/D。")
    print("加减键调整步长；Enter 保存；Esc 取消；中间圆形按钮归零。")

    try:
        demo.init_vision()
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, 960, 720)
        cv2.setMouseCallback(WINDOW_NAME, editor.on_mouse)

        while editor.finished is None:
            frames = demo.detector.pipeline.wait_for_frames()
            frames = demo.detector.align.process(frames)
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            image = np.asanyarray(color_frame.get_data())
            demo.frame_idx += 1
            analysis = demo._analyze_visual_frame(image)
            preview = image.copy()
            _draw_lines(
                preview,
                analysis.get("outer_meridian_lines"),
                editor.offset_x,
                editor.offset_y,
                (255, 0, 255),
                3,
            )
            _draw_lines(
                preview,
                analysis.get("meridian_lines"),
                editor.offset_x,
                editor.offset_y,
                (0, 255, 0),
                2,
            )
            tracking = str(analysis.get("visual_status", "search")).upper()
            cv2.putText(preview, tracking, (16, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (20, 20, 20), 4, cv2.LINE_AA)
            cv2.putText(preview, tracking, (16, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (80, 220, 255), 1, cv2.LINE_AA)
            editor.draw_controls(preview)
            cv2.imshow(WINDOW_NAME, preview)

            editor.consume_mouse_action()
            editor.handle_key(cv2.waitKeyEx(20))

        if editor.finished == "save":
            saved_path = _save_offset(config_path, editor.offset_x, editor.offset_y)
            print(
                f"已保存四条膀胱经线偏移: "
                f"x={editor.offset_x:+.0f}px, y={editor.offset_y:+.0f}px"
            )
            print(f"配置文件: {saved_path}")
            return 0

        print("已取消，未修改已保存的偏移。")
        return 1
    finally:
        pipeline = getattr(getattr(demo, "detector", None), "pipeline", None)
        if pipeline is not None:
            try:
                pipeline.stop()
            except Exception:
                pass
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="Adjust the four back meridian lines on a live camera image.")
    parser.add_argument("--config", default=str(ft.BACK_LINE_OFFSET_FILE))
    parser.add_argument("--step", type=float, default=2.0)
    args = parser.parse_args()
    raise SystemExit(run(args.config, args.step))


if __name__ == "__main__":
    main()
