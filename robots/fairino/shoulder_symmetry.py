"""侧卧时双肩沿身体头脚方向的对称性预览；只读取相机，不连接机械臂。"""

import argparse
from collections import deque
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "shared" / "vision"))
from yolo import infer_best_pose_with_rotations  # noqa: E402


def draw_english_overlay(image, result_text, points=None):
    cv2.rectangle(image, (0, 0), (image.shape[1], 76), (0, 0, 0), -1)
    cv2.putText(image, result_text, (10, 28), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (255, 255, 255), 2)
    cv2.putText(image, "r: retest | q/Esc: quit", (10, 56),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 255, 180), 1)
    if points is not None:
        left, right = points
        cv2.putText(image, "L", (left[0] + 8, left[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        cv2.putText(image, "R", (right[0] + 8, right[1] - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    return image


def symmetry_measure(kpts, min_confidence=0.6):
    """按摩床头脚方向在画面中水平；比较双肩沿此方向的位置差。"""
    indices = (5, 6, 11, 12)  # COCO: 左肩、右肩、左髋、右髋
    if kpts is None or len(kpts) < 13:
        return None, "未检测到完整躯干"
    if any(float(kpts[i][2]) < min_confidence for i in indices):
        return None, "肩/髋关键点置信度不足"

    points_2d = np.asarray([kpts[i][:2] for i in indices], dtype=np.float64)
    if not np.isfinite(points_2d).all():
        return None, "关键点坐标无效"
    ls, right_shoulder, lh, rh = points_2d
    shoulder_mid = (ls + right_shoulder) / 2.0
    hip_mid = (lh + rh) / 2.0
    torso_vector = hip_mid - shoulder_mid
    shoulder_vector = right_shoulder - ls
    shoulder_width = float(np.linalg.norm(shoulder_vector))
    torso_length = float(np.linalg.norm(torso_vector))
    if shoulder_width < 1e-6 or torso_length < shoulder_width * 0.65:
        return None, "躯干方向不可靠"
    if abs(torso_vector[0]) < max(shoulder_width * 0.65, abs(torso_vector[1]) * 2.0):
        return None, "身体头脚方向未与画面水平方向对齐"

    # 图像 x 轴对应床的头脚方向；y 轴的上下肩分离不会进入计算。
    footward_offset = float(shoulder_vector[0] * np.sign(torso_vector[0]))
    ratio = footward_offset / shoulder_width
    if abs(ratio) > 0.4:
        return None, "双肩定位异常，请调整姿势或视角"
    return {
        "ratio": ratio,
        "offset_px": footward_offset,
        "points_2d": points_2d,
        "body_axis_2d": (shoulder_mid, np.array([hip_mid[0], shoulder_mid[1]])),
    }, None


def stable_result(history, threshold, min_frames):
    """只有同一分类占多数且偏移波动较小时才锁定。"""
    if not history:
        return None
    newest_time = history[-1][0]
    recent = [sample for sample in history if sample[0] >= newest_time - 1.5]
    if len(recent) < min_frames or recent[-1][0] - recent[0][0] < 1.0:
        return None
    ratios = np.asarray([sample[1] for sample in recent], dtype=np.float64)
    median = float(np.median(ratios))
    mad = float(np.median(np.abs(ratios - median)))
    if mad > 0.02:
        return None
    labels = np.where(ratios > threshold, 1, np.where(ratios < -threshold, -1, 0))
    label = 1 if median > threshold else -1 if median < -threshold else 0
    if float(np.mean(labels == label)) < 0.85:
        return None
    return median


def main():
    parser = argparse.ArgumentParser(description="侧卧双肩对称性实时检测（仅相机）")
    parser.add_argument("--threshold", type=float, default=0.05,
                        help="偏移量/肩宽的判断门槛，默认 0.05（测试参数，非临床标准）")
    parser.add_argument("--min-confidence", type=float, default=0.6)
    parser.add_argument("--frames", type=int, default=20, help="锁定前至少采集的有效帧数")
    parser.add_argument("--delay", type=int, default=0, help="启动检测前等待的秒数")
    parser.add_argument("--once", action="store_true", help="锁定一次结果后显示固定画面，按 q/Esc 关闭")
    parser.add_argument("--debug-save", type=Path, help="可选：保存本次锁定画面和肩髋点数据的文件名前缀")
    args = parser.parse_args()
    if not 0 < args.threshold < 0.4 or not 0 < args.min_confidence <= 1 or args.frames < 7 or args.delay < 0:
        parser.error("参数超出范围")

    if args.delay:
        print(f"{args.delay} 秒后开始检测，请在这段时间躺好并保持姿势。", flush=True)
        time.sleep(args.delay)
    model_path = ROOT / "shared" / "vision" / "yolo11l-pose.pt"
    if not model_path.is_file():
        parser.error(f"找不到姿态模型：{model_path}")
    model = YOLO(str(model_path))
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    started = False
    history = deque()
    locked_image = None
    locked_message = None
    last_state = None
    last_print = 0.0
    try:
        pipeline.start(config)
        started = True
        started_at = time.monotonic()
        print("侧卧双肩对称性检测已启动；稳定后自动锁定结果，按 r 重测，按 q 或 Esc 退出。", flush=True)
        while True:
            if args.once and locked_image is None and time.monotonic() - started_at > 45.0:
                print("本次未能在 45 秒内获得稳定结果。", flush=True)
                return 1
            if locked_image is not None:
                image = locked_image
            else:
                frames = pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                if not color_frame:
                    continue
                image = np.asanyarray(color_frame.get_data()).copy()
                raw_image = image.copy() if args.debug_save else None
                pose = infer_best_pose_with_rotations(model, image, conf=0.5)
                measure, reason = symmetry_measure(pose["kpts"], args.min_confidence)
                now = time.monotonic()
                while history and now - history[0][0] > 4.0:
                    history.popleft()
                if measure is None:
                    message = f"等待有效姿态：{reason}"
                    state = "等待有效姿态"
                    overlay = "Waiting for clear shoulder / hip keypoints"
                    shoulder_marks = None
                else:
                    history.append((now, measure["ratio"], pose["kpts"][[5, 6, 11, 12], :].tolist()))
                    pts = measure["points_2d"]
                    left = tuple(np.rint(pts[0]).astype(int))
                    right = tuple(np.rint(pts[1]).astype(int))
                    cv2.line(image, left, right, (0, 255, 255), 2)
                    cv2.circle(image, left, 6, (255, 0, 0), -1)
                    cv2.circle(image, right, 6, (0, 0, 255), -1)
                    shoulder_mid, hip_mid = measure["body_axis_2d"]
                    cv2.line(image, tuple(np.rint(shoulder_mid).astype(int)),
                             tuple(np.rint(hip_mid).astype(int)), (255, 255, 0), 2)
                    shoulder_marks = (left, right)
                    result = stable_result(history, args.threshold, args.frames)
                    if result is None:
                        valid_count = min(
                            sum(sample[0] >= now - 1.5 for sample in history),
                            args.frames,
                        )
                        message = f"采样中：{valid_count}/{args.frames} 有效帧；请保持姿势"
                        state = "采样中"
                        overlay = f"Sampling {valid_count}/{args.frames} | hold still"
                    else:
                        ratio = result
                        if ratio > args.threshold:
                            label = "左肩偏高"
                            english = "Left shoulder higher"
                        elif ratio < -args.threshold:
                            label = "右肩偏高"
                            english = "Right shoulder higher"
                        else:
                            label = "基本对称"
                            english = "Shoulders symmetric"
                        value = f"肩宽的 {abs(ratio) * 100:.1f}%"
                        value_en = f"{abs(ratio) * 100:.1f}% shoulder width"
                        locked_message = f"结果已锁定：{label}；沿头脚方向偏移：{value}；仅代表当前侧卧姿态"
                        message = locked_message
                        state = "结果已锁定"
                        overlay = f"LOCKED: {english} | {value_en}"

                image = draw_english_overlay(image, overlay, shoulder_marks)
                if state != last_state or now - last_print >= 5.0:
                    print(message, flush=True)
                    last_state, last_print = state, now
                if locked_message is not None:
                    locked_image = image.copy()
                    if args.debug_save:
                        prefix = args.debug_save
                        prefix.parent.mkdir(parents=True, exist_ok=True)
                        cv2.imwrite(str(prefix) + "_raw.png", raw_image)
                        cv2.imwrite(str(prefix) + "_marked.png", locked_image)
                        (Path(str(prefix) + "_points.json")).write_text(
                            json.dumps({
                                "result": locked_message,
                                "rotation": pose["rotation"],
                                "threshold_ratio": args.threshold,
                                "keypoint_order": ["left_shoulder", "right_shoulder", "left_hip", "right_hip"],
                                "samples": [
                                    {"ratio": sample[1], "keypoints": sample[2]}
                                    for sample in history
                                ],
                            }, ensure_ascii=False, indent=2),
                            encoding="utf-8",
                        )
                        print(f"诊断数据已保存：{prefix}_raw.png、{prefix}_marked.png、{prefix}_points.json", flush=True)
            cv2.imshow("Side-lying shoulder symmetry", image)
            key = cv2.waitKey(30 if locked_image is not None else 1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("r"):
                history.clear()
                locked_image = None
                locked_message = None
                last_state = None
                print("已重新开始采样。", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        if started:
            pipeline.stop()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
