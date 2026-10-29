# RTMPOSE.py D435i 实时部署说明

当前版本的 `RTMPOSE.py` 默认直接读取当前 RealSense D435i 的实时彩色图和对齐深度图，用 MMPose / RTMPose 检测人体关键点，并绘制同侧髋部到膝盖的连线。

ROS bag 处理仍保留为可选模式：需要时显式加 `--source bag --bags <file.bag>`。

---

## 当前机器已配置

使用项目现有虚拟环境：

```bash
/home/franka/massage/env/.venv/bin/python
```

已补齐的主要依赖：

```text
torch 2.5.1+cu121
opencv-python 4.10.0.84
numpy 1.26.4
pyrealsense2
rosbags
mmengine 0.10.7
mmcv-lite 2.1.0
mmdet 3.3.0
mmpose 1.3.2
```

说明：当前 PyTorch 版本没有直接可用的完整 `mmcv` wheel，因此使用 `mmcv-lite`，并在当前 venv 内对 MMPose/Mmdet 的包级导入做了轻量兼容处理，跳过 RTMPose 不需要的检测/Transformer 扩展算子。

---

## 权重与配置

RTMPose-M COCO 配置文件由脚本自动从当前 venv 的 `mmpose` 包内解析：

```text
/home/franka/massage/env/.venv/lib/python3.10/site-packages/mmpose/.mim/configs/body_2d_keypoint/rtmpose/coco/rtmpose-m_8xb256-420e_coco-256x192.py
```

权重已下载到：

```text
/home/franka/massage/robots/fairino/weights/rtmpose-m_simcc-coco_pt-aic-coco_420e-256x192-d8dd5ca4_20230127.pth
```

离线重新部署时，手动下载地址：

```text
https://download.openmmlab.com/mmpose/v1/projects/rtmposev1/rtmpose-m_simcc-coco_pt-aic-coco_420e-256x192-d8dd5ca4_20230127.pth
```

---

## 实时运行

默认就是 D435i 实时模式：

```bash
cd /home/franka/massage/robots/fairino
/home/franka/massage/env/.venv/bin/python RTMPOSE.py
```

也可以用小写包装入口：

```bash
cd /home/franka/massage/robots/fairino
/home/franka/massage/env/.venv/bin/python rtmpose.py
```

窗口中按 `q` 或 `ESC` 退出。

常用参数：

```bash
/home/franka/massage/env/.venv/bin/python RTMPOSE.py \
  --device cuda:0 \
  --side nearest \
  --width 640 \
  --height 480 \
  --fps 30
```

参数说明：

| 参数 | 含义 |
|------|------|
| `--source realsense` | 默认值，读取 D435i 实时画面 |
| `--width / --height / --fps` | RealSense 彩色和深度流分辨率、帧率 |
| `--no-align-depth` | 关闭深度对齐到彩色图，默认开启 |
| `--side` | `auto` / `nearest` / `left` / `right`，选择哪条腿 |
| `--kpt-thr` | 髋/膝关键点置信度阈值 |
| `--rotation` | `none` / `ccw` / `cw`，实时默认不旋转 |
| `--try-rotations` | 每帧尝试三个旋转方向，准确但明显更慢 |
| `--no-display` | 不打开 OpenCV 窗口 |
| `--no-save-json` | 实时退出时不保存检测 JSON |
| `--max-frames` | 调试用，处理 N 帧后自动退出 |

实时检测结果默认保存到：

```text
rtmpose_hip_knee_output/realsense_rtmpose_hip_knee_*.json
```

---

## ROS bag 可选模式

如果以后还需要处理 bag：

```bash
cd /home/franka/massage/robots/fairino
/home/franka/massage/env/.venv/bin/python RTMPOSE.py \
  --source bag \
  --bags /path/to/file.bag \
  --output-dir rtmpose_hip_knee_output
```

bag topic 自动匹配：

- 彩色：`/color/image_raw` 或 `/rgb/image_raw`
- 深度：`/aligned_depth_to_color/image_raw`、`/depth/image_rect_raw`、`/depth/image_raw`

自动匹配失败时可加：

```bash
--color-topic <彩色topic> --depth-topic <深度topic>
```

---

## 自检命令

依赖导入：

```bash
/home/franka/massage/env/.venv/bin/python - <<'PY'
import cv2, numpy, torch, pyrealsense2
from mmpose.apis import init_model, inference_topdown
print("ok")
PY
```

模型初始化和单帧推理：

```bash
cd /home/franka/massage/robots/fairino
/home/franka/massage/env/.venv/bin/python - <<'PY'
import numpy as np
from RTMPOSE import DEFAULT_RTMPOSE_CONFIG, DEFAULT_RTMPOSE_WEIGHTS, RTMPoseHipKneeDetector

detector = RTMPoseHipKneeDetector(
    pose2d=DEFAULT_RTMPOSE_CONFIG,
    pose2d_weights=DEFAULT_RTMPOSE_WEIGHTS,
    device="cuda:0",
    side="nearest",
    kpt_thr=0.25,
)
img = np.zeros((480, 640, 3), dtype=np.uint8)
_, record = detector.detect(img, None, 0, 0.0)
print(record.valid, record.reason)
PY
```

黑图上输出关键点置信度不足是正常的；重点是模型能成功加载并完成一次推理。
