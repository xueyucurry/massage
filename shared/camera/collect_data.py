import pyrealsense2 as rs
import numpy as np
import cv2
import os
import time

# --------------------------
# 1. 配置保存路径与显示选项
# --------------------------
save_path = "./collected_data"
if not os.path.exists(save_path):
    os.makedirs(save_path)
    print(f"文件夹 {save_path} 创建成功")

# 显示选项（为减少卡顿，默认只显示彩色，不拼接）
SHOW_COLOR = True       # 显示彩色画面
SHOW_DEPTH = False      # 显示深度画面（需要将此项改为 True）
USE_COLOR_MAP = False   # 为深度图使用伪彩色渲染（SHOW_DEPTH 为 True 时可选）

# --------------------------
# 2. 初始化相机流
# --------------------------
# 检查是否有可用的 RealSense 设备
ctx = rs.context()
devices = ctx.query_devices()
if len(devices) == 0:
    print("错误：未检测到 RealSense 相机！")
    print("请检查：")
    print("1. 相机是否正确连接到 USB 端口")
    print("2. USB 线是否支持数据传输（不是仅充电线）")
    print("3. 相机驱动是否正确安装")
    exit(1)

print(f"检测到 {len(devices)} 个 RealSense 设备")

pipeline = rs.pipeline()
config = rs.config()

# 配置分辨率 (640x480 是 D435i 最稳定的分辨率)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# 开始采集
try:
    profile = pipeline.start(config)
    print("相机配置成功，正在初始化...")
except Exception as e:
    print(f"启动相机失败：{e}")
    print("可能的原因：")
    print("1. 相机被其他程序占用")
    print("2. 相机配置不匹配（分辨率/帧率）")
    print("3. USB 连接不稳定")
    exit(1)

# *** 关键步骤：创建对齐对象 ***
# 这会让深度图自动变换，去匹配彩色图的视角
align_to = rs.stream.color
align = rs.align(align_to)

# 等待几帧让相机稳定（跳过前几帧）
print("等待相机稳定...")
for _ in range(30):
    pipeline.wait_for_frames()

print("相机已启动...")
print("按 's' 保存当前帧")
print("按 'q' 退出程序")

try:
    count = 0
    while True:
        # 等待一帧数据（设置超时时间为 10 秒）
        try:
            frames = pipeline.wait_for_frames(timeout_ms=10000)
        except RuntimeError as e:
            print(f"获取帧数据超时：{e}")
            print("请检查相机连接，或按 'q' 退出")
            # 继续循环，尝试重新获取
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            continue
        
        # *** 关键步骤：执行对齐 ***
        aligned_frames = align.process(frames)
        
        # 获取对齐后的深度帧和彩色帧
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not aligned_depth_frame or not color_frame:
            continue

        # 转换为 numpy 数组
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # 选择显示内容（去掉 np.hstack，减少拷贝开销）
        if SHOW_DEPTH:
            # 深度图可选伪彩色，避免额外开销时可关闭
            if USE_COLOR_MAP:
                depth_display = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_image, alpha=0.03),
                    cv2.COLORMAP_JET
                )
            else:
                # 转 8-bit 方便显示，但不保存此版本
                depth_display = cv2.convertScaleAbs(depth_image, alpha=0.03)
            display_image = depth_display
        else:
            display_image = color_image

        cv2.imshow('RealSense Data Collection', display_image)

        # 监听键盘输入
        key = cv2.waitKey(1)
        
        # 按 's' 保存数据
        if key & 0xFF == ord('s'):
            timestamp = int(time.time())
            
            # 1. 保存彩色图 (.jpg)
            cv2.imwrite(f"{save_path}/color_{count}_{timestamp}.jpg", color_image)
            
            # 2. 保存原始深度数据 (.png 16-bit) -> 只有这种格式才能保留真实的毫米级距离信息
            # 注意：普通的 jpg 会丢失深度精度，必须存为 png 或 npy
            cv2.imwrite(f"{save_path}/depth_raw_{count}_{timestamp}.png", depth_image)
            
            # 3. (可选) 保存可视化的深度图 (.jpg) -> 方便人眼查看
            # 如果开启伪彩色，则用伪彩色；否则用灰度深度
            depth_view_to_save = (
                cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
                if USE_COLOR_MAP else cv2.convertScaleAbs(depth_image, alpha=0.03)
            )
            cv2.imwrite(f"{save_path}/depth_view_{count}_{timestamp}.jpg", depth_view_to_save)
            
            print(f"已保存第 {count} 组数据到 {save_path}")
            count += 1

        # 按 'q' 退出
        elif key & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()