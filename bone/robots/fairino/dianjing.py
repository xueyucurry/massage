# 兼容三种场景：
# 1) 已安装第三方包 fairino -> from fairino import Robot
# 2) 项目内作为包导入 -> from src.user_functions.fairino import Robot
# 3) 直接运行脚本且本地同目录下有 fairino 目录 -> 动态加入本地路径后 import fairino
try:
    from fairino import Robot  # 第三方包/同目录模块（若运行目录包含当前目录）
except Exception:
    try:
        from src.user_functions.fairino import Robot  # 项目内绝对导入
    except Exception:
        import os, sys, importlib
        _dir = os.path.dirname(__file__)
        if _dir not in sys.path:
            sys.path.insert(0, _dir)
        Robot = importlib.import_module('fairino').Robot
import glob
import json
import os


def _load_camera_to_robot_matrix(path="camera_to_robot.json"):
    search_paths = []
    if path:
        search_paths.append(path)

    module_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.abspath(os.path.join(module_dir, "..", ".."))
    search_paths.extend(
        [
            os.path.join(module_dir, "camera_to_robot.json"),
            os.path.join(repo_root, "shared", "calibration", "camera_to_robot.json"),
        ]
    )

    picked = None
    for candidate in search_paths:
        if candidate and os.path.isfile(candidate):
            picked = candidate
            break
    if picked is None:
        return None
    try:
        with open(picked, "r", encoding="utf-8") as f:
            data = json.load(f)
        m = data.get("matrix", None)
        if not isinstance(m, list) or len(m) != 4:
            return None
        return m
    except Exception:
        return None


def _transform_points(points_xyz, T4):
    out = []
    for p in points_xyz:
        x, y, z = p[0], p[1], p[2]
        qx = T4[0][0] * x + T4[0][1] * y + T4[0][2] * z + T4[0][3]
        qy = T4[1][0] * x + T4[1][1] * y + T4[1][2] * z + T4[1][3]
        qz = T4[2][0] * x + T4[2][1] * y + T4[2][2] * z + T4[2][3]
        out.append([qx, qy, qz])
    return out


def _pick_latest_meridian_json(output_dir="robot_meridian_output"):
    pattern = os.path.join(output_dir, "meridian_data_*.json")
    files = glob.glob(pattern)
    if not files:
        return None
    files.sort(key=os.path.getmtime, reverse=True)
    return files[0]


def _load_points_from_meridian_json(json_path, side="left"):
    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    side = side.lower().strip()
    if side not in ("left", "right"):
        raise ValueError("side 必须是 left 或 right")

    # 优先机械臂坐标；没有则尝试相机坐标（需要用户自己先转换）
    robot_key = f"{side}_meridian_robot"
    cam_key = f"{side}_meridian_camera"
    if robot_key in data and data[robot_key]:
        points = data[robot_key]
        frame = "robot"
    elif cam_key in data and data[cam_key]:
        points = data[cam_key]
        frame = "camera"
    else:
        raise ValueError(f"{json_path} 中没有可用的 {side} 轨迹点")

    cleaned = []
    for p in points:
        if isinstance(p, (list, tuple)) and len(p) >= 3:
            cleaned.append([float(p[0]), float(p[1]), float(p[2])])
    if len(cleaned) < 2:
        raise ValueError("轨迹点数量不足（至少需要2个点）")
    return cleaned, frame


def _load_points_prefer_current_calibration(json_path, side="left", prefer_camera_retransform=True):
    """
    为避免使用旧标定写入的 *_meridian_robot，默认优先:
    *_meridian_camera + 当前 camera_to_robot.json 进行实时转换。
    """
    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    side = side.lower().strip()
    robot_key = f"{side}_meridian_robot"
    cam_key = f"{side}_meridian_camera"

    if prefer_camera_retransform and cam_key in data and data[cam_key]:
        pts = [[float(p[0]), float(p[1]), float(p[2])] for p in data[cam_key] if len(p) >= 3]
        return pts, "camera"

    return _load_points_from_meridian_json(json_path, side=side)


def _to_mm_points(points_xyz):
    """
    自动将点列转为毫米：
    - 若最大绝对值 < 10，视为米，乘 1000
    - 否则视为毫米，保持不变
    """
    m = max(abs(v) for p in points_xyz for v in p)
    scale = 1000.0 if m < 10.0 else 1.0
    return [[p[0] * scale, p[1] * scale, p[2] * scale] for p in points_xyz], scale


def _bbox(points_xyz):
    xs = [p[0] for p in points_xyz]
    ys = [p[1] for p in points_xyz]
    zs = [p[2] for p in points_xyz]
    return (min(xs), max(xs)), (min(ys), max(ys)), (min(zs), max(zs))


def move_by_meridian_json(
    side="left",
    json_path="",
    robot_ip="192.168.58.2",
    speed=8,
    tool=0,
    user=0,
    rx=-178.190,
    ry=1.724,
    rz=-1.187,
    approach_height_mm=25.0,
    press_depth_mm=0.0,
    sample_step=2,
    prefer_camera_retransform=True,
    passes=2,
    spline_avg_time_ms=2000,
):
    """
    读取 yolo 导出的膀胱经轨迹，使用样条运动驱动法奥机械臂沿经络平滑滑动。

    流程:
      1. 接近: MoveCart 到起点上方 -> 下降到接触面
      2. 沿经络滑动: NewSpline 样条平滑运动（每个 pass 沿轨迹走一遍）
      3. 抬起: 结束后抬起到安全高度

    参数:
      passes: 沿经络来回滑动的遍数（默认2，奇数遍正向，偶数遍反向）
      spline_avg_time_ms: 样条点间平均衔接时间(ms)，越大越慢越平滑
    """
    if not json_path:
        json_path = _pick_latest_meridian_json()
    if not json_path or not os.path.isfile(json_path):
        raise FileNotFoundError("未找到轨迹 JSON，请先在 yolo.py 中按 s 保存一帧")

    points, frame = _load_points_prefer_current_calibration(
        json_path,
        side=side,
        prefer_camera_retransform=prefer_camera_retransform,
    )
    if frame == "camera":
        T4 = _load_camera_to_robot_matrix()
        if T4 is None:
            raise RuntimeError(
                "轨迹是相机坐标，但未找到 camera_to_robot.json。"
                "请先做手眼标定并提供 matrix。"
            )
        points = _transform_points(points, T4)
        frame = "robot(from_camera_to_robot)"
    points_mm, scale = _to_mm_points(points)
    sampled = points_mm[::max(1, int(sample_step))]
    if sampled[-1] != points_mm[-1]:
        sampled.append(points_mm[-1])

    print(f"加载轨迹: {json_path}")
    print(f"侧别: {side} | 来源坐标系: {frame}")
    print(f"点数: {len(points_mm)} -> 执行点数: {len(sampled)} | 遍数: {passes}")
    print(f"单位缩放: x{scale} (转为毫米)")
    bx, by, bz = _bbox(sampled)
    print(
        f"轨迹范围(mm): "
        f"X[{bx[0]:.1f},{bx[1]:.1f}] "
        f"Y[{by[0]:.1f},{by[1]:.1f}] "
        f"Z[{bz[0]:.1f},{bz[1]:.1f}]"
    )

    robot = Robot.RPC(robot_ip)
    robot.SetSpeed(speed)

    # --- 1. 接近：移动到起点上方，再下降到接触面 ---
    first = sampled[0]
    approach_pose = [first[0], first[1], first[2] + approach_height_mm, rx, ry, rz]
    contact_pose = [first[0], first[1], first[2] - press_depth_mm, rx, ry, rz]

    print(">>> 移动到起点上方...")
    rtn = robot.MoveCart(desc_pos=approach_pose, tool=tool, user=user, blendT=0.0)
    if rtn != 0:
        print(f"到达起始上方失败，err={rtn}")
        robot.CloseRPC()
        return rtn

    print(">>> 下降到接触面...")
    rtn = robot.MoveCart(desc_pos=contact_pose, tool=tool, user=user, blendT=0.0)
    if rtn != 0:
        print(f"到达起始接触点失败，err={rtn}")
        robot.CloseRPC()
        return rtn

    # --- 2. 沿经络样条滑动 ---
    for p_idx in range(passes):
        # 奇数遍正向，偶数遍反向
        if p_idx % 2 == 0:
            path = sampled
        else:
            path = list(reversed(sampled))

        direction = "正向(颈→尾)" if p_idx % 2 == 0 else "反向(尾→颈)"
        print(f">>> 第 {p_idx + 1}/{passes} 遍 ({direction})，样条运动 {len(path)} 点...")

        rtn = robot.NewSplineStart(type=0, averageTime=spline_avg_time_ms)
        if rtn != 0:
            print(f"NewSplineStart 失败，err={rtn}，回退到逐点 MoveL")
            # 回退：用 MoveL + blendR 实现近似平滑
            for p in path:
                pose = [p[0], p[1], p[2] - press_depth_mm, rx, ry, rz]
                rtn = robot.MoveL(desc_pos=pose, tool=tool, user=user, blendR=50.0)
                if rtn != 0:
                    print(f"MoveL 失败，pose={pose}, err={rtn}")
                    break
            continue

        for i, p in enumerate(path):
            last_flag = 1 if i == len(path) - 1 else 0
            pose = [p[0], p[1], p[2] - press_depth_mm, rx, ry, rz]
            rtn = robot.NewSplinePoint(
                desc_pos=pose, tool=tool, user=user, lastFlag=last_flag
            )
            if rtn != 0:
                print(f"NewSplinePoint 失败，index={i}, pose={pose}, err={rtn}")
                break

        rtn = robot.NewSplineEnd()
        if rtn != 0:
            print(f"NewSplineEnd 失败，err={rtn}")

        print(f"    第 {p_idx + 1} 遍完成")

    # --- 3. 抬起 ---
    last = sampled[-1] if passes % 2 != 0 else sampled[0]
    lift_pose = [last[0], last[1], last[2] + approach_height_mm, rx, ry, rz]
    print(">>> 抬起到安全高度...")
    rtn = robot.MoveCart(desc_pos=lift_pose, tool=tool, user=user, blendT=0.0)

    robot.CloseRPC()
    print(f"轨迹执行完成，err={rtn}")
    return rtn


def dianjing(duration_min: int = 0):
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = Robot.RPC('192.168.58.2')

    desc_pos2 = [221.524,23.448,522.072,88.433,-84.559,72.683]
    desc_pos3 = [147.987,444.138,423.132,-177.959,-4.403,-5.038]
    desc_pos4 = [448.696,413.187,13.058,-179.606,1.734,-38.013]
    desc_pos5 = [448.862,413.610,-29.400,-177.022,1.731,-38.106]

    offset_pos = [0, 0, 0, 0, 0, 0]
    epos = [0, 0, 0, 0]
    tool = 0
    user = 0
    vel = 100.0
    acc = 100.0
    ovl = 100.0
    blendT = 0.0
    blendR = 0.0
    flag = 0
    search = 0

    robot.SetSpeed(15)

    #rtn = robot.MoveCart(desc_pos=desc_pos2, tool=tool, user=user, blendT=blendT)
    #rtn = robot.MoveCart(desc_pos=desc_pos3, tool=tool, user=user, blendT=blendT)
    
    # rtn = robot.MoveCart(desc_pos=desc_pos4, tool=tool, user=user, blendT=blendT)
    # rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    # rtn = robot.MoveCart(desc_pos=desc_pos4, tool=tool, user=user, blendT=blendT)
    # rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    # rtn = robot.MoveCart(desc_pos=desc_pos4, tool=tool, user=user, blendT=blendT)
    # rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    # rtn = robot.MoveL(desc_pos=desc_pos5, tool=tool, user=user)
    
    #第一遍
    desc_pos5 = [107.857,613.762,50.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    #模拟按压
    desc_pos5 = [107.857,613.762,40.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [107.857,613.762,50.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [107.857,613.762,40.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [107.857,613.762,50.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    
    desc_pos5 = [7.857,613.762,55.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [7.857,613.762,45.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [7.857,613.762,55.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [7.857,613.762,45.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [7.857,613.762,55.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    
    
    
    desc_pos5 = [-107.857,613.762,49.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [-107.857,613.762,39.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [-107.857,613.762,49.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [-107.857,613.762,39.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [-107.857,613.762,49.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    
    
    
    
    desc_pos5 = [-207.857,613.762,49.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [-207.857,613.762,39.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [-207.857,613.762,49.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [-207.857,613.762,39.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [-207.857,613.762,49.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    
    
    desc_pos5 = [107.857,613.762,50.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    
    
    desc_pos5 = [107.857,693.762,50.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [107.857,693.762,40.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [107.857,693.762,50.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [107.857,693.762,40.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [107.857,693.762,50.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    
    
    
    desc_pos5 = [7.857,693.762,55.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [7.857,693.762,45.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [7.857,693.762,55.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [7.857,693.762,45.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [7.857,693.762,55.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    
    
    desc_pos5 = [-107.857,693.762,49.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [-107.857,693.762,39.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [-107.857,693.762,49.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [-107.857,693.762,39.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [-107.857,693.762,49.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    
    
    desc_pos5 = [-207.857,693.762,49.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [-207.857,693.762,39.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [-207.857,693.762,49.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [-207.857,693.762,39.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [-207.857,693.762,49.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    
    desc_pos5 = [107.857,693.762,50.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    
    
    
    
    #第二遍
    desc_pos5 = [107.857,613.762,50.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    #模拟按压
    desc_pos5 = [107.857,613.762,40.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [107.857,613.762,50.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [107.857,613.762,40.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [107.857,613.762,50.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    
    desc_pos5 = [7.857,613.762,55.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [7.857,613.762,45.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [7.857,613.762,55.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [7.857,613.762,45.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [7.857,613.762,55.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    
    
    
    desc_pos5 = [-107.857,613.762,49.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [-107.857,613.762,39.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [-107.857,613.762,49.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [-107.857,613.762,39.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [-107.857,613.762,49.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    
    
    
    
    desc_pos5 = [-207.857,613.762,49.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [-207.857,613.762,39.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [-207.857,613.762,49.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [-207.857,613.762,39.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [-207.857,613.762,49.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    
    
    desc_pos5 = [107.857,613.762,50.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    
    
    desc_pos5 = [107.857,693.762,50.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [107.857,693.762,40.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [107.857,693.762,50.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [107.857,693.762,40.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [107.857,693.762,50.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    
    
    
    desc_pos5 = [7.857,693.762,55.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [7.857,693.762,45.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [7.857,693.762,55.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [7.857,693.762,45.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [7.857,693.762,55.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    
    
    desc_pos5 = [-107.857,693.762,49.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [-107.857,693.762,39.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [-107.857,693.762,49.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [-107.857,693.762,39.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [-107.857,693.762,49.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    
    
    desc_pos5 = [-207.857,693.762,49.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [-207.857,693.762,39.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [-207.857,693.762,49.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [-207.857,693.762,39.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    desc_pos5 = [-207.857,693.762,49.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    
    desc_pos5 = [107.857,693.762,50.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    
    
    
    
    
    
    desc_pos5 = [7.857,693.762,279.997,-178.190,1.724,-1.187]
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    
    
    print(f"movel errcode: {rtn}")
    robot.CloseRPC()
    return rtn


def main(duration_min: int = 0):
    mode = input("选择模式：1=固定程序，2=按膀胱经轨迹执行（默认2）: ").strip() or "2"
    if mode == "1":
        return dianjing(duration_min=duration_min)

    side = input("选择轨迹侧别 left/right（默认left）: ").strip() or "left"
    json_path = input("输入轨迹JSON路径（留空则读取最新 meridian_data_*.json）: ").strip()
    speed_str = input("输入机械臂速度(1-100，默认8): ").strip() or "8"
    sample_step_str = input("轨迹抽稀步长(默认2，越大点越少): ").strip() or "2"
    press_str = input("按压深度mm(默认0，纯轨迹跟随): ").strip() or "0"
    approach_str = input("接近高度mm(默认25): ").strip() or "25"
    passes_str = input("沿经络来回遍数(默认2): ").strip() or "2"
    spline_time_str = input("样条衔接时间ms(默认2000，越大越慢越平滑): ").strip() or "2000"
    prefer_str = input("是否优先使用当前标定重算轨迹(y/n，默认y): ").strip().lower() or "y"

    return move_by_meridian_json(
        side=side,
        json_path=json_path,
        speed=int(float(speed_str)),
        sample_step=int(float(sample_step_str)),
        press_depth_mm=float(press_str),
        approach_height_mm=float(approach_str),
        passes=int(float(passes_str)),
        spline_avg_time_ms=int(float(spline_time_str)),
        prefer_camera_retransform=(prefer_str != "n"),
    )


if __name__ == "__main__":
    main()
