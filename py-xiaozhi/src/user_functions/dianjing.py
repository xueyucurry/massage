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
    return dianjing(duration_min=duration_min)


if __name__ == "__main__":
    main()
