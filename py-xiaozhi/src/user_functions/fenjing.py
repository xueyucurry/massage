try:
    from fairino import Robot
except Exception:
    try:
        from src.user_functions.fairino import Robot
    except Exception:
        import os, sys, importlib
        _dir = os.path.dirname(__file__)
        if _dir not in sys.path:
            sys.path.insert(0, _dir)
        Robot = importlib.import_module('fairino').Robot


def fenjing(duration_min: int = 0):
    # 与机器人控制器建立连接，连接成功返回一个机器人对象
    robot = Robot.RPC('192.168.58.2')

    desc_pos2 = [221.524,23.448,522.072,88.433,-84.559,72.683]
    desc_pos3 = [147.987,444.138,423.132,-177.959,-4.403,-5.038]
    desc_pos4 = [448.696,413.187,13.058,-179.606,1.734,-38.013]
    desc_pos5 = [448.862,413.610,-29.400,-177.022,1.731,-38.106]
    desc_pos6 = [322.637,425.399,-22.323,-178.408,1.719,-26.734]
    desc_pos7 = [579.033,408.673,-37.185,179.360,1.726,-46.969]

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

    robot.SetSpeed(30)

    #rtn = robot.MoveCart(desc_pos=desc_pos2, tool=tool, user=user, blendT=blendT)
    #rtn = robot.MoveCart(desc_pos=desc_pos3, tool=tool, user=user, blendT=blendT)
    #rtn = robot.MoveCart(desc_pos=desc_pos4, tool=tool, user=user, blendT=blendT)
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    rtn = robot.MoveCart(desc_pos=desc_pos6, tool=tool, user=user, blendT=blendT)
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    rtn = robot.MoveCart(desc_pos=desc_pos7, tool=tool, user=user, blendT=blendT)
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)
    rtn = robot.MoveCart(desc_pos=desc_pos6, tool=tool, user=user, blendT=blendT)
    rtn = robot.MoveCart(desc_pos=desc_pos5, tool=tool, user=user, blendT=blendT)

    print(f"movel errcode: {rtn}")
    robot.CloseRPC()
    return rtn


def main(duration_min: int = 0):
    return fenjing(duration_min=duration_min)


if __name__ == "__main__":
    main()
