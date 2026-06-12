#!/usr/bin/env python3
from __future__ import annotations

import math
import time

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from rm_msgs.msg import ArmState
from std_msgs.msg import Empty


JOINTS = [
    ("rm_base", "rm_link1", (0.0, 0.0, 0.2405), (0.0, 0.0, 0.0), (0.0, 0.0, -1.0)),
    ("rm_link1", "rm_link2", (0.0, 0.0, 0.0), (1.5708, -1.5708, 0.0), (0.0, 0.0, 1.0)),
    ("rm_link2", "rm_link3", (0.256, 0.0, 0.0), (0.0, 0.0, 1.5708), (0.0, 0.0, 1.0)),
    ("rm_link3", "rm_link4", (0.0, -0.21, 0.0), (1.5708, 0.0, 0.0), (0.0, 0.0, 1.0)),
    ("rm_link4", "rm_link5", (0.0, 0.0, 0.0), (-1.5708, 0.0, 0.0), (0.0, 0.0, 1.0)),
    ("rm_link5", "rm_link6", (0.0, -0.144, 0.0), (1.5708, 0.0, 0.0), (0.0, 0.0, 1.0)),
]

# Controller tool frame read from the arm: mas_rub relative to flange/default tool.
MAS_RUB_TOOL = ("rm_link6", "rm_mas_rub", (0.0, -0.073916, 0.110916), (0.785, 0.0, 0.0))


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def matvec(m, v):
    return [sum(m[i][j] * v[j] for j in range(3)) for i in range(3)]


def transpose(m):
    return [[m[j][i] for j in range(3)] for i in range(3)]


def hmat(xyz, rot):
    return [
        [rot[0][0], rot[0][1], rot[0][2], xyz[0]],
        [rot[1][0], rot[1][1], rot[1][2], xyz[1]],
        [rot[2][0], rot[2][1], rot[2][2], xyz[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def hmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def hinv(t):
    rot = [[t[i][j] for j in range(3)] for i in range(3)]
    pos = [t[i][3] for i in range(3)]
    rt = transpose(rot)
    inv_pos = [-v for v in matvec(rt, pos)]
    return hmat(inv_pos, rt)


def hxyz(t):
    return (t[0][3], t[1][3], t[2][3])


def hrot(t):
    return [[t[i][j] for j in range(3)] for i in range(3)]


def rpy_to_matrix(roll: float, pitch: float, yaw: float):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = [[1, 0, 0], [0, cr, -sr], [0, sr, cr]]
    ry = [[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]
    rz = [[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]
    return matmul(matmul(rz, ry), rx)


def quat_to_matrix(x: float, y: float, z: float, w: float):
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return [
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ]


def axis_angle_to_matrix(axis, angle: float):
    x, y, z = axis
    n = math.sqrt(x * x + y * y + z * z)
    if n <= 0.0:
        return [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    x, y, z = x / n, y / n, z / n
    c = math.cos(angle)
    s = math.sin(angle)
    t = 1.0 - c
    return [
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ]


def matrix_to_quat(m):
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        return ((m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, 0.25 * s)
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        return (0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s)
    if m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        return ((m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s, (m[0][2] - m[2][0]) / s)
    s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
    return ((m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s, (m[1][0] - m[0][1]) / s)


def make_tf(parent: str, child: str, xyz, rot_matrix, stamp):
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.translation.x = float(xyz[0])
    msg.transform.translation.y = float(xyz[1])
    msg.transform.translation.z = float(xyz[2])
    qx, qy, qz, qw = matrix_to_quat(rot_matrix)
    msg.transform.rotation.x = qx
    msg.transform.rotation.y = qy
    msg.transform.rotation.z = qz
    msg.transform.rotation.w = qw
    return msg


class RmLinkTfBroadcaster:
    def __init__(self):
        self.joints = [0.0] * 6
        self.tool_pos = (0.0, 0.0, 0.0)
        self.tool_quat = (0.0, 0.0, 0.0, 1.0)
        self.last_state_time = 0.0
        self.br = tf2_ros.TransformBroadcaster()
        self.state_pub = rospy.Publisher("/rm_driver/GetCurrentArmState", Empty, queue_size=1)
        rospy.Subscriber("/rm_driver/ArmCurrentState", ArmState, self.on_state, queue_size=5)

    def on_state(self, msg: ArmState):
        self.joints = [float(v) for v in msg.joint[:6]]
        p = msg.Pose.position
        q = msg.Pose.orientation
        self.tool_pos = (float(p.x), float(p.y), float(p.z))
        self.tool_quat = (float(q.x), float(q.y), float(q.z), float(q.w))
        self.last_state_time = time.time()

    def build_transforms(self):
        stamp = rospy.Time.now()
        relative = []
        predicted_tcp = hmat((0.0, 0.0, 0.0), rpy_to_matrix(0.0, 0.0, 0.0))
        for idx, (parent, child, xyz, origin_rpy, axis) in enumerate(JOINTS):
            rot = matmul(rpy_to_matrix(*origin_rpy), axis_angle_to_matrix(axis, self.joints[idx]))
            relative.append(make_tf(parent, child, xyz, rot, stamp))
            predicted_tcp = hmul(predicted_tcp, hmat(xyz, rot))
        parent, child, xyz, rpy = MAS_RUB_TOOL
        tool_rot = rpy_to_matrix(*rpy)
        relative.append(make_tf(parent, child, xyz, tool_rot, stamp))
        predicted_tcp = hmul(predicted_tcp, hmat(xyz, tool_rot))

        actual_tcp = hmat(self.tool_pos, quat_to_matrix(*self.tool_quat))
        world_to_base = hmul(actual_tcp, hinv(predicted_tcp))
        out = [make_tf("world", "rm_base", hxyz(world_to_base), hrot(world_to_base), stamp)]
        out.extend(relative)
        return out

    def spin(self):
        rate = rospy.Rate(20.0)
        next_request = 0.0
        while not rospy.is_shutdown():
            now = time.time()
            if now >= next_request:
                self.state_pub.publish(Empty())
                next_request = now + 0.2
            self.br.sendTransform(self.build_transforms())
            rate.sleep()


def main():
    rospy.init_node("rm_link_tf_broadcaster", anonymous=False)
    node = RmLinkTfBroadcaster()
    rospy.loginfo("Publishing RM65 link TF frames: world -> rm_base -> rm_link1..rm_link6 -> rm_mas_rub")
    node.spin()


if __name__ == "__main__":
    main()
