"""Helpers compartilhados entre os nós ROS deste pacote."""
import math


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Quaternion (x,y,z,w) → yaw em rad. Convenção ROS REP-103."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_pi(a: float) -> float:
    """Envelopa ângulo em (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))
