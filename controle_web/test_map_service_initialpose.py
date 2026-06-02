import math
from map_service import build_initialpose
from builtin_interfaces.msg import Time


def test_build_initialpose_frame_position_quat_cov():
    msg = build_initialpose(1.0, 2.0, math.pi / 2, Time())
    assert msg.header.frame_id == 'map'
    assert abs(msg.pose.pose.position.x - 1.0) < 1e-9
    assert abs(msg.pose.pose.position.y - 2.0) < 1e-9
    # yaw=pi/2 -> qz=qw=0.7071
    assert abs(msg.pose.pose.orientation.z - 0.70710678) < 1e-3
    assert abs(msg.pose.pose.orientation.w - 0.70710678) < 1e-3
    assert msg.pose.covariance[0] == 0.25     # var x
    assert msg.pose.covariance[7] == 0.25     # var y
    assert msg.pose.covariance[35] > 0.0      # var yaw
