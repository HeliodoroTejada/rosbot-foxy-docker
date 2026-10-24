"""Planar kinematics, with no dependency on ROS."""
import math


def planar_pose(position, quaternion):
    """Return x, y, yaw after validating and normalizing the quaternion."""
    if not all(math.isfinite(v) for v in (*position, *quaternion)):
        raise ValueError('Pose contains non-finite values')
    norm = math.sqrt(sum(v * v for v in quaternion))
    if norm < 1e-8:
        raise ValueError('Pose has a zero quaternion')
    x, y, z, w = (v / norm for v in quaternion)
    yaw = math.atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z))
    return position[0], position[1], yaw


def body_twist(previous, current, dt):
    """Finite-difference world position; express planar velocity in current body."""
    if not math.isfinite(dt) or dt <= 0.0:
        raise ValueError('dt must be positive')
    vx = (current[0] - previous[0]) / dt
    vy = (current[1] - previous[1]) / dt
    yaw = current[2]
    dyaw = math.atan2(math.sin(yaw-previous[2]), math.cos(yaw-previous[2]))
    return (math.cos(yaw)*vx + math.sin(yaw)*vy,
            -math.sin(yaw)*vx + math.cos(yaw)*vy, dyaw/dt)
