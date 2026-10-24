#!/usr/bin/env python3
"""Adapt existing planar /pose to /odom + TF; this is not a new estimator.

By default twist comes from finite differences, never from /cmd_vel.
The optional velocity mode requires measured, body-frame /velocity in SI units.
Covariances default to large placeholders, not calibrated uncertainties.
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from planar_math import planar_pose, body_twist


class PoseToOdom(Node):
    def __init__(self):
        super().__init__('pose_to_odom')
        defaults = {
            'pose_topic': '/pose', 'odom_topic': '/odom',
            'odom_frame': 'odom', 'base_frame': 'base_link',
            'twist_source': 'pose', 'velocity_topic': '/velocity',
            'max_velocity_age': 0.25, 'max_pose_gap': 0.5,
            'pose_variance': 1e6, 'twist_variance': 1e6,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.config = {key: self.get_parameter(key).value for key in defaults}
        c = self.config
        if c['twist_source'] not in ('pose', 'velocity'):
            raise ValueError('twist_source must be pose or velocity')
        if not c['odom_frame'] or not c['base_frame'] or c['odom_frame'] == c['base_frame']:
            raise ValueError('Distinct, non-empty odom_frame and base_frame required')
        if any(c[k] <= 0.0 or not math.isfinite(c[k]) for k in
               ('max_velocity_age', 'max_pose_gap', 'pose_variance', 'twist_variance')):
            raise ValueError('Timing and variance parameters must be finite and positive')
        self.previous = None
        self.velocity = None
        self.warned = set()
        self.publisher = self.create_publisher(Odometry, c['odom_topic'], 10)
        self.broadcaster = TransformBroadcaster(self)
        self.pose_sub = self.create_subscription(
            PoseStamped, c['pose_topic'], self.on_pose, qos_profile_sensor_data)
        self.velocity_sub = None
        if c['twist_source'] == 'velocity':
            self.velocity_sub = self.create_subscription(
                Twist, c['velocity_topic'], self.on_velocity, qos_profile_sensor_data)
        self.get_logger().info('Adapting planar /pose: no new fusion or covariance estimate. '
                               'twist_source=' + c['twist_source'])

    def warn_once(self, key, message):
        if key not in self.warned:
            self.warned.add(key)
            self.get_logger().warning(message)

    def on_velocity(self, message):
        if all(math.isfinite(v) for v in
               (message.linear.x, message.linear.y, message.angular.z)):
            self.velocity = (self.get_clock().now().nanoseconds, message)

    def on_pose(self, message):
        c = self.config
        if message.header.frame_id.lstrip('/') != c['odom_frame']:
            self.warn_once('frame', 'Rejected /pose: header.frame_id must describe coordinates '
                           'in ' + c['odom_frame'] + '. Inspect the firmware; do not just rename frames.')
            return
        stamp_ns = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
        if stamp_ns <= 0:
            self.warn_once('stamp', 'Rejected zero timestamp; check rosserial time synchronization.')
            return
        p, q = message.pose.position, message.pose.orientation
        try:
            current = planar_pose((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
        except ValueError as error:
            self.warn_once('pose', str(error))
            return
        if self.previous and stamp_ns <= self.previous[0]:
            self.previous = None
            self.warn_once('time_reset', 'Non-increasing pose time: sample dropped, derivative reset.')
            return
        twist = None
        if c['twist_source'] == 'pose' and self.previous:
            dt = (stamp_ns - self.previous[0]) * 1e-9
            if 1e-4 <= dt <= c['max_pose_gap']:
                twist = body_twist(self.previous[1], current, dt)
        elif c['twist_source'] == 'velocity' and self.velocity:
            age = (self.get_clock().now().nanoseconds - self.velocity[0]) * 1e-9
            if 0.0 <= age <= c['max_velocity_age']:
                v = self.velocity[1]
                twist = (v.linear.x, v.linear.y, v.angular.z)
        self.previous = (stamp_ns, current)
        odom = Odometry()
        odom.header.stamp = message.header.stamp
        odom.header.frame_id = c['odom_frame']
        odom.child_frame_id = c['base_frame']
        odom.pose.pose.position.x, odom.pose.pose.position.y = current[:2]
        odom.pose.pose.orientation.z = math.sin(current[2] / 2.0)
        odom.pose.pose.orientation.w = math.cos(current[2] / 2.0)
        for i in range(6):
            odom.pose.covariance[i*7] = c['pose_variance']
            odom.twist.covariance[i*7] = 1e6
        if twist is not None:
            odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z = twist
            for index in (0, 7, 35):
                odom.twist.covariance[index] = c['twist_variance']
        else:
            self.warn_once('twist', 'No valid velocity for a pose sample: zero twist with large '
                           'covariance. For velocity mode, verify fresh measured /velocity before Nav2.')
        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = odom.child_frame_id
        transform.transform.translation.x = current[0]
        transform.transform.translation.y = current[1]
        transform.transform.rotation = odom.pose.pose.orientation
        self.publisher.publish(odom)
        self.broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = PoseToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
