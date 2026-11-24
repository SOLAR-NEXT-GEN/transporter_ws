#!/usr/bin/env python3
"""Differential drive kinematics.

Subscribes
    cmd_vel     geometry_msgs/Twist
Publishes
    wheels/cmd   std_msgs/Float64MultiArray  data=[left, right] wheel speed (rad/s)

If cmd_vel stops arriving for `cmd_timeout` seconds, zero is published.
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


def clamp(v, lim):
    return max(-lim, min(lim, v))


class MobileController(Node):

    def __init__(self):
        super().__init__('mobile_controller')
        p = self.declare_parameter
        self.wheel_radius = p('wheel_radius', 0.139).value          # m
        self.wheel_separation = p('wheel_separation', 4.0).value  # m
        self.max_linear = p('max_linear_velocity', 0.2).value     # m/s
        self.max_angular = p('max_angular_velocity', 0.1).value   # rad/s
        self.cmd_timeout = p('cmd_timeout', 0.5).value            # s
        rate = p('publish_rate', 50.0).value

        self.pub = self.create_publisher(Float64MultiArray, 'wheels/cmd', 10)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.create_timer(1.0 / rate, self.update)

        self.cmd = Twist()
        self.last_cmd_time = None
        self.get_logger().info(
            f'Mobile controller: r={self.wheel_radius} m, L={self.wheel_separation} m')

    def cmd_callback(self, msg):
        self.cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def update(self):
        v = w = 0.0
        if self.last_cmd_time is not None:
            age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
            if age < self.cmd_timeout:
                v = clamp(self.cmd.linear.x, self.max_linear)
                w = clamp(self.cmd.angular.z, self.max_angular)

        half = self.wheel_separation / 2.0
        left = (v - w * half) / self.wheel_radius
        right = (v + w * half) / self.wheel_radius
        self.pub.publish(Float64MultiArray(data=[left, right]))


def main(args=None):
    rclpy.init(args=args)
    node = MobileController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.pub.publish(Float64MultiArray(data=[0.0, 0.0]))
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
