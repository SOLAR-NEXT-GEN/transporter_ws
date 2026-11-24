#!/usr/bin/env python3
"""Bridge between high level manipulator commands and the micro-ROS firmware.

Subscribes
    manipulator/cmd  std_msgs/Int8MultiArray  data=[left, right]  (from joy, etc.)
    manipulator/hw_state       std_msgs/Int8MultiArray  state reported by firmware
Publishes
    manipulator/hw_cmd         std_msgs/Int8MultiArray  data=[left, right]  -> firmware

Values: 1 = up, -1 = down, 0 = hold.
manipulator/hw_cmd is re-published at a fixed rate (the firmware holds both sides if it
gets nothing for 500 ms). If manipulator/cmd is not received for `cmd_timeout`
seconds, HOLD is sent.

Firmware output table (transport_mani):
    up   : PUMP_EN=1  UP=1  DOWN=0
    down : PUMP_EN=1  UP=0  DOWN=1
    hold : PUMP_EN=0  UP=0  DOWN=0
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray

UP, HOLD, DOWN = 1, 0, -1
NAMES = {UP: 'UP', HOLD: 'HOLD', DOWN: 'DOWN'}


def sanitize(v):
    return v if v in (UP, DOWN) else HOLD


class ManipulatorController(Node):

    def __init__(self):
        super().__init__('manipulator_controller')
        rate = self.declare_parameter('publish_rate', 20.0).value
        self.cmd_timeout = self.declare_parameter('cmd_timeout', 0.3).value

        self.pub = self.create_publisher(Int8MultiArray, 'manipulator/hw_cmd', 10)
        self.create_subscription(Int8MultiArray, 'manipulator/cmd', self.cmd_callback, 10)
        self.create_subscription(Int8MultiArray, 'manipulator/hw_state', self.state_callback, 10)
        self.create_timer(1.0 / rate, self.update)

        self.cmd = (HOLD, HOLD)
        self.last_cmd_time = None
        self.fw_state = None
        self.get_logger().info('Manipulator controller ready (publishing manipulator/hw_cmd)')

    def cmd_callback(self, msg):
        if len(msg.data) >= 2:
            self.cmd = (sanitize(msg.data[0]), sanitize(msg.data[1]))
        else:
            self.cmd = (HOLD, HOLD)
        self.last_cmd_time = self.get_clock().now()

    def state_callback(self, msg):
        if len(msg.data) < 2:
            return
        state = (int(msg.data[0]), int(msg.data[1]))
        if state != self.fw_state:
            self.get_logger().info(
                f'Firmware state L={NAMES.get(state[0], state[0])} '
                f'R={NAMES.get(state[1], state[1])}')
            self.fw_state = state

    def update(self):
        cmd = (HOLD, HOLD)
        if self.last_cmd_time is not None:
            age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
            if age < self.cmd_timeout:
                cmd = self.cmd
        self.pub.publish(Int8MultiArray(data=list(cmd)))


def main(args=None):
    rclpy.init(args=args)
    node = ManipulatorController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.pub.publish(Int8MultiArray(data=[HOLD, HOLD]))
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
