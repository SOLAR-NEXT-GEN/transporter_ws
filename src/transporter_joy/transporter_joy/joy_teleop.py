#!/usr/bin/env python3
"""PS5 teleop for the transporter.

Mapping (indices are parameters, see transporter_bringup/config/joy.yaml):

    L1                 deadman - nothing moves unless L1 is held
    L1 + D-pad         mobile: up/down = forward/backward, left/right = turn
    L1 + PS            ODrive index search (rising edge, one call per press)
    L1 + Triangle      left manipulator up
    L1 + Square        left manipulator down
    L1 + Circle        right manipulator up
    L1 + Cross (X)     right manipulator down

Nothing pressed / L1 released / joystick lost -> zero velocity and both
manipulators HOLD.

Joystick safety
  * No /joy for `joy_timeout` s (controller off, out of range, unplugged,
    joy_node dead) -> robot disabled. joy_node must run with autorepeat_rate
    (20 Hz) so a connected but untouched controller still publishes.
  * After the joystick (re)connects, L1 must be RELEASED once before control
    is enabled, so a button held/stuck during reconnect cannot move the robot.

Publishes
    cmd_vel           geometry_msgs/Twist
    manipulator/cmd   std_msgs/Int8MultiArray  data=[left, right]
                      1 = up, -1 = down, 0 = hold
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8MultiArray
from std_srvs.srv import Trigger

UP, HOLD, DOWN = 1, 0, -1


class JoyTeleop(Node):

    def __init__(self):
        super().__init__('transporter_joy')

        p = self.declare_parameter
        self.enable_mobile = p('enable_mobile', True).value
        self.enable_manipulator = p('enable_manipulator', True).value
        publish_rate = p('publish_rate', 20.0).value
        self.joy_timeout = p('joy_timeout', 0.5).value
        self.linear_speed = p('linear_speed', 0.2).value     # m/s
        self.angular_speed = p('angular_speed', 0.1).value    # rad/s

        # Default indices: ROS 2 `joy` package (SDL GameController) + DualSense
        self.btn_deadman = p('button_deadman', 9).value          # L1
        self.btn_index_search = p('button_index_search', 5).value  # PS
        self.btn_left_up = p('button_left_up', 3).value          # Triangle
        self.btn_left_down = p('button_left_down', 2).value      # Square
        self.btn_right_up = p('button_right_up', 1).value        # Circle
        self.btn_right_down = p('button_right_down', 0).value    # Cross

        # D-pad: 'buttons' (joy / SDL) or 'axes' (joy_linux / hat axes)
        self.dpad_source = p('dpad_source', 'buttons').value
        self.btn_dpad_up = p('button_dpad_up', 11).value
        self.btn_dpad_down = p('button_dpad_down', 12).value
        self.btn_dpad_left = p('button_dpad_left', 13).value
        self.btn_dpad_right = p('button_dpad_right', 14).value
        self.axis_dpad_x = p('axis_dpad_x', 6).value  # +1 = left
        self.axis_dpad_y = p('axis_dpad_y', 7).value  # +1 = up

        index_srv = p('index_search_service', '/odrive/index_search').value

        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.mani_pub = self.create_publisher(Int8MultiArray, 'manipulator/cmd', 10)
        self.index_client = self.create_client(Trigger, index_srv)
        self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.create_timer(1.0 / publish_rate, self.update)

        self.joy = None
        self.last_joy_time = None
        self.prev_index_combo = False
        self.prev_mani = (HOLD, HOLD)
        self.connected = False
        # After the joystick (re)connects, L1 must be released once before
        # anything is enabled -> no motion from a button held during reconnect.
        self.locked = True
        self.last_wait_log = 0.0

        self.get_logger().info(
            f'PS5 teleop ready (mobile={self.enable_mobile}, '
            f'manipulator={self.enable_manipulator}, dpad={self.dpad_source}). '
            'Hold L1 to enable.')

    # ------------------------------------------------------------------ input
    def joy_callback(self, msg):
        self.joy = msg
        self.last_joy_time = self.get_clock().now()

    def button(self, idx):
        b = self.joy.buttons
        return 0 <= idx < len(b) and b[idx] == 1

    def axis(self, idx):
        a = self.joy.axes
        return a[idx] if 0 <= idx < len(a) else 0.0

    def dpad(self):
        """Return (forward, left) each in {-1, 0, 1}."""
        if self.dpad_source == 'axes':
            y, x = self.axis(self.axis_dpad_y), self.axis(self.axis_dpad_x)
            fwd = 1 if y > 0.5 else (-1 if y < -0.5 else 0)
            left = 1 if x > 0.5 else (-1 if x < -0.5 else 0)
            return fwd, left
        fwd = int(self.button(self.btn_dpad_up)) - int(self.button(self.btn_dpad_down))
        left = int(self.button(self.btn_dpad_left)) - int(self.button(self.btn_dpad_right))
        return fwd, left

    @staticmethod
    def side_cmd(up, down):
        if up and not down:
            return UP
        if down and not up:
            return DOWN
        return HOLD

    def joy_alive(self):
        if self.joy is None:
            return False
        age = (self.get_clock().now() - self.last_joy_time).nanoseconds * 1e-9
        return age < self.joy_timeout

    # ----------------------------------------------------------------- output
    def check_connection(self):
        """Track joystick connection. Returns True when control is allowed."""
        alive = self.joy_alive()
        now = self.get_clock().now().nanoseconds * 1e-9

        if alive and not self.connected:
            self.connected = True
            self.locked = True
            self.get_logger().info('Joystick connected - release L1 once to enable control')
        elif not alive and self.connected:
            self.connected = False
            self.locked = True
            self.get_logger().error(
                'Joystick LOST - mobile stopped, manipulators HOLD')
        elif not alive and now - self.last_wait_log > 5.0:
            self.last_wait_log = now
            self.get_logger().warn('Waiting for joystick (/joy) - robot disabled')

        if not alive:
            return False
        if self.locked:
            if not self.button(self.btn_deadman):
                self.locked = False
                self.get_logger().info('Control enabled - hold L1 to operate')
            return False
        return True

    def update(self):
        twist = Twist()
        mani = (HOLD, HOLD)
        index_combo = False

        if self.check_connection() and self.button(self.btn_deadman):
            fwd, left = self.dpad()
            twist.linear.x = fwd * self.linear_speed
            twist.angular.z = left * self.angular_speed

            mani = (
                self.side_cmd(self.button(self.btn_left_up), self.button(self.btn_left_down)),
                self.side_cmd(self.button(self.btn_right_up), self.button(self.btn_right_down)),
            )
            index_combo = self.button(self.btn_index_search)

        if self.enable_mobile:
            self.twist_pub.publish(twist)
            if index_combo and not self.prev_index_combo:
                self.request_index_search()
        self.prev_index_combo = index_combo

        if self.enable_manipulator:
            self.mani_pub.publish(Int8MultiArray(data=list(mani)))
            if mani != self.prev_mani:
                names = {UP: 'UP', HOLD: 'HOLD', DOWN: 'DOWN'}
                self.get_logger().info(
                    f'Manipulator L={names[mani[0]]} R={names[mani[1]]}')
            self.prev_mani = mani

    def request_index_search(self):
        if not self.index_client.service_is_ready():
            self.get_logger().warn(
                f'Index search service {self.index_client.srv_name} not available')
            return
        self.get_logger().info('Requesting ODrive index search')
        future = self.index_client.call_async(Trigger.Request())
        future.add_done_callback(self._index_done)

    def _index_done(self, future):
        try:
            res = future.result()
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f'Index search call failed: {e}')
            return
        log = self.get_logger().info if res.success else self.get_logger().warn
        log(f'Index search: {res.message}')

    def stop(self):
        if self.enable_mobile:
            self.twist_pub.publish(Twist())
        if self.enable_manipulator:
            self.mani_pub.publish(Int8MultiArray(data=[HOLD, HOLD]))


def main(args=None):
    rclpy.init(args=args)
    node = JoyTeleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
