#!/usr/bin/env python3
"""ODrive driver for the transporter skid-steer base (4 ODrives, axis0 each).

Based on the tested standalone script (ODrive firmware 0.6.x, USB).

Subscribes
    wheels/cmd              std_msgs/Float64MultiArray [left, right] wheel speed (rad/s)
Publishes
    wheels/state            std_msgs/Float64MultiArray [left, right] mean wheel speed (rad/s)
    odrive/state           std_msgs/String  DISCONNECTED | IDLE | INDEX_SEARCH | ARMING | ARMED
Services (std_srvs/Trigger)
    odrive/index_search    index search on all wheels, then arm (closed loop)
    odrive/idle            disarm all wheels
    odrive/clear_errors

Safety
  * The base can ONLY be armed by a successful index search, and an index
    search is only accepted when ALL ODrives are connected. There is no way to
    arm without it.
  * After any fault (wheel drops out of closed loop, failed arm, USB lost /
    ODrive reconnected) the index is invalidated -> index search again
    (L1 + PS) before the robot can move.
  * ODrive watchdog is fed every control cycle; if this node dies the drives stop.
  * wheels/cmd older than `cmd_timeout` -> zero velocity.
  * If any wheel drops out of closed loop, ALL wheels go idle.
  * Side commands are scaled together so neither exceeds the motor limit
    (keeps the turning curvature).

Install on the robot:  pip install odrive
"""
import math
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

TWO_PI = 2.0 * math.pi


def _enums():
    """Legacy ODrive enum names with numeric fallbacks."""
    try:
        from odrive import enums as e
    except ImportError:
        e = None

    def g(name, default):
        return getattr(e, name, default) if e else default

    return {
        'IDLE': g('AXIS_STATE_IDLE', 1),
        'INDEX': g('AXIS_STATE_ENCODER_INDEX_SEARCH', 6),
        'CLOSED': g('AXIS_STATE_CLOSED_LOOP_CONTROL', 8),
        'VEL_CTRL': g('CONTROL_MODE_VELOCITY_CONTROL', 2),
        'VEL_RAMP': g('INPUT_MODE_VEL_RAMP', 2),
    }


class Wheel:
    def __init__(self, name, serial, side, sign):
        self.name, self.serial, self.side, self.sign = name, serial, side, sign
        self.drv = None

    @property
    def ax(self):
        return self.drv.axis0


class ODriveNode(Node):

    def __init__(self):
        super().__init__('odrive_node')
        p = self.declare_parameter
        names = p('wheel_names', ['FL', 'RL', 'FR', 'RR']).value
        serials = p('wheel_serials',
                    ['003CF6745E24', '003CF67455A2', '003CF6744C10', '003CF6744BA2']).value
        sides = p('wheel_sides', ['L', 'L', 'R', 'R']).value
        signs = p('wheel_signs', [-1.0, 1.0, 1.0, -1.0]).value
        if not (len(names) == len(serials) == len(sides) == len(signs)):
            raise ValueError('wheel_names/serials/sides/signs must have equal length')
        self.wheels = [Wheel(n, s, sd.upper(), float(sg))
                       for n, s, sd, sg in zip(names, serials, sides, signs)]

        self.gear_ratio = p('gear_ratio', 21.097).value          # motor turns / wheel turn
        motor_max_rpm = p('motor_max_rpm', 300.0).value
        speed_fraction = p('speed_fraction', 0.95).value
        self.max_vel = motor_max_rpm / 60.0 * speed_fraction     # motor turns/s
        self.vel_ramp_rate = p('vel_ramp_rate', 200.0).value     # turns/s^2
        self.use_watchdog = p('use_watchdog', True).value
        self.watchdog_timeout = p('watchdog_timeout', 0.5).value
        self.cmd_timeout = p('cmd_timeout', 0.5).value
        self.connect_timeout = p('connect_timeout', 15.0).value
        self.index_timeout = p('index_search_timeout', 15.0).value
        self.arm_timeout = p('arm_timeout', 3.0).value
        # what to do once all drives are connected: none | index_search
        self.startup_action = p('startup_action', 'none').value
        if self.startup_action not in ('none', 'index_search'):
            raise ValueError("startup_action must be 'none' or 'index_search'")
        rate = p('control_rate', 100.0).value

        self.E = _enums()
        self.lock = threading.RLock()
        self.state = 'DISCONNECTED'
        self.state_since = time.monotonic()
        self.connecting = False
        self.cmd = (0.0, 0.0)
        self.last_cmd_time = None
        self.tick = 0
        self.index_done = False
        self.last_not_armed_warn = 0.0

        self.wheel_pub = self.create_publisher(Float64MultiArray, 'wheels/state', 10)
        self.state_pub = self.create_publisher(String, 'odrive/state', 10)
        self.create_subscription(Float64MultiArray, 'wheels/cmd', self.cmd_callback, 10)
        self.create_service(Trigger, 'odrive/index_search', self.srv_index_search)
        self.create_service(Trigger, 'odrive/idle', self.srv_idle)
        self.create_service(Trigger, 'odrive/clear_errors', self.srv_clear_errors)
        self.create_timer(1.0 / rate, self.update)

        try:
            import odrive  # noqa: F401
        except ImportError:
            self.get_logger().fatal('python package `odrive` not installed (pip install odrive)')
            raise

        self.get_logger().info(
            f'Max motor vel {self.max_vel:.2f} turns/s '
            f'= {self.max_vel / self.gear_ratio * TWO_PI:.3f} wheel rad/s')
        self.start_connect()

    # ------------------------------------------------------------- state
    def set_state(self, s):
        if s != self.state:
            self.get_logger().info(f'State {self.state} -> {s}')
            self.state = s
            self.state_since = time.monotonic()
            self.state_pub.publish(String(data=s))

    def in_state_for(self):
        return time.monotonic() - self.state_since

    # -------------------------------------------------------- connection
    def start_connect(self):
        if self.connecting:
            return
        self.connecting = True
        threading.Thread(target=self._connect_worker, daemon=True).start()

    def _connect_worker(self):
        import odrive
        try:
            while rclpy.ok():
                missing = [w for w in self.wheels if w.drv is None]
                if not missing:
                    break
                for w in missing:
                    self.get_logger().info(f'[{w.name}] searching for ODrive {w.serial or "(any)"} ...')
                    try:
                        drv = (odrive.find_any(serial_number=w.serial, timeout=self.connect_timeout)
                               if w.serial else odrive.find_any(timeout=self.connect_timeout))
                    except TimeoutError:
                        self.get_logger().warn(f'[{w.name}] not found, retrying')
                        continue
                    except Exception as e:  # noqa: BLE001
                        self.get_logger().warn(f'[{w.name}] search error: {e}')
                        time.sleep(1.0)
                        continue
                    with self.lock:
                        w.drv = drv
                        try:
                            self._configure(w)
                        except Exception as e:  # noqa: BLE001
                            self.get_logger().error(f'[{w.name}] configure failed: {e}')
                            w.drv = None
                            continue
            if not rclpy.ok():
                return
            with self.lock:
                self.set_state('IDLE')
                self.get_logger().info('All ODrives connected')
                if self.startup_action == 'index_search':
                    self._begin_index_search()
                else:
                    self.get_logger().warn(
                        'NOT ARMED - press L1 + PS (odrive/index_search) to run index search')
        finally:
            self.connecting = False

    def _configure(self, w):
        drv, ax = w.drv, w.ax
        self.get_logger().info(
            f'[{w.name}] connected, fw {drv.fw_version_major}.{drv.fw_version_minor}.'
            f'{drv.fw_version_revision}, vbus {drv.vbus_voltage:.1f} V')
        drv.clear_errors()
        time.sleep(0.1)
        if ax.active_errors:
            self.get_logger().warn(
                f'[{w.name}] error 0x{ax.active_errors:08X} still active after clear '
                '(check vbus / supply / brake resistor)')
        ax.controller.config.control_mode = self.E['VEL_CTRL']
        ax.controller.config.input_mode = self.E['VEL_RAMP']
        ax.controller.config.vel_ramp_rate = self.vel_ramp_rate
        ax.controller.input_vel = 0.0
        if self.use_watchdog:
            ax.config.watchdog_timeout = self.watchdog_timeout
            ax.config.enable_watchdog = True
            ax.watchdog_feed()
        else:
            ax.config.enable_watchdog = False

    def _lost(self, err):
        self.get_logger().error(f'ODrive communication error: {err}')
        for w in self.wheels:
            if w.drv is not None:
                try:
                    w.ax.controller.input_vel = 0.0
                    w.ax.requested_state = self.E['IDLE']
                except Exception:  # noqa: BLE001
                    w.drv = None
        # Re-find any drive that did not answer
        for w in self.wheels:
            if w.drv is not None:
                try:
                    _ = w.ax.current_state
                except Exception:  # noqa: BLE001
                    w.drv = None
        self.index_done = False
        self.set_state('DISCONNECTED')
        self.start_connect()

    # ----------------------------------------------------------- actions
    def _all_idle(self):
        self.index_done = False  # any stop by fault/request -> index search again
        for w in self.wheels:
            w.ax.controller.input_vel = 0.0
            w.ax.requested_state = self.E['IDLE']

    def _begin_index_search(self):
        if any(w.drv is None for w in self.wheels):
            raise RuntimeError('not all ODrives connected')
        self.index_done = False
        for w in self.wheels:
            w.drv.clear_errors()
            w.ax.controller.input_vel = 0.0
            if self.use_watchdog:
                w.ax.watchdog_feed()
            w.ax.requested_state = self.E['INDEX']
        self.set_state('INDEX_SEARCH')

    def _begin_arm(self):
        if not self.index_done or any(w.drv is None for w in self.wheels):
            raise RuntimeError('refusing to arm: index search not completed on all ODrives')
        for w in self.wheels:
            w.drv.clear_errors()
            w.ax.controller.input_vel = 0.0
            if self.use_watchdog:
                w.ax.watchdog_feed()
            w.ax.requested_state = self.E['CLOSED']
        self.set_state('ARMING')

    def _report_errors(self, what):
        for w in self.wheels:
            ax = w.ax
            self.get_logger().error(
                f'[{w.name}] {what}: state={ax.current_state} '
                f'active_errors=0x{ax.active_errors:08X} '
                f'disarm_reason=0x{ax.disarm_reason:08X} '
                f'procedure_result={ax.procedure_result}')

    # ------------------------------------------------------------ ROS IO
    def cmd_callback(self, msg):
        if len(msg.data) >= 2:
            self.cmd = (float(msg.data[0]), float(msg.data[1]))
            self.last_cmd_time = self.get_clock().now()

    def _active_cmd(self):
        if self.last_cmd_time is None:
            return 0.0, 0.0
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        return self.cmd if age < self.cmd_timeout else (0.0, 0.0)

    def _warn_not_armed(self, why):
        """Tell the operator why the robot ignores a drive command (max every 2 s)."""
        if any(abs(c) > 1e-6 for c in self._active_cmd()):
            now = time.monotonic()
            if now - self.last_not_armed_warn > 2.0:
                self.last_not_armed_warn = now
                self.get_logger().warn(f'Drive command ignored: {why}')

    def _side_motor_vel(self):
        """Wheel rad/s -> motor turns/s per side, scaled together to max_vel."""
        left, right = (w / TWO_PI * self.gear_ratio for w in self._active_cmd())
        peak = max(abs(left), abs(right))
        if peak > self.max_vel:
            s = self.max_vel / peak
            left, right = left * s, right * s
        return {'L': left, 'R': right}

    def update(self):
        with self.lock:
            if self.state == 'DISCONNECTED':
                self._warn_not_armed('waiting for all ODrives')
                return
            if self.state != 'ARMED':
                self._warn_not_armed(f'state {self.state}, run index search (L1 + PS)')
            try:
                self._update()
            except Exception as e:  # noqa: BLE001  (USB unplug etc.)
                self._lost(e)

    def _update(self):
        if self.use_watchdog:
            for w in self.wheels:
                w.ax.watchdog_feed()

        if self.state == 'INDEX_SEARCH':
            if self.in_state_for() > 0.3 and all(
                    w.ax.current_state == self.E['IDLE'] for w in self.wheels):
                if any(w.ax.procedure_result != 0 or w.ax.active_errors for w in self.wheels):
                    self._report_errors('index search failed')
                    self.set_state('IDLE')
                else:
                    self.get_logger().info('Index search done on all wheels')
                    self.index_done = True
                    self._begin_arm()
            elif self.in_state_for() > self.index_timeout:
                self._report_errors('index search timeout')
                self._all_idle()
                self.set_state('IDLE')

        elif self.state == 'ARMING':
            if all(w.ax.current_state == self.E['CLOSED'] for w in self.wheels):
                self.set_state('ARMED')
            elif self.in_state_for() > self.arm_timeout:
                self._report_errors('arm failed')
                self._all_idle()
                self.set_state('IDLE')

        elif self.state == 'ARMED':
            dropped = [w for w in self.wheels if w.ax.current_state != self.E['CLOSED']]
            if dropped:
                self._report_errors(f'{",".join(w.name for w in dropped)} dropped out of closed loop')
                self._all_idle()  # one wheel faulting stops ALL wheels
                self.set_state('IDLE')
            else:
                side = self._side_motor_vel()
                for w in self.wheels:
                    w.ax.controller.input_vel = w.sign * side[w.side]

        # telemetry at ~10 Hz
        self.tick += 1
        if self.tick % 10 == 0:
            acc = {'L': [], 'R': []}
            for w in self.wheels:
                motor = w.sign * w.ax.pos_vel_mapper.vel
                acc[w.side].append(motor / self.gear_ratio * TWO_PI)
            mean = [sum(v) / len(v) if v else 0.0 for v in (acc['L'], acc['R'])]
            self.wheel_pub.publish(Float64MultiArray(data=mean))
            self.state_pub.publish(String(data=self.state))

    # ---------------------------------------------------------- services
    def _service(self, res, fn, ok_msg, allowed=None):
        with self.lock:
            if self.state == 'DISCONNECTED':
                res.success, res.message = False, 'ODrives not connected'
                return res
            if allowed and self.state not in allowed:
                res.success, res.message = False, f'Busy ({self.state})'
                return res
            try:
                fn()
                res.success, res.message = True, ok_msg
            except Exception as e:  # noqa: BLE001
                res.success, res.message = False, str(e)
                self._lost(e)
        return res

    def srv_index_search(self, _req, res):
        return self._service(res, self._begin_index_search, 'Index search started',
                             allowed=('IDLE', 'ARMED'))

    def srv_idle(self, _req, res):
        def go():
            self._all_idle()
            self.set_state('IDLE')
        return self._service(res, go, 'Idle (index search required to re-arm)')

    def srv_clear_errors(self, _req, res):
        return self._service(res, lambda: [w.drv.clear_errors() for w in self.wheels],
                             'Errors cleared')

    def shutdown(self):
        with self.lock:
            for w in self.wheels:
                if w.drv is None:
                    continue
                try:
                    w.ax.controller.input_vel = 0.0
                    w.ax.requested_state = self.E['IDLE']
                    w.ax.config.enable_watchdog = False
                    self.get_logger().info(f'[{w.name}] idle')
                except Exception as e:  # noqa: BLE001
                    self.get_logger().warn(f'[{w.name}] disarm error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ODriveNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
