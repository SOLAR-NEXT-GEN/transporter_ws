#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy

class TryManiControl(Node):
    def __init__(self):
        super().__init__('try_mani_control')
        
        # Subscribers
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.state_sub = self.create_subscription(Float64MultiArray, 'hinge_state', self.state_callback, 10)

        # Service clients
        self.mani_up_client = self.create_client(Trigger, 'mani_control_up')
        self.mani_down_client = self.create_client(Trigger, 'mani_control_down')
        
        # State tracking
        self.prev_x_btn = 0
        self.prev_stop_btn = 0
        self.current_states = [0, 0]
        self.flex_sensors = [0, 0]
        self.is_walking = False

        self.flex_timer = None
        self.flex_triggered = False
        
        self.get_logger().info('Try Mani Control Node started')
        self.get_logger().info('Button[0]: Start (UP -> walk)')
        self.get_logger().info('Button[2]: Stop (stop -> DOWN)')
        self.get_logger().info('Flex sensor: Auto stop when flex=1 detected')

    # Replace the state_callback method:
    def state_callback(self, msg):
        """Monitor hinge states and flex sensors"""
        if len(msg.data) >= 4:
            old_flex = self.flex_sensors.copy()
            self.current_states = [int(msg.data[0]), int(msg.data[1])]
            self.flex_sensors = [int(msg.data[2]), int(msg.data[3])]
            
            # Check for flex sensor trigger (auto stop with 2-second delay)
            if self.is_walking and (self.flex_sensors[0] == 1 or self.flex_sensors[1] == 1):
                if old_flex != self.flex_sensors and not self.flex_triggered:  # Only trigger on change
                    self.get_logger().info(f'FLEX SENSOR TRIGGERED! FL={self.flex_sensors[0]}, BL={self.flex_sensors[1]}')
                    self.get_logger().info('Walking for 2 more seconds before stopping...')
                    self.flex_triggered = True
                    
                    # Start 2-second timer
                    if self.flex_timer:
                        self.flex_timer.cancel()
                    self.flex_timer = self.create_timer(2.0, self.flex_timer_callback)

    # Add this new method:
    def flex_timer_callback(self):
        """Called after 2-second delay from flex trigger"""
        self.get_logger().info('2-second delay complete - now stopping')
        self.flex_timer.cancel()
        self.flex_timer = None
        self.flex_triggered = False
        self.handle_stop("flex sensor trigger (after 2s delay)")

    def joy_callback(self, msg: Joy):
        """Handle joystick button presses"""
        if not msg.buttons or len(msg.buttons) < 3:
            return
            
        x_btn = msg.buttons[0]
        stop_btn = msg.buttons[2]
        
        # Start button pressed
        if self.prev_x_btn == 0 and x_btn == 1:
            if not self.is_walking:
                self.handle_start()
            else:
                self.get_logger().info('Already walking - ignoring start button')
        
        # Stop button pressed
        elif self.prev_stop_btn == 0 and stop_btn == 1:
            if self.is_walking:
                self.handle_stop("button press")
            else:
                self.get_logger().info('Not walking - ignoring stop button')
        
        self.prev_x_btn = x_btn
        self.prev_stop_btn = stop_btn

    def handle_start(self):
        """Handle start sequence: UP -> walk"""
        self.get_logger().info('=== START SEQUENCE ===')
        
        # Call UP service and wait for success
        if not self.mani_up_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Manipulator UP service not available')
            return
            
        self.get_logger().info('Calling UP service...')
        request = Trigger.Request()
        
        try:
            future = self.mani_up_client.call_async(request)
            future.add_done_callback(self.up_service_callback)
                
        except Exception as e:
            self.get_logger().error(f'UP service call failed: {str(e)}')

    def handle_stop(self, trigger_source):
        """Handle stop sequence: stop -> DOWN"""
        self.get_logger().info(f'=== STOP SEQUENCE (triggered by {trigger_source}) ===')
        
        # Log stop
        self.get_logger().info('🛑 STOP - Robot stopped walking')
        self.is_walking = False
        
        # Call DOWN service and wait for success
        if not self.mani_down_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Manipulator DOWN service not available')
            return
            
        self.get_logger().info('Calling DOWN service...')
        request = Trigger.Request()
        
        try:
            future = self.mani_down_client.call_async(request)
            future.add_done_callback(self.down_service_callback)
                
        except Exception as e:
            self.get_logger().error(f'DOWN service call failed: {str(e)}')

    def up_service_callback(self, future):
        """Handle UP service response"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'UP SUCCESS: {response.message}')
                self.get_logger().info('🚶 WALK - Robot is now walking!')
                self.is_walking = True
            else:
                self.get_logger().error(f'UP FAILED: {response.message}')
        except Exception as e:
            self.get_logger().error(f'UP service callback failed: {str(e)}')

    def down_service_callback(self, future):
        """Handle DOWN service response"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'DOWN SUCCESS: {response.message}')
                self.get_logger().info('⬇️  DOWN - Manipulator lowered')
            else:
                self.get_logger().error(f'DOWN FAILED: {response.message}')
        except Exception as e:
            self.get_logger().error(f'DOWN service callback failed: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = TryManiControl()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Try Mani Control Node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()