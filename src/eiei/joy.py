#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class TransporterJoy(Node):
    def __init__(self):
        super().__init__('transporter_joy')
        
        # Constants for velocities
        self.LINEAR_VEL = 0.1225  # m/s
        self.ANGULAR_VEL = 0.064  # rad/s
        
        # Create publisher for cmd_vel topic
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Create publisher for cmd_hinge topic
        self.hinge_pub = self.create_publisher(Float64MultiArray, '/cmd_hinge', 10)
        
        # Create subscriber for joy topic
        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
        # Initialize Twist message
        self.twist_msg = Twist()
        
        # Button state tracking for debouncing
        self.prev_button1 = False  # down
        self.prev_button3 = False  # up
        
        # Current hinge command state
        self.current_hinge_cmd = [0.0, 0.0]
        
        # Timer for continuous publishing of hinge commands
        self.hinge_timer = self.create_timer(0.1, self.publish_hinge_cmd)  # 10 Hz
        
        self.get_logger().info('Transporter Joy node initialized')
        self.get_logger().info(f'Linear velocity: {self.LINEAR_VEL} m/s')
        self.get_logger().info(f'Angular velocity: {self.ANGULAR_VEL} rad/s')
        self.get_logger().info('Listening for joystick input on /joy topic...')
        self.get_logger().info('Button 1: Down command [-1,-1], Button 3: Up command [1,1]')
        self.get_logger().info('Not pressed: [0,0]')
    
    def joy_callback(self, msg):
        """
        Process joystick input and publish Twist messages
        Handle button presses for direct hinge command publishing
        """
        
        # Reset twist message
        self.twist_msg.linear.x = 0.0
        self.twist_msg.linear.y = 0.0
        self.twist_msg.linear.z = 0.0
        self.twist_msg.angular.x = 0.0
        self.twist_msg.angular.y = 0.0
        self.twist_msg.angular.z = 0.0
        
        # Check if we have enough axes and buttons
        if len(msg.axes) > 7 and len(msg.buttons) > 3:
            # Process linear velocity (axis 7)
            if msg.axes[7] > 0:
                self.twist_msg.linear.x = self.LINEAR_VEL
            elif msg.axes[7] < 0:
                self.twist_msg.linear.x = -self.LINEAR_VEL
            
            # Process angular velocity (axis 6)
            if msg.axes[6] > 0:
                self.twist_msg.angular.z = self.ANGULAR_VEL
            elif msg.axes[6] < 0:
                self.twist_msg.angular.z = -self.ANGULAR_VEL
            
            # Publish the twist message
            self.twist_pub.publish(self.twist_msg)
            
            # Handle button presses for hinge control
            self.handle_button_presses(msg.buttons)
            
            # Log current velocities (optional - comment out if too verbose)
            if self.twist_msg.linear.x != 0.0 or self.twist_msg.angular.z != 0.0:
                self.get_logger().debug(
                    f'Publishing: Vx={self.twist_msg.linear.x:.4f} m/s, '
                    f'Wz={self.twist_msg.angular.z:.4f} rad/s'
                )
        else:
            self.get_logger().warn(
                f'Not enough axes or buttons in Joy message. '
                f'Expected at least 8 axes and 4 buttons, got {len(msg.axes)} axes and {len(msg.buttons)} buttons'
            )
    
    def handle_button_presses(self, buttons):
        """Handle button presses for direct hinge command publishing"""
        
        # Check current button states
        button1_pressed = buttons[1]  # down
        button3_pressed = buttons[3]  # up
        
        # Set hinge command based on button states
        if button1_pressed and button3_pressed:
            # Both pressed - stop (shouldn't happen but handle it)
            self.current_hinge_cmd = [0.0, 0.0]
        elif button1_pressed:
            # Button 1 pressed - down command
            self.current_hinge_cmd = [-1.0, -1.0]
            if not self.prev_button1:  # Log only on press, not hold
                self.get_logger().info('Button 1 pressed - sending down command [-1, -1]')
        elif button3_pressed:
            # Button 3 pressed - up command
            self.current_hinge_cmd = [1.0, 1.0]
            if not self.prev_button3:  # Log only on press, not hold
                self.get_logger().info('Button 3 pressed - sending up command [1, 1]')
        else:
            # No buttons pressed - stop
            self.current_hinge_cmd = [0.0, 0.0]
            if self.prev_button1 or self.prev_button3:  # Log only when releasing
                self.get_logger().info('Buttons released - sending stop command [0, 0]')
        
        # Update previous button states
        self.prev_button1 = button1_pressed
        self.prev_button3 = button3_pressed
    
    def publish_hinge_cmd(self):
        """Publish current hinge command at 10 Hz"""
        hinge_msg = Float64MultiArray()
        hinge_msg.data = self.current_hinge_cmd
        self.hinge_pub.publish(hinge_msg)
        
        # Debug log (comment out if too verbose)
        # self.get_logger().debug(f'Publishing hinge command: {self.current_hinge_cmd}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = TransporterJoy()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()