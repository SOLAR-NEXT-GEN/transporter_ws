#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Float64MultiArray
from enum import Enum
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading

class OperationState(Enum):
    IDLE = 0
    WAITING_UP_COMPLETION = 1
    WAITING_DOWN_COMPLETION = 2
    SENDING_RESET = 3

class ManiControlServer(Node):
    def __init__(self):
        super().__init__('mani_control_server')
        
        # Create reentrant callback group for services
        self.service_callback_group = ReentrantCallbackGroup()
        
        # Publisher to send commands to micro-ROS
        self.cmd_publisher = self.create_publisher(
            Float64MultiArray, 
            'cmd_hinge', 
            10
        )
        
        # Subscriber to monitor hinge states
        self.state_subscriber = self.create_subscription(
            Float64MultiArray,
            'hinge_state',
            self.state_callback,
            10
        )
        
        # Service servers with reentrant callback group
        self.up_service = self.create_service(
            Trigger,
            'mani_control_up',
            self.handle_up_request,
            callback_group=self.service_callback_group
        )
        
        self.down_service = self.create_service(
            Trigger,
            'mani_control_down', 
            self.handle_down_request,
            callback_group=self.service_callback_group
        )
        
        # Timer for checking states (non-blocking)
        self.timer = self.create_timer(0.01, self.check_completion)  # 100Hz
        
        # State tracking
        self.current_states = [0, 0]
        self.flex_sensors = [0, 0]
        self.operation_state = OperationState.IDLE
        self.start_time = None
        self.timeout_duration = 15.0
        
        # Completion tracking
        self.completion_event = threading.Event()
        self.operation_success = False
        self.operation_message = ""
        
        self.get_logger().info('Non-blocking Manipulator Control Server started')
        self.get_logger().info('Services available:')
        self.get_logger().info('  - /mani_control_up')  
        self.get_logger().info('  - /mani_control_down')

    def state_callback(self, msg):
        """Monitor current hinge states and flex sensors"""
        if len(msg.data) >= 4:
            old_states = self.current_states.copy()
            self.current_states = [int(msg.data[0]), int(msg.data[1])]
            self.flex_sensors = [int(msg.data[2]), int(msg.data[3])]

            print(f'Current states: FL={self.current_states[0]}, BL={self.current_states[1]}')
            
            # Log state changes for debugging
            if old_states != self.current_states:
                self.get_logger().info(f'State changed: FL {old_states[0]} -> {self.current_states[0]}, BL {old_states[1]} -> {self.current_states[1]}')

    def send_command(self, fl_cmd, bl_cmd):
        """Send command to micro-ROS"""
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [float(fl_cmd), float(bl_cmd)]
        
        self.cmd_publisher.publish(cmd_msg)
        self.get_logger().info(f'Sent command: FL={fl_cmd}, BL={bl_cmd}')

    def check_completion(self):
        """Timer callback to check for completion states (non-blocking)"""
        if self.operation_state == OperationState.IDLE:
            return
            
        # Check for timeout
        current_time = self.get_clock().now().nanoseconds / 1e9
        if self.start_time and (current_time - self.start_time) > self.timeout_duration:
            self.get_logger().warn(f'Operation timed out! Final states: FL={self.current_states[0]}, BL={self.current_states[1]}')
            self.operation_success = False
            self.operation_message = f'Operation timed out. Final states: FL={self.current_states[0]}, BL={self.current_states[1]}'
            self.completion_event.set()
            self.reset_operation()
            return
        
        fl_state = self.current_states[0]
        bl_state = self.current_states[1]
        
        if self.operation_state == OperationState.WAITING_UP_COMPLETION:
            # Check if both hinges completed UP operation (state 2 or 3)
            if fl_state in [2, 3] and bl_state in [2, 3]:
                self.get_logger().info(f'UP operation completed: FL={fl_state}, BL={bl_state}')
                self.get_logger().info('Sending reset command (0, 0)')
                self.send_command(0, 0)
                self.operation_state = OperationState.SENDING_RESET
                
        elif self.operation_state == OperationState.WAITING_DOWN_COMPLETION:
            # Check if both hinges completed DOWN operation (state 2 or 3)  
            if fl_state in [2, 3] and bl_state in [2, 3]:
                self.get_logger().info(f'DOWN operation completed: FL={fl_state}, BL={bl_state}')
                self.get_logger().info('Sending reset command (0, 0)')
                self.send_command(0, 0)
                self.operation_state = OperationState.SENDING_RESET
                
        elif self.operation_state == OperationState.SENDING_RESET:
            # Reset sent, operation complete
            self.get_logger().info('Reset command sent, operation complete')
            self.operation_success = True
            self.operation_message = 'Operation completed and reset'
            self.completion_event.set()
            self.reset_operation()

    def reset_operation(self):
        """Reset operation state"""
        self.operation_state = OperationState.IDLE
        self.start_time = None

    def handle_up_request(self, request, response):
        """Handle hinges up service request (non-blocking)"""
        try:
            if self.operation_state != OperationState.IDLE:
                response.success = False
                response.message = 'Another operation is in progress'
                return response
                
            self.get_logger().info(f'UP request - Current states: FL={self.current_states[0]}, BL={self.current_states[1]}')
            
            # Reset completion tracking
            self.completion_event.clear()
            self.operation_success = False
            self.operation_message = ""
            
            # Send up command
            self.send_command(-1, -1)
            
            # Set up non-blocking completion tracking
            self.operation_state = OperationState.WAITING_UP_COMPLETION
            self.start_time = self.get_clock().now().nanoseconds / 1e9
            
            self.get_logger().info('UP command sent, waiting for completion...')
            
            # Wait for completion without blocking ROS callbacks
            # This blocks the service thread but allows other callbacks to continue
            completed = self.completion_event.wait(timeout=self.timeout_duration + 1)
            
            if completed:
                response.success = self.operation_success
                response.message = self.operation_message
            else:
                response.success = False
                response.message = 'Service call timed out waiting for completion'
                self.reset_operation()
            
            return response
                
        except Exception as e:
            response.success = False
            response.message = f'Error: {str(e)}'
            self.get_logger().error(f'Error in up service: {str(e)}')
            self.reset_operation()
            return response

    def handle_down_request(self, request, response):
        """Handle hinges down service request (non-blocking)"""
        try:
            if self.operation_state != OperationState.IDLE:
                response.success = False
                response.message = 'Another operation is in progress'
                return response
                
            self.get_logger().info(f'DOWN request - Current states: FL={self.current_states[0]}, BL={self.current_states[1]}')
            
            # Reset completion tracking
            self.completion_event.clear()
            self.operation_success = False
            self.operation_message = ""
            
            # Send down command
            self.send_command(1, 1)
            
            # Set up non-blocking completion tracking
            self.operation_state = OperationState.WAITING_DOWN_COMPLETION
            self.start_time = self.get_clock().now().nanoseconds / 1e9
            
            self.get_logger().info('DOWN command sent, waiting for completion...')
            
            # Wait for completion without blocking ROS callbacks
            # This blocks the service thread but allows other callbacks to continue
            completed = self.completion_event.wait(timeout=self.timeout_duration + 1)
            
            if completed:
                response.success = self.operation_success
                response.message = self.operation_message
            else:
                response.success = False
                response.message = 'Service call timed out waiting for completion'
                self.reset_operation()
            
            return response
                
        except Exception as e:
            response.success = False
            response.message = f'Error: {str(e)}'
            self.get_logger().error(f'Error in down service: {str(e)}')
            self.reset_operation()
            return response


def main(args=None):
    rclpy.init(args=args)
    
    node = ManiControlServer()
    
    # Use MultiThreadedExecutor to allow concurrent callback processing
    executor = MultiThreadedExecutor()
    
    try:
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Manipulator Control Server...')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()