"""Manipulator only: PS5 joy -> manipulator_controller -> micro-ROS agent -> transport_mani."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory('transporter_bringup'), 'config')
    joy_cfg = os.path.join(config, 'joy.yaml')
    mani_cfg = os.path.join(config, 'manipulator.yaml')
    use_joy = LaunchConfiguration('use_joy')

    return LaunchDescription([
        DeclareLaunchArgument('use_joy', default_value='true',
                              description='Start joy_node and transporter_joy'),
        DeclareLaunchArgument('mani_device', default_value='/dev/ttyTransporterLeftMani',
                              description='Serial port of the transport_mani board'),
        DeclareLaunchArgument('mani_baudrate', default_value='115200',
                              description='Must match LPUART1 baud rate in firmware'),

        Node(package='joy', executable='joy_node', name='joy_node',
             parameters=[joy_cfg], output='screen', condition=IfCondition(use_joy)),
        Node(package='transporter_joy', executable='joy_teleop', name='transporter_joy',
             parameters=[joy_cfg, {'enable_mobile': False, 'enable_manipulator': True}],
             output='screen', condition=IfCondition(use_joy)),

        Node(package='transporter_manipulator', executable='manipulator_controller',
             name='manipulator_controller', parameters=[mani_cfg], output='screen'),

        Node(package='micro_ros_agent', executable='micro_ros_agent',
             name='micro_ros_agent_mani', output='screen',
             arguments=['serial', '--dev', LaunchConfiguration('mani_device'),
                        '-b', LaunchConfiguration('mani_baudrate')]),
    ])
