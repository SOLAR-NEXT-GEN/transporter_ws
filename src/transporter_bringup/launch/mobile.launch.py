"""Mobile base only: PS5 joy -> mobile_controller -> ODrive."""
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
    mobile_cfg = os.path.join(config, 'mobile.yaml')
    use_joy = LaunchConfiguration('use_joy')

    return LaunchDescription([
        DeclareLaunchArgument('use_joy', default_value='true',
                              description='Start joy_node and transporter_joy'),

        Node(package='joy', executable='joy_node', name='joy_node',
             parameters=[joy_cfg], output='screen', condition=IfCondition(use_joy)),
        Node(package='transporter_joy', executable='joy_teleop', name='transporter_joy',
             parameters=[joy_cfg, {'enable_mobile': True, 'enable_manipulator': False}],
             output='screen', condition=IfCondition(use_joy)),

        Node(package='transporter_mobile', executable='mobile_controller',
             name='mobile_controller', parameters=[mobile_cfg], output='screen'),
        Node(package='transporter_odrive', executable='odrive_node',
             name='odrive_node', parameters=[mobile_cfg], output='screen'),
    ])
