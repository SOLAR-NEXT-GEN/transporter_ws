"""Manual mode: one PS5 controller drives both the mobile base and the manipulator."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('transporter_bringup')
    joy_cfg = os.path.join(share, 'config', 'joy.yaml')

    def include(name, extra=None):
        args = {'use_joy': 'false'}
        args.update(extra or {})
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', name)),
            launch_arguments=args.items())

    return LaunchDescription([
        DeclareLaunchArgument('mani_device', default_value='/dev/ttyTransporterLeftMani'),
        DeclareLaunchArgument('mani_baudrate', default_value='115200'),

        Node(package='joy', executable='joy_node', name='joy_node',
             parameters=[joy_cfg], output='screen'),
        Node(package='transporter_joy', executable='joy_teleop', name='transporter_joy',
             parameters=[joy_cfg, {'enable_mobile': True, 'enable_manipulator': True}],
             output='screen'),

        include('mobile.launch.py'),
        include('manipulator.launch.py', {
            'mani_device': LaunchConfiguration('mani_device'),
            'mani_baudrate': LaunchConfiguration('mani_baudrate'),
        }),
    ])
