from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('drive')
    twist_cfg = os.path.join(pkg_share, 'config', 'twist.yaml')

    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{'dev': '/dev/input/js0'}],
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[twist_cfg],
        ),
        Node(
            package='drive',
            executable='joy_dock',
            name='joy_dock',
            output='screen',
            emulate_tty=True,
            respawn=True,
            arguments=['--ros-args', '--log-level', 'info'],
        ),
    ])

