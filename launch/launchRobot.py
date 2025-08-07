from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
                os.path.join(
                    get_package_share_directory('rosbridge_server'),
                    'launch',
                    'rosbridge_websocket_launch.xml'
                )
        ),
        Node(
            package='wheel_control',
            executable='transcriber',
            name='wheel_control',
            respawn=True,
            respawn_delay=4,
        ),
        Node(
            package='stepper_control',
            executable='transcriber',
            name='stepper_control',
            respawn=True,
            respawn_delay=4,
        ),
        # Node(
        #     package='coordinate_sender',
        #     executable='sender',
        #     name='coordinate_sender',
        #     respawn=True,
        #     respawn_delay=4,
        # ),
        Node(
            package='ai',
            executable='img_send',
            name='img_send',
            respawn=True,
            respawn_delay=4,
        ),
    ])
