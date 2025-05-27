from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
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
        Node(
            package='ai',
            executable='img_send',
            name='img_send',
            respawn=True,
            respawn_delay=4,
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_control',
            respawn=True,
            respawn_delay=4,
            parameters=[{
                'dev': '/dev/input/js0',
                #'autorepeat_rate': 200.0
            }]
        ),
    ])