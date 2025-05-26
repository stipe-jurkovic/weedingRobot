from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wheel_control',
            executable='transcriber',
            name='wheel_control'
        ),
        Node(
            package='ai',
            executable='img_send',
            name='img_send'
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_control'
        ),
    ])