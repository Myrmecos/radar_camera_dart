from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hikrobot_camera',
            executable='hikrobot_camera',
            name='hikrobot_camera_node',
            output='screen'
        )
    ])