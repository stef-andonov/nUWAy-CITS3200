from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gsml',
            executable='pub1.py',
            name='camera_front',
            output='screen',
        ),
        Node(
            package='gsml',
            executable='pub2.py',
            name='camera_left',
            output='screen',
        ),
        Node(
            package='gsml',
            executable='pub3.py',
            name='camera_right',
            output='screen',
        ),
        Node(
            package='gsml',
            executable='pub4.py',
            name='camera_rear',
            output='screen',
        ),
    ])
