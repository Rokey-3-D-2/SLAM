from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rokey_pjt',
            executable='nav_through_poses',
            name='nav_through_poses',
            output='screen'
        ),
        Node(
            package='rokey_pjt',
            executable='object_xyz_marker',
            name='object_xyz_marker',
            output='screen'
        )
    ])
