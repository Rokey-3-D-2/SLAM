from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # object_xyz_marker 노드
        Node(
            package='rokey_pjt',
            executable='object_xyz_marker',
            name='object_xyz_marker',
            namespace='robot1',
            output='screen',
            remappings=[
                ('/tf', '/robot1/tf'),
                ('/tf_static', '/robot1/tf_static'),
            ]
        ),

        # sc_follow_waypoints 노드
        Node(
            package='rokey_pjt',
            executable='sc_nav_to_pose',
            name='sc_nav_to_pose',
            namespace='robot1',
            output='screen'
        )
    ])
