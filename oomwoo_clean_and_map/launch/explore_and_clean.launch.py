from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Launch the explorer, redirect its action client, and point it to /map
        Node(
            package='explore_lite',
            executable='explore',
            name='explore_node',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'costmap_topic': 'map',
                'costmap_updates_topic': 'map_updates',
                'visualize': True
            }],
            remappings=[
                ('navigate_to_pose', 'intercepted_navigate_to_pose') 
            ]
        ),

        # 2. Launch our custom C++ Interceptor
        Node(
            package='oomwoo_clean_and_map',
            executable='frontier_interceptor',
            name='frontier_interceptor',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])