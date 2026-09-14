#!/usr/bin/env python3
"""
Clean-and-Map Square Room Launch File
=====================================
Launches automated cleaning and simultaneous mapping in a clean 6x6m square room:
  1. Gazebo Sim with square_room.world
  2. Robot state publisher & ros_gz_bridge
  3. SLAM Toolbox (online_async_launch)
  4. Boustrophedon Coverage Planner (oomwoo_coverage)
  5. RViz2 visualization
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    default_world = os.path.join(
        get_package_share_directory('oomwoo_sim_support'),
        'worlds',
        'square_room.world'
    )
    world = LaunchConfiguration('world', default=default_world)
    headless = LaunchConfiguration('headless', default='false')

    # 1. Gazebo Sim World + Robot
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('oomwoo_gazebo'), 'launch', 'sim.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': world,
            'headless': headless,
        }.items()
    )

    # 2. SLAM Toolbox
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # 3. Autonomous Boustrophedon Coverage Planner
    coverage_planner_node = Node(
        package='oomwoo_coverage',
        executable='coverage_planner',
        name='coverage_planner',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'sweep_step_m': 0.35,
            'robot_radius_m': 0.17,
            'reach_tolerance_m': 0.20,
            'align_tolerance_rad': 0.35,
            'cruise_speed': 0.25,
            'rotate_speed': 0.8,
            'executor': 'reactive',
        }],
        remappings=[
            ('/map', '/map'),
            ('/cmd_vel', '/cmd_vel'),
        ]
    )

    # 4. RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('headless', default_value='false'),
        gazebo_launch,
        slam_launch,
        coverage_planner_node,
        rviz_node,
    ])
