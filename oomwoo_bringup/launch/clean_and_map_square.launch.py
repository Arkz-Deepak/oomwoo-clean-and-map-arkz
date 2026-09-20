#!/usr/bin/env python3
"""
Clean-and-Map Square Room Launch File
=====================================
Launches automated cleaning and simultaneous mapping in square_room.world:
  1. Gazebo Sim with square_room.world & robot spawner (via oomwoo_gazebo sim.launch.py)
  2. SLAM Toolbox (online_async_launch)
  3. Boustrophedon Coverage Planner (oomwoo_coverage)
  4. RViz2 (conditional on rviz:=true)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('oomwoo_bringup')
    pkg_sim_support = get_package_share_directory('oomwoo_sim_support')
    pkg_gazebo = get_package_share_directory('oomwoo_gazebo')
    pkg_slam = get_package_share_directory('slam_toolbox')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    default_world = os.path.join(pkg_sim_support, 'worlds', 'square_room.world')
    world = LaunchConfiguration('world', default=default_world)
    headless = LaunchConfiguration('headless', default='true')
    rviz = LaunchConfiguration('rviz', default='false')

    # 1. Gazebo Sim Server (Headless or GUI)
    gz_server_headless = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', world],
        output='screen',
        additional_env={'GZ_SIM_RESOURCE_PATH': os.path.join(pkg_gazebo, 'sdf')},
        condition=IfCondition(headless),
    )

    gz_server_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world],
        output='screen',
        additional_env={'GZ_SIM_RESOURCE_PATH': os.path.join(pkg_gazebo, 'sdf')},
        condition=UnlessCondition(headless),
    )

    # 2. ROS-Gazebo Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/bumper_left@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/bumper_right@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # 3. Robot State Publisher & URDF
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command(
                ['xacro ', os.path.join(pkg_bringup, 'urdf', 'robot.urdf.xacro')]
            )
        }],
    )

    # 4. Spawn Robot Entity in Gazebo
    spawn_entity = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=['-name', 'oomwoo', '-topic', 'robot_description'],
                output='screen',
            )
        ],
    )

    # 5. RViz2 (Conditional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_gazebo, 'rviz', 'oomwoo.rviz')],
        condition=IfCondition(rviz),
    )

    # 6. SLAM Toolbox (Online Async)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 7. Autonomous Coverage Planner
    coverage_planner_node = Node(
        package='oomwoo_coverage',
        executable='coverage_planner',
        name='coverage_planner',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'executor': 'reactive',
            'v_cruise': 0.30,
            'rotate_speed': 0.8,
            'robot_radius': 0.17,
            'reach_tol': 0.20,
            'align_tol': 0.35,
        }],
        remappings=[
            ('/map', '/map'),
            ('/cmd_vel', '/cmd_vel'),
        ]
    )

    return LaunchDescription([
        SetEnvironmentVariable('GZ_IP', '127.0.0.1'),
        SetEnvironmentVariable('__EGL_VENDOR_LIBRARY_FILENAMES', '/usr/share/glvnd/egl_vendor.d/50_mesa.json'),
        SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'mesa'),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='false'),
        gz_server_headless,
        gz_server_gui,
        bridge,
        robot_state_pub,
        spawn_entity,
        rviz_node,
        slam_launch,
        coverage_planner_node,
    ])
