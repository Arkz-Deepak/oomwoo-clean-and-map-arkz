#!/usr/bin/env python3
"""
Clean-and-Map Living Room Launch File
=====================================
Single-package launch file for automated cleaning and SLAM mapping in the
cluttered Living Room world (with tables, chairs, sofa, TV stand):
  1. Gazebo Sim with living_room.world, models resource path, & robot spawner at (0.32, 1.59)
  2. ROS-Gazebo Parameter Bridge (clock, odom, tf, scan, cmd_vel, joint_states, bumpers)
  3. Robot State Publisher with URDF xacro & use_sim_time
  4. SLAM Toolbox (online_async_launch)
  5. Autonomous Coverage Planner (reactive executor with furniture un-wedging)
  6. Optional RViz2 visualization
"""

import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, Shutdown, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Clean up any orphaned Gazebo processes from previous runs to prevent multiple robot spawns
    subprocess.run(['killall', '-q', '-9', 'gz-sim-server', 'gz-sim-gui'], stderr=subprocess.DEVNULL)

    pkg_name = 'oomwoo_clean_and_map'
    pkg_dir = get_package_share_directory(pkg_name)
    pkg_slam = get_package_share_directory('slam_toolbox')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    default_world = os.path.join(pkg_dir, 'worlds', 'living_room.world')
    world = LaunchConfiguration('world', default=default_world)
    headless = LaunchConfiguration('headless', default='false')
    rviz = LaunchConfiguration('rviz', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.32')
    y_pose = LaunchConfiguration('y_pose', default='1.59')
    yaw = LaunchConfiguration('yaw', default='0.0')

    models_dir = os.path.join(pkg_dir, 'models')
    gui_config = os.path.join(pkg_dir, 'config', 'gazebo_gui_living_room.config')

    # 1. Gazebo Sim Server (Headless or GUI)
    gz_server_headless = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', '--headless-rendering', world],
        output='screen',
        on_exit=Shutdown(),
        condition=IfCondition(headless),
    )

    gz_server_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '--gui-config', gui_config, world],
        output='screen',
        on_exit=Shutdown(),
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
            'use_sim_time': use_sim_time,
            'robot_description': Command(
                ['xacro ', os.path.join(pkg_dir, 'urdf', 'robot.urdf.xacro')]
            )
        }],
    )

    # 4. Spawn Robot Entity in Gazebo (spawn at clear floor cell away from table)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'default',
            '-name', 'oomwoo',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.05',
            '-Y', yaw,
            '-allow_renaming', 'false',
        ],
        output='screen',
    )

    # 5. RViz2 (Conditional)
    rviz_config = os.path.join(pkg_dir, 'config', 'clean_and_map.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(rviz),
    )

    # 6. SLAM Toolbox (Online Async)
    slam_params = os.path.join(pkg_dir, 'config', 'slam_square_room.yaml')
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params,
        }.items()
    )

    # 7. Autonomous Coverage Planner
    coverage_planner_node = Node(
        package=pkg_name,
        executable='coverage_planner_node.py',
        name='coverage_planner',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'executor': 'reactive',
            'v_cruise': 0.25,
            'rotate_speed': 0.4,
            'robot_radius': 0.17,
            'reach_tol': 0.08,
            'align_tol': 0.30,
            'obstacle_inflation': 0.12,
            'no_progress_sec': 2.5,
        }],
        remappings=[
            ('/map', '/map'),
            ('/cmd_vel', '/cmd_vel'),
        ]
    )

    return LaunchDescription([
        SetEnvironmentVariable('GZ_IP', '127.0.0.1'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', models_dir),
        SetEnvironmentVariable('__EGL_VENDOR_LIBRARY_FILENAMES', '/usr/share/glvnd/egl_vendor.d/50_mesa.json'),
        SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'mesa'),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('x_pose', default_value='0.32'),
        DeclareLaunchArgument('y_pose', default_value='1.59'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        gz_server_headless,
        gz_server_gui,
        bridge,
        robot_state_pub,
        spawn_entity,
        rviz_node,
        slam_launch,
        coverage_planner_node,
    ])
