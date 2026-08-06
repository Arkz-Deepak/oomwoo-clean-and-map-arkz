import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='living_room.world')
    robot_model = LaunchConfiguration('robot_model', default='makerspet_snoopy')
    headless = LaunchConfiguration('headless', default='false')

    # 1. Gazebo World + Robot Bringup
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('oomwoo_gazebo'), 'launch', 'world.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': world,
            'robot_model': robot_model,
            'headless': headless,
        }.items()
    )

    # 2. SLAM Toolbox Online Async Node
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # 3. Phase 3 Coverage Path Planner Node (Boustrophedon Cellular Decomposition)
    coverage_planner_node = Node(
        package='oomwoo_coverage',
        executable='coverage_planner',
        name='coverage_planner',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'sweep_step_m': 0.3,
            'robot_radius_m': 0.18,
            'reach_tolerance_m': 0.25,
            'align_tolerance_rad': 0.35,
            'cruise_speed': 0.22,
            'rotate_speed': 0.6,
        }],
        remappings=[
            ('/map', '/map'),
            ('/cmd_vel', '/cmd_vel'),
        ]
    )

    
    # 4. RViz Graphical Visualization Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='living_room.world'),
        DeclareLaunchArgument('robot_model', default_value='makerspet_snoopy'),
        DeclareLaunchArgument('headless', default_value='false'),
        gazebo_launch,
        slam_toolbox_launch,
        coverage_planner_node,
        rviz_node,
    ])
