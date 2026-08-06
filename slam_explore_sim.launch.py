import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='living_room.world')
    robot_model = LaunchConfiguration('robot_model', default='makerspet_snoopy')
    headless = LaunchConfiguration('headless', default='true')

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

    # 3. m-explore-ros2 (explore_lite) Frontier Exploration Node
    explore_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('explore_lite'), 'launch', 'explore.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value='living_room.world'),
        DeclareLaunchArgument('robot_model', default_value='makerspet_snoopy'),
        DeclareLaunchArgument('headless', default_value='true'),
        gazebo_launch,
        slam_toolbox_launch,
        explore_launch,
    ])
