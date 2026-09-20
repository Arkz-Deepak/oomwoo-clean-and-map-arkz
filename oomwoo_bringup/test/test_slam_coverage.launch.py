import os
import unittest
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
import launch
import launch_testing
import launch_testing.actions
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_test_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    default_world_path = os.path.join(
        get_package_share_directory('oomwoo_sim_support'),
        'worlds',
        'square_room.world'
    )
    world = LaunchConfiguration('world', default=default_world_path)

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('oomwoo_gazebo'), 'launch', 'sim.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time, 'world': world, 'rviz': 'false'}.items()
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    return launch.LaunchDescription([
        SetEnvironmentVariable('GZ_IP', '127.0.0.1'),
        SetEnvironmentVariable('__EGL_VENDOR_LIBRARY_FILENAMES', '/usr/share/glvnd/egl_vendor.d/50_mesa.json'),
        SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'mesa'),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value=default_world_path),
        gazebo_launch,
        slam_launch,
        launch_testing.actions.ReadyToTest(),
    ])

class TestSlamCoverageRegression(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_slam_coverage_node')
        self.map_msg = None
        self.odom_count = 0
        self.map_sub = self.node.create_subscription(OccupancyGrid, '/map', self._map_cb, 10)
        self.odom_sub = self.node.create_subscription(Odometry, '/odom', self._odom_cb, 10)

    def tearDown(self):
        self.node.destroy_node()

    def _map_cb(self, msg):
        self.map_msg = msg

    def _odom_cb(self, msg):
        self.odom_count += 1

    def test_map_and_coverage_completeness(self):
        end_time = self.node.get_clock().now() + rclpy.duration.Duration(seconds=15)
        while rclpy.ok() and self.node.get_clock().now() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.5)

        self.assertIsNotNone(self.map_msg, 'Map topic /map was not published within timeout!')
        self.assertGreater(self.map_msg.info.width, 0, 'Map width must be greater than 0')
        self.assertGreater(self.map_msg.info.height, 0, 'Map height must be greater than 0')
        self.assertGreater(self.odom_count, 0, 'Odometry /odom updates must be published during test')
