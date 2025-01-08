import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def generate_launch_description():
    
    # LIDARを起動するlaunchファイル名とパッケージ名を指定(別のLIDARを使用する場合は変更すること)
    LIDAR_LAUNCH_FILE = '/sllidar_a2m8_launch.py'
    LIDAR_PACKAGE = 'sllidar_ros2'

    # OpenCRを接続するUSBポートに関する設定
    usb_port = LaunchConfiguration('usb_port', default='/dev/ttyACM0')

    # ロボットのパラメータに関する設定
    robot_param_dir = LaunchConfiguration(
        'robot_param_dir',
        default=os.path.join(get_package_share_directory('avatar_bringup'), 'param', 'avatar.yaml'))

    # LIDARのパッケージに関する設定
    lidar_pkg_dir = LaunchConfiguration(
        'lidar_pkg_dir',
        default=os.path.join(get_package_share_directory(LIDAR_PACKAGE), 'launch'))

    # シミュレーション時間を使用するか(シミュレーションの場合はtrue, 実機の場合はfalse)
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'usb_port',
            default_value=usb_port,
            description='Connected USB port with OpenCR'),

        DeclareLaunchArgument(
            'robot_param_dir',
            default_value=robot_param_dir,
            description='Full path to robot parameter file to load'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('turtlebot3_bringup'), 'launch'), '/turtlebot3_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidar_pkg_dir, LIDAR_LAUNCH_FILE]),
            launch_arguments={'serial_port': '/dev/ttyUSB0', 'frame_id': 'base_scan'}.items(),
        ),

        Node(
            package='turtlebot3_node',
            executable='turtlebot3_ros',
            parameters=[robot_param_dir],
            arguments=['-i', usb_port],
            output='screen'),
    ])