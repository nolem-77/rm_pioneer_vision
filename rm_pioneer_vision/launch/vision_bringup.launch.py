import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # robot type
    robot_type_launch_arg = DeclareLaunchArgument(
        name='robot', default_value='guard')
    robot = LaunchConfiguration('robot')

    # params file path
    config_dir = os.path.join(get_package_share_directory(
        'rm_pioneer_vision'), 'config')
    params_file = [config_dir, '/', robot, '_params.yaml']
    camera_info_url = ['package://rm_pioneer_vision/config/',
                       robot, '_camera_info.yaml']

    # xacro file path
    urdf_dir = os.path.join(get_package_share_directory(
        'rm_pioneer_description'), 'urdf')
    xacro_file = Command(
        ['xacro ', urdf_dir, '/', robot, '.urdf.xacro'])

    mv_camera_node = Node(
        package='mindvision_camera',
        executable='mindvision_camera_node',
        output='screen',
        emulate_tty=True,
        parameters=[params_file, {
            'camera_info_url': camera_info_url,
            'use_sensor_data_qos': False,
        }],
    )

    detector_node = Node(
        package='armor_detector',
        executable='rgb_detector_node',
        output='screen',
        emulate_tty=True,
        parameters=[params_file, {
            'detect_color': 0,
            'debug': False,
        }],
    )

    processor_node = Node(
        package='armor_processor',
        executable='armor_processor_node',
        output='screen',
        emulate_tty=True,
        parameters=[params_file, {'debug': False}],
    )

    rm_serial_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rm_serial_driver'),
                'launch', 'serial_driver.launch.py')))

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': xacro_file}]
    )

    return LaunchDescription([
        robot_type_launch_arg,
        mv_camera_node,
        detector_node,
        processor_node,
        rm_serial_launch,
        robot_state_publisher,
    ])
