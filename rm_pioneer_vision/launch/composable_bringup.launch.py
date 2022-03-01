import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command

from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

import yaml


def generate_launch_description():
    # robot type
    robot = os.environ['ROBOT']

    # params file path
    params_file = os.path.join(get_package_share_directory(
        'rm_pioneer_vision'), 'config', robot + '_params.yaml')
    camera_info_url = 'package://rm_pioneer_vision/config/' + robot + '_camera_info.yaml'

    # xacro file path
    urdf_dir = os.path.join(get_package_share_directory(
        'rm_pioneer_description'), 'urdf/')
    xacro_file = Command(
        ['xacro ', urdf_dir, robot, '.urdf.xacro'])

    # load params for composable node
    with open(params_file, 'r') as f:
        camera_params = yaml.safe_load(f)['/mv_camera']['ros__parameters']
    with open(params_file, 'r') as f:
        detector_params = yaml.safe_load(f)['/armor_detector']['ros__parameters']

    camera_detector_container = ComposableNodeContainer(
        name='camera_detector_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
                ComposableNode(
                    package='mindvision_camera',
                    plugin='mindvision_camera::MVCameraNode',
                    name='mv_camera',
                    parameters=[camera_params, {
                        'camera_info_url': camera_info_url,
                        'use_sensor_data_qos': False,
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}]),

                ComposableNode(
                    package='armor_detector',
                    plugin='rm_auto_aim::RgbDetectorNode',
                    name='armor_detector',
                    parameters=[detector_params, {
                        'detect_color': 0,
                        'debug': False,
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}])
        ],
        output='screen',
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
        parameters=[{'robot_description': xacro_file,
                     'publish_frequency': 1000.0}]
    )

    return LaunchDescription([
        camera_detector_container,
        processor_node,
        rm_serial_launch,
        robot_state_publisher,
    ])
