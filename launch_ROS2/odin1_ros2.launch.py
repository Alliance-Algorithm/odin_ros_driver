
# USAGE: ros2 launch odin_ros_driver odin1_ros2.launch.py
import os
import yaml 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def _resolve_config_file(package_dir, config_file):
    if os.path.isabs(config_file):
        return config_file
    candidate = os.path.join(package_dir, config_file)
    if os.path.exists(candidate):
        return candidate
    return config_file


def _build_actions(context):
    package_dir = get_package_share_directory('odin_ros_driver')
    config_file = _resolve_config_file(
        package_dir, LaunchConfiguration('config_file').perform(context))

    host_sdk_node = Node(
        package='odin_ros_driver',
        executable='host_sdk_sample',
        name='host_sdk_sample',
        output='screen',
        parameters=[{
            'config_file': config_file
        }]
    )

    with open(config_file, 'r', encoding='utf-8') as f:
        control_params = yaml.safe_load(f) or {}

    pcd2depth_params = dict(control_params)
    pcd2depth_calib_path = os.path.join(package_dir, 'config', 'calib.yaml')
    pcd2depth_params['calib_file_path'] = pcd2depth_calib_path 
    pcd2depth_params['config_file'] = config_file
    pcd2depth_node = Node(
        package='odin_ros_driver',
        executable='pcd2depth_ros2_node',  
        name='pcd2depth_ros2_node',
        output='screen',
        parameters=[pcd2depth_params]
    )

    reprojection_params = dict(control_params)
    reprojection_calib_path = os.path.join(package_dir, 'config', 'calib.yaml')
    reprojection_params['calib_file_path'] = reprojection_calib_path 
    reprojection_params['config_file'] = config_file
    cloud_reprojection_node = Node(
        package='odin_ros_driver',
        executable='cloud_reprojection_ros2_node',  
        name='cloud_reprojection_ros2_node',
        output='screen',
        parameters=[reprojection_params]
    )
    
    # Bridge node: converts odin1/cloud_raw → odin1/cloud_lio (point_lio OUST64 format)
    odin_to_lio_bridge_node = Node(
        package='odin_ros_driver',
        executable='odin_to_lio_bridge',
        name='odin_to_lio_bridge',
        output='screen',
    )

    return [
        host_sdk_node,
        pcd2depth_node,
        cloud_reprojection_node,
        odin_to_lio_bridge_node,
    ]


def generate_launch_description():
    package_dir = get_package_share_directory('odin_ros_driver')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(package_dir, 'config', 'control_command.yaml'),
            description='Path to the control config YAML file'
        ),
        OpaqueFunction(function=_build_actions),
    ])
