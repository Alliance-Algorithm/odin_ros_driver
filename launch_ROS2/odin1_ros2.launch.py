# USAGE: ros2 launch odin_ros_driver odin1_ros2.launch.py
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def load_yaml_params(package_dir, config_name):
    config_path = os.path.join(package_dir, "config", config_name)
    with open(config_path, "r") as file:
        return yaml.safe_load(file)


def generate_launch_description():
    # Get package directory
    package_dir = get_package_share_directory("odin_ros_driver")
    default_runtime_dir = os.path.join(
        os.path.expanduser("~"), ".ros", "odin_ros_driver"
    )

    # Declare configuration parameter
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=os.path.join(package_dir, "config", "control_command.yaml"),
        description="Path to the control config YAML file",
    )

    runtime_dir_arg = DeclareLaunchArgument(
        "runtime_dir",
        default_value=default_runtime_dir,
        description="Writable directory for generated calibration and runtime outputs",
    )

    calib_file_arg = DeclareLaunchArgument(
        "calib_file_path",
        default_value=PathJoinSubstitution(
            [LaunchConfiguration("runtime_dir"), "calib.yaml"]
        ),
        description="Path to the generated calibration YAML file",
    )

    # Add RViz2 configuration file parameter
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(package_dir, "config", "odin_ros2.rviz"),
        description="Path to RViz2 config file",
    )

    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value="false",
        description="Start RViz2. Keep false on headless MiniPC/runtime containers.",
    )

    # Create main node
    host_sdk_node = Node(
        package="odin_ros_driver",
        executable="host_sdk_sample",
        name="host_sdk_sample",
        output="screen",
        # arguments=['--ros-args', '--log-level', 'debug'],
        parameters=[
            {
                "config_file": LaunchConfiguration("config_file"),
                "runtime_dir": LaunchConfiguration("runtime_dir"),
                "calib_file_path": LaunchConfiguration("calib_file_path"),
            }
        ],
    )

    pcd2depth_params = load_yaml_params(package_dir, "control_command.yaml")
    pcd2depth_node = Node(
        package="odin_ros_driver",
        executable="pcd2depth_ros2_node",
        name="pcd2depth_ros2_node",
        output="screen",
        parameters=[
            pcd2depth_params,
            {
                "config_file": LaunchConfiguration("config_file"),
                "calib_file_path": LaunchConfiguration("calib_file_path"),
            },
        ],
    )

    # Cloud reprojection node
    reprojection_params = load_yaml_params(package_dir, "control_command.yaml")
    cloud_reprojection_node = Node(
        package="odin_ros_driver",
        executable="cloud_reprojection_ros2_node",
        name="cloud_reprojection_ros2_node",
        output="screen",
        parameters=[
            reprojection_params,
            {
                "config_file": LaunchConfiguration("config_file"),
                "calib_file_path": LaunchConfiguration("calib_file_path"),
            },
        ],
    )

    # Image overlay node - overlays reprojected points on camera image
    overlay_params = load_yaml_params(package_dir, "control_command.yaml")
    image_overlay_node = Node(
        package="odin_ros_driver",
        executable="image_overlay_node",
        name="image_overlay_node",
        output="screen",
        parameters=[overlay_params],
    )

    # Create RViz2 node - loads specified configuration file
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
    )

    # Create launch description
    ld = LaunchDescription()
    ld.add_action(config_file_arg)
    ld.add_action(runtime_dir_arg)
    ld.add_action(calib_file_arg)
    ld.add_action(rviz_config_arg)  # Add RViz configuration argument
    ld.add_action(launch_rviz_arg)
    ld.add_action(host_sdk_node)
    ld.add_action(pcd2depth_node)
    ld.add_action(cloud_reprojection_node)
    ld.add_action(image_overlay_node)
    ld.add_action(rviz_node)  # Add RViz node

    return ld
