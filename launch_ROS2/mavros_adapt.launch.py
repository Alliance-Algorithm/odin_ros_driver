import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory("odin_ros_driver")

    remapping_script = os.path.join(package_dir, "script", "remapping.py")
    remapping = Node(
        executable=remapping_script,
        name="flight_mavros_remapping",
        output="screen",
    )

    mavros_share = get_package_share_directory("mavros")
    mavros = Node(
        package="mavros",
        executable="mavros_node",
        namespace="mavros",
        parameters=[
            os.path.join(mavros_share, "launch", "px4_pluginlists.yaml"),
            os.path.join(mavros_share, "launch", "px4_config.yaml"),
            {
                "fcu_url": "serial:///dev/ttyUSB0:460800",
                "gcs_url": "",
                "tgt_system": 1,
                "tgt_component": 1,
                "fcu_protocol": "v2.0",
            },
        ],
        respawn=True,
        respawn_delay=1.0,
        output="screen",
        emulate_tty=True,
    )

    ld = LaunchDescription()
    ld.add_action(remapping)
    ld.add_action(mavros)

    return ld
