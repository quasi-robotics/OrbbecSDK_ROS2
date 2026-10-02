import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Include launch files
    package_dir = get_package_share_directory("orbbec_camera")
    launch_file_dir = os.path.join(
        package_dir,
        "examples",
        "multi_camera_shared_container",
    )
    component_container_name = "shared_orbbec_container"

    shared_orbbec_container = Node(
        name=component_container_name,
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
    )

    launch1_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_series_shared_container.launch.py")
        ),
        launch_arguments={
            "camera_name": "camera_01",
            "usb_port": "2-1",
            "device_num": "2",
            "attach_to_shared_component_container": "true",
            "component_container_name": component_container_name,
            "use_intra_process_comms": "true",
        }.items(),
    )

    launch2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_series_shared_container.launch.py")
        ),
        launch_arguments={
            "camera_name": "camera_02",
            "usb_port": "2-2",
            "device_num": "2",
            "attach_to_shared_component_container": "true",
            "component_container_name": component_container_name,
            "use_intra_process_comms": "true",
        }.items(),
    )

    # If you need more cameras, just add more launch_include here, and change the usb_port and device_num

    # Launch description
    ld = LaunchDescription(
        [
            shared_orbbec_container,
            TimerAction(period=0.0, actions=[GroupAction([launch1_include])]),
            TimerAction(period=2.0, actions=[GroupAction([launch2_include])]),
        ]
    )

    return ld
