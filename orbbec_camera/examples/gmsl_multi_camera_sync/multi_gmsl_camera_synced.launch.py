import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Include launch files
    package_dir = get_package_share_directory("orbbec_camera")
    launch_file_dir = os.path.join(package_dir, "launch")
    config_file_dir = os.path.join(package_dir, "config")
    secondary_config_file_path = os.path.join(config_file_dir, "camera_secondary_params.yaml")

    launch1_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_series.launch.py")
        ),
        launch_arguments={
            "camera_name": "camera_01",
            "usb_port": "gmsl2-1",
            "device_num": "2",
            "sync_mode": "secondary_synced",
            "gmsl_trigger_fps": "3000",
            "enable_gmsl_trigger": "true",
            "config_file_path": secondary_config_file_path,
        }.items(),
    )

    launch2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "gemini_330_series.launch.py")
        ),
        launch_arguments={
            "camera_name": "camera_02",
            "usb_port": "gmsl2-3",
            "device_num": "2",
            "sync_mode": "secondary_synced",
            "config_file_path": secondary_config_file_path,
        }.items(),
    )

    # Launch description
    ld = LaunchDescription(
        [
            TimerAction(period=0.0, actions=[GroupAction([launch2_include])]),
            TimerAction(period=2.0, actions=[GroupAction([launch1_include])]),
            # The camera that enables the GMSL trigger should be launched at last
        ]
    )

    return ld
