from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    bluesea2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("bluesea2"), "/launch/udp_lidar.launch"]
        )
    )

    lpms_ig1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("lpms_ig1"), "/launch/lpms_si1_launch.py"]
        )
    )

    vcu_ctrl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("vcu_ctrl"), "/launch/vcu_ctrl.launch.py"]
        )
    )

    return LaunchDescription([
        lpms_ig1_launch,
        vcu_ctrl_launch,
        bluesea2_launch,
    ])