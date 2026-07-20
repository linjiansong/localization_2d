import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    map_file = "/home/linjs/solex_ws/src/maps/cartographer.yaml"

    return LaunchDescription([
        # 1. 启动 map_server 节点
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_file, 'use_sim_time': True}]
        ),

        # 2. 启动 lifecycle_manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{'autostart': True, 'node_names': ['map_server'], 'use_sim_time': True}]
        ),

        # 3. 启动 RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # 4. 你的定位节点
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='localization_2d',
                    executable='localization_2d_node',
                    name='localization_2d_node',
                    output='screen',
                    parameters=[{'use_sim_time': True}]
                )
            ]
        )
    ])