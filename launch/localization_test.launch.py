import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
from launch.substitutions import Command

def generate_launch_description():
    pkg_path = get_package_share_directory('urdf_ctrl')
    config_file = os.path.join(pkg_path, 'config', 'urdf_ctrl.yaml')
    
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)
    
    # 2. 从配置文件中获取 xacro 文件路径并转为 URDF 描述
    xacro_file_path = config.get('xacro_file', os.path.join(pkg_path, 'urdf', 'indoor01.urdf.xacro'))
    xacro_file = os.path.join(pkg_path, xacro_file_path)
    robot_description = Command(['xacro ', xacro_file])

    map_file = "/home/solex/maps/test1.yaml"

    return LaunchDescription([
        # 1. 启动 robot_state_publisher 节点（发布机器人 TF 和状态）
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
            output='screen'
        ),

        # 2. 启动 map_server 节点
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_file, 'use_sim_time': False}]
        ),

        # 3. 启动 lifecycle_manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{'autostart': True, 'node_names': ['map_server'], 'use_sim_time': False}]
        ),

        # 4. 启动 RViz2
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     parameters=[{'use_sim_time': False}]
        # ),

        # 5. 你的定位节点
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='localization_2d',
                    executable='localization_2d_node',
                    name='localization_2d_node',
                    output='screen',
                    parameters=[{'use_sim_time': False}]
                )
            ]
        )
    ])