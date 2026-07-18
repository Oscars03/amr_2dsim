import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'amr_2dsim'
    pkg_share = get_package_share_directory(pkg_name)
    
    # ดึง Path ของไฟล์ Config และ RViz
    my_config_file = os.path.join(pkg_share, 'config', 'mapping_params.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'mapping.rviz')
    
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'slam_params_file': my_config_file  
        }.items()
    )



    # ย้าย RViz2 มาเปิดในไฟล์นี้
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_display',
        arguments=['-d', rviz_config]
    )

    return LaunchDescription([
        slam_toolbox_launch,
        rviz_node  # <--- เพิ่มเข้ามาใน Launch Description
    ])