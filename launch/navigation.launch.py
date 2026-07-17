import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node # เพิ่มไลบรารีสำหรับรันโหนด ROS 2

def generate_launch_description():
    pkg_name = 'amr_2dsim'
    pkg_share = get_package_share_directory(pkg_name)
    
    # Path to config, default map, and rviz
    default_map_file = os.path.join(pkg_share, 'maps', 'map_1782965660.yaml')
    default_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    default_rviz_config_file = os.path.join(pkg_share, 'rviz', 'nav2_default.rviz') # เพิ่ม Path ไฟล์ RViz
    
    # Launch Arguments
    declare_map_yaml = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Full path to map yaml file to load'
    )
        
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file'
    )
        
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    declare_rviz_config_file = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config_file,
        description='Full path to the RVIZ config file to use'
    )

    # Include standard Nav2 bringup
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    # กำหนดโหนด RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )

    return LaunchDescription([
        declare_map_yaml,
        declare_params_file,
        declare_use_sim_time,
        declare_rviz_config_file,
        nav2_bringup_launch,
        rviz_node # สั่งรัน RViz2
    ])