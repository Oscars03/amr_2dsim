# =============================================================
#  navigation.launch.py  —  AMR_2dsim  (ROS2 Jazzy)
#  Runs: Map Server + AMCL + Nav2 stack + RViz2
#  Does NOT run: simulator_node (launch separately)
# =============================================================
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def static_tf(parent, child, xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=f"static_tf_{parent}_to_{child}",
        arguments=[
            str(xyz[0]), str(xyz[1]), str(xyz[2]),
            str(rpy[0]), str(rpy[1]), str(rpy[2]),
            parent, child,
        ],
        output="screen",
    )

def generate_launch_description():
    pkg_dir          = get_package_share_directory("amr_2dsim")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    # ── Declare args ─────────────────────────────────────────────
    declare_map = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(pkg_dir, "maps", "map_1782965660.yaml"),
        description="Full path to map YAML file",
    )
    declare_params = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(pkg_dir, "config", "nav2_params.yaml"),
        description="Full path to nav2_params.yaml",
    )
    declare_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",          # ← string "false", not Python False
        description="Use simulation clock",
    )
    declare_autostart = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Auto-start Nav2 lifecycle nodes",
    )
    declare_use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz2",
    )
    declare_rviz_cfg = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(pkg_dir, "rviz", "amr_2dsim.rviz"),
        description="Path to RViz2 config",
    )
    declare_log = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging verbosity",
    )

    # ── LaunchConfiguration handles ──────────────────────────────
    map_yaml     = LaunchConfiguration("map")
    params_file  = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart    = LaunchConfiguration("autostart")
    use_rviz     = LaunchConfiguration("use_rviz")
    rviz_config  = LaunchConfiguration("rviz_config")
    log_level    = LaunchConfiguration("log_level")

    # ── Read URDF at parse time (plain Python string) ─────────────
    urdf_path = os.path.join(pkg_dir, "urdf", "amr.urdf")
    with open(urdf_path, "r") as f:
        robot_description = f.read()

    # ─────────────────────────────────────────────────────────────
    # NODES
    # ─────────────────────────────────────────────────────────────

    # 1. Static TF: base_link → laser_link
    tf_base_laser = static_tf(
        "base_link", "laser_link",
        xyz=(0.0, 0.0, 0.18),
    )

    # 2. Robot State Publisher
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,  # plain string ✓
            "use_sim_time":      use_sim_time,        # LaunchConfig ✓
        }],
    )

    # 3. Map Server
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            params_file,
            {"yaml_filename": map_yaml,
             "use_sim_time":  use_sim_time},
        ],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # 4. AMCL
    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[
            params_file,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[("scan", "/scan")],
        arguments=["--ros-args", "--log-level", log_level],
    )

    # 5. Lifecycle Manager — Localisation
    lifecycle_loc = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart":    autostart,
            "node_names":   ["map_server", "amcl"],
        }],
    )

    # 6. Nav2 navigation stack
    #    ⚠️  launch_arguments values must be substitutions or plain strings,
    #        NOT raw Python booleans (True/False) or bare words (true/false)
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments=[
            ("use_sim_time",    use_sim_time),   # LaunchConfiguration ✓
            ("autostart",       autostart),       # LaunchConfiguration ✓
            ("params_file",     params_file),     # LaunchConfiguration ✓
            ("use_composition", "False"),         # plain string        ✓
            ("log_level",       log_level),       # LaunchConfiguration ✓
        ],
    )

    # 7. RViz2
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    # ─────────────────────────────────────────────────────────────
    return LaunchDescription([
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),

        # ── args ─────────────────────────────────────────────────
        declare_map,
        declare_params,
        declare_sim_time,
        declare_autostart,
        declare_use_rviz,
        declare_rviz_cfg,
        declare_log,

        # ── info ─────────────────────────────────────────────────
        LogInfo(msg=["[navigation.launch] map    : ", map_yaml]),
        LogInfo(msg=["[navigation.launch] params : ", params_file]),

        # ── nodes ────────────────────────────────────────────────
        tf_base_laser,
        rsp,
        map_server,
        amcl,
        lifecycle_loc,
        navigation,
        rviz,
    ])