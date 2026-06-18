import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rname = LaunchConfiguration("rname")
    world = LaunchConfiguration("world")
    map_yaml = LaunchConfiguration("map_yaml")
    nav2_params = LaunchConfiguration("nav2_params")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    autostart = LaunchConfiguration("autostart")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart_param = ParameterValue(autostart, value_type=bool)
    use_sim_time_param = ParameterValue(use_sim_time, value_type=bool)

    rl_sar_share = FindPackageShare("rl_sar")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([rl_sar_share, "launch", "gazebo.launch.py"])
        ),
        launch_arguments={
            "rname": rname,
            "world": [world, TextSubstitution(text=".world")],
            "use_sim_time": use_sim_time,
        }.items(),
    )

    default_map_yaml = PathJoinSubstitution([rl_sar_share, "maps", "a1_nav_world.yaml"])
    default_nav2_params = PathJoinSubstitution([rl_sar_share, "config", "a1_nav2.yaml"])
    default_rviz_config = PathJoinSubstitution([rl_sar_share, "config", "a1_nav2.rviz"])

    rl_sim_node = Node(
        package="rl_sar",
        executable="rl_sim",
        name="rl_sim_node",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time_param,
            "auto_start_locomotion": autostart_param,
            "navigation_mode_default": True,
            "cmd_vel_timeout_sec": 0.25,
            "min_vx": -0.25,
            "max_vx": 0.45,
            "min_vy": 0.0,
            "max_vy": 0.0,
            "max_wz": 0.70,
            "cmd_vel_yaw_scale": 1.0,
            "acc_lim_x": 0.60,
            "acc_lim_y": 0.0,
            "acc_lim_wz": 1.20,
        }],
    )

    ground_truth_odom = Node(
        package="rl_sar",
        executable="gazebo_ground_truth_odom.py",
        name="gazebo_ground_truth_odom",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time_param,
            "source_odom_topic": "/ground_truth/odom_raw",
            "odom_topic": "/odom",
            "odom_frame": "odom",
            "base_frame": "base",
            "publish_tf": True,
            "zero_z": False,
        }],
    )

    ground_truth_odom_eval = Node(
        package="rl_sar",
        executable="gazebo_ground_truth_odom.py",
        name="gazebo_ground_truth_odom_eval",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time_param,
            "source_odom_topic": "/ground_truth/odom_raw",
            "odom_topic": "/odom_gt",
            "odom_frame": "odom_gt",
            "base_frame": "base",
            "publish_tf": False,
            "zero_z": False,
        }],
    )

    map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_static_tf",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="screen",
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time_param,
            "yaml_filename": map_yaml,
        }],
    )

    map_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time_param,
            "autostart": True,
            "node_names": ["map_server"],
        }],
    )

    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params,
            "autostart": "true",
        }.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(use_rviz),
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time_param}],
    )

    return LaunchDescription([
        DeclareLaunchArgument("rname", default_value=TextSubstitution(text="a1")),
        DeclareLaunchArgument("world", default_value=TextSubstitution(text="a1_nav_world")),
        DeclareLaunchArgument("map_yaml", default_value=default_map_yaml),
        DeclareLaunchArgument("nav2_params", default_value=default_nav2_params),
        DeclareLaunchArgument("use_rviz", default_value=TextSubstitution(text="true")),
        DeclareLaunchArgument("rviz_config", default_value=default_rviz_config),
        DeclareLaunchArgument("autostart", default_value=TextSubstitution(text="true")),
        DeclareLaunchArgument("use_sim_time", default_value=TextSubstitution(text="true")),
        gazebo_launch,
        rl_sim_node,
        ground_truth_odom,
        ground_truth_odom_eval,
        map_to_odom,
        map_server,
        map_lifecycle,
        nav2_navigation,
        rviz_node,
    ])
