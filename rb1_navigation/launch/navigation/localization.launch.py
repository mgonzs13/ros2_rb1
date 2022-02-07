import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory("nav2_bringup")

    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_y = LaunchConfiguration("initial_pose_y")
    initial_pose_z = LaunchConfiguration("initial_pose_z")
    initial_pose_yaw = LaunchConfiguration("initial_pose_yaw")

    namespace = LaunchConfiguration("namespace")
    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")
    lifecycle_nodes = ["map_server", "amcl"]

    remappings = [("/tf", "tf"),
                  ("/tf_static", "tf_static")]

    param_substitutions = {
        "use_sim_time": use_sim_time,
        "yaml_filename": map_yaml_file,
        "initial_pose.x": initial_pose_x,
        "initial_pose.y": initial_pose_y,
        "initial_pose.z": initial_pose_z,
        "initial_pose.yaw": initial_pose_yaw}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    return LaunchDescription([

        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),

        DeclareLaunchArgument(
            "namespace", default_value="",
            description="Top-level namespace"),

        DeclareLaunchArgument(
            "map",
            default_value=os.path.join(
                bringup_dir, "maps", "turtlebot3_world.yaml"),
            description="Full path to map yaml file to load"),

        DeclareLaunchArgument(
            "use_sim_time", default_value="False",
            description="Use simulation (Gazebo) clock if True"),

        DeclareLaunchArgument(
            "autostart", default_value="True",
            description="Automatically startup the nav2 stack"),

        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(
                bringup_dir, "config", "nav2_params.yaml"),
            description="Full path to the ROS2 parameters file to use"),

        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[configured_params],
            remappings=remappings),

        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[configured_params],
            remappings=remappings),

        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time},
                        {"autostart": autostart},
                        {"node_names": lifecycle_nodes}])
    ])
