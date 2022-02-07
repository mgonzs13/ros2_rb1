import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import Node


def generate_launch_description():

    bringup_dir = get_package_share_directory("rb1_navigation")
    launch_dir = os.path.join(bringup_dir, "launch/navigation")

    rviz_config_dir = os.path.join(
        bringup_dir,
        "rviz",
        "nav2_view.rviz")

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1")

    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    cmd_vel_topic_cmd = DeclareLaunchArgument(
        "cmd_vel_topic",
        default_value="/cmd_vel",
        description="cmd_vel topic (for remmaping)")

    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_rviz_cmd = DeclareLaunchArgument(
        "launch_rviz",
        default_value="True",
        description="Whether launch rviz")

    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_x_cmd = DeclareLaunchArgument(
        "initial_pose_x",
        default_value="1.336",
        description="Initial pose x")

    initial_pose_y = LaunchConfiguration("initial_pose_y")
    initial_pose_y_cmd = DeclareLaunchArgument(
        "initial_pose_y",
        default_value="6.544",
        description="Initial pose y")

    initial_pose_z = LaunchConfiguration("initial_pose_z")
    initial_pose_z_cmd = DeclareLaunchArgument(
        "initial_pose_z",
        default_value="0.0",
        description="Initial pose z")

    initial_pose_yaw = LaunchConfiguration("initial_pose_yaw")
    initial_pose_yaw_cmd = DeclareLaunchArgument(
        "initial_pose_yaw",
        default_value="0.0",
        description="Initial pose yaw")

    namespace = LaunchConfiguration("namespace")
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace")

    use_namespace = LaunchConfiguration("use_namespace")
    declare_use_namespace_cmd = DeclareLaunchArgument(
        "use_namespace",
        default_value="False",
        description="Whether to apply a namespace to the navigation stack")

    slam = LaunchConfiguration("slam")
    declare_slam_cmd = DeclareLaunchArgument(
        "slam",
        default_value="False",
        description="Whether run a SLAM")

    map_yaml_file = LaunchConfiguration(
        "map",
        default=os.path.join(
            bringup_dir,
            "maps/apartamento_leon",
            "apartamento_leon_gimp_con_mesa_tv.yaml"))
    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=map_yaml_file,
        description="Full path to map yaml file to load")

    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if True")

    param_file_name = "nav2_params" + ".yaml"
    params_file = LaunchConfiguration(
        "params_file",
        default=os.path.join(
            bringup_dir,
            "config",
            param_file_name))
    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=params_file,
        description="Full path to the ROS2 parameters file to use for all launched nodes")

    default_bt_xml_filename = LaunchConfiguration("default_bt_xml_filename")
    declare_bt_xml_cmd = DeclareLaunchArgument(
        "default_bt_xml_filename",
        default_value=os.path.join(
            get_package_share_directory("nav2_bt_navigator"),
            "behavior_trees", "navigate_w_replanning_and_recovery.xml"),
        description="Full path to the behavior tree xml file to use")

    autostart = LaunchConfiguration("autostart")
    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart", default_value="True",
        description="Automatically startup the nav2 stack")

    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, "slam.launch.py")),
            condition=IfCondition(slam),
            launch_arguments={"namespace": namespace,
                              "use_sim_time": use_sim_time,
                              "autostart": autostart,
                              "params_file": params_file}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir,
                                                       "localization.launch.py")),
            condition=IfCondition(PythonExpression(["not ", slam])),
            launch_arguments={"initial_pose_x": initial_pose_x,
                              "initial_pose_y": initial_pose_y,
                              "initial_pose_z": initial_pose_z,
                              "initial_pose_yaw": initial_pose_yaw,
                              "namespace": namespace,
                              "map": map_yaml_file,
                              "use_sim_time": use_sim_time,
                              "autostart": autostart,
                              "params_file": params_file,
                              "use_lifecycle_mgr": "False"}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                launch_dir, "navigation.launch.py")),
            launch_arguments={"cmd_vel_topic": cmd_vel_topic,
                              "namespace": namespace,
                              "use_sim_time": use_sim_time,
                              "autostart": autostart,
                              "params_file": params_file,
                              "default_bt_xml_filename": default_bt_xml_filename,
                              "use_lifecycle_mgr": "False",
                              "map_subscribe_transient_local": "True"}.items()),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_dir],
            parameters=[{"use_sim_time": use_sim_time}],
            # output="log",
            condition=IfCondition(PythonExpression([launch_rviz]))),
    ])

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(cmd_vel_topic_cmd)
    ld.add_action(launch_rviz_cmd)

    ld.add_action(initial_pose_x_cmd)
    ld.add_action(initial_pose_y_cmd)
    ld.add_action(initial_pose_z_cmd)
    ld.add_action(initial_pose_yaw_cmd)

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_bt_xml_cmd)

    ld.add_action(bringup_cmd_group)

    return ld
