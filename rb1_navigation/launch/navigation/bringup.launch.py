# Copyright (C) 2023  Miguel Ángel González Santamarta

# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("rb1_navigation")
    launch_dir = os.path.join(bringup_dir, "launch/navigation")

    rviz_config_dir = os.path.join(
        bringup_dir,
        "rviz",
        "nav2_view.rviz")

    # Create the launch configuration variables
    namespace = LaunchConfiguration("namespace")
    use_namespace = LaunchConfiguration("use_namespace")
    slam = LaunchConfiguration("slam")
    map_yaml_file = LaunchConfiguration(
        "map",
        default=os.path.join(
            bringup_dir,
            "maps/apartamento_leon",
            "apartamento_leon_gimp_con_mesa_tv.yaml"))
    use_sim_time = LaunchConfiguration("use_sim_time")
    param_file_name = "nav2_params" + ".yaml"
    params_file = LaunchConfiguration(
        "params_file",
        default=os.path.join(
            bringup_dir,
            "config",
            param_file_name))
    autostart = LaunchConfiguration("autostart")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    launch_rviz = LaunchConfiguration("launch_rviz")
    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_y = LaunchConfiguration("initial_pose_y")
    initial_pose_z = LaunchConfiguration("initial_pose_z")
    initial_pose_yaw = LaunchConfiguration("initial_pose_yaw")

    remappings = [("/tf", "tf"),
                  ("/tf_static", "tf_static"),
                  ("/cmd_vel", cmd_vel_topic)]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        "use_sim_time": use_sim_time,
        "yaml_filename": map_yaml_file}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1")

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Top-level namespace")

    declare_use_namespace_cmd = DeclareLaunchArgument(
        "use_namespace",
        default_value="false",
        description="Whether to apply a namespace to the navigation stack")

    declare_slam_cmd = DeclareLaunchArgument(
        "slam",
        default_value="False",
        description="Whether run a SLAM")

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        description="Full path to map yaml file to load")

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true")

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "params", "nav2_params.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes")

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart", default_value="true",
        description="Automatically startup the nav2 stack")

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition", default_value="True",
        description="Whether to use composed bringup")

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn", default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.")

    cmd_vel_topic_cmd = DeclareLaunchArgument(
        "cmd_vel_topic",
        default_value="/cmd_vel",
        description="cmd_vel topic (for remmaping)")

    launch_rviz_cmd = DeclareLaunchArgument(
        "launch_rviz",
        default_value="True",
        description="Whether launch rviz")

    initial_pose_x_cmd = DeclareLaunchArgument(
        "initial_pose_x",
        default_value="1.336",
        description="Initial pose x")

    initial_pose_y_cmd = DeclareLaunchArgument(
        "initial_pose_y",
        default_value="6.544",
        description="Initial pose y")

    initial_pose_z_cmd = DeclareLaunchArgument(
        "initial_pose_z",
        default_value="0.0",
        description="Initial pose z")

    initial_pose_yaw_cmd = DeclareLaunchArgument(
        "initial_pose_yaw",
        default_value="0.0",
        description="Initial pose yaw")

    # Specify the actions
    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

        Node(
            condition=IfCondition(use_composition),
            name="nav2_container",
            package="rclcpp_components",
            executable="component_container_isolated",
            parameters=[configured_params, {"autostart": autostart}],
            remappings=remappings,
            output="screen"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, "slam.launch.py")),
            condition=IfCondition(slam),
            launch_arguments={"namespace": namespace,
                              "use_sim_time": use_sim_time,
                              "autostart": autostart,
                              "use_respawn": use_respawn,
                              "params_file": params_file}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir,
                                                       "localization.launch.py")),
            condition=IfCondition(PythonExpression(["not ", slam])),
            launch_arguments={"namespace": namespace,
                              "initial_pose_x": initial_pose_x,
                              "initial_pose_y": initial_pose_y,
                              "initial_pose_z": initial_pose_z,
                              "initial_pose_yaw": initial_pose_yaw,
                              "map": map_yaml_file,
                              "use_sim_time": use_sim_time,
                              "autostart": autostart,
                              "params_file": params_file,
                              "use_composition": use_composition,
                              "use_respawn": use_respawn,
                              "container_name": "nav2_container"}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                launch_dir, "navigation.launch.py")),
            launch_arguments={"namespace": namespace,
                              "cmd_vel_topic": cmd_vel_topic,
                              "use_sim_time": use_sim_time,
                              "autostart": autostart,
                              "params_file": params_file,
                              "use_composition": use_composition,
                              "use_respawn": use_respawn,
                              "container_name": "nav2_container"}.items()),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_dir],
            parameters=[{"use_sim_time": use_sim_time}],
            # output="log",
            condition=IfCondition(PythonExpression([launch_rviz]))),
    ])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(cmd_vel_topic_cmd)
    ld.add_action(launch_rviz_cmd)

    ld.add_action(initial_pose_x_cmd)
    ld.add_action(initial_pose_y_cmd)
    ld.add_action(initial_pose_z_cmd)
    ld.add_action(initial_pose_yaw_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    return ld
