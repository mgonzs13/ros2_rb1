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
from launch.actions import (
    IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    bringup_dir = get_package_share_directory("rb1_navigation")
    launch_dir = os.path.join(bringup_dir, "launch/navigation")

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1")

    params_file = os.path.join(
        bringup_dir,
        "config",
        "nav2_params.yaml")

    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_rviz_cmd = DeclareLaunchArgument(
        "launch_rviz",
        default_value="True",
        description="Whether launch rviz")

    slam = LaunchConfiguration("slam")
    slam_cmd = DeclareLaunchArgument(
        "slam",
        default_value="False",
        description="Whether run a SLAM")

    map_yaml_file = LaunchConfiguration(
        "map",
        default=os.path.join(
            bringup_dir,
            "maps/apartamento_leon",
            "apartamento_leon_gimp_con_mesa_tv.yaml"))
    map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=map_yaml_file,
        description="Full path to map yaml file to load")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation (Gazebo) clock if True")

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

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "bringup.launch.py")),
        launch_arguments={"launch_rviz": launch_rviz,
                          "slam": slam,
                          "params_file": params_file,
                          "map": map_yaml_file,
                          "use_sim_time": use_sim_time,
                          "initial_pose_x": initial_pose_x,
                          "initial_pose_y": initial_pose_y,
                          "initial_pose_z": initial_pose_z,
                          "initial_pose_yaw": initial_pose_yaw,
                          "cmd_vel_topic": "/cmd_vel"}.items())

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)

    ld.add_action(launch_rviz_cmd)
    ld.add_action(slam_cmd)
    ld.add_action(map_yaml_cmd)
    ld.add_action(use_sim_time_cmd)
    ld.add_action(initial_pose_x_cmd)
    ld.add_action(initial_pose_y_cmd)
    ld.add_action(initial_pose_z_cmd)
    ld.add_action(initial_pose_yaw_cmd)

    ld.add_action(bringup_cmd)

    return ld
