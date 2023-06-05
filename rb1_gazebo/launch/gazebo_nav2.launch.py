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
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    ### ARGS ###
    world = LaunchConfiguration("world")
    world_cmd = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(
            get_package_share_directory("rb1_gazebo"), "world", "empty.world"),
        description="Gazebo world")

    map_yaml_file = LaunchConfiguration(
        "map",
        default=os.path.join(
            get_package_share_directory("rb1_navigation"),
            "maps/apartamento_leon",
            "apartamento_leon_gimp_con_mesa_tv.yaml"))
    map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=map_yaml_file,
        description="Full path to map yaml file to load")

    ## NAV ##
    initial_nav_pose_x = LaunchConfiguration("initial_nav_pose_x")
    initial_nav_pose_x_cmd = DeclareLaunchArgument(
        "initial_nav_pose_x",
        default_value="1.336",
        description="Initial navigation pose x")

    initial_nav_pose_y = LaunchConfiguration("initial_nav_pose_y")
    initial_nav_pose_y_cmd = DeclareLaunchArgument(
        "initial_nav_pose_y",
        default_value="6.544",
        description="Initial navigation pose x")

    initial_nav_pose_z = LaunchConfiguration("initial_nav_pose_z")
    initial_nav_pose_z_cmd = DeclareLaunchArgument(
        "initial_nav_pose_z",
        default_value="0.0",
        description="Initial navigation pose z")

    ## GAZ ##
    initial_nav_pose_yaw = LaunchConfiguration("initial_nav_pose_yaw")
    initial_nav_pose_yaw_cmd = DeclareLaunchArgument(
        "initial_nav_pose_yaw",
        default_value="0.0",
        description="Initial navigation pose yaw")

    initial_gaz_pose_x = LaunchConfiguration("initial_gaz_pose_x")
    initial_gaz_pose_x_cmd = DeclareLaunchArgument(
        "initial_gaz_pose_x",
        default_value="0.0",
        description="Initial gazigation pose x")

    initial_gaz_pose_y = LaunchConfiguration("initial_gaz_pose_y")
    initial_gaz_pose_y_cmd = DeclareLaunchArgument(
        "initial_gaz_pose_y",
        default_value="0.0",
        description="Initial gazebo pose x")

    initial_gaz_pose_z = LaunchConfiguration("initial_gaz_pose_z")
    initial_gaz_pose_z_cmd = DeclareLaunchArgument(
        "initial_gaz_pose_z",
        default_value="0.0",
        description="Initial gazebo pose z")

    initial_gaz_pose_yaw = LaunchConfiguration("initial_gaz_pose_yaw")
    initial_gaz_pose_yaw_cmd = DeclareLaunchArgument(
        "initial_gaz_pose_yaw",
        default_value="0.0",
        description="Initial gazebo pose yaw")

    ### LAUNCHES ###
    rb1_nav_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("rb1_navigation"), "launch", "nav2_bringup.launch.py")),
        launch_arguments={"map": map_yaml_file,
                          "use_sim_time": "True",
                          "initial_pose_x": initial_nav_pose_x,
                          "initial_pose_y": initial_nav_pose_y,
                          "initial_pose_z": initial_nav_pose_z,
                          "initial_pose_yaw": initial_nav_pose_yaw}.items()
    )

    rb1_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("rb1_gazebo"), "launch", "gazebo.launch.py")),
        launch_arguments={"launch_gui": "True",
                          "launch_rviz": "False",
                          "world": world,
                          "initial_pose_x": initial_gaz_pose_x,
                          "initial_pose_y": initial_gaz_pose_y,
                          "initial_pose_z": initial_gaz_pose_z,
                          "initial_pose_yaw": initial_gaz_pose_yaw}.items()
    )

    ld = LaunchDescription()

    ld.add_action(world_cmd)
    ld.add_action(map_yaml_cmd)
    ld.add_action(initial_nav_pose_x_cmd)
    ld.add_action(initial_nav_pose_y_cmd)
    ld.add_action(initial_nav_pose_z_cmd)
    ld.add_action(initial_nav_pose_yaw_cmd)
    ld.add_action(initial_gaz_pose_x_cmd)
    ld.add_action(initial_gaz_pose_y_cmd)
    ld.add_action(initial_gaz_pose_z_cmd)
    ld.add_action(initial_gaz_pose_yaw_cmd)

    ld.add_action(rb1_nav_cmd)
    ld.add_action(rb1_gazebo_cmd)

    return ld
