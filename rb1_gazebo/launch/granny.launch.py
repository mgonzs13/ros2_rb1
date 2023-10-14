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
    IncludeLaunchDescription, SetEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    pkg_dir = get_package_share_directory("rb1_gazebo")

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1")

    map_yaml_file = os.path.join(
        pkg_dir,
        "maps/granny",
        "GrannyAnnie.yaml")

    world = os.path.join(pkg_dir, "worlds", "GrannyAnnie.world")

    gazebo_nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("rb1_gazebo"),
                         "launch", "gazebo_nav2.launch.py")),
        launch_arguments={"map": map_yaml_file,
                          "world": world,
                          "initial_nav_pose_x": "0.0",
                          "initial_nav_pose_y": "0.0",
                          "initial_nav_pose_yaw": "3.1415",
                          "initial_gaz_pose_x": "0.0",
                          "initial_gaz_pose_y": "0.0",
                          "initial_gaz_pose_yaw": "0.0"}.items())

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(gazebo_nav2_cmd)

    return ld
