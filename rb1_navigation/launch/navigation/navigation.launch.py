import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    bringup_dir = get_package_share_directory("nav2_bringup")

    logging_buf_stream_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1")

    ### ARGS ###
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    cmd_vel_topic_cmd = DeclareLaunchArgument(
        "cmd_vel_topic",
        default_value="/cmd_vel",
        description="cmd_vel topic (for remmaping)")

    namespace = LaunchConfiguration("namespace")
    namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="",
        description="Top-level namespace")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time", default_value="False",
        description="Use simulation (Gazebo) clock if True")

    autostart = LaunchConfiguration("autostart")
    autostart_cmd = DeclareLaunchArgument(
        "autostart", default_value="True",
        description="Automatically startup the nav2 stack")

    params_file = LaunchConfiguration("params_file")
    params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            bringup_dir, "config", "nav2_params.yaml"),
        description="Full path to the ROS2 parameters file to use")

    default_bt_xml_filename = LaunchConfiguration("default_bt_xml_filename")
    default_bt_xml_filename_cmd = DeclareLaunchArgument(
        "default_bt_xml_filename",
        default_value=os.path.join(
            get_package_share_directory("nav2_bt_navigator"),
            "behavior_trees", "navigate_w_replanning_and_recovery.xml"),
        description="Full path to the behavior tree xml file to use")

    map_subscribe_transient_local = LaunchConfiguration(
        "map_subscribe_transient_local")
    map_subscribe_transient_local_cmd = DeclareLaunchArgument(
        "map_subscribe_transient_local", default_value="False",
        description="Whether to set the map subscriber QoS to transient local")

    lifecycle_nodes = ["controller_server",
                       "planner_server",
                       "recoveries_server",
                       "bt_navigator",
                       "waypoint_follower"]

    remappings = [("/tf", "tf"),
                  ("/tf_static", "tf_static"),
                  ("/cmd_vel", cmd_vel_topic)]

    param_substitutions = {
        "use_sim_time": use_sim_time,
        "default_bt_xml_filename": default_bt_xml_filename,
        "autostart": autostart,
        "map_subscribe_transient_local": map_subscribe_transient_local}

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    ### NODES ###
    controller_server_cmd = Node(
        package="nav2_controller",
        executable="controller_server",
        output="screen",
        parameters=[configured_params],
        remappings=remappings)

    planner_server_cmd = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[configured_params],
        remappings=remappings)

    recoveries_server_cmd = Node(
        package="nav2_recoveries",
        executable="recoveries_server",
        name="recoveries_server",
        output="screen",
        parameters=[configured_params],
        remappings=remappings)

    bt_navigator_cmd = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[configured_params],
        remappings=remappings)

    waypoint_follower_cmd = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        output="screen",
        parameters=[configured_params],
        remappings=remappings)

    lifecycle_manager_navigation_cmd = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time},
                    {"autostart": autostart},
                    {"node_names": lifecycle_nodes}])

    ld = LaunchDescription()

    ld.add_action(logging_buf_stream_envvar)

    ld.add_action(cmd_vel_topic_cmd)
    ld.add_action(namespace_cmd)
    ld.add_action(use_sim_time_cmd)
    ld.add_action(autostart_cmd)
    ld.add_action(params_file_cmd)
    ld.add_action(default_bt_xml_filename_cmd)
    ld.add_action(map_subscribe_transient_local_cmd)

    ld.add_action(controller_server_cmd)
    ld.add_action(planner_server_cmd)
    ld.add_action(recoveries_server_cmd)
    ld.add_action(bt_navigator_cmd)
    ld.add_action(waypoint_follower_cmd)
    ld.add_action(lifecycle_manager_navigation_cmd)

    return ld
