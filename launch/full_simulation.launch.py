
import os.path
import time

import nav_msgs.msg
import rclpy.qos
import sensor_msgs.msg
import std_msgs.msg
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from pydoc_data.topics import topics

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def kill_daemon():
    pid = (
        os.popen("ps aux | grep -i ros2-daemon | grep -v grep | awk '{print $2}'")
        .read()
        .strip()
    )
    if pid:
        print(f"killing daemon with pid: {pid}")
        os.system(f"kill -9 {pid}")
        time.sleep(1)


def kill_ros_processes():
    pids = (
        os.popen("ps aux | grep -i ros-args | grep -v grep | awk '{print $2}'")
        .read()
        .strip()
    )
    pids = [pid.strip() for pid in pids.split("\n") if pid.strip() != ""]
    for pid in pids:
        print(f"shutting down ros process with pid: {pid}")
        os.system(f"kill -9 {pid}")
        time.sleep(1)
    if pids:
        time.sleep(3)


def generate_launch_description():
    # kill_daemon()
    kill_ros_processes()

    pkg_nav2_mod_planner = get_package_share_directory("nav2_mod_planner")

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_nav2_mod_planner, "launch", "turtlebot4_ignition.launch.py")
                ),
            )

    navigation = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(pkg_nav2_mod_planner, "launch", "navigation.launch")
        ), 
        launch_arguments={
            "sim": "true", 
            "params_file": os.path.join(pkg_nav2_mod_planner, "params", "navigation.yaml")
            }.items(),
    )
    
    start_mapping_node = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(pkg_nav2_mod_planner, "launch", "mapping.launch")
        )
    )

    ld = LaunchDescription()

  
    ld.add_action(gazebo)
    ld.add_action(navigation)
    ld.add_action(start_mapping_node)

    return ld


def main(argv=None):
    launch_service = launch.LaunchService(debug=False)
    launch_service.include_launch_description(generate_launch_description())
    return launch_service.run()


if __name__ == "__main__":
    main()