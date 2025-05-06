from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    robot_driver_node = Node (
        package="robot_driver",
        executable="robot_driver_canrx",
    )
    ld.add_action(robot_driver_node)
    return ld