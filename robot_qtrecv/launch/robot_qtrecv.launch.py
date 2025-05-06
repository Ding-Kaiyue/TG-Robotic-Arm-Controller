from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    robot_qtrecv_node = Node (
        package="robot_qtrecv",
        executable='robot_qtrecv',
    )

    ld.add_action(robot_qtrecv_node)
    return ld