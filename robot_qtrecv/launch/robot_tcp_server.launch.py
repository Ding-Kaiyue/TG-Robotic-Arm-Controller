from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    robot_tcp_server = Node (
        package="robot_qtrecv",
        executable='robot_tcp_server',
    )

    ld.add_action(robot_tcp_server)
    return ld