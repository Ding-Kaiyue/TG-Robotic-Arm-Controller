from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    control_node = Node(
        package='robot_control',
        executable='robot_control',
        # parameters=[
        #     {"follow": False}
        # ],
        output='screen',
    )
    ld.add_action(control_node)
    return ld