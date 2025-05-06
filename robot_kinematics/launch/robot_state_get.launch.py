from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.actions import SetParameter

def generate_launch_description():
    ld = LaunchDescription()
    
    robot_state_get = Node (
        package = 'robot_kinematics',
        executable ='robot_state_get',
    )
    ld.add_action(robot_state_get)
    return ld
