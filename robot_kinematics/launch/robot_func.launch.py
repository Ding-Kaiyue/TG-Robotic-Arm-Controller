from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.actions import SetParameter

def generate_launch_description():
    ld = LaunchDescription()
    
    robot_functions = Node (
        package = 'robot_kinematics',
        executable ='robot_func',
    )
    ld.add_action(robot_functions)
    return ld
