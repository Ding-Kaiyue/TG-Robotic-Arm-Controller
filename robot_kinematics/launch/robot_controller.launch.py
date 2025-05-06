from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.actions import SetParameter

def generate_launch_description():
    ld = LaunchDescription()
    
    robot_key_controller = Node (
        package = 'robot_kinematics',
        executable ='robot_key_controller',
        parameters=[{
            'robot_model_name': 'arm620'
        }]
    )
    ld.add_action(robot_key_controller)
    return ld
