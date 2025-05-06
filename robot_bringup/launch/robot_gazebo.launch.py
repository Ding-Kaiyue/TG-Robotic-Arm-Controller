import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_moveit_config(context, *args, **kwargs):
    robot_model_name = context.launch_configurations['robot_model_name']

    if robot_model_name == 'arm620':
        config_pkg = 'arm620_config'
    elif robot_model_name == 'arm380':
        config_pkg = 'arm380_config'
    else:
        raise ValueError(f'Unsupported robot model name: {robot_model_name}')
    
    moveit_config_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(config_pkg), 'launch', 'gazebo_moveit_demo.launch.py')
        ),
        launch_arguments=[
            ('robot_model_name', robot_model_name)
        ]
    )
    return [moveit_config_launch]


def generate_launch_description():
    robot_model_arg = DeclareLaunchArgument(
        'robot_model_name',
        default_value='arm620',  # 默认是arm620
        description='Robot model name (e.g., arm620 or arm380)'
    )

    robot_gazebo_up = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('robot_gazebo'), 'launch', 'robot_gazebo.launch.py')
        ),
        launch_arguments=[
            ('robot_model_name', LaunchConfiguration('robot_model_name'))
        ]
    )

    robot_moveit_config = OpaqueFunction(function=launch_moveit_config)

    return LaunchDescription([
        robot_model_arg,
        robot_gazebo_up,
        robot_moveit_config
    ])

     