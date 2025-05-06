import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription


def launch_moveit_config(context, *args, **kwargs):
    robot_model_name = LaunchConfiguration('robot_model_name').perform(context)

    if robot_model_name == 'arm620':
        config_pkg = 'arm620_config'
    elif robot_model_name == 'arm380':
        config_pkg = 'arm380_config'
    else:
        raise ValueError(f'Unsupported robot model name: {robot_model_name}')
    
    moveit_config_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(config_pkg), 'launch', 'real_moveit_demo.launch.py')
        )
    )
    return [moveit_config_launch]


def generate_launch_description():

    robot_model_arg = DeclareLaunchArgument(
        'robot_model_name',
        default_value='arm620',  # 默认是arm620
        description='Robot model name (e.g., arm620 or arm380)'
    )

    robot_driver = IncludeLaunchDescription (
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('robot_driver'), 'launch', 'robot_driver.launch.py'))
    )

    robto_tcp_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('robot_qtrecv'), 'launch', 'robot_tcp_server.launch.py'))
    )

    robot_func = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('robot_kinematics'), 'launch', 'robot_func.launch.py'))
    )

    robot_current_state_get = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('robot_kinematics'), 'launch', 'robot_state_get.launch.py'))
    )

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('robot_description'), 'launch', 'robot_description.launch.py')),
        launch_arguments={'robot_model_name': LaunchConfiguration('robot_model_name')}.items()
    )

    robot_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('robot_control'), 'launch', 'robot_control.launch.py'))
    )

    robot_moveit_config = OpaqueFunction(function=launch_moveit_config)

    robot_key_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('robot_kinematics'), 'launch', 'robot_controller.launch.py')),
        launch_arguments={'robot_model_name': LaunchConfiguration('robot_model_name')}.items()
    )

    return LaunchDescription([
        robot_model_arg,
        robot_driver,
        robto_tcp_server,
        robot_func,
        robot_current_state_get,
        robot_description,
        robot_control,
        robot_moveit_config,
        robot_key_controller,
    ])