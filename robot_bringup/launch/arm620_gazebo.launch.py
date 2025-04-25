import os
from  ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # For Realman
    robot_gazebo_up =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('robot_gazebo')), 'launch', 'arm620_gazebo.launch.py'))
    )

    robot_moveit_up = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(('arm620_config')), 'launch', 'gazebo_moveit_demo.launch.py'))
    )

    return LaunchDescription ([
        robot_gazebo_up,
        robot_moveit_up
    ])

     