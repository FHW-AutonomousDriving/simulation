# from https://answers.ros.org/question/314607/
# and https://answers.ros.org/question/373495/

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('gazebo_ros'), '/launch/gazebo.launch.py']),
        # from https://stackoverflow.com/questions/57696569/
        launch_arguments = {
          'verbose': 'true',
          'world': get_package_share_directory('car_demo')+'/worlds/fhw.world'
        }.items()
    )

    urdf = open(get_package_share_directory('prius_description')+'/urdf/prius.urdf', 'r').read()
    robot_state = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        name = 'robot_state_publisher',
        output = 'screen',
        parameters = [
            {'robot_description': urdf}
        ]
    )
    robot_spawner = Node(
        package = 'gazebo_ros',
        executable = 'spawn_entity.py',
        name = 'robot_spawner',
        output = 'screen',
        arguments = '-entity prius -topic /robot_description -x 3 -y -7 -z 0.5 -Y -1.570796'.split()
    )

    return LaunchDescription([
        gazebo,
        robot_state,
        robot_spawner
    ])