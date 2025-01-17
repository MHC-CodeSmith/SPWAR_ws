from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

import os

def generate_launch_description():
    pkg_share = FindPackageShare(package='spot_description').find('spot_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'spot_with_arm.urdf')

    # Certifique-se de carregar o conteúdo do URDF como uma string
    with open(urdf_file, 'r') as infp:
        robot_description_content = infp.read()

    # Publica o modelo no tópico `/robot_description`
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # Comando para carregar o modelo no Gazebo
    spawn_entity_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-entity', 'spot', '-file', urdf_file,
            '-x', '0', '-y', '0', '-z', '0.1'
        ],
        output='screen'
    )

    return LaunchDescription([
        # Gazebo server e client
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        robot_state_publisher_node,
        spawn_entity_cmd
    ])
