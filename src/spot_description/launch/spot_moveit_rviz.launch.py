from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    urdf_file = PathJoinSubstitution(
        [FindPackageShare("spot_description"), "urdf", "spot_with_arm.urdf"]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("spot_description"), "config", "display.rviz"]
    )
    
    # Nó para publicar o estado da junta
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher',
        output='screen',
    )

    # Nó para publicar a descrição do robô
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        [{
                "robot_description": ParameterValue(Command(["xacro ", urdf_file]), value_type=str)
            }],
    )

    # Nó do RViz com MoveIt
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=["-d", rviz_config_file]
    )

    return LaunchDescription([
        joint_state_publisher,
        robot_state_publisher,
        rviz_node
    ])
