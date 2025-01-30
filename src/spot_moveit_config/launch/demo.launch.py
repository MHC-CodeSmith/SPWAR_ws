from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
def generate_launch_description():
        moveit_config = (
            MoveItConfigsBuilder("spot", package_name="spot_moveit_config")
            .robot_description(file_path="config/spot.urdf.xacro")
            .robot_description_semantic(file_path="config/spot.srdf")
            .robot_description_kinematics(file_path="config/kinematics.yaml")
            .trajectory_execution(file_path="config/moveit_controllers.yaml")
            .planning_pipelines(default_planning_pipeline="ompl")
            .joint_limits(file_path="config/joint_limits.yaml")  # Adiciona os limites das juntas
            .to_moveit_configs()
        )

        move_group_demo = Node(
            package="hello_moveit",
            executable="hello_moveit",
            output="screen",
            parameters=[
                moveit_config.to_dict()  # Garante que os parâmetros sejam passados corretamente
            ],
        )

        return LaunchDescription([
            generate_demo_launch(moveit_config),
            move_group_demo,
        ])
