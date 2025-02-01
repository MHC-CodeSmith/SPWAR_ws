#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <thread>

int main(int argc, char *argv[]) {
    // Inicializa o ROS 2
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        "plan_around_objects",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    auto logger = rclcpp::get_logger("plan_around_objects");

    // Thread separada para processar o estado do robô
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    // Interface do MoveIt para o grupo de juntas do braço robótico
    moveit::planning_interface::MoveGroupInterface move_group(node, "spot_arm");

    // Define o pipeline e o planejador
    move_group.setPlanningPipelineId("ompl");
    move_group.setPlannerId("RRTConnectkConfigDefault");

    // Define tempo máximo de planejamento e escalas de velocidade/aceleração
    move_group.setPlanningTime(1.0);
    move_group.setMaxVelocityScalingFactor(1.0);
    move_group.setMaxAccelerationScalingFactor(1.0);

    // Exibe as configurações de planejamento no terminal
    RCLCPP_INFO(logger, "Planning pipeline: %s", move_group.getPlanningPipelineId().c_str());
    RCLCPP_INFO(logger, "Planner ID: %s", move_group.getPlannerId().c_str());
    RCLCPP_INFO(logger, "Planning time: %.2f", move_group.getPlanningTime());

    // Define a posição alvo do end-effector
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.8593;
    target_pose.position.y = -0.2360;
    target_pose.position.z = 0.2703;
    target_pose.orientation.x = 4.85e-05;
    target_pose.orientation.y = 5.63e-05;
    target_pose.orientation.z = -1.10e-05;
    target_pose.orientation.w = 0.9999;

    move_group.setPoseTarget(target_pose);

    // Planejamento e execução
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        move_group.execute(plan);
        RCLCPP_INFO(logger, "Movimento executado com sucesso!");
    } else {
        RCLCPP_ERROR(logger, "Falha no planejamento do movimento!");
    }

    // Finaliza o ROS 2
    rclcpp::shutdown();
    spinner.join();
    return 0;
}
