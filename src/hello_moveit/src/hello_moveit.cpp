#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <rclcpp/rclcpp.hpp>
#include <thread>

class MoveItNode : public rclcpp::Node {
public:
    MoveItNode() : Node("hello_moveit") {
        // O construtor agora está vazio, evitando chamadas a shared_from_this()
    }

    void initialize(std::shared_ptr<MoveItNode> node_shared) {
        // Inicializa a interface do MoveIt para o grupo "spot_arm"
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            node_shared, "spot_arm");

        // Inicializa a interface da cena de planejamento
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

        // Inicializa o monitor da cena de planejamento
        planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
            node_shared, "robot_description");

        // Inicia os monitores da cena e do estado do robô
        planning_scene_monitor_->startSceneMonitor();
        planning_scene_monitor_->startStateMonitor();
        planning_scene_monitor_->startWorldGeometryMonitor();

        // Inicia a monitoração do estado do robô no MoveGroup
        move_group_->startStateMonitor();

        // Aguarda a sincronização do estado do robô
        rclcpp::sleep_for(std::chrono::milliseconds(500));

        // Verifica se o estado do robô foi atualizado
        auto robot_state = move_group_->getCurrentState(10.0);
        if (!robot_state) {
            RCLCPP_ERROR(this->get_logger(), "Falha ao obter o estado atual do robô após a inicialização.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Estado do robô sincronizado com sucesso!");
        }
    }


    bool isTargetReachable(const geometry_msgs::msg::Pose& target_pose) {
        auto robot_state = move_group_->getCurrentState(10.0);
        if (!robot_state) {
            RCLCPP_WARN(this->get_logger(), "Não foi possível obter o estado atual do robô!");
            return false;
        }
        
        const moveit::core::JointModelGroup* joint_model_group = move_group_->getRobotModel()->getJointModelGroup("spot_arm");
        bool ik_success = robot_state->setFromIK(joint_model_group, target_pose);

        if (!ik_success) {
            RCLCPP_ERROR(this->get_logger(), "O alvo está fora do espaço de trabalho!");
            return false;
        }
        return true;
    }

    bool isColliding(const geometry_msgs::msg::Pose& target_pose) {
        auto robot_state = move_group_->getCurrentState(10.0);
        if (!robot_state) return true;

        const moveit::core::JointModelGroup* joint_model_group = move_group_->getRobotModel()->getJointModelGroup("spot_arm");
        robot_state->setFromIK(joint_model_group, target_pose);

        collision_detection::CollisionRequest collision_request;
        collision_detection::CollisionResult collision_result;

        if (!planning_scene_monitor_) {
            RCLCPP_ERROR(this->get_logger(), "Falha ao acessar o monitor da cena de planejamento!");
            return true;
        }

        planning_scene_monitor_->requestPlanningSceneState();
        auto planning_scene = planning_scene_monitor_->getPlanningScene();
        planning_scene->checkCollision(collision_request, collision_result, *robot_state);

        if (collision_result.collision) {
            RCLCPP_ERROR(this->get_logger(), "O alvo causaria colisão com o próprio robô!");
            return true;
        }
        return false;
    }

    bool isObstacleCollision(const geometry_msgs::msg::Pose& target_pose) {
        double obstacle_x_min = 1.0, obstacle_x_max = 3.0;
        if (target_pose.position.x >= obstacle_x_min && target_pose.position.x <= obstacle_x_max) {
            RCLCPP_WARN(this->get_logger(), "O alvo está dentro da zona de colisão com um obstáculo externo! Movendo até a última posição segura.");
            return true;
        }
        return false;
    }

    void moveToPose(const geometry_msgs::msg::Pose& target_pose) {
        if (!isTargetReachable(target_pose)) {
            RCLCPP_ERROR(this->get_logger(), "Movimento cancelado: Ponto inalcançável!");
            return;
        }

        if (isColliding(target_pose)) {
            RCLCPP_ERROR(this->get_logger(), "Movimento cancelado: Colisão detectada com o próprio robô!");
            return;
        }

        geometry_msgs::msg::Pose adjusted_pose = target_pose;
        if (isObstacleCollision(target_pose)) {
            adjusted_pose.position.x = (target_pose.position.x < 1.0) ? target_pose.position.x : 1.0;
            RCLCPP_INFO(this->get_logger(), "Movendo apenas até a última posição segura (x=%.2f)...", adjusted_pose.position.x);
        }
        
        move_group_->setPoseTarget(adjusted_pose);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            move_group_->execute(plan);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Falha no planejamento!");
        }
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // Criamos um shared_ptr antes de chamar initialize()
    auto node = std::make_shared<MoveItNode>();
    node->initialize(node);

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 1.5;
    target_pose.position.y = 0.4;
    target_pose.position.z = 0.4;
    target_pose.orientation.w = 1.0;
    
    node->moveToPose(target_pose);

    rclcpp::shutdown();
    return 0;
}
