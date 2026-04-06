/*
 * Documentation FR: src/muto_utils/src/diagnostics_node.cpp
 * Role: composant C++ du pipeline MUTO RS avec exigences de latence et surete.
 * Details: ce fichier implemente des fonctions critiques pour la perception, le controle ou le hardware.
 * Contraintes: eviter les regressions de timing, conserver les unites physiques et la coherence des interfaces.
 * Maintenance: documenter toute hypothese metier (seuils, conversions, heuristiques) lors des modifications.
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

class DiagnosticsNode : public rclcpp::Node {
public:
  DiagnosticsNode() : Node("diagnostics_node") {
    pub_ = create_publisher<std_msgs::msg::Float32>("/system_health_score", 10);
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      std_msgs::msg::Float32 msg;
      msg.data = 1.0F;
      pub_->publish(msg);
    });
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DiagnosticsNode>());
  rclcpp::shutdown();
  return 0;
}
