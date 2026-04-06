/*
 * Documentation FR: src/muto_perception/include/muto_perception/lidar_processor_node.hpp
 * Role: composant C++ du pipeline MUTO RS avec exigences de latence et surete.
 * Details: ce fichier implemente des fonctions critiques pour la perception, le controle ou le hardware.
 * Contraintes: eviter les regressions de timing, conserver les unites physiques et la coherence des interfaces.
 * Maintenance: documenter toute hypothese metier (seuils, conversions, heuristiques) lors des modifications.
 */

#pragma once

#include "muto_msgs/msg/obstacle_list.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace muto_perception {

class LidarProcessorNode : public rclcpp::Node {
public:
  LidarProcessorNode();

private:
  static uint64_t monotonic_now_ns();

  void process_scan();

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<muto_msgs::msg::ObstacleList>::SharedPtr obs_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace muto_perception
