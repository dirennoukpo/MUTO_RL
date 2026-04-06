/*
 * Documentation FR: src/muto_perception/include/muto_perception/depth_processor_node.hpp
 * Role: composant C++ du pipeline MUTO RS avec exigences de latence et surete.
 * Details: ce fichier implemente des fonctions critiques pour la perception, le controle ou le hardware.
 * Contraintes: eviter les regressions de timing, conserver les unites physiques et la coherence des interfaces.
 * Maintenance: documenter toute hypothese metier (seuils, conversions, heuristiques) lors des modifications.
 */

#pragma once

#include <atomic>

#include "muto_msgs/msg/height_map.hpp"
#include "muto_msgs/msg/obstacle_list.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace muto_perception {

class DepthProcessorNode : public rclcpp::Node {
public:
  DepthProcessorNode();

private:
  static uint64_t monotonic_now_ns();

  void process_depth();
  void thermal_monitor();
  void set_processing_rate(double hz);
  float read_gpu_freq_ratio() const;

  std::atomic<int> thermal_throttle_count_{0};
  double nominal_rate_hz_{30.0};
  double throttled_rate_hz_{15.0};

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
  rclcpp::Publisher<muto_msgs::msg::ObstacleList>::SharedPtr obs_pub_;
  rclcpp::Publisher<muto_msgs::msg::HeightMap>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr process_timer_;
  rclcpp::TimerBase::SharedPtr thermal_timer_;
};

}  // namespace muto_perception
