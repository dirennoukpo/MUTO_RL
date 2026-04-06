/*
 * Documentation FR: src/muto_perception/src/lidar_processor_node.cpp
 * Role: composant C++ du pipeline MUTO RS avec exigences de latence et surete.
 * Details: ce fichier implemente des fonctions critiques pour la perception, le controle ou le hardware.
 * Contraintes: eviter les regressions de timing, conserver les unites physiques et la coherence des interfaces.
 * Maintenance: documenter toute hypothese metier (seuils, conversions, heuristiques) lors des modifications.
 */

#include "muto_perception/lidar_processor_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <pthread.h>
#include <time.h>

namespace muto_perception {

uint64_t LidarProcessorNode::monotonic_now_ns() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + static_cast<uint64_t>(ts.tv_nsec);
}

LidarProcessorNode::LidarProcessorNode() : Node("lidar_processor_node") {
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(2, &cpuset);
  (void)pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);

  rclcpp::QoS qos(1);
  qos.best_effort();
  qos.durability_volatile();
  qos.history(rclcpp::HistoryPolicy::KeepLast);
  scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/lidar/scan_filtered", qos);
  obs_pub_ = create_publisher<muto_msgs::msg::ObstacleList>("/lidar/obstacles", qos);
  timer_ = create_wall_timer(std::chrono::milliseconds(100),
                             std::bind(&LidarProcessorNode::process_scan, this));
}

void LidarProcessorNode::process_scan() {
  const uint64_t start_ns = monotonic_now_ns();

  sensor_msgs::msg::LaserScan scan;
  scan.header.stamp = now();
  scan.header.frame_id = "base_link";
  scan.angle_min = -1.57F;
  scan.angle_max = 1.57F;
  scan.angle_increment = 0.01F;
  scan.range_min = 0.05F;
  scan.range_max = 3.0F;

  const int n = static_cast<int>((scan.angle_max - scan.angle_min) / scan.angle_increment);
  scan.ranges.resize(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {
    const float angle = scan.angle_min + static_cast<float>(i) * scan.angle_increment;
    float r = 1.5F + 0.6F * std::sin(angle * 3.0F);
    if (r < scan.range_min || r > scan.range_max) {
      r = scan.range_max;
    }
    scan.ranges[static_cast<std::size_t>(i)] = r;
  }

  muto_msgs::msg::ObstacleList obstacles;
  obstacles.header = scan.header;
  obstacles.obstacles.resize(64);
  size_t obs_count = 0;

  bool active_cluster = false;
  float cluster_min = scan.range_max;
  int cluster_start = 0;
  for (int i = 0; i < n; ++i) {
    const float r = scan.ranges[static_cast<std::size_t>(i)];
    const bool in_range = (r > scan.range_min && r < 1.0F);

    if (in_range && !active_cluster) {
      active_cluster = true;
      cluster_min = r;
      cluster_start = i;
    } else if (in_range && active_cluster) {
      cluster_min = std::min(cluster_min, r);
    } else if (!in_range && active_cluster) {
      const float angle = scan.angle_min + static_cast<float>(cluster_start) * scan.angle_increment;
      muto_msgs::msg::Obstacle o;
      o.center.x = cluster_min * std::cos(angle);
      o.center.y = cluster_min * std::sin(angle);
      o.center.z = 0.0;
      o.radius = 0.08F;
      o.height = 0.2F;
      o.confidence = 0.7F;
      if (obs_count < 64) {
        obstacles.obstacles[obs_count++] = o;
      }
      active_cluster = false;
    }
  }
  obstacles.obstacles.resize(obs_count);

  scan_pub_->publish(scan);
  obs_pub_->publish(obstacles);

  RCLCPP_DEBUG(get_logger(), "lidar mono start=%llu end=%llu",
               static_cast<unsigned long long>(start_ns),
               static_cast<unsigned long long>(monotonic_now_ns()));
}

}  // namespace muto_perception

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<muto_perception::LidarProcessorNode>());
  rclcpp::shutdown();
  return 0;
}
