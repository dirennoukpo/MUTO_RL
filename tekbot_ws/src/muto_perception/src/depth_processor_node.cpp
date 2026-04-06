/*
 * Documentation FR: src/muto_perception/src/depth_processor_node.cpp
 * Role: composant C++ du pipeline MUTO RS avec exigences de latence et surete.
 * Details: ce fichier implemente des fonctions critiques pour la perception, le controle ou le hardware.
 * Contraintes: eviter les regressions de timing, conserver les unites physiques et la coherence des interfaces.
 * Maintenance: documenter toute hypothese metier (seuils, conversions, heuristiques) lors des modifications.
 */

#include "muto_perception/depth_processor_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <pthread.h>
#include <time.h>

#include "geometry_msgs/msg/point.hpp"
#include "pcl/ModelCoefficients.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"

namespace muto_perception {

uint64_t DepthProcessorNode::monotonic_now_ns() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + static_cast<uint64_t>(ts.tv_nsec);
}

DepthProcessorNode::DepthProcessorNode() : Node("depth_processor_node") {
  nominal_rate_hz_ = declare_parameter("rate_hz", 30.0);
  throttled_rate_hz_ = declare_parameter("throttle_rate_hz", 15.0);

  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(0, &cpuset);
  CPU_SET(1, &cpuset);
  (void)pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);

  rclcpp::QoS qos(1);
  qos.best_effort();
  qos.durability_volatile();
  qos.history(rclcpp::HistoryPolicy::KeepLast);
  pc_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/depth/pointcloud", qos);
  obs_pub_ = create_publisher<muto_msgs::msg::ObstacleList>("/depth/obstacles", qos);
  map_pub_ = create_publisher<muto_msgs::msg::HeightMap>("/depth/height_map", qos);

  set_processing_rate(nominal_rate_hz_);
  thermal_timer_ = create_wall_timer(std::chrono::milliseconds(200),
                                     std::bind(&DepthProcessorNode::thermal_monitor, this));
}

void DepthProcessorNode::set_processing_rate(double hz) {
  const auto period_ms = static_cast<int>(std::round(1000.0 / std::max(1.0, hz)));
  process_timer_ = create_wall_timer(std::chrono::milliseconds(period_ms),
                                     std::bind(&DepthProcessorNode::process_depth, this));
}

float DepthProcessorNode::read_gpu_freq_ratio() const {
  std::ifstream f("/sys/devices/gpu.0/devfreq/17000000.gp10b/cur_freq");
  std::ifstream fn("/sys/devices/gpu.0/devfreq/17000000.gp10b/max_freq");
  if (!f.good() || !fn.good()) {
    return 1.0F;
  }
  double cur = 0.0;
  double max = 0.0;
  f >> cur;
  fn >> max;
  if (max <= 0.0) {
    return 1.0F;
  }
  return static_cast<float>(cur / max);
}

void DepthProcessorNode::thermal_monitor() {
  const float ratio = read_gpu_freq_ratio();
  if (ratio < 0.8F) {
    const int c = thermal_throttle_count_.fetch_add(1) + 1;
    if (c >= 25) {
      set_processing_rate(throttled_rate_hz_);
    }
  } else {
    thermal_throttle_count_.store(0);
    set_processing_rate(nominal_rate_hz_);
  }
}

void DepthProcessorNode::process_depth() {
  const uint64_t start_ns = monotonic_now_ns();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->points.resize(30000);
  cloud->width = 30000;
  cloud->height = 1;
  for (int i = 0; i < 30000; ++i) {
    cloud->points[static_cast<size_t>(i)].x = static_cast<float>(i % 200) * 0.005F;
    cloud->points[static_cast<size_t>(i)].y = static_cast<float>((i / 200) % 150) * 0.005F;
    cloud->points[static_cast<size_t>(i)].z = static_cast<float>((i % 30) * 0.002F);
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(0.02F, 0.02F, 0.02F);
  vg.filter(*filtered);

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.02);
  seg.setInputCloud(filtered);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
  seg.segment(*inliers, *coeff);

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(filtered);
  extract.setIndices(inliers);
  extract.setNegative(true);
  pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground(new pcl::PointCloud<pcl::PointXYZ>());
  extract.filter(*no_ground);

  muto_msgs::msg::ObstacleList obstacles;
  obstacles.header.stamp = now();
  obstacles.header.frame_id = "base_link";
  obstacles.obstacles.resize(64);
  size_t obs_count = 0;
  for (const auto &p : no_ground->points) {
    const float d = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    if (d < 0.5F) {
      muto_msgs::msg::Obstacle o;
      o.center.x = p.x;
      o.center.y = p.y;
      o.center.z = p.z;
      o.radius = 0.05F;
      o.height = std::max(0.0F, p.z);
      o.confidence = 0.8F;
      obstacles.obstacles[obs_count++] = o;
      if (obs_count >= 64) {
        break;
      }
    }
  }
  obstacles.obstacles.resize(obs_count);

  muto_msgs::msg::HeightMap map;
  map.header = obstacles.header;
  map.width = 11;
  map.height = 11;
  map.resolution = 0.05F;
  map.data.assign(121, 0.0F);
  for (const auto &p : no_ground->points) {
    const int ix = std::clamp(static_cast<int>(std::round((p.x + 0.25F) / 0.05F)), 0, 10);
    const int iy = std::clamp(static_cast<int>(std::round((p.y + 0.25F) / 0.05F)), 0, 10);
    const int idx = iy * 11 + ix;
    map.data[static_cast<std::size_t>(idx)] = std::max(map.data[static_cast<std::size_t>(idx)], p.z);
  }

  sensor_msgs::msg::PointCloud2 out;
  out.header = obstacles.header;

  pc_pub_->publish(out);
  obs_pub_->publish(obstacles);
  map_pub_->publish(map);

  RCLCPP_DEBUG(get_logger(), "depth mono start=%llu end=%llu",
               static_cast<unsigned long long>(start_ns),
               static_cast<unsigned long long>(monotonic_now_ns()));
}

}  // namespace muto_perception

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<muto_perception::DepthProcessorNode>());
  rclcpp::shutdown();
  return 0;
}
