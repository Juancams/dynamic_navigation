// Copyright 2024 Juan Carlos Manzanares Serrano
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DYNAMIC_NAVIGATION__OBSTACLE_DETECTOR_HPP_
#define DYNAMIC_NAVIGATION__OBSTACLE_DETECTOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <memory>
#include <cmath>
#include <iostream>

#include "std_msgs/msg/string.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/transform_listener.h>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/costmap_2d_publisher.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include "nav_msgs/msg/map_meta_data.hpp"

#include <tf2_ros/message_filter.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <message_filters/subscriber.h>
#include <list>

#include "dynamic_navigation_interfaces/srv/metadata.hpp"

namespace dynamic_navigation
{

class ObstacleDetector : public rclcpp_lifecycle::LifecycleNode
{
public:
  ObstacleDetector();
  ~ObstacleDetector();

  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_in);
  void posCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pos);
  void step();

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pos_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
  nav2_costmap_2d::Costmap2D cost_map_;
  float posRobot_x_, posRobot_y_, posRobot_w_, max_lenght_, min_lenght_;
  int cost_inc_, cost_dec_;
  bool pos_ready_, scan_ready_;
  unsigned int cells_size_x_, cells_size_y_;
  double resolution_, origin_x_, origin_y_;
  unsigned char default_value_;
  nav2_costmap_2d::Costmap2DPublisher cost_map_publisher_;

  std::string baseFrameId_, laser_topic_, srv_name_;
  rclcpp::Time last_exec_inc_, last_exec_dec_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  nav_msgs::msg::MapMetaData metadata_;
  std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> tfScanSub_;
  message_filters::Subscriber<sensor_msgs::msg::LaserScan> scanSub_;
  std::list<geometry_msgs::msg::TransformStamped> point_list_;
  std::list<std::vector<unsigned int>> p_vector_list_;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread thread_;

  nav_msgs::msg::MapMetaData getMetadata();
  int getQuadrant(tf2::Transform p, tf2::Transform p_robot);
  void decrementCost(unsigned int cells_x, unsigned int cells_y);
  void setDecrementCost1Q(
    unsigned int point_A_x, unsigned int point_A_y, unsigned int point_B_x,
    unsigned int point_B_y, int mod_x, int mod_y);
  void setDecrementCost2Q(
    unsigned int point_A_x, unsigned int point_A_y, unsigned int point_B_x,
    unsigned int point_B_y, int mod_x, int mod_y);
  void setDecrementCost3Q(
    unsigned int point_A_x, unsigned int point_A_y, unsigned int point_B_x,
    unsigned int point_B_y, int mod_x, int mod_y);
  void setDecrementCost4Q(
    unsigned int point_A_x, unsigned int point_A_y, unsigned int point_B_x,
    unsigned int point_B_y, int mod_x, int mod_y);
  void cleanCostMap(int quadrant, tf2::Transform p, tf2::Transform p_robot);
  unsigned int distance2cell(float n);
  bool cellsOK(unsigned int x, unsigned int y);
  void incrementCostProcedure();
  void decrementCostProcedure();
  void updateCostmap();
};

}  // namespace dynamic_navigation

#endif // DYNAMIC_NAVIGATION__OBSTACLE_DETECTOR_HPP_
