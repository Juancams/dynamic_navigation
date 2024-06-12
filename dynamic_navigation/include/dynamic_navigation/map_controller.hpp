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

#ifndef DYNAMIC_NAVIGATION__MAP_CONTROLLER_HPP_
#define DYNAMIC_NAVIGATION__MAP_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "dynamic_navigation_interfaces/srv/clean_map.hpp"

namespace dynamic_navigation
{

class MapController : public rclcpp::Node
{
public:
  MapController();

  bool cleanMapCallback(
    const std::shared_ptr<dynamic_navigation_interfaces::srv::CleanMap::Request> request,
    std::shared_ptr<dynamic_navigation_interfaces::srv::CleanMap::Response> response);
  void step();
  nav_msgs::msg::OccupancyGrid getMap(const std::string srv_name);

private:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr shortTermMap_sub;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr longTermMap_pub;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr effectiveMap_pub;
  nav_msgs::msg::OccupancyGrid::SharedPtr static_map, shortTerm_map, longTerm_map, effective_map;
  std::string static_map_srv_name, longterm_map_srv_name;
  bool map_ready;
  int longterm_cost_dec;

  void shortTermMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr short_map);
  void updateLongTermMap(const nav_msgs::msg::OccupancyGrid::SharedPtr e_map);
  void buildEffectiveMap(
    const nav_msgs::msg::OccupancyGrid::SharedPtr s_map,
    const nav_msgs::msg::OccupancyGrid::SharedPtr l_map);
  void publishAll();
};

}  // namespace dynamic_navigation

#endif // DYNAMIC_NAVIGATION__MAP_CONTROLLER_HPP_
