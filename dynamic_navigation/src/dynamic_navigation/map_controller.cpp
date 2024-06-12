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

#include "dynamic_navigation/map_controller.hpp"

namespace dynamic_navigation
{

MapController::MapController()
: Node("map_controller"),
  shortTerm_map(),
  longTerm_map(),
  effective_map(),
  map_ready(false),
  longterm_cost_dec(1)
{
  shortTermMap_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap_auto", 5,
    std::bind(&MapController::shortTermMapCallback, this, std::placeholders::_1));

  longTermMap_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/long_map", 5);
  effectiveMap_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 5);   // Effective_map

  this->declare_parameter("static_map_srv_name", "map");
  this->declare_parameter("longterm_map_srv_name", "long_map");
  this->declare_parameter("longterm_cost_dec", 1);

  this->get_parameter("static_map_srv_name", static_map_srv_name);
  this->get_parameter("longterm_map_srv_name", longterm_map_srv_name);
  this->get_parameter("longterm_cost_dec", longterm_cost_dec);

  static_map = std::make_shared<nav_msgs::msg::OccupancyGrid>(getMap(static_map_srv_name));
  longTerm_map = std::make_shared<nav_msgs::msg::OccupancyGrid>(getMap(longterm_map_srv_name));

  effective_map->info = longTerm_map->info;
  effective_map->info.width = longTerm_map->info.width;
  effective_map->info.height = longTerm_map->info.height;
  effective_map->info.origin = longTerm_map->info.origin;
  effective_map->data.resize(longTerm_map->info.width * longTerm_map->info.height);
}

void MapController::shortTermMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr short_map)
{
  shortTerm_map = short_map;
  map_ready = true;
}

nav_msgs::msg::OccupancyGrid MapController::getMap(const std::string srv_name)
{
  auto client = this->create_client<nav_msgs::srv::GetMap>(srv_name);
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service");
      return nav_msgs::msg::OccupancyGrid();
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }

  auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
  auto result = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "Service call failed");
    return nav_msgs::msg::OccupancyGrid();
  }

  return result.get()->map;
}

bool MapController::cleanMapCallback(
  const std::shared_ptr<dynamic_navigation_interfaces::srv::CleanMap::Request> request,
  std::shared_ptr<dynamic_navigation_interfaces::srv::CleanMap::Response> response)
{
  (void)request;
  (void)response;

  RCLCPP_INFO(this->get_logger(), "Cleaning map");
  longTerm_map = std::make_shared<nav_msgs::msg::OccupancyGrid>(getMap(longterm_map_srv_name));
  publishAll();
  return true;
}

void MapController::updateLongTermMap(const nav_msgs::msg::OccupancyGrid::SharedPtr s_map)
{
  for (size_t i = 0; i < s_map->data.size(); ++i) {
    if (s_map->data[i] > 95) {
      longTerm_map->data[i] = s_map->data[i];
    } else if (s_map->data[i] < 5 && s_map->data[i] >= 0 &&
      longTerm_map->data[i] >= longterm_cost_dec &&
      static_map->data[i] < 95)
    {
      longTerm_map->data[i] -= longterm_cost_dec;
    }
    if (longTerm_map->data[i] < 0) {
      longTerm_map->data[i] = 0;
    }
  }
}

void MapController::buildEffectiveMap(
  const nav_msgs::msg::OccupancyGrid::SharedPtr s_map,
  const nav_msgs::msg::OccupancyGrid::SharedPtr l_map)
{
  for (size_t i = 0; i < s_map->data.size(); ++i) {
    effective_map->data[i] = std::max(static_map->data[i], l_map->data[i]);
  }
}

void MapController::publishAll()
{
  effective_map->header.stamp = this->now();
  effectiveMap_pub->publish(*effective_map);
  longTerm_map->header.stamp = this->now();
  longTermMap_pub->publish(*longTerm_map);
}

void MapController::step()
{
  if (map_ready) {
    buildEffectiveMap(shortTerm_map, longTerm_map);
    updateLongTermMap(shortTerm_map);
    publishAll();
  }
}

}  // namespace dynamic_navigation
