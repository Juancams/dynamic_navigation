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

#include "dynamic_navigation/obstacle_detector.hpp"

namespace dynamic_navigation

{
	
	ObstacleDetector::ObstacleDetector()
	: LifecycleNode("obstacle_detector"),
	  cost_map_(),
    cost_map_publisher_(shared_from_this(), &cost_map_, "map", "costmap_auto", true),
    baseFrameId_("base_footprint"),
    laser_topic_("/scan"),
    srv_name_("MapMetadata"),
    last_exec_inc_(rclcpp::Clock().now()),
    last_exec_dec_(rclcpp::Clock().now()),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
    tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_)) 
  {	
    // scanSub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(this->get_node_base_interface(), laser_topic_, 5);

    pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/amcl_pose", 5, std::bind(&ObstacleDetector::posCallback, this, std::placeholders::_1));

    // tfScanSub_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
    //     *scanSub_, tf_listener_, baseFrameId_, 5, this->get_node_logging_interface(), this->get_node_clock_interface());

    // tfScanSub_->registerCallback(std::bind(&ObstacleDetector::laserCallback, this, std::placeholders::_1));
  

		metadata_ = getMetadata();

		cells_size_x_ = metadata_.width;
		cells_size_y_ = metadata_.height;
		resolution_ = metadata_.resolution;
		origin_x_ = metadata_.origin.position.x;
		origin_y_ = metadata_.origin.position.y;
		default_value_ = 255; /*30*/
		scan_ready_ = false;
		pos_ready_ = false;
		cost_map_.resizeMap(cells_size_x_, cells_size_y_, resolution_, origin_x_, origin_y_);
		cost_map_.setDefaultValue(default_value_);
		cost_map_.resetMap(0, 0, cost_map_.getSizeInCellsX(), cost_map_.getSizeInCellsY());

		this->declare_parameter("cost_inc", cost_inc_);
		this->declare_parameter("cost_dec", cost_dec_);
		this->declare_parameter("min_lenght", min_lenght_);
		this->declare_parameter("max_lenght", max_lenght_);
	}

	void ObstacleDetector::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_in) {
		float o_t_min, o_t_max, o_t_inc;

		float x, y, z;
		float ax, ay, az;
		tf2::Transform scan_sensor, point_in_cells_;
    geometry_msgs::msg::PointStamped scan_sensor_point, point_in_cells_point;

		tf2::Transform bf2obj, map2obj;
		tf2::Quaternion q;
		geometry_msgs::msg::TransformStamped map2bf;
		geometry_msgs::msg::TransformStamped from_map_to_base_footprint;
    unsigned int cells_x, cells_y;
		std::vector<unsigned int> object_point(2);

		o_t_min = scan_in->angle_min;
		o_t_max = scan_in->angle_max;
		o_t_inc = scan_in->angle_increment;

		int num_points = (int)2.0 * o_t_max / o_t_inc;
		if (tf_buffer_->canTransform("map", "base_footprint", tf2::TimePointZero)) {
			map2bf = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
			for (int i = 0; i < num_points; ++i) {
				float theta = o_t_min + i * o_t_inc;
				float r = scan_in->ranges[i];
				if (r > max_lenght_) {
					r = max_lenght_;
				}
				if (r > min_lenght_ && !std::isnan(r)) {
					scan_sensor.setRotation(tf2::Quaternion(r*cos(theta), r*sin(theta), 0.0, 1.0));
          scan_sensor_point.point.x = r*cos(theta);
          scan_sensor_point.point.y = r*sin(theta);
          scan_sensor_point.point.z = 0.0;
          scan_sensor_point.header = scan_in->header;
          geometry_msgs::msg::TransformStamped scan2bf = tf_buffer_->lookupTransform(scan_in->header.frame_id, 
                                                                                     baseFrameId_, 
                                                                                     tf2::TimePointZero);
                                                                                  
					tf2::doTransform(scan_sensor_point, point_in_cells_point, scan2bf);

					x = point_in_cells_point.point.x;
					y = point_in_cells_point.point.y;
					z = 0.0;
					ax = 0.0;
					ay = 0.0;
					az = 0.0;
					bf2obj.setOrigin(tf2::Vector3(x, y, z));
					q.setRPY(ax, ay, az);
					bf2obj.setRotation(q);
          geometry_msgs::msg::TransformStamped tf_stamped;
          tf2::convert(bf2obj, tf_stamped.transform);

					point_list_.push_back(tf_stamped);
					if (r != max_lenght_) {
            tf2::Transform map2obj_transform(tf2::Quaternion(map2bf.transform.rotation.x, 
                                                              map2bf.transform.rotation.y, 
                                                              map2bf.transform.rotation.z, 
                                                              map2bf.transform.rotation.w),
                                              tf2::Vector3(map2bf.transform.translation.x, 
                                                           map2bf.transform.translation.y, 
                                                           map2bf.transform.translation.z));
            map2obj = map2obj_transform * bf2obj;

						// tf2::doTransform(bf2obj, map2obj, map2bf);
						cells_x = distance2cell(map2obj.getOrigin().x());
						cells_y = distance2cell(map2obj.getOrigin().y());
						object_point[0] = cells_x;
						object_point[1] = cells_y;
						p_vector_list_.push_back(object_point);
					}
				}
			}
			scan_ready_ = true;
		}
	}

	void ObstacleDetector::posCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pos) {
		posRobot_x_ = pos->pose.pose.position.x;
		posRobot_y_ = pos->pose.pose.position.y; //Pos del robot respecto al origen
		double yaw = tf2::getYaw(pos->pose.pose.orientation) - M_PI;
		float degrees = yaw * 180 / M_PI;
		posRobot_w_ = 360 + degrees;
		pos_ready_ = true;
	}

	void ObstacleDetector::step() {
		updateCostmap();
		cost_map_publisher_.publishCostmap();
	}

	nav_msgs::msg::MapMetaData ObstacleDetector::getMetadata() {
		auto client = this->create_client<dynamic_navigation_interfaces::srv::Metadata>(srv_name_);
		auto request = std::make_shared<dynamic_navigation_interfaces::srv::Metadata::Request>();
		while (!client->wait_for_service(std::chrono::seconds(1))) {
			if (!rclcpp::ok()) {
				RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
				return nav_msgs::msg::MapMetaData();
			}
			RCLCPP_INFO(this->get_logger(), "Waiting for service to appear...");
		}
		auto result = client->async_send_request(request);
		if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS) {
			RCLCPP_INFO(this->get_logger(), "GetMetadata Success");
			return result.get()->metadata_res;
		} else {
			RCLCPP_ERROR(this->get_logger(), "Failed to call service Metadata");
			return nav_msgs::msg::MapMetaData();
		}
	}

	int ObstacleDetector::getQuadrant(tf2::Transform p, tf2::Transform p_robot) {
		float p_x, p_y, p_robot_x, p_robot_y;
		p_x = p.getOrigin().x();
		p_y = p.getOrigin().y();

		p_robot_x = p_robot.getOrigin().x();
		p_robot_y = p_robot.getOrigin().y();

		if (p_x == p_robot_x && p_y == p_robot_y) {
			return 0;
		} else if (p_x >= p_robot_x && p_y >= p_robot_y) {
			return 1;
		} else if (p_x <= p_robot_x && p_y >= p_robot_y) {
			return 2;
		} else if (p_x <= p_robot_x && p_y <= p_robot_y) {
			return 3;
		} else {
			return 4;
		}
	}

	void ObstacleDetector::decrementCost(unsigned int cells_x, unsigned int cells_y) {
		unsigned char cost_now;
		if (cellsOK(cells_x, cells_y)) {
			cost_now = cost_map_.getCost(cells_x, cells_y);
			if (cost_now == 255) {
				cost_map_.setCost(cells_x, cells_y, 0);
			} else if (cost_now > cost_dec_ - 1) {
				cost_map_.setCost(cells_x, cells_y, cost_now - cost_dec_);
			} else {
				cost_map_.setCost(cells_x, cells_y, 0);
			}
		}
	}

	void ObstacleDetector::setDecrementCost1Q(unsigned int point_A_x, unsigned int point_A_y, unsigned int point_B_x, unsigned int point_B_y, int mod_x, int mod_y) {
		while (point_A_x > point_B_x || point_A_y > point_B_y) {
			if (point_A_x > point_B_x) {
				point_A_x = point_A_x - mod_x;
			}
			if (point_A_y > point_B_y) {
				point_A_y = point_A_y - mod_y;
			}
			decrementCost(point_A_x, point_A_y);
		}
	}

	void ObstacleDetector::setDecrementCost2Q(unsigned int point_A_x, unsigned int point_A_y, unsigned int point_B_x, unsigned int point_B_y, int mod_x, int mod_y) {
		while (point_A_x < point_B_x || point_A_y > point_B_y) {
			if (point_A_x < point_B_x) {
				point_A_x = point_A_x + mod_x;
			}
			if (point_A_y > point_B_y) {
				point_A_y = point_A_y - mod_y;
			}
			decrementCost(point_A_x, point_A_y);
		}
	}

	void ObstacleDetector::setDecrementCost3Q(unsigned int point_A_x, unsigned int point_A_y, unsigned int point_B_x, unsigned int point_B_y, int mod_x, int mod_y) {
		while (point_A_x < point_B_x || point_A_y < point_B_y) {
			if (point_A_x < point_B_x) {
				point_A_x = point_A_x + mod_x;
			}
			if (point_A_y < point_B_y) {
				point_A_y = point_A_y + mod_y;
			}
			decrementCost(point_A_x, point_A_y);
		}
	}

	void ObstacleDetector::setDecrementCost4Q(unsigned int point_A_x, unsigned int point_A_y, unsigned int point_B_x, unsigned int point_B_y, int mod_x, int mod_y) {
		while (point_A_x > point_B_x || point_A_y < point_B_y) {
			if (point_A_x > point_B_x) {
				point_A_x = point_A_x - mod_x;
			}
			if (point_A_y < point_B_y) {
				point_A_y = point_A_y + mod_y;
			}
			decrementCost(point_A_x, point_A_y);
		}
	}

	void ObstacleDetector::cleanCostMap(int quadrant, tf2::Transform p, tf2::Transform p_robot) {
		unsigned int p_cell_x, p_cell_y, p_robot_cell_x, p_robot_cell_y;
		p_cell_x = distance2cell(p.getOrigin().x());
		p_cell_y = distance2cell(p.getOrigin().y());
		p_robot_cell_x = distance2cell(p_robot.getOrigin().x());
		p_robot_cell_y = distance2cell(p_robot.getOrigin().y());
		if (!cellsOK(p_cell_x, p_cell_y)) {
			return;
		}
		switch (quadrant) {
		case 1:
			setDecrementCost1Q(p_cell_x, p_cell_y, p_robot_cell_x, p_robot_cell_y, 1, 1);
			break;
		case 2:
			setDecrementCost2Q(p_cell_x, p_cell_y, p_robot_cell_x, p_robot_cell_y, 1, 1);
			break;
		case 3:
			setDecrementCost3Q(p_cell_x, p_cell_y, p_robot_cell_x, p_robot_cell_y, 1, 1);
			break;
		case 4:
			setDecrementCost4Q(p_cell_x, p_cell_y, p_robot_cell_x, p_robot_cell_y, 1, 1);
			break;
		default:
			break;
		}
	}

	unsigned int ObstacleDetector::distance2cell(float n) {
		return round(n / resolution_);
	}

	bool ObstacleDetector::cellsOK(unsigned int x, unsigned int y) {
		return (x <= cells_size_x_ && y <= cells_size_y_);
	}

	void ObstacleDetector::incrementCostProcedure() {
		tf2::Stamped<tf2::Transform> map2bf;
		tf2::Transform map2obj, point;
		unsigned int cells_x, cells_y;
		int cost_now;
		if (rclcpp::Clock().now() > last_exec_inc_ + rclcpp::Duration(std::chrono::seconds(1))) {
			for (auto &p : p_vector_list_) {
				cells_x = p[0];
				cells_y = p[1];
				if (cellsOK(cells_x, cells_y)) {
					cost_now = cost_map_.getCost(cells_x, cells_y);
					if (cost_now == 255) {
						cost_map_.setCost(cells_x, cells_y, 0);
					} else if (cost_now < 254 - cost_inc_) {
						cost_map_.setCost(cells_x, cells_y, cost_now + cost_inc_);
					} else {
						cost_map_.setCost(cells_x, cells_y, 254);
					}
				}
			}
			last_exec_inc_ = rclcpp::Clock().now();
		}
	}

	void ObstacleDetector::decrementCostProcedure() {
		geometry_msgs::msg::TransformStamped map2bf;
		tf2::Transform point, p_robot, map2obj;
		int quadrant;
    p_robot.setOrigin(tf2::Vector3(posRobot_x_, posRobot_y_, 0));

		map2bf = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
		if (rclcpp::Clock().now() > last_exec_dec_ + rclcpp::Duration(std::chrono::seconds(1))) {
			for (auto &point : point_list_) {
        tf2::Transform map2bf_trans(tf2::Quaternion(map2bf.transform.rotation.x, 
                                                    map2bf.transform.rotation.y, 
                                                    map2bf.transform.rotation.z, 
                                                    map2bf.transform.rotation.w),
                                    tf2::Vector3(map2bf.transform.translation.x, 
                                                 map2bf.transform.translation.y, 
                                                 map2bf.transform.translation.z));
        tf2::Transform point_temp(tf2::Quaternion(point.transform.rotation.x, 
                                                 point.transform.rotation.y, 
                                                 point.transform.rotation.z, 
                                                 point.transform.rotation.w),
                                 tf2::Vector3(point.transform.translation.x, 
                                              point.transform.translation.y, 
                                              point.transform.translation.z));
        map2obj = map2bf_trans * point_temp;
				quadrant = getQuadrant(map2obj, p_robot);
				cleanCostMap(quadrant, map2obj, p_robot);
			}
			last_exec_dec_ = rclcpp::Clock().now();
		}
	}

	void ObstacleDetector::updateCostmap() {
		if (scan_ready_ && pos_ready_) {
			incrementCostProcedure();
			decrementCostProcedure();
		}
		p_vector_list_.clear();
		point_list_.clear();
	}

} // namespace dynamic_navigation