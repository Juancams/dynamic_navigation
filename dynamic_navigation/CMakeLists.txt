cmake_minimum_required(VERSION 3.8)
project(dynamic_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(dynamic_navigation_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  nav_msgs
  nav2_msgs
  nav2_map_server
  yaml_cpp_vendor
  std_msgs
  nav2_util
  dynamic_navigation_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_costmap_2d
  geometry_msgs
  sensor_msgs)

include_directories(include)

add_library(map_server_library SHARED
  src/dynamic_navigation/map_saver.cpp)

add_library(obstacle_detector_library SHARED
  src/dynamic_navigation/obstacle_detector.cpp)

  add_library(map_controller_library SHARED
  src/dynamic_navigation/map_controller.cpp)

ament_target_dependencies(map_server_library ${dependencies})
ament_target_dependencies(obstacle_detector_library ${dependencies})
ament_target_dependencies(map_controller_library ${dependencies})

add_executable(map_saver_node src/map_saver_node.cpp)
ament_target_dependencies(map_saver_node ${dependencies})
target_link_libraries(map_saver_node map_server_library)

add_executable(obstacle_detector_node src/obstacle_detector_node.cpp)
ament_target_dependencies(obstacle_detector_node ${dependencies})
target_link_libraries(obstacle_detector_node obstacle_detector_library)

add_executable(map_controller_node src/map_controller_node.cpp)
ament_target_dependencies(map_controller_node ${dependencies})
target_link_libraries(map_controller_node map_controller_library)
  
install(DIRECTORY include/
  DESTINATION include/)


install(TARGETS
  map_server_library
  obstacle_detector_library
  map_controller_library
  map_saver_node
  obstacle_detector_node
  map_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY launch maps
  DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_export_libraries(map_server_library obtacle_detector_library)
ament_package()
