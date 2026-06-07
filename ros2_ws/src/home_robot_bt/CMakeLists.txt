cmake_minimum_required(VERSION 3.8)
project(home_robot_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(bt_patrol src/bt_patrol.cpp)
target_compile_features(bt_patrol PUBLIC cxx_std_17)
ament_target_dependencies(bt_patrol
  rclcpp
  rclcpp_action
  ament_index_cpp
  behaviortree_cpp
  nav2_msgs
  geometry_msgs
  std_msgs
  yaml_cpp_vendor
)
target_link_libraries(bt_patrol yaml-cpp)

install(TARGETS bt_patrol
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
