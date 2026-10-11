cmake_minimum_required(VERSION 3.5)
project(carrierbot_guidance_test)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Bản thử nghiệm của fablab_guidance_node (tên khác để không trùng bản gốc)
add_executable(guidance_test_node src/guidance_test_node.cpp)
ament_target_dependencies(guidance_test_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
)

install(
  TARGETS guidance_test_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/odom_to_pose.py
    scripts/send_route.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
