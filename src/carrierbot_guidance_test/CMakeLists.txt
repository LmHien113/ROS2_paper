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
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(carrierbot_msgs REQUIRED)

# Bản thử nghiệm của fablab_guidance_node (tên khác để không trùng bản gốc)
add_executable(guidance_test_node src/guidance_test_node.cpp)
ament_target_dependencies(guidance_test_node
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
)

# Ban thu nghiem cua hardware interface (dau tung banh cau hinh duoc).
# Ten lop khac ban goc: carrierbot_guidance_test::CarrierbotInterfaceTest
add_library(carrierbot_interface_test SHARED src/carrierbot_interface_test.cpp)
target_include_directories(carrierbot_interface_test PRIVATE include)
target_compile_definitions(carrierbot_interface_test PRIVATE "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
ament_target_dependencies(carrierbot_interface_test
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  carrierbot_msgs
  std_msgs
)
pluginlib_export_plugin_description_file(hardware_interface carrierbot_interface_test.xml)

install(
  TARGETS carrierbot_interface_test
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS guidance_test_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/odom_to_pose.py
    scripts/send_route.py
    scripts/drive_test.py
    scripts/step_test.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch urdf config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
