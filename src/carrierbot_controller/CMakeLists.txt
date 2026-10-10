cmake_minimum_required(VERSION 3.8)
project(carrierbot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(simple_controller src/simple_controller.cpp)

ament_target_dependencies(simple_controller
  rclcpp
  std_msgs
  geometry_msgs
  ${Eigen3_LIBRARIES}
)

add_executable(cmd_vel_relay src/cmd_vel_relay.cpp)

ament_target_dependencies(cmd_vel_relay
  rclcpp
  std_msgs
)

install(
  DIRECTORY include
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS 
  simple_controller
  cmd_vel_relay
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto QUIET)
  if(ament_lint_auto_FOUND)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
  endif()
endif()
ament_package()