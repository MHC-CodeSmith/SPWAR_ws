cmake_minimum_required(VERSION 3.8)
project(spot_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Encontrar pacotes necessários
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(rviz_visual_tools REQUIRED)

add_executable(spot_teleop src/spot_teleop.cpp)
add_executable(spot_teleop src/simple_moveto.cpp)

target_include_directories(spot_teleop PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(spot_teleop PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  spot_teleop
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_servo
  rviz_visual_tools
  rclcpp
)

install(TARGETS spot_teleop
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
