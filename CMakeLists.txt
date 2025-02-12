cmake_minimum_required(VERSION 3.8)
project(handrobot_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(
  handrobot_ros2_control
  SHARED
  hardware/src/handRobot.cpp
)
target_compile_features(handrobot_ros2_control PUBLIC cxx_std_17)
target_include_directories(handrobot_ros2_control PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/handrobot_ros2_control>
)
ament_target_dependencies(
  handrobot_ros2_control PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(hardware_interface handrobot_ros2_control.xml)

install(
  DIRECTORY hardware/include/
  DESTINATION include/handrobot_ros2_control
)

install(
  DIRECTORY description/launch description/ros2_control description/urdf description/meshes description/rviz
  DESTINATION share/handrobot_ros2_control
)

install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/handrobot_ros2_control
)

install(TARGETS handrobot_ros2_control
  EXPORT export_handrobot_ros2_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_handrobot_ros2_control HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})


add_executable(keyboard_controller keyboard_controller/src/keyboard_controller.cpp)
ament_target_dependencies(keyboard_controller rclcpp std_msgs)

install(TARGETS keyboard_controller DESTINATION lib/${PROJECT_NAME})

ament_package()
