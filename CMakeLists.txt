cmake_minimum_required(VERSION 3.16)
project(vmxpi_ros2 LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

include_directories( # <- #include <"">
  include 
  /usr/local/include/vmxpi 
  /usr/local/include/studica_drivers
  # ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)
 

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  vmxpi_ros2
  SHARED
  hardware/rrbot.cpp
  hardware/diffbot_system.cpp
  hardware/titan_system.cpp
)
target_compile_features(vmxpi_ros2 PUBLIC cxx_std_17)
target_include_directories(vmxpi_ros2 PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/vmxpi_ros2>
)
ament_target_dependencies(
  vmxpi_ros2 PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface vmxpi_ros2.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/vmxpi_ros2
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf description/gazebo description/diffbot
  DESTINATION share/vmxpi_ros2
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/vmxpi_ros2
)
install(TARGETS vmxpi_ros2
  EXPORT export_vmxpi_ros2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(example_rrbot_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(view_example_rrbot_launch test/test_view_robot_launch.py)
  ament_add_pytest_test(run_example_rrbot_launch test/test_rrbot_launch.py)

  ament_add_pytest_test(example_diffbot_urdf_xacro test/test_diffbot_urdf_xacro.py)
  ament_add_pytest_test(view_example_diffbot_launch test/test_view_robot_diffbot_launch.py)
  ament_add_pytest_test(run_example_diffbot_launch test/test_diffbot_launch.py)
endif()

## EXPORTS
ament_export_targets(export_vmxpi_ros2 HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
