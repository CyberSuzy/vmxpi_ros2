cmake_minimum_required(VERSION 3.16)
project(vmxpi_ros2 LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
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

# Find VMXPi library and headers (CONDITIONAL)
find_library(VMXPI_LIBRARY
  NAMES libstudica_drivers.so # Check for libstudica_drivers.so
  PATHS /usr/local/lib
        /usr/local/lib/studica_drivers # Adjust path if needed
  )

find_path(VMXPI_INCLUDE_DIR
  NAMES VMXPi.h # Check for a header file from studica_drivers
  PATHS /usr/local/include
        /usr/local/include/studica_drivers # Adjust path if needed
        /usr/local/include/vmxpi
  )

# Check if VMXPI library and headers were found
if(VMXPI_LIBRARY AND VMXPI_INCLUDE_DIR)
  message(STATUS "VMXPi library and headers FOUND: Using hardware interface.")
  set(VMXPI_FOUND TRUE)
  set(VMXPI_LIBRARIES ${VMXPI_LIBRARY})
  set(VMXPI_INCLUDE_DIRS ${VMXPI_INCLUDE_DIR})
else()
  message(WARNING "VMXPi library and headers NOT FOUND: Building without hardware interface support.")
  set(VMXPI_FOUND FALSE)
  set(VMXPI_LIBRARIES "")
  set(VMXPI_INCLUDE_DIRS "")
endif()

if(VMXPI_FOUND)
  include_directories( # <- #include <"">
    include
    ${VMXPI_INCLUDE_DIRS} # Include VMXPi headers if found
  )
else()
  include_directories(include) # Still include 'include' directory for package's own headers
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(backward_ros REQUIRED)
# For control
find_package(ros2_control REQUIRED)
find_package(gazebo_ros2_control REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(velocity_controllers REQUIRED)
find_package(rclpy REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  vmxpi_ros2
  SHARED
  hardware/rrbot.cpp
  hardware/diffbot_system.cpp
  # Conditionally include hardware-specific source files
  ${${PROJECT_NAME}_HARDWARE_SOURCES}
)

# Conditionally link and compile hardware interface code
if(VMXPI_FOUND)
  target_sources(vmxpi_ros2
    PRIVATE
      hardware/titan_system.cpp # Include titan_system.cpp only if VMXPI_FOUND
  )
  target_link_libraries(vmxpi_ros2
    PUBLIC
    /usr/local/lib/libstudica_drivers.so
    # ${VMXPI_LIBRARIES} # Link against VMXPi library if found
  )
else()
  message(WARNING "Skipping compilation and linking of hardware interface code.")
  # Optionally, you could add a dummy/mock hardware interface source file here
  # target_sources(vmxpi_ros2 PRIVATE src/mock_hardware_interface.cpp)
endif()


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
  DIRECTORY description  
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
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

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