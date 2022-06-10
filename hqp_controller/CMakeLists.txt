cmake_minimum_required(VERSION 3.5)
project(hqp_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(forward_command_controller REQUIRED)
find_package(urdfdom REQUIRED)

# find 3rd party dependencies
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(SRCS
  src/hqp_controller.cpp
  src/trajectory.cpp
  src/model.cpp
  src/quadratic_programming.cpp
)
add_library(hqp_controller SHARED
  ${SRCS}
)
target_include_directories(hqp_controller PRIVATE include)
target_link_libraries (hqp_controller
  Eigen3::Eigen
  -lrbdl
  -lrbdl_urdfreader
  -lqpOASES
)
ament_target_dependencies(hqp_controller
  rclcpp
  controller_interface
  std_msgs
  control_msgs
  hardware_interface
  control_toolbox
  pluginlib
  rclcpp_lifecycle
  realtime_tools
  forward_command_controller
  urdfdom
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(hqp_controller PRIVATE "HQP_CONTROLLER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface hqp_controller_plugin.xml)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS hqp_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  rclcpp
  controller_interface
  control_msgs
  std_msgs
  hardware_interface
  control_toolbox
  rclcpp_lifecycle
  forward_command_controller
  urdfdom
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  hqp_controller
)

ament_package()
