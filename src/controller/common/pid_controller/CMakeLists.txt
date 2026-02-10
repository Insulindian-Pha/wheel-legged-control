cmake_minimum_required(VERSION 3.8)
project(pid_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PidState.msg"
  DEPENDENCIES std_msgs
)

# Create library (shared library for proper linking)
# NOTE: the target name must NOT collide with the rosidl_generate_interfaces() custom target,
# which uses ${PROJECT_NAME} as its target name.
add_library(pid_controller_lib SHARED
  src/pid.cpp
  src/pid_ros.cpp
)

set_target_properties(pid_controller_lib PROPERTIES OUTPUT_NAME pid_controller)

target_include_directories(pid_controller_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(pid_controller_lib
  PUBLIC
    rclcpp
    rclcpp_lifecycle
    rcl_interfaces
    std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(pid_controller_lib PUBLIC ${cpp_typesupport_target})

# Install headers
install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp"
)

# Install library
install(TARGETS pid_controller_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.yaml"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(pid_controller_lib)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rclcpp rclcpp_lifecycle rcl_interfaces std_msgs rosidl_default_runtime)
ament_package()

