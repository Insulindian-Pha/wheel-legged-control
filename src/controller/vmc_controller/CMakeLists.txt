cmake_minimum_required(VERSION 3.8)
project(vmc_controller)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_input_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pid_controller REQUIRED)
find_package(rosidl_default_generators REQUIRED)
list(APPEND CMAKE_PREFIX_PATH "/opt/ros/humble/lib/x86_64-linux-gnu/cmake/pinocchio")
find_package(pinocchio REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VMCState.msg"
  DEPENDENCIES std_msgs
)

# Build the controller library
# NOTE: the target name must NOT collide with the rosidl_generate_interfaces() custom target,
# which uses ${PROJECT_NAME} as its target name.
add_library(vmc_controller_lib SHARED
    src/vmc_controller.cpp 
    src/vmc_calc.cpp)

set_target_properties(vmc_controller_lib PROPERTIES OUTPUT_NAME vmc_controller)

target_include_directories(vmc_controller_lib
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(
        vmc_controller_lib PUBLIC
        rclcpp
        sensor_msgs
        std_msgs
        control_input_msgs
        controller_interface
        hardware_interface
        pluginlib
        rclcpp_lifecycle
        pid_controller
)

target_link_libraries(vmc_controller_lib PUBLIC pinocchio::pinocchio)
message(STATUS "Pinocchio backend enabled (required).")

# Link message typesupport
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(vmc_controller_lib PUBLIC ${cpp_typesupport_target})

# Ensure messages are generated before building the library
add_dependencies(vmc_controller_lib ${PROJECT_NAME}__cpp)

# Plugin export
pluginlib_export_plugin_description_file(controller_interface vmc_controller_plugin.xml)

install(TARGETS
        vmc_controller_lib
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(FILES vmc_controller_plugin.xml
        DESTINATION share/${PROJECT_NAME})

# Install message files
install(DIRECTORY msg/
        DESTINATION share/${PROJECT_NAME}/msg
        FILES_MATCHING PATTERN "*.msg")

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_export_dependencies(rclcpp sensor_msgs std_msgs control_input_msgs controller_interface hardware_interface pluginlib rclcpp_lifecycle pid_controller rosidl_default_runtime)
ament_package()

