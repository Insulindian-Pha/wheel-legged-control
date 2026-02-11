cmake_minimum_required(VERSION 3.8)
project(lqr_controller)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(vmc_controller REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LQRState.msg"
  DEPENDENCIES std_msgs
)

# Build the controller library
add_library(lqr_controller_lib SHARED
    src/lqr_controller.cpp)

set_target_properties(lqr_controller_lib PROPERTIES OUTPUT_NAME lqr_controller)

target_include_directories(lqr_controller_lib
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(
        lqr_controller_lib PUBLIC
        rclcpp
        std_msgs
        sensor_msgs
        controller_interface
        hardware_interface
        pluginlib
        rclcpp_lifecycle
        vmc_controller
)

# Link message typesupport
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(lqr_controller_lib PUBLIC ${cpp_typesupport_target})

# Ensure messages are generated before building the library
add_dependencies(lqr_controller_lib ${PROJECT_NAME}__cpp)

# Plugin export
pluginlib_export_plugin_description_file(controller_interface lqr_controller_plugin.xml)

install(TARGETS
        lqr_controller_lib
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(FILES lqr_controller_plugin.xml
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h")

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

ament_export_dependencies(rclcpp std_msgs sensor_msgs controller_interface hardware_interface pluginlib rclcpp_lifecycle vmc_controller rosidl_default_runtime)
ament_package()

