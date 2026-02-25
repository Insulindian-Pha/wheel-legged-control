cmake_minimum_required(VERSION 3.8)
project(rl_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(vmc_controller REQUIRED)
find_package(pid_controller REQUIRED)
find_package(control_input_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate RLState message for debug topic
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RLState.msg"
  DEPENDENCIES std_msgs
)

# ONNX Runtime from third_party
set(ONNXRUNTIME_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/onnxruntime")
find_package(onnxruntime CONFIG REQUIRED
  HINTS "${ONNXRUNTIME_ROOT}/lib/cmake/onnxruntime")

add_library(rl_controller_lib SHARED src/rl_controller.cpp)
set_target_properties(rl_controller_lib PROPERTIES
  OUTPUT_NAME rl_controller
  INSTALL_RPATH "\$ORIGIN"
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH_USE_LINK_PATH FALSE)

target_include_directories(rl_controller_lib
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_include_directories(rl_controller_lib PRIVATE
  "${ONNXRUNTIME_ROOT}/include")

ament_target_dependencies(rl_controller_lib PUBLIC
  rclcpp
  std_msgs
  sensor_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  vmc_controller
  pid_controller
  control_input_msgs
  ament_index_cpp)
target_link_libraries(rl_controller_lib PUBLIC onnxruntime::onnxruntime)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(rl_controller_lib PUBLIC ${cpp_typesupport_target})
add_dependencies(rl_controller_lib ${PROJECT_NAME}__cpp)

pluginlib_export_plugin_description_file(controller_interface rl_controller_plugin.xml)

# Install onnxruntime shared lib so rl_controller.so can load it at runtime (RPATH $ORIGIN)
set(ONNXRUNTIME_LIB_DIR "${ONNXRUNTIME_ROOT}/lib64")
if(NOT EXISTS "${ONNXRUNTIME_LIB_DIR}/libonnxruntime.so.1.24.1")
  set(ONNXRUNTIME_LIB_DIR "${ONNXRUNTIME_ROOT}/lib")
endif()
install(FILES
  "${ONNXRUNTIME_LIB_DIR}/libonnxruntime.so.1.24.1"
  "${ONNXRUNTIME_LIB_DIR}/libonnxruntime.so.1"
  DESTINATION lib)

install(TARGETS rl_controller_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(FILES rl_controller_plugin.xml
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h")
install(DIRECTORY msg/
  DESTINATION share/${PROJECT_NAME}/msg
  FILES_MATCHING PATTERN "*.msg")

# Install default policy (from workspace resources) so policy_path can be left empty
set(POLICY_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../resources/policy")
if(EXISTS "${POLICY_SOURCE_DIR}/walk.onnx")
  install(FILES "${POLICY_SOURCE_DIR}/walk.onnx" DESTINATION share/${PROJECT_NAME}/policy)
endif()
if(EXISTS "${POLICY_SOURCE_DIR}/policy.onnx")
  install(FILES "${POLICY_SOURCE_DIR}/policy.onnx" DESTINATION share/${PROJECT_NAME}/policy)
endif()

ament_export_dependencies(rclcpp std_msgs sensor_msgs controller_interface
  hardware_interface pluginlib rclcpp_lifecycle vmc_controller
  pid_controller control_input_msgs ament_index_cpp rosidl_default_runtime)
ament_package()
