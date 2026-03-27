cmake_minimum_required(VERSION 3.16)

project(librmcs_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(LIBUSB REQUIRED IMPORTED_TARGET libusb-1.0)

find_package(librmcs QUIET CONFIG)

if(TARGET librmcs)
  set(LIBRMCS_TARGET librmcs)
elseif(TARGET librmcs::librmcs)
  set(LIBRMCS_TARGET librmcs::librmcs)
else()
  set(LIBRMCS_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/lib" CACHE PATH
      "Path containing the vendored librmcs headers when no installed librmcs package is available")
  if(NOT EXISTS "${LIBRMCS_SOURCE_DIR}/librmcs/client/cboard.hpp")
    message(FATAL_ERROR
      "Could not locate librmcs headers. Set LIBRMCS_SOURCE_DIR to a directory containing librmcs/client/cboard.hpp.")
  endif()

  add_library(librmcs_headers INTERFACE)
  target_include_directories(librmcs_headers INTERFACE "${LIBRMCS_SOURCE_DIR}")
  set(LIBRMCS_TARGET librmcs_headers)
endif()

add_library(librmcs_hardware SHARED
  src/system/librmcs_system.cpp
  src/driver/grouped_can_command.cpp
  src/devices/bm_motor_device.cpp
  src/devices/dji_motor_device.cpp
  src/devices/dm_motor_device.cpp
  src/driver/librmcs_robot_driver.cpp
  src/devices/lk_motor_device.cpp
  src/devices/motor_device_base.cpp
  src/devices/motor_device_factory.cpp
)

target_include_directories(librmcs_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(librmcs_hardware
  PRIVATE
    ${LIBRMCS_TARGET}
    PkgConfig::LIBUSB
)

ament_target_dependencies(librmcs_hardware
  PUBLIC
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface librmcs_hardware_plugin.xml)

install(
  TARGETS librmcs_hardware
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES librmcs_hardware_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
