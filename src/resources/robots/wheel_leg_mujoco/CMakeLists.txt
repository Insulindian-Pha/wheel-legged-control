cmake_minimum_required(VERSION 3.8)
project(wheel_leg_mujoco)

# Find ament_cmake
find_package(ament_cmake REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install URDF directory (contains xacro files)
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)

# Install MJCF directory (contains model files and meshes)
install(DIRECTORY MJCF
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python files (Simulation.py, VMC.py, etc.)
install(PROGRAMS
  Simulation.py
  VMC.py
  environment.py
  keyboard.py
  caculation.py
  Controller.py
  DESTINATION share/${PROJECT_NAME}/
)

# Install scripts directory
install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}/
  USE_SOURCE_PERMISSIONS
)

ament_package()

