#!/usr/bin/env bash
# 显式指定 Python，避免 snap 版 CMake 与 FindPythonInterp 组合下缺少 PYTHON_EXECUTABLE，
# 导致 rosidl_generate_interfaces（如 vmc_controller、librmcs_hardware）配置失败。
PYTHON3="$(command -v python3)"
[ -n "$PYTHON3" ] || PYTHON3=/usr/bin/python3
colcon build --symlink-install --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DPYTHON_EXECUTABLE="${PYTHON3}"