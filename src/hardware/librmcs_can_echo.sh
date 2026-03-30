#!/usr/bin/env bash
# 通过 librmcs（USB 控制板）发送 CAN 并打印接收帧。
# 使用前请先 source 本工作区的 install/setup.bash，例如:
#   source install/setup.bash
#   ./src/hardware/librmcs_can_echo.sh --help

set -euo pipefail
exec ros2 run librmcs_hardware librmcs_can_echo "$@"
