#!/bin/bash

# 自动检测并设置 MUJOCO_DIR 环境变量
if [ -z "$MUJOCO_DIR" ]; then
    # 尝试从 Python 包中找到 MuJoCo
    MUJOCO_PYTHON_PATH=$(python3 -c "import mujoco; import os; print(os.path.dirname(mujoco.__file__))" 2>/dev/null)
    if [ -n "$MUJOCO_PYTHON_PATH" ] && [ -d "$MUJOCO_PYTHON_PATH/include" ]; then
        # 检查库文件是否存在（支持多种命名格式）
        MUJOCO_LIB=$(find "$MUJOCO_PYTHON_PATH" -maxdepth 1 -name "libmujoco.so*" -type f 2>/dev/null | head -1)
        if [ -n "$MUJOCO_LIB" ]; then
            # Python 包的库文件在根目录，但 CMake 期望在 lib/ 目录下
            # 创建临时 lib 目录的符号链接（如果不存在）
            if [ ! -d "$MUJOCO_PYTHON_PATH/lib" ]; then
                # 创建 lib 目录并创建符号链接
                mkdir -p "$MUJOCO_PYTHON_PATH/lib"
                ln -sf "$MUJOCO_LIB" "$MUJOCO_PYTHON_PATH/lib/libmujoco.so"
                echo "Created lib directory and symlink for MuJoCo library"
            fi
            export MUJOCO_DIR="$MUJOCO_PYTHON_PATH"
            echo "Found MuJoCo at: $MUJOCO_DIR"
        else
            MUJOCO_PYTHON_PATH=""
        fi
    fi
    if [ -z "$MUJOCO_PYTHON_PATH" ]; then
        echo "Warning: MUJOCO_DIR not set and MuJoCo not found automatically."
        echo "Please set MUJOCO_DIR environment variable to the MuJoCo installation directory."
        echo "Example: export MUJOCO_DIR=/path/to/mujoco-3.x.x"
    fi
fi

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON