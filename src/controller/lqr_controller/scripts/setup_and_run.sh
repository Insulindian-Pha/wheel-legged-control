#!/bin/bash
# 设置虚拟环境并运行 lagrange.py 计算 K 值

echo "======================================================"
echo "LQR K 值计算 - 环境设置与运行"
echo "======================================================"

# 虚拟环境目录
VENV_DIR="venv_lagrange"

# 检查虚拟环境是否存在
if [ ! -d "$VENV_DIR" ]; then
    echo "创建虚拟环境..."
    python3 -m venv $VENV_DIR
    
    echo "激活虚拟环境..."
    source $VENV_DIR/bin/activate
    
    echo "安装依赖包（NumPy 和 SciPy）..."
    pip install --upgrade pip
    pip install "numpy<2.0" scipy
    
    echo "虚拟环境设置完成！"
else
    echo "虚拟环境已存在，激活中..."
    source $VENV_DIR/bin/activate
fi

echo ""
echo "======================================================"
echo "运行 lagrange.py 计算 K 值"
echo "======================================================"
echo ""

# 运行脚本
python3 run_lagrange.py

# 保存退出码
EXIT_CODE=$?

# 提示
echo ""
echo "======================================================"
if [ $EXIT_CODE -eq 0 ]; then
    echo "计算成功完成！"
else
    echo "计算失败，退出码：$EXIT_CODE"
fi
echo "======================================================"

# 退出虚拟环境
deactivate 2>/dev/null || true

exit $EXIT_CODE

