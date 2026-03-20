#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
运行 lagrange.py 计算 LQR 的 K 值

使用方法：
    python3 run_lagrange.py

参数说明：
    ll, lr: 左右腿长度（米）
    q1-q10: LQR 状态权重矩阵 Q 的对角元素
    r1-r4: LQR 控制权重矩阵 R 的对角元素
"""

import sys
import os

# 添加当前目录到路径
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    from lagrange import K
    import numpy as np
    
    # 设置打印选项
    np.set_printoptions(precision=4, suppress=True, linewidth=300)
    
    print("=" * 60)
    print("LQR K 值计算")
    print("=" * 60)
    
    # 参数设置
    ll = 0.2  # 左腿长度（米）
    lr = 0.2  # 右腿长度（米）
    
    # LQR 权重矩阵 Q 的对角元素（10个状态变量）
    # Q = diag(q1, q2, q3, q4, q5, q6, q7, q8, q9, q10)
    # 对应状态：s, dot_s, fai, dot_fai, theta_ll, dot_theta_ll, theta_lr, dot_theta_lr, theta_b, dot_theta_b
    q1 = 10   # 位移 s
    q2 = 5    # 速度 dot_s
    q3 = 10   # 航向角 fai
    q4 = 5    # 航向角速度 dot_fai
    q5 = 15   # 左腿角度 theta_ll
    q6 = 10    # 左腿角速度 dot_theta_ll
    q7 = 15   # 右腿角度 theta_lr
    q8 = 10    # 右腿角速度 dot_theta_lr
    q9 = 3000   # 身体角度 theta_b
    q10 = 50   # 身体角速度 dot_theta_b
    
    # LQR 权重矩阵 R 的对角元素（4个控制输入）
    # R = diag(r1, r2, r3, r4)
    r1 = 1    # 控制输入 1
    r2 = 1  # 控制输入 2
    r3 = 1    # 控制输入 3
    r4 = 1    # 控制输入 4
    
    print(f"\n输入参数：")
    print(f"  腿长: ll={ll} m, lr={lr} m")
    print(f"  Q权重: [{q1}, {q2}, {q3}, {q4}, {q5}, {q6}, {q7}, {q8}, {q9}, {q10}]")
    print(f"  R权重: [{r1}, {r2}, {r3}, {r4}]")
    
    print(f"\n正在计算 LQR K 值...")
    
    # 计算 K 值
    k_matrix = K(ll, lr, q1, q2, q3, q4, q5, q6, q7, q8, q9, q10, r1, r2, r3, r4)
    
    print(f"\n计算完成！\n")
    print("K 矩阵 (4x10):")
    print(k_matrix)
    
    # 状态变量说明（对应 K 矩阵的列）
    state_names = [
        "s (位移)",
        "dot_s (速度)", 
        "fai (航向角)",
        "dot_fai (航向角速度)",
        "theta_ll (左腿角度)",
        "dot_theta_ll (左腿角速度)",
        "theta_lr (右腿角度)",
        "dot_theta_lr (右腿角速度)",
        "theta_b (身体角度)",
        "dot_theta_b (身体角速度)"
    ]
    
    # 控制输入说明（对应 K 矩阵的行）
    control_names = [
        "左轮力矩",
        "右轮力矩",
        "左腿力矩",
        "右腿力矩"
    ]
    
    # 输出与 cod_lqr_vmc_config.yaml 第161-215行完全一致的片段，便于整段复制粘贴替换
    print("\n\n--- 以下可直接复制粘贴到 cod_lqr_vmc_config.yaml 替换 LQR gains 段 ---\n")
    print("    left_wheel_lqr_gains:")
    for j in range(10):
        print(f"      {j}: {k_matrix[0, j]:.4f} # {state_names[j]} -> 左轮力矩")
    print("    right_wheel_lqr_gains:")
    for j in range(10):
        print(f"      {j}: {k_matrix[1, j]:.4f} # {state_names[j]} -> 右轮力矩")
    print("    left_leg_lqr_gains:")
    for j in range(10):
        print(f"      {j}: {k_matrix[2, j]:.4f} # {state_names[j]} -> 左腿力矩")
    print("    right_leg_lqr_gains:")
    for j in range(10):
        print(f"      {j}: {k_matrix[3, j]:.4f} # {state_names[j]} -> 右腿力矩")
    print("\n--- 复制到此处为止 ---")
    
except ImportError as e:
    print(f"错误：无法导入 lagrange 模块")
    print(f"详细信息：{e}")
    print("\n可能的解决方案：")
    print("  1. 检查 NumPy 和 SciPy 版本是否兼容")
    print("  2. 尝试：pip install numpy==1.24.3 scipy")
    print("  3. 或使用虚拟环境安装兼容版本")
    sys.exit(1)
except Exception as e:
    print(f"错误：{e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

