#!/usr/bin/env python3
"""
手动输入关节位置的工具脚本
用于确定VMC控制器的电机零点位置

使用方法：
1. 将机器人放置在期望的初始位置（零点位置）
2. 从日志、话题或其他方式获取关节位置值
3. 运行此脚本并手动输入各关节位置
4. 脚本会自动计算偏移值并生成配置文件格式
"""

import math


def input_float(prompt, allow_negative=True):
    """安全地读取浮点数输入"""
    while True:
        try:
            value = float(input(prompt))
            if not allow_negative and value < 0:
                print("  错误：请输入非负值")
                continue
            return value
        except ValueError:
            print("  错误：请输入有效的数字")
        except KeyboardInterrupt:
            print("\n\n已取消")
            exit(0)


def input_unit():
    """让用户选择输入单位"""
    print("\n请选择输入单位：")
    print("  1. 弧度 (rad)")
    print("  2. 度 (degree)")
    
    while True:
        choice = input("请输入选项 (1 或 2): ").strip()
        if choice == '1':
            return 'rad'
        elif choice == '2':
            return 'deg'
        else:
            print("  错误：请输入 1 或 2")


def deg_to_rad(deg):
    """将度转换为弧度"""
    return deg * math.pi / 180.0


def main():
    print("=" * 70)
    print("VMC控制器电机零点位置校准工具")
    print("=" * 70)
    print("\n说明：")
    print("  1. 请确保机器人处于期望的初始位置（零点位置）")
    print("  2. 从以下方式之一获取关节位置值：")
    print("     - 运行: ros2 topic echo /joint_states --once")
    print("     - 查看控制器日志输出")
    print("     - 从其他工具或传感器读取")
    print("  3. 输入各关节的当前位置值")
    print("=" * 70)
    
    # 选择输入单位
    unit = input_unit()
    unit_name = "弧度 (rad)" if unit == 'rad' else "度 (degree)"
    
    print(f"\n请输入各关节的当前位置值（单位：{unit_name}）")
    print("-" * 70)
    
    # 定义关节信息
    joints = [
        ("Left_front_joint", "左前关节"),
        ("Left_rear_joint", "左后关节"),
        ("Right_front_joint", "右前关节"),
        ("Right_rear_joint", "右后关节")
    ]
    
    # 读取各关节位置
    joint_positions = {}
    for joint_name, joint_desc in joints:
        prompt = f"  {joint_desc} ({joint_name}): "
        value = input_float(prompt)
        
        # 转换为弧度
        if unit == 'deg':
            value_rad = deg_to_rad(value)
            print(f"    -> {value_rad:.6f} rad")
        else:
            value_rad = value
        
        joint_positions[joint_name] = value_rad
    
    # 计算偏移值
    print("\n" + "=" * 70)
    print("计算结果")
    print("=" * 70)
    print("\n原始关节位置（弧度）:")
    print("-" * 70)
    
    offset_mapping = {
        'Left_front_joint': 'left_front_joint_offset',
        'Left_rear_joint': 'left_rear_joint_offset',
        'Right_front_joint': 'right_front_joint_offset',
        'Right_rear_joint': 'right_rear_joint_offset'
    }
    
    for joint_name, position in joint_positions.items():
        print(f"  {joint_name}: {position:.6f} rad ({position * 180 / math.pi:.2f} deg)")
    
    print("\n计算得到的偏移值（offset = -position）:")
    print("-" * 70)
    
    offsets = {}
    for joint_name, position in joint_positions.items():
        offset = -position
        config_name = offset_mapping[joint_name]
        offsets[config_name] = offset
        print(f"  {config_name}: {offset:.6f}  # {joint_name}")
    
    # 生成YAML配置格式
    print("\n" + "=" * 70)
    print("YAML配置文件格式（可直接复制到 cod_vmc_config.yaml）:")
    print("=" * 70)
    print("\n    # 关节初始角度偏置 (单位: rad)")
    print("    # 计算公式: pos = pos_raw + offset，要使pos=0，则 offset = -pos_raw")
    print("    # 如果设置为0.0，则使用当前关节位置作为初始角度（需要在启动时手动设置）")
    
    for joint_name, position in joint_positions.items():
        offset = -position
        config_name = offset_mapping[joint_name]
        joint_desc = next(desc for name, desc in joints if name == joint_name)
        print(f"    {config_name}: {offset:.6f}  # {joint_desc}")
    
    print("\n" + "=" * 70)
    print("完成！请将上述配置复制到配置文件中。")
    print("=" * 70)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\n已取消")
        exit(0)
