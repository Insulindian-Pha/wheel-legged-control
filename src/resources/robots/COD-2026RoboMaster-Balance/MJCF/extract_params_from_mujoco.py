#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
从 MuJoCo XML 文件中提取机器人物理参数
用于 LQR 控制器参数计算
"""

import xml.etree.ElementTree as ET
import re
import numpy as np
from collections import defaultdict


def parse_mujoco_xml(xml_file):
    """
    解析 MuJoCo XML 文件，提取所有 body 的质量和转动惯量
    使用正则表达式方法，直接搜索所有 inertial 标签及其父 body
    
    返回:
        bodies: dict, 包含所有 body 的信息
    """
    with open(xml_file, 'r', encoding='utf-8') as f:
        lines = f.readlines()
    
    bodies = {}
    
    # 找到所有 body 标签的位置和名称
    body_info = {}  # {line_num: {'name': ..., 'pos': ..., 'start': ..., 'end': ...}}
    body_stack = []  # 用于跟踪嵌套的 body
    
    for i, line in enumerate(lines):
        # 查找 body 开始标签
        body_start_match = re.search(r'<body\s+name="([^"]+)"', line)
        if body_start_match:
            body_name = body_start_match.group(1)
            pos_match = re.search(r'pos="([^"]+)"', line)
            pos = [0, 0, 0]
            if pos_match:
                try:
                    pos = [float(x) for x in pos_match.group(1).split()]
                    if len(pos) < 3:
                        pos = pos + [0] * (3 - len(pos))
                except:
                    pass
            
            body_info[i] = {
                'name': body_name,
                'pos': pos,
                'start': i,
                'end': None
            }
            body_stack.append(i)
        
        # 查找 body 结束标签
        if '</body>' in line:
            if body_stack:
                start_line = body_stack.pop()
                body_info[start_line]['end'] = i
    
    # 在每个 body 的范围内查找 inertial 标签
    for start_line, info in body_info.items():
        body_name = info['name']
        end_line = info['end'] if info['end'] else len(lines)
        
        # 在这个 body 的范围内查找 inertial
        body_content = ''.join(lines[start_line:end_line+1])
        
        # 查找 inertial 标签
        inertial_pattern = r'<inertial[^>]*>'
        inertial_match = re.search(inertial_pattern, body_content)
        
        if inertial_match:
            # 提取 inertial 标签的完整内容（可能跨多行）
            inertial_start = inertial_match.start()
            # 找到对应的结束标签
            inertial_end = body_content.find('</inertial>', inertial_start)
            if inertial_end == -1:
                # 自闭合标签
                inertial_end = body_content.find('/>', inertial_start)
            if inertial_end == -1:
                inertial_end = inertial_start + 500  # 限制长度
            
            inertial_tag = body_content[inertial_start:inertial_end+2]
            
            # 提取 mass
            mass_match = re.search(r'mass="([^"]+)"', inertial_tag)
            mass = 0
            if mass_match:
                try:
                    mass = float(mass_match.group(1))
                except:
                    pass
            
            # 提取 diaginertia
            inertia_match = re.search(r'diaginertia="([^"]+)"', inertial_tag)
            diaginertia = [0, 0, 0]
            if inertia_match:
                try:
                    diaginertia_str = inertia_match.group(1)
                    diaginertia = [float(x) for x in diaginertia_str.split()]
                    if len(diaginertia) < 3:
                        diaginertia = diaginertia + [0] * (3 - len(diaginertia))
                except:
                    pass
            
            bodies[body_name] = {
                'mass': mass,
                'inertia': diaginertia,
                'pos': info['pos'],
                'parent': ''
            }
        else:
            # 没有 inertial 标签
            bodies[body_name] = {
                'mass': 0,
                'inertia': [0, 0, 0],
                'pos': info['pos'],
                'parent': ''
            }
    
    return bodies


def classify_bodies(bodies):
    """
    将 body 分类为：轮子、左腿、右腿、身体等
    
    返回:
        classified: dict, 分类后的 body 信息
    """
    classified = {
        'wheels': [],
        'left_leg': [],
        'right_leg': [],
        'base': [],
        'other': []
    }
    
    for name, info in bodies.items():
        name_lower = name.lower()
        
        if 'wheel' in name_lower:
            classified['wheels'].append((name, info))
        elif 'left' in name_lower and ('front' in name_lower or 'rear' in name_lower):
            classified['left_leg'].append((name, info))
        elif 'right' in name_lower and ('front' in name_lower or 'rear' in name_lower):
            classified['right_leg'].append((name, info))
        elif 'base' in name_lower:
            classified['base'].append((name, info))
        else:
            classified['other'].append((name, info))
    
    return classified


def calculate_aggregated_params(classified, all_bodies):
    """
    计算聚合参数，用于 LQR 控制器
    
    参数:
        classified: 分类后的 body 信息
        all_bodies: 所有 body 的字典
    
    返回:
        params: dict, 包含 m_w, m_l, m_b, I_w, I_b, I_z 等参数
    """
    params = {}
    
    # 计算轮子总质量和转动惯量
    wheel_mass = 0
    wheel_inertia = [0, 0, 0]
    for name, info in classified['wheels']:
        wheel_mass += info['mass']
        # 累加转动惯量（简化处理，实际应该考虑平行轴定理）
        for i in range(3):
            wheel_inertia[i] += info['inertia'][i]
    
    # 单个轮子的质量（假设左右轮质量相同）
    params['m_w'] = wheel_mass / len(classified['wheels']) if classified['wheels'] else 0
    # 单个轮子的转动惯量（取平均值）
    if classified['wheels']:
        avg_inertia = [x / len(classified['wheels']) for x in wheel_inertia]
        # 轮子主要绕 y 轴旋转，取 Iyy
        params['I_w'] = avg_inertia[1] if len(avg_inertia) > 1 else avg_inertia[0]
    else:
        params['I_w'] = 0
    
    # 计算左腿总质量
    left_leg_mass = sum(info['mass'] for _, info in classified['left_leg'])
    # 计算右腿总质量
    right_leg_mass = sum(info['mass'] for _, info in classified['right_leg'])
    # 单条腿的质量（取平均值）
    params['m_l'] = (left_leg_mass + right_leg_mass) / 2 if (left_leg_mass + right_leg_mass) > 0 else 0
    
    # 计算左腿转动惯量（简化处理）
    left_leg_inertia = [0, 0, 0]
    for _, info in classified['left_leg']:
        for i in range(3):
            left_leg_inertia[i] += info['inertia'][i]
    
    right_leg_inertia = [0, 0, 0]
    for _, info in classified['right_leg']:
        for i in range(3):
            right_leg_inertia[i] += info['inertia'][i]
    
    # 单条腿的转动惯量（取平均值）
    if left_leg_mass + right_leg_mass > 0:
        avg_leg_inertia = [(left_leg_inertia[i] + right_leg_inertia[i]) / 2 for i in range(3)]
        params['I_ll'] = avg_leg_inertia[1] if len(avg_leg_inertia) > 1 else avg_leg_inertia[0]
        params['I_lr'] = params['I_ll']  # 假设左右腿相同
    else:
        params['I_ll'] = 0
        params['I_lr'] = 0
    
    # 计算身体质量（base 可能没有 inertial，需要手动设置或从其他部分计算）
    base_mass = sum(info['mass'] for _, info in classified['base'])
    # 如果没有 base 的质量，可以从总质量减去其他部分
    if base_mass == 0:
        total_mass = sum(info['mass'] for _, info in all_bodies.items())
        params['m_b'] = total_mass - wheel_mass - left_leg_mass - right_leg_mass
    else:
        params['m_b'] = base_mass
    
    # 计算身体转动惯量
    base_inertia = [0, 0, 0]
    for _, info in classified['base']:
        for i in range(3):
            base_inertia[i] += info['inertia'][i]
    
    # I_b: 身体绕自身轴的转动惯量（取 Iyy 或平均值）
    if sum(base_inertia) > 0:
        params['I_b'] = base_inertia[1] if len(base_inertia) > 1 else sum(base_inertia) / 3
    else:
        # 如果没有，使用估算值
        params['I_b'] = 0.01  # 默认值，需要根据实际情况调整
    
    # I_z: 绕 z 轴的转动惯量（航向角）
    # 这需要从所有部件的转动惯量计算，简化处理
    total_inertia_z = sum(info['inertia'][2] for _, info in all_bodies.items() if len(info['inertia']) > 2)
    params['I_z'] = total_inertia_z if total_inertia_z > 0 else 0.1  # 默认值
    
    return params


def extract_geometry_params(xml_file):
    """
    从 XML 文件中提取几何参数（轮子半径、轮距等）
    这些参数可能需要从 pos 属性或其他地方推断
    """
    # 注意：这里不再用 ET.parse()，避免 XML 标签不严格时直接崩溃。
    # 目前简单给出一个“可运行的默认值”，你可以按模型实际尺寸修改。
    #
    # 轮距的一半 R_l：可以用左右轮 body 的 y 方向 pos 差值 / 2。
    # 轮子半径 R_w：XML 里是 mesh，没有直接给半径；建议你从 CAD/mesh 尺寸或仿真配置确认。

    geometry = {
        'R_w': 0.06,  # 轮子半径，需要根据实际模型调整
        'R_l': 0.18,  # 轮距的一半，从 pos 推断
        'l_c': 0,     # 质心偏移
        'g': 9.8      # 重力加速度
    }
    
    return geometry


def print_params_for_lagrange(params, geometry):
    """
    打印用于 lagrange.py 的参数格式
    """
    print("=" * 60)
    print("从 MuJoCo XML 提取的物理参数")
    print("=" * 60)
    print("\n用于 lagrange.py 的参数设置：")
    print(f"R_w, R_l, l_c, m_w, m_l, m_b, I_w, I_b, I_z, g = "
          f"{geometry['R_w']}, {geometry['R_l']}, {geometry['l_c']}, "
          f"{params['m_w']:.6f}, {params['m_l']:.6f}, {params['m_b']:.6f}, "
          f"{params['I_w']:.8f}, {params['I_b']:.8f}, {params['I_z']:.8f}, "
          f"{geometry['g']}")
    
    print("\n详细参数说明：")
    print(f"  R_w (轮子半径): {geometry['R_w']} m")
    print(f"  R_l (轮距的一半): {geometry['R_l']} m")
    print(f"  l_c (质心偏移): {geometry['l_c']} m")
    print(f"  m_w (单个轮子质量): {params['m_w']:.6f} kg")
    print(f"  m_l (单条腿质量): {params['m_l']:.6f} kg")
    print(f"  m_b (身体质量): {params['m_b']:.6f} kg")
    print(f"  I_w (轮子转动惯量): {params['I_w']:.8f} kg·m²")
    print(f"  I_b (身体转动惯量): {params['I_b']:.8f} kg·m²")
    print(f"  I_z (绕z轴转动惯量): {params['I_z']:.8f} kg·m²")
    print(f"  g (重力加速度): {geometry['g']} m/s²")
    
    print("\n注意：")
    print("  1. 轮子半径 R_w 需要根据实际模型调整")
    print("  2. 转动惯量的计算是简化处理，实际应该考虑平行轴定理")
    print("  3. 如果某些参数为 0 或异常，请手动检查并调整")


if __name__ == "__main__":
    import sys
    
    # XML 文件路径
    if len(sys.argv) > 1:
        xml_file = sys.argv[1]
    else:
        xml_file = "COD-2026RoboMaster-Balance.xml"
    
    print(f"正在解析 MuJoCo XML 文件: {xml_file}\n")
    
    try:
        # 解析 XML
        bodies = parse_mujoco_xml(xml_file)
        
        print(f"找到 {len(bodies)} 个 body 元素\n")
        
        # 分类
        classified = classify_bodies(bodies)
        
        print("分类结果：")
        print(f"  轮子: {len(classified['wheels'])} 个")
        for name, info in classified['wheels']:
            print(f"    - {name}: 质量={info['mass']:.4f} kg, 转动惯量={info['inertia']}")
        
        print(f"\n  左腿部件: {len(classified['left_leg'])} 个")
        for name, info in classified['left_leg']:
            print(f"    - {name}: 质量={info['mass']:.4f} kg")
        
        print(f"\n  右腿部件: {len(classified['right_leg'])} 个")
        for name, info in classified['right_leg']:
            print(f"    - {name}: 质量={info['mass']:.4f} kg")
        
        print(f"\n  身体部件: {len(classified['base'])} 个")
        for name, info in classified['base']:
            print(f"    - {name}: 质量={info['mass']:.4f} kg")
        
        # 计算聚合参数
        params = calculate_aggregated_params(classified, bodies)
        
        # 提取几何参数
        geometry = extract_geometry_params(xml_file)
        
        # 打印结果
        print_params_for_lagrange(params, geometry)
        
    except FileNotFoundError:
        print(f"错误：找不到文件 {xml_file}")
        print("使用方法: python extract_params_from_mujoco.py <xml_file>")
    except ET.ParseError as e:
        print(f"错误：XML 解析失败: {e}")
    except Exception as e:
        print(f"错误：{e}")
        import traceback
        traceback.print_exc()

