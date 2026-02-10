#!/usr/bin/env python3
"""
从MJCF文件中提取所有连杆的长度信息
"""
import xml.etree.ElementTree as ET
import numpy as np
import os
import sys

def parse_pos(pos_str):
    """解析位置字符串，返回[x, y, z]"""
    if pos_str is None:
        return [0.0, 0.0, 0.0]
    try:
        parts = pos_str.strip().split()
        return [float(x) for x in parts[:3]]
    except (ValueError, AttributeError):
        return [0.0, 0.0, 0.0]

def calculate_length(pos):
    """计算向量的长度（欧几里得距离）"""
    return np.linalg.norm(pos)

def extract_link_lengths(xml_file):
    """
    从MJCF文件中提取连杆长度信息
    
    参数:
        xml_file: MJCF文件路径
    
    返回:
        dict: 包含每个body及其相对于父body的位置和长度的字典
    """
    tree = ET.parse(xml_file)
    root = tree.getroot()
    
    # 存储body的层次结构和位置信息
    body_info = {}
    
    def traverse_body(body_elem, parent_name="world", parent_pos=[0, 0, 0]):
        """递归遍历body元素"""
        body_name = body_elem.get("name", "unnamed")
        body_pos_str = body_elem.get("pos", "0 0 0")
        body_pos = parse_pos(body_pos_str)
        
        # 计算相对于父body的位置长度（即连杆长度）
        length = calculate_length(body_pos)
        
        # 存储信息
        body_info[body_name] = {
            "parent": parent_name,
            "position": body_pos,
            "length": length,
            "joints": []
        }
        
        # 提取该body下的所有joint
        for joint in body_elem.findall("joint"):
            joint_name = joint.get("name", "unnamed")
            joint_pos_str = joint.get("pos", "0 0 0")
            joint_pos = parse_pos(joint_pos_str)
            joint_axis = joint.get("axis", "0 0 1")
            joint_range = joint.get("range", "")
            joint_damping = joint.get("damping", "")
            joint_ref = joint.get("ref", "")
            
            body_info[body_name]["joints"].append({
                "name": joint_name,
                "position": joint_pos,
                "axis": joint_axis,
                "range": joint_range,
                "damping": joint_damping,
                "ref": joint_ref
            })
        
        # 递归处理子body
        for child_body in body_elem.findall("body"):
            traverse_body(child_body, body_name, body_pos)
    
    # 从worldbody开始遍历
    worldbody = root.find("worldbody")
    if worldbody is not None:
        for body in worldbody.findall("body"):
            traverse_body(body, "world", [0, 0, 0])
    
    return body_info

def print_link_lengths(body_info):
    """打印连杆长度信息"""
    print("=" * 100)
    print("机器人连杆长度信息（MJCF格式）")
    print("=" * 100)
    print(f"{'连杆名称':<30} {'父连杆':<30} {'位置(x,y,z)':<30} {'长度(m)':<15} {'关节数':<10}")
    print("-" * 100)
    
    # 按长度排序
    sorted_bodies = sorted(
        body_info.items(),
        key=lambda x: x[1]['length'],
        reverse=True
    )
    
    for body_name, info in sorted_bodies:
        pos_str = f"({info['position'][0]:.6f}, {info['position'][1]:.6f}, {info['position'][2]:.6f})"
        print(f"{body_name:<30} {info['parent']:<30} {pos_str:<30} {info['length']:<15.6f} {len(info['joints']):<10}")
        
        # 打印该body下的关节信息
        if info['joints']:
            for joint in info['joints']:
                range_str = joint['range'] if joint['range'] else "无限制"
                damping_str = f"阻尼={joint['damping']}" if joint['damping'] else ""
                ref_str = f"ref={joint['ref']}" if joint['ref'] else ""
                extra = " ".join([s for s in [damping_str, ref_str] if s])
                print(f"  └─ 关节: {joint['name']:<25} 轴: {joint['axis']:<15} 范围: {range_str:<15} {extra}")
    
    print("=" * 100)
    
    # 统计信息
    print("\n统计信息:")
    print(f"总连杆数量: {len(body_info)}")
    total_joints = sum(len(info['joints']) for info in body_info.values())
    print(f"总关节数量: {total_joints}")
    
    # 列出所有非零长度的连杆
    print("\n非零长度的连杆（按长度排序）:")
    non_zero_bodies = [
        (name, info['length']) 
        for name, info in body_info.items() 
        if info['length'] > 0.001
    ]
    non_zero_bodies.sort(key=lambda x: x[1], reverse=True)
    for name, length in non_zero_bodies:
        print(f"  {name:<30}: {length:.6f} m")
    
    # 列出所有零长度或接近零长度的连杆（可能是旋转关节）
    print("\n零长度或接近零长度的连杆（可能是旋转关节）:")
    zero_bodies = [
        (name, info['length']) 
        for name, info in body_info.items() 
        if info['length'] <= 0.001
    ]
    if zero_bodies:
        for name, length in zero_bodies:
            joints_info = ", ".join([j['name'] for j in body_info[name]['joints']])
            print(f"  {name:<30}: {length:.6f} m (关节: {joints_info})")
    else:
        print("  无")

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="从MJCF文件中提取连杆长度信息")
    parser.add_argument(
        "xml_file", 
        nargs="?", 
        help="MJCF文件路径（可选，默认使用COD-2026RoboMaster-Balance.xml）"
    )
    
    args = parser.parse_args()
    
    if args.xml_file:
        xml_file = args.xml_file
    else:
        # 默认使用COD-2026RoboMaster-Balance.xml
        script_dir = os.path.dirname(os.path.abspath(__file__))
        xml_file = os.path.join(script_dir, "..", "MJCF", "COD-2026RoboMaster-Balance.xml")
    
    if not os.path.exists(xml_file):
        print(f"错误: 找不到文件 {xml_file}")
        return 1
    
    print(f"正在解析文件: {xml_file}\n")
    
    try:
        body_info = extract_link_lengths(xml_file)
        print_link_lengths(body_info)
        
        return 0
        
    except Exception as e:
        print(f"解析文件时出错: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == "__main__":
    sys.exit(main())

