#!/usr/bin/env python3
"""
从URDF文件中提取所有连杆的长度信息
"""
import xml.etree.ElementTree as ET
import numpy as np
import os
import sys

def parse_xyz(xyz_str):
    """解析xyz字符串，返回[x, y, z]"""
    if xyz_str is None:
        return [0.0, 0.0, 0.0]
    try:
        parts = xyz_str.strip().split()
        return [float(x) for x in parts[:3]]
    except (ValueError, AttributeError):
        return [0.0, 0.0, 0.0]

def calculate_distance(pos1, pos2):
    """计算两点之间的欧几里得距离"""
    return np.linalg.norm(np.array(pos1) - np.array(pos2))

def extract_link_lengths(urdf_file):
    """
    从URDF文件中提取连杆长度信息
    
    参数:
        urdf_file: URDF文件路径
    
    返回:
        dict: 包含每个关节和连杆信息的字典
    """
    tree = ET.parse(urdf_file)
    root = tree.getroot()
    
    # 存储link和joint的信息
    links = {}  # link名称 -> link信息
    joints = {}  # joint名称 -> joint信息
    link_positions = {}  # link名称 -> 在全局坐标系中的位置
    
    # 首先收集所有link的信息
    for link in root.findall('link'):
        link_name = link.get('name')
        if link_name:
            links[link_name] = {
                'name': link_name,
                'inertial': None,
                'visual': None,
                'collision': None
            }
    
    # 收集所有joint的信息
    for joint in root.findall('joint'):
        joint_name = joint.get('name')
        if not joint_name:
            continue
            
        parent_elem = joint.find('parent')
        child_elem = joint.find('child')
        origin_elem = joint.find('origin')
        
        if parent_elem is None or child_elem is None:
            continue
            
        parent_link = parent_elem.get('link')
        child_link = child_elem.get('link')
        
        # 解析origin的xyz和rpy
        xyz = [0.0, 0.0, 0.0]
        if origin_elem is not None:
            xyz_str = origin_elem.get('xyz')
            if xyz_str:
                xyz = parse_xyz(xyz_str)
        
        # 获取关节轴
        axis_elem = joint.find('axis')
        axis = [0.0, 0.0, 1.0]
        if axis_elem is not None:
            axis_str = axis_elem.get('xyz')
            if axis_str:
                axis = parse_xyz(axis_str)
        
        joints[joint_name] = {
            'name': joint_name,
            'type': joint.get('type', 'unknown'),
            'parent': parent_link,
            'child': child_link,
            'origin_xyz': xyz,
            'axis': axis
        }
    
    # 构建从base_link开始的树结构，计算每个link的全局位置
    def build_tree(link_name, parent_pos=[0.0, 0.0, 0.0], visited=None):
        """递归构建树并计算位置"""
        if visited is None:
            visited = set()
        
        if link_name in visited:
            return
        
        visited.add(link_name)
        link_positions[link_name] = parent_pos
        
        # 找到所有以该link为parent的joint
        for joint_name, joint_info in joints.items():
            if joint_info['parent'] == link_name:
                child_link = joint_info['child']
                # 计算子link的位置（父位置 + joint origin）
                child_pos = [
                    parent_pos[0] + joint_info['origin_xyz'][0],
                    parent_pos[1] + joint_info['origin_xyz'][1],
                    parent_pos[2] + joint_info['origin_xyz'][2]
                ]
                # 递归处理子link
                build_tree(child_link, child_pos, visited)
    
    # 从base_link开始构建树
    if 'base_link' in links:
        build_tree('base_link', [0.0, 0.0, 0.0])
    else:
        # 如果没有base_link，找到所有没有parent的link
        parent_links = {j['parent'] for j in joints.values()}
        root_links = [name for name in links.keys() if name not in parent_links]
        if root_links:
            build_tree(root_links[0], [0.0, 0.0, 0.0])
    
    # 计算每个连杆的长度（从parent joint到child joint的距离）
    link_lengths = {}
    for joint_name, joint_info in joints.items():
        parent_link = joint_info['parent']
        child_link = joint_info['child']
        origin_xyz = joint_info['origin_xyz']
        
        # 连杆长度就是joint origin的xyz向量的模长
        length = calculate_distance([0, 0, 0], origin_xyz)
        
        # 使用child_link名称作为连杆名称
        link_lengths[child_link] = {
            'link_name': child_link,
            'parent_link': parent_link,
            'joint_name': joint_name,
            'joint_type': joint_info['type'],
            'origin_xyz': origin_xyz,
            'length': length,
            'axis': joint_info['axis']
        }
    
    return {
        'links': links,
        'joints': joints,
        'link_lengths': link_lengths,
        'link_positions': link_positions
    }

def print_link_lengths(info):
    """打印连杆长度信息"""
    print("=" * 100)
    print("机器人连杆长度信息")
    print("=" * 100)
    print(f"{'连杆名称':<30} {'父连杆':<30} {'关节名称':<30} {'长度(m)':<15} {'位置(x,y,z)':<30}")
    print("-" * 100)
    
    # 按长度排序
    sorted_lengths = sorted(
        info['link_lengths'].items(),
        key=lambda x: x[1]['length'],
        reverse=True
    )
    
    for link_name, link_info in sorted_lengths:
        pos_str = f"({link_info['origin_xyz'][0]:.6f}, {link_info['origin_xyz'][1]:.6f}, {link_info['origin_xyz'][2]:.6f})"
        print(f"{link_name:<30} {link_info['parent_link']:<30} {link_info['joint_name']:<30} {link_info['length']:<15.6f} {pos_str:<30}")
    
    print("=" * 100)
    
    # 统计信息
    print("\n统计信息:")
    print(f"总连杆数量: {len(info['link_lengths'])}")
    print(f"总关节数量: {len(info['joints'])}")
    
    # 列出所有非零长度的连杆
    print("\n非零长度的连杆（按长度排序）:")
    non_zero_links = [
        (name, data['length']) 
        for name, data in info['link_lengths'].items() 
        if data['length'] > 0.001
    ]
    non_zero_links.sort(key=lambda x: x[1], reverse=True)
    for name, length in non_zero_links:
        print(f"  {name:<30}: {length:.6f} m")
    
    # 列出所有零长度或接近零长度的连杆（可能是旋转关节）
    print("\n零长度或接近零长度的连杆（可能是旋转关节）:")
    zero_links = [
        (name, data['length']) 
        for name, data in info['link_lengths'].items() 
        if data['length'] <= 0.001
    ]
    for name, length in zero_links:
        print(f"  {name:<30}: {length:.6f} m (关节类型: {info['link_lengths'][name]['joint_type']})")

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="从URDF文件中提取连杆长度信息")
    parser.add_argument(
        "urdf_file", 
        nargs="?", 
        help="URDF文件路径（可选，默认使用COD_2026_Balance_2_0.xacro）"
    )
    
    args = parser.parse_args()
    
    if args.urdf_file:
        urdf_file = args.urdf_file
    else:
        # 默认使用COD_2026_Balance_2_0.xacro
        script_dir = os.path.dirname(os.path.abspath(__file__))
        urdf_file = os.path.join(script_dir, "..", "urdf", "COD_2026_Balance_2_0.xacro")
    
    if not os.path.exists(urdf_file):
        print(f"错误: 找不到文件 {urdf_file}")
        return 1
    
    print(f"正在解析文件: {urdf_file}\n")
    
    try:
        info = extract_link_lengths(urdf_file)
        print_link_lengths(info)
        
        return 0
        
    except Exception as e:
        print(f"解析文件时出错: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == "__main__":
    sys.exit(main())

