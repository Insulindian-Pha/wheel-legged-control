#!/usr/bin/env python3
"""
VMC计算验证脚本
通过监听ROS2话题来验证VMC控制器中的计算是否正确
"""

import sys
import os
import math
import argparse
import yaml
from typing import Optional, Dict, Tuple
from collections import deque

# 添加VMC.py的路径
# 尝试多个可能的路径
script_dir = os.path.dirname(os.path.abspath(__file__))
possible_paths = [
    os.path.join(script_dir, '..'),  # 开发环境：scripts/../
    os.path.join(script_dir, '..', '..', 'share', 'wheel_leg_mujoco'),  # 安装环境：lib/wheel_leg_mujoco/../../share/wheel_leg_mujoco
    os.path.join(os.path.dirname(script_dir), 'share', 'wheel_leg_mujoco'),  # 另一种安装路径
]

for path in possible_paths:
    abs_path = os.path.abspath(path)
    if os.path.exists(os.path.join(abs_path, 'VMC.py')):
        sys.path.insert(0, abs_path)
        break

from VMC import leg_VMC

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float64MultiArray


class VMCVerifier(Node):
    """VMC计算验证器"""
    
    def __init__(self, config_file: Optional[str] = None):
        super().__init__('vmc_verifier')
        
        # 加载配置
        self.load_config(config_file)
        
        # 初始化VMC实例
        self.left_leg = leg_VMC()
        self.right_leg = leg_VMC()
        
        # 初始化杆长参数
        self.left_leg.l1 = self.l1
        self.left_leg.l2 = self.l2
        self.left_leg.l3 = self.l3
        self.left_leg.l4 = self.l4
        self.left_leg.l5 = self.l5
        
        self.right_leg.l1 = self.l1
        self.right_leg.l2 = self.l2
        self.right_leg.l3 = self.l3
        self.right_leg.l4 = self.l4
        self.right_leg.l5 = self.l5
        
        # 数据存储
        self.joint_positions = {}  # 存储关节位置
        # 初始化pitch为0（默认平衡状态）
        self.pitch = 0.0
        self.pitch_gyro = 0.0
        self.left_F0 = 0.0
        self.left_Tp = 0.0
        self.right_F0 = 0.0
        self.right_Tp = 0.0
        
        # 时间戳
        self.last_joint_time = None
        self.last_imu_time = None
        self.last_update_time = None
        
        # 数据接收标志
        self.received_joint_states = False
        self.received_imu = False
        self.received_force_command = False
        
        # 偏移值自动检测功能
        self.auto_calibrate_mode = self.config.get('auto_calibrate', False)
        self.balance_samples = deque(maxlen=100)  # 存储平衡状态下的样本
        self.balance_threshold = self.config.get('balance_threshold', 0.05)  # pitch角度阈值（rad）
        self.balance_stable_time = self.config.get('balance_stable_time', 2.0)  # 稳定时间（秒）
        self.balance_start_time = None
        self.calibration_complete = False
        
        # 打印计数器
        self.print_counter = 0
        self.print_frequency = self.config.get('print_frequency', 10)  # 默认10Hz
        
        # 创建订阅者
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # IMU订阅（可选，如果未启用自动校准或不需要IMU，可以不订阅）
        self.imu_sub = None
        if self.auto_calibrate_mode or self.config.get('enable_imu', True):
            self.imu_sub = self.create_subscription(
                Imu,
                self.config.get('imu_topic', '/imu/data'),
                self.imu_callback,
                10
            )
        
        self.force_command_sub = self.create_subscription(
            Float64MultiArray,
            self.config.get('force_command_topic', '/vmc_controller/force_command'),
            self.force_command_callback,
            10
        )
        
        # 创建定时器用于定期验证和打印
        timer_period = 1.0 / self.print_frequency
        self.timer = self.create_timer(timer_period, self.verify_and_print)
        
        self.get_logger().info('VMC验证器已启动')
        self.get_logger().info(f'订阅关节状态: /joint_states')
        if self.imu_sub is not None:
            self.get_logger().info(f'订阅IMU: {self.config.get("imu_topic", "/imu/data")}')
        else:
            self.get_logger().info('IMU订阅已禁用，使用默认值 pitch=0')
        self.get_logger().info(f'订阅力命令: {self.config.get("force_command_topic", "/vmc_controller/force_command")}')
        self.get_logger().info(f'打印频率: {self.print_frequency} Hz')
        if self.auto_calibrate_mode:
            self.get_logger().info('='*60)
            self.get_logger().info('🔧 自动校准模式已启用')
            if self.imu_sub is None:
                self.get_logger().info('⚠️  IMU未启用，pitch使用默认值0（假设机器人已平衡）')
            else:
                self.get_logger().info(f'平衡阈值: ±{math.degrees(self.balance_threshold):.2f}°')
            self.get_logger().info(f'稳定时间: {self.balance_stable_time}秒')
            self.get_logger().info('请确保机器人处于平衡状态')
            self.get_logger().info('='*60)
    
    def load_config(self, config_file: Optional[str] = None):
        """加载配置参数"""
        # 默认配置
        self.config = {
            'joint_names': {
                'left_front': 'jIJ',
                'left_rear': 'jIO',
                'right_front': 'jAB',
                'right_rear': 'jAG'
            },
            'l1': 0.215,
            'l2': 0.258,
            'l3': 0.258,
            'l4': 0.215,
            'l5': 0.0,
            'left_front_joint_offset': 0.003,
            'left_rear_joint_offset': -1.3,
            'right_front_joint_offset': 0.027,
            'right_rear_joint_offset': 1.3,
            'auto_calibrate': False,
            'balance_threshold': 0.05,  # 5度
            'balance_stable_time': 2.0,  # 2秒
            'enable_imu': False,  # 默认关闭IMU监听
            'max_torque': 90.0,
            'imu_topic': '/imu/data',
            'force_command_topic': '/vmc_controller/force_command',
            'print_frequency': 10
        }
        
        # 从配置文件加载
        if config_file and os.path.exists(config_file):
            with open(config_file, 'r') as f:
                yaml_config = yaml.safe_load(f)
                if 'vmc_controller' in yaml_config and 'ros__parameters' in yaml_config['vmc_controller']:
                    params = yaml_config['vmc_controller']['ros__parameters']
                    # 更新配置
                    for key in ['l1', 'l2', 'l3', 'l4', 'l5',
                               'left_front_joint_offset', 'left_rear_joint_offset',
                               'right_front_joint_offset', 'right_rear_joint_offset',
                               'max_torque', 'imu_topic', 'force_command_topic',
                               'auto_calibrate', 'balance_threshold', 'balance_stable_time', 'enable_imu']:
                        if key in params:
                            self.config[key] = params[key]
                    # 更新关节名称
                    if 'left_front_joint_name' in params:
                        self.config['joint_names']['left_front'] = params['left_front_joint_name']
                    if 'left_rear_joint_name' in params:
                        self.config['joint_names']['left_rear'] = params['left_rear_joint_name']
                    if 'right_front_joint_name' in params:
                        self.config['joint_names']['right_front'] = params['right_front_joint_name']
                    if 'right_rear_joint_name' in params:
                        self.config['joint_names']['right_rear'] = params['right_rear_joint_name']
        
        # 提取参数
        self.l1 = self.config['l1']
        self.l2 = self.config['l2']
        self.l3 = self.config['l3']
        self.l4 = self.config['l4']
        self.l5 = self.config['l5']
        self.left_front_joint_offset = self.config['left_front_joint_offset']
        self.left_rear_joint_offset = self.config['left_rear_joint_offset']
        self.right_front_joint_offset = self.config['right_front_joint_offset']
        self.right_rear_joint_offset = self.config['right_rear_joint_offset']
        self.max_torque = self.config['max_torque']
        self.joint_names = self.config['joint_names']
    
    def joint_state_callback(self, msg: JointState):
        """关节状态回调"""
        self.received_joint_states = True
        self.last_joint_time = msg.header.stamp
        
        # 提取关节位置
        for i, name in enumerate(msg.name):
            if name in [self.joint_names['left_front'], self.joint_names['left_rear'],
                       self.joint_names['right_front'], self.joint_names['right_rear']]:
                self.joint_positions[name] = msg.position[i]
    
    def imu_callback(self, msg: Imu):
        """IMU回调 - 提取pitch和pitch_gyro（与控制器逻辑一致）"""
        self.received_imu = True
        self.last_imu_time = msg.header.stamp
        
        # 使用与控制器相同的四元数到pitch转换
        self.pitch = self.quaternion_to_pitch(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        self.pitch_gyro = msg.angular_velocity.y
        
        # 自动校准模式：检测平衡状态
        if self.auto_calibrate_mode and not self.calibration_complete:
            self.detect_balance_state()
    
    def force_command_callback(self, msg: Float64MultiArray):
        """力命令回调"""
        self.received_force_command = True
        
        if len(msg.data) >= 4:
            self.left_F0 = msg.data[0]
            self.left_Tp = msg.data[1]
            self.right_F0 = msg.data[2]
            self.right_Tp = msg.data[3]
        else:
            self.get_logger().warn(f'力命令数据长度不足: {len(msg.data)}, 期望4')
    
    def quaternion_to_pitch(self, x: float, y: float, z: float, w: float) -> float:
        """四元数转pitch角度（与控制器逻辑一致）"""
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        return pitch
    
    def is_valid_number(self, value: float, name: str = "") -> Tuple[bool, str]:
        """检查数值有效性"""
        if math.isnan(value):
            return False, f"{name} is NaN"
        if math.isinf(value):
            return False, f"{name} is Inf"
        if abs(value) > 1e10:
            return False, f"{name} is too large: {value}"
        return True, ""
    
    def verify_and_print(self):
        """执行VMC计算并验证"""
        # 检查是否有足够的数据
        if not self.received_joint_states:
            self.get_logger().warn('等待关节状态数据...')
            return
        
        # IMU检查：如果没有IMU数据，使用默认值pitch=0
        if not self.received_imu:
            self.pitch = 0.0
            self.pitch_gyro = 0.0
            self.get_logger().debug('未接收到IMU数据，使用默认值 pitch=0')
        
        # 检查关节位置是否完整
        required_joints = [
            self.joint_names['left_front'],
            self.joint_names['left_rear'],
            self.joint_names['right_front'],
            self.joint_names['right_rear']
        ]
        
        missing_joints = [j for j in required_joints if j not in self.joint_positions]
        if missing_joints:
            self.get_logger().warn(f'缺少关节位置: {missing_joints}')
            return
        
        # 计算时间步长
        current_time = self.get_clock().now()
        if self.last_update_time is None:
            dt = 0.002  # 默认500Hz
        else:
            dt = (current_time - self.last_update_time).nanoseconds / 1e9
            if dt <= 0.0 or dt > 0.1:
                dt = 0.002
        self.last_update_time = current_time
        
        # 提取关节位置并应用偏移量（与控制器逻辑一致）
        left_front_pos_raw = self.joint_positions[self.joint_names['left_front']]
        left_rear_pos_raw = self.joint_positions[self.joint_names['left_rear']]
        right_front_pos_raw = self.joint_positions[self.joint_names['right_front']]
        right_rear_pos_raw = self.joint_positions[self.joint_names['right_rear']]
        
        left_front_pos = left_front_pos_raw + self.left_front_joint_offset
        left_rear_pos = left_rear_pos_raw + self.left_rear_joint_offset
        right_front_pos = right_front_pos_raw + self.right_front_joint_offset
        right_rear_pos = right_rear_pos_raw + self.right_rear_joint_offset
        
        # 更新关节角度（注意：与控制器逻辑一致）
        # 根据关节名称判断机器人类型
        # COD-2026RoboMaster-Balance: 关节名称包含 "Left_front_joint", "Right_front_joint" 等
        # wheel_leg_mujoco: 关节名称是 "jIJ", "jIO", "jAB", "jAG" 等
        is_cod_robot = any(name in ['Left_front_joint', 'Left_rear_joint', 
                                     'Right_front_joint', 'Right_rear_joint'] 
                          for name in self.joint_names.values())
        
        if is_cod_robot:
            # COD-2026RoboMaster-Balance 映射（与 vmc_controller.cpp 一致）
            # 左腿：phi1 = left_rear + π, phi4 = left_front
            # 右腿：phi1 = right_front + π, phi4 = right_rear
            self.left_leg.phi1 = math.pi + left_rear_pos   # Left_rear_joint -> phi1
            self.left_leg.phi4 = left_front_pos            # Left_front_joint -> phi4 (不加π)
            self.right_leg.phi1 = math.pi + right_front_pos  # Right_front_joint -> phi1
            self.right_leg.phi4 = right_rear_pos          # Right_rear_joint -> phi4 (不加π)
        else:
            # wheel_leg_mujoco 映射（与 Simulation.py 逻辑一致）
            # phi1需要加π，但phi4不需要加π
            # Simulation.py中：右腿使用jAB(phi1)和jAG(phi4)，左腿使用jIO(phi1)和jIJ(phi4)
            # 控制器中左右腿交换：左腿使用jAB和jAG，右腿使用jIO和jIJ
            self.left_leg.phi1 = math.pi + right_front_pos  # jAB -> phi1
            self.left_leg.phi4 = right_rear_pos             # jAG -> phi4 (不加π)
            self.right_leg.phi1 = math.pi + left_rear_pos   # jIO -> phi1 (注意：使用left_rear，不是left_front)
            self.right_leg.phi4 = left_front_pos            # jIJ -> phi4 (不加π，注意：使用left_front，不是left_rear)
        
        # 设置F0和Tp
        self.left_leg.F0 = self.left_F0
        self.left_leg.Tp = self.left_Tp
        self.right_leg.F0 = self.right_F0
        self.right_leg.Tp = self.right_Tp
        
        # 执行VMC计算
        # 左腿
        self.left_leg.vmc_calc_pos(
            dt=dt,
            phi1=self.left_leg.phi1,
            phi4=self.left_leg.phi4,
            pitch=self.pitch,
            gyro=self.pitch_gyro
        )
        self.left_leg.vmc_calc_torque()
        
        # 右腿
        self.right_leg.vmc_calc_pos(
            dt=dt,
            phi1=self.right_leg.phi1,
            phi4=self.right_leg.phi4,
            pitch=self.pitch,
            gyro=self.pitch_gyro
        )
        self.right_leg.vmc_calc_torque()
        
        # 获取计算结果
        left_front_torque = self.left_leg.torque_set[1]  # Front关节力矩
        left_rear_torque = self.left_leg.torque_set[0]   # Rear关节力矩
        right_front_torque = self.right_leg.torque_set[1]  # Front关节力矩
        right_rear_torque = self.right_leg.torque_set[0]   # Rear关节力矩
        
        # 限制力矩
        left_front_torque = max(-self.max_torque, min(self.max_torque, left_front_torque))
        left_rear_torque = max(-self.max_torque, min(self.max_torque, left_rear_torque))
        right_front_torque = max(-self.max_torque, min(self.max_torque, right_front_torque))
        right_rear_torque = max(-self.max_torque, min(self.max_torque, right_rear_torque))
        
        # 验证数值有效性
        errors = []
        warnings = []
        
        # 验证中间值
        for leg_name, leg in [('Left', self.left_leg), ('Right', self.right_leg)]:
            # L0验证
            valid, msg = self.is_valid_number(leg.L0, f'{leg_name} L0')
            if not valid:
                errors.append(msg)
            
            # phi0验证
            valid, msg = self.is_valid_number(leg.phi0, f'{leg_name} phi0')
            if not valid:
                errors.append(msg)
            
            # theta验证
            valid, msg = self.is_valid_number(leg.theta, f'{leg_name} theta')
            if not valid:
                errors.append(msg)
            
            # d_theta验证
            valid, msg = self.is_valid_number(leg.d_theta, f'{leg_name} d_theta')
            if not valid:
                errors.append(msg)
            
            # 雅可比矩阵系数验证
            for j_name, j_val in [('j11', leg.j11), ('j12', leg.j12),
                                 ('j21', leg.j21), ('j22', leg.j22)]:
                valid, msg = self.is_valid_number(j_val, f'{leg_name} {j_name}')
                if not valid:
                    errors.append(msg)
        
        # 验证最终力矩
        for name, torque in [('Left Front', left_front_torque),
                           ('Left Rear', left_rear_torque),
                           ('Right Front', right_front_torque),
                           ('Right Rear', right_rear_torque)]:
            valid, msg = self.is_valid_number(torque, f'{name} Torque')
            if not valid:
                errors.append(msg)
            elif abs(torque) > self.max_torque * 0.95:
                warnings.append(f'{name} Torque接近限制: {torque:.3f} Nm')
        
        # 打印结果
        self.print_counter += 1
        if self.print_counter % (self.print_frequency // 2) == 0:  # 每0.5秒打印一次详细信息
            self.print_detailed_results(
                left_front_pos_raw, left_rear_pos_raw,
                right_front_pos_raw, right_rear_pos_raw,
                left_front_pos, left_rear_pos,
                right_front_pos, right_rear_pos,
                left_front_torque, left_rear_torque,
                right_front_torque, right_rear_torque,
                errors, warnings
            )
    
    def print_detailed_results(self, left_front_raw, left_rear_raw,
                              right_front_raw, right_rear_raw,
                              left_front, left_rear,
                              right_front, right_rear,
                              left_front_torque, left_rear_torque,
                              right_front_torque, right_rear_torque,
                              errors, warnings):
        """打印详细的计算结果"""
        timestamp = self.get_clock().now()
        
        print("\n" + "="*80)
        print(f"时间戳: {timestamp}")
        print("="*80)
        
        # 输入数据摘要
        print("\n【输入数据】")
        print(f"  连杆长度 (原始):")
        print(f"    l1: {self.l1:8.4f} m")
        print(f"    l2: {self.l2:8.4f} m")
        print(f"    l3: {self.l3:8.4f} m")
        print(f"    l4: {self.l4:8.4f} m")
        print(f"    l5: {self.l5:8.4f} m")
        print(f"  关节位置 (原始):")
        print(f"    {self.joint_names['left_front']}: {left_front_raw:8.4f} rad")
        print(f"    {self.joint_names['left_rear']}: {left_rear_raw:8.4f} rad")
        print(f"    {self.joint_names['right_front']}: {right_front_raw:8.4f} rad")
        print(f"    {self.joint_names['right_rear']}: {right_rear_raw:8.4f} rad")
        print(f"  关节位置 (应用偏移后):")
        print(f"    Left Front: {left_front:8.4f} rad (offset: {self.left_front_joint_offset:8.4f})")
        print(f"    Left Rear:  {left_rear:8.4f} rad (offset: {self.left_rear_joint_offset:8.4f})")
        print(f"    Right Front: {right_front:8.4f} rad (offset: {self.right_front_joint_offset:8.4f})")
        print(f"    Right Rear:  {right_rear:8.4f} rad (offset: {self.right_rear_joint_offset:8.4f})")
        print(f"  Pitch: {self.pitch:8.4f} rad ({math.degrees(self.pitch):7.2f} deg)")
        print(f"  Pitch Gyro: {self.pitch_gyro:8.4f} rad/s")
        print(f"  力命令:")
        print(f"    Left:  F0={self.left_F0:8.3f} N,  Tp={self.left_Tp:8.3f} Nm")
        print(f"    Right: F0={self.right_F0:8.3f} N,  Tp={self.right_Tp:8.3f} Nm")
        
        # 中间计算值
        print("\n【中间计算值 - 左腿】")
        print(f"  L0:     {self.left_leg.L0:8.4f} m")
        print(f"  phi0:   {self.left_leg.phi0:8.4f} rad ({math.degrees(self.left_leg.phi0):7.2f} deg)")
        print(f"  theta:  {self.left_leg.theta:8.4f} rad ({math.degrees(self.left_leg.theta):7.2f} deg)")
        print(f"  d_theta: {self.left_leg.d_theta:8.4f} rad/s")
        print(f"  雅可比矩阵:")
        print(f"    j11: {self.left_leg.j11:10.6f}")
        print(f"    j12: {self.left_leg.j12:10.6f}")
        print(f"    j21: {self.left_leg.j21:10.6f}")
        print(f"    j22: {self.left_leg.j22:10.6f}")
        
        print("\n【中间计算值 - 右腿】")
        print(f"  L0:     {self.right_leg.L0:8.4f} m")
        print(f"  phi0:   {self.right_leg.phi0:8.4f} rad ({math.degrees(self.right_leg.phi0):7.2f} deg)")
        print(f"  theta:  {self.right_leg.theta:8.4f} rad ({math.degrees(self.right_leg.theta):7.2f} deg)")
        print(f"  d_theta: {self.right_leg.d_theta:8.4f} rad/s")
        print(f"  雅可比矩阵:")
        print(f"    j11: {self.right_leg.j11:10.6f}")
        print(f"    j12: {self.right_leg.j12:10.6f}")
        print(f"    j21: {self.right_leg.j21:10.6f}")
        print(f"    j22: {self.right_leg.j22:10.6f}")
        
        # 最终力矩
        print("\n【最终力矩计算结果】")
        print(f"  Left Front:  {left_front_torque:8.3f} Nm")
        print(f"  Left Rear:   {left_rear_torque:8.3f} Nm")
        print(f"  Right Front: {right_front_torque:8.3f} Nm")
        print(f"  Right Rear:  {right_rear_torque:8.3f} Nm")
        print(f"  最大力矩限制: ±{self.max_torque:.1f} Nm")
        
        # 错误和警告
        if errors:
            print("\n【错误】")
            for error in errors:
                print(f"  ❌ {error}")
        
        if warnings:
            print("\n【警告】")
            for warning in warnings:
                print(f"  ⚠️  {warning}")
        
        if not errors and not warnings:
            print("\n【状态】✅ 所有计算值正常")
        
        # 自动校准模式：显示校准进度
        if self.auto_calibrate_mode and not self.calibration_complete:
            self.print_calibration_status()
        
        print("="*80 + "\n")
    
    def detect_balance_state(self):
        """检测机器人是否处于平衡状态"""
        current_time = self.get_clock().now()
        
        # 如果没有IMU数据，假设pitch=0（平衡状态）
        if not self.received_imu:
            self.pitch = 0.0
            self.pitch_gyro = 0.0
        
        # 检查pitch角度是否在阈值内（如果没有IMU，pitch=0总是满足条件）
        if abs(self.pitch) < self.balance_threshold and abs(self.pitch_gyro) < 0.1:
            # 记录平衡样本
            if self.received_joint_states and len(self.joint_positions) == 4:
                sample = {
                    'time': current_time,
                    'pitch': self.pitch,
                    'pitch_gyro': self.pitch_gyro,
                    'joints': {
                        self.joint_names['left_front']: self.joint_positions.get(self.joint_names['left_front'], 0),
                        self.joint_names['left_rear']: self.joint_positions.get(self.joint_names['left_rear'], 0),
                        self.joint_names['right_front']: self.joint_positions.get(self.joint_names['right_front'], 0),
                        self.joint_names['right_rear']: self.joint_positions.get(self.joint_names['right_rear'], 0),
                    }
                }
                self.balance_samples.append(sample)
                
                # 检查是否稳定足够长时间
                if len(self.balance_samples) >= 50:  # 至少50个样本
                    if self.balance_start_time is None:
                        self.balance_start_time = current_time
                    
                    elapsed_time = (current_time - self.balance_start_time).nanoseconds / 1e9
                    if elapsed_time >= self.balance_stable_time:
                        self.calculate_offsets()
        else:
            # 不平衡，重置
            self.balance_samples.clear()
            self.balance_start_time = None
    
    def calculate_offsets(self):
        """根据平衡状态下的关节位置计算偏移值"""
        if len(self.balance_samples) < 50:
            return
        
        # 计算平均关节位置
        avg_joints = {
            'left_front': 0.0,
            'left_rear': 0.0,
            'right_front': 0.0,
            'right_rear': 0.0
        }
        avg_pitch = 0.0
        
        for sample in self.balance_samples:
            avg_joints['left_front'] += sample['joints'][self.joint_names['left_front']]
            avg_joints['left_rear'] += sample['joints'][self.joint_names['left_rear']]
            avg_joints['right_front'] += sample['joints'][self.joint_names['right_front']]
            avg_joints['right_rear'] += sample['joints'][self.joint_names['right_rear']]
            avg_pitch += sample['pitch']
        
        n = len(self.balance_samples)
        for key in avg_joints:
            avg_joints[key] /= n
        avg_pitch /= n
        
        # 根据VMC算法要求计算偏移值
        # 
        # VMC期望的零点：
        #   - phi1 ≈ π (前关节)
        #   - phi4 ≈ 0 (后关节)
        #
        # 根据Simulation.py的逻辑：
        #   右腿：phi1 = joint_pos[0] + π (jAB), phi4 = joint_pos[1] (jAG)
        #   左腿：phi1 = joint_pos[3] + π (jIO), phi4 = joint_pos[2] (jIJ)
        #
        # 但控制器中左右腿交换了（见vmc_controller.cpp）：
        #   左腿使用：jAB (phi1), jAG (phi4)
        #   右腿使用：jIJ (phi1), jIO (phi4)
        #
        # 偏移值的计算公式：
        #   在environment.py中：joint_pos = sensor_raw + offset
        #   在Simulation.py中：phi1 = joint_pos + π, phi4 = joint_pos
        #
        # 要使VMC得到期望的零点：
        #   - phi1 = π: 需要 joint_pos = 0，即 sensor_raw + offset = 0，所以 offset = -sensor_raw
        #   - phi4 = 0: 需要 joint_pos = 0，即 sensor_raw + offset = 0，所以 offset = -sensor_raw
        #
        # 因此，偏移值 = -传感器原始读数（当机器人处于期望平衡姿态时）
        
        # 注意：这里的avg_joints已经是传感器原始读数（未应用偏移）
        # 所以偏移值就是这些值的负值
        recommended_offsets = {
            'left_front': -avg_joints['left_front'],   # jIJ -> 右腿phi4，期望0
            'left_rear': -avg_joints['left_rear'],     # jIO -> 右腿phi1，期望π（但偏移值计算时不需要π）
            'right_front': -avg_joints['right_front'], # jAB -> 左腿phi1，期望π（但偏移值计算时不需要π）
            'right_rear': -avg_joints['right_rear']    # jAG -> 左腿phi4，期望0
        }
        
        # 对于phi1的关节，需要额外减去π（因为phi1 = sensor + offset + π）
        # 但根据Simulation.py，phi1的关节在计算时已经加了π
        # 所以对于phi1关节：offset应该使 sensor + offset = 0，即 offset = -sensor
        # 对于phi4关节：offset应该使 sensor + offset = 0，即 offset = -sensor
        
        # 但是，我们需要考虑Simulation.py中的偏移值
        # Simulation.py中：右前+0.027, 右后+1.3, 左前+0.003, 左后-1.3
        # 这些偏移值是为了将传感器读数转换为VMC坐标系
        
        # 实际上，我们需要的是：当机器人平衡时，传感器读数应该是什么
        # 然后偏移值 = -传感器读数（使偏移后的值为0）
        
        self.print_calibration_results(avg_joints, avg_pitch, recommended_offsets)
        self.calibration_complete = True
    
    def print_calibration_status(self):
        """打印校准状态"""
        if len(self.balance_samples) > 0:
            elapsed = 0.0
            if self.balance_start_time:
                elapsed = (self.get_clock().now() - self.balance_start_time).nanoseconds / 1e9
            
            progress = min(100, (elapsed / self.balance_stable_time) * 100)
            print(f"\n【校准进度】样本数: {len(self.balance_samples)}, "
                  f"稳定时间: {elapsed:.2f}/{self.balance_stable_time:.2f}秒 ({progress:.1f}%)")
            print(f"  Pitch: {math.degrees(self.pitch):.2f}°, "
                  f"Pitch Gyro: {math.degrees(self.pitch_gyro):.2f}°/s")
    
    def print_calibration_results(self, avg_joints, avg_pitch, recommended_offsets):
        """打印校准结果"""
        print("\n" + "="*80)
        print("🎯 偏移值自动校准完成！")
        print("="*80)
        
        print("\n【平衡状态下的平均关节位置】")
        print(f"  {self.joint_names['left_front']}:  {avg_joints['left_front']:8.6f} rad ({math.degrees(avg_joints['left_front']):7.3f}°)")
        print(f"  {self.joint_names['left_rear']}:   {avg_joints['left_rear']:8.6f} rad ({math.degrees(avg_joints['left_rear']):7.3f}°)")
        print(f"  {self.joint_names['right_front']}: {avg_joints['right_front']:8.6f} rad ({math.degrees(avg_joints['right_front']):7.3f}°)")
        print(f"  {self.joint_names['right_rear']}:  {avg_joints['right_rear']:8.6f} rad ({math.degrees(avg_joints['right_rear']):7.3f}°)")
        print(f"  平均Pitch: {math.degrees(avg_pitch):7.3f}°")
        
        print("\n【推荐的偏移值配置（YAML格式）】")
        print("vmc_controller:")
        print("  ros__parameters:")
        print(f"    left_front_joint_offset: {recommended_offsets['left_front']:.6f}  # {self.joint_names['left_front']}")
        print(f"    left_rear_joint_offset: {recommended_offsets['left_rear']:.6f}  # {self.joint_names['left_rear']}")
        print(f"    right_front_joint_offset: {recommended_offsets['right_front']:.6f}  # {self.joint_names['right_front']}")
        print(f"    right_rear_joint_offset: {recommended_offsets['right_rear']:.6f}  # {self.joint_names['right_rear']}")
        
        print("\n【说明】")
        print("  这些偏移值是基于当前平衡状态下的关节位置计算得出的。")
        print("  使用这些偏移值后，当机器人处于相同姿态时，VMC算法将得到期望的零点。")
        print("  注意：这些值可能需要根据实际VMC算法要求进行微调。")
        print("="*80 + "\n")


def main(args=None):
    """主函数"""
    parser = argparse.ArgumentParser(description='VMC计算验证脚本')
    parser.add_argument(
        '--config',
        type=str,
        default=None,
        help='配置文件路径（YAML格式，包含vmc_controller配置）'
    )
    parser.add_argument(
        '--print-frequency',
        type=float,
        default=10.0,
        help='打印频率（Hz），默认10Hz'
    )
    parser.add_argument(
        '--auto-calibrate',
        action='store_true',
        help='启用自动校准模式，通过监听话题自动确定偏移值'
    )
    parser.add_argument(
        '--balance-threshold',
        type=float,
        default=0.05,
        help='平衡状态检测的pitch角度阈值（弧度），默认0.05 rad (约2.9°)'
    )
    parser.add_argument(
        '--balance-stable-time',
        type=float,
        default=2.0,
        help='平衡状态需要保持的稳定时间（秒），默认2.0秒'
    )
    
    args = parser.parse_args()
    
    rclpy.init(args=sys.argv)
    
    verifier = VMCVerifier(config_file=args.config)
    
    # 应用命令行参数
    if args.print_frequency:
        verifier.print_frequency = args.print_frequency
        verifier.timer.cancel()
        timer_period = 1.0 / verifier.print_frequency
        verifier.timer = verifier.create_timer(timer_period, verifier.verify_and_print)
    
    if args.auto_calibrate:
        verifier.auto_calibrate_mode = True
        verifier.balance_threshold = args.balance_threshold
        verifier.balance_stable_time = args.balance_stable_time
        verifier.get_logger().info('自动校准模式已通过命令行启用')
    
    try:
        rclpy.spin(verifier)
    except KeyboardInterrupt:
        pass
    finally:
        verifier.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

