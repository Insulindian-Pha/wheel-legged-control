#!/usr/bin/env python3
"""
诊断VMC控制器offset配置问题的工具脚本
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import sys
import time


class OffsetDiagnostic(Node):
    def __init__(self):
        super().__init__('offset_diagnostic')
        self.joint_positions = {}
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('VMC控制器Offset诊断工具')
        self.get_logger().info('=' * 70)
        self.get_logger().info('正在监听 /joint_states 话题...')
        self.get_logger().info('请确保：')
        self.get_logger().info('  1. 机器人系统已启动')
        self.get_logger().info('  2. VMC控制器已加载')
        self.get_logger().info('  3. 机器人处于期望的初始位置（零点位置）')
        self.get_logger().info('=' * 70)
        
    def joint_state_callback(self, msg):
        # 查找VMC控制器使用的关节
        vmc_joints = {
            'Left_front_joint': None,
            'Left_rear_joint': None,
            'Right_front_joint': None,
            'Right_rear_joint': None
        }
        
        for i, joint_name in enumerate(msg.name):
            if joint_name in vmc_joints:
                vmc_joints[joint_name] = msg.position[i]
        
        # 检查是否所有关节都找到了
        if all(v is not None for v in vmc_joints.values()):
            self.get_logger().info('\n' + '=' * 70)
            self.get_logger().info('当前关节位置（原始值）:')
            self.get_logger().info('-' * 70)
            
            for joint_name, position in vmc_joints.items():
                self.get_logger().info(f'  {joint_name}: {position:.6f} rad ({position * 180 / 3.14159:.2f} deg)')
            
            self.get_logger().info('\n' + '-' * 70)
            self.get_logger().info('如果offset配置正确，计算后的位置应该接近0:')
            self.get_logger().info('-' * 70)
            self.get_logger().info('计算公式: pos_after_offset = pos_raw + offset')
            self.get_logger().info('要使 pos_after_offset = 0，需要: offset = -pos_raw')
            self.get_logger().info('\n建议的offset值:')
            self.get_logger().info('-' * 70)
            
            offset_mapping = {
                'Left_front_joint': 'left_front_joint_offset',
                'Left_rear_joint': 'left_rear_joint_offset',
                'Right_front_joint': 'right_front_joint_offset',
                'Right_rear_joint': 'right_rear_joint_offset'
            }
            
            for joint_name, position in vmc_joints.items():
                suggested_offset = -position
                config_name = offset_mapping[joint_name]
                self.get_logger().info(f'  {config_name}: {suggested_offset:.6f}  # {joint_name}')
            
            self.get_logger().info('\n' + '=' * 70)
            self.get_logger().info('请检查：')
            self.get_logger().info('  1. 配置文件 cod_balance_controllers.yaml 中的offset值')
            self.get_logger().info('  2. 控制器启动日志中的offset值（应该与配置文件一致）')
            self.get_logger().info('  3. 如果修改了配置文件，需要重新编译和重启控制器')
            self.get_logger().info('=' * 70)
            
            # 只输出一次
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    
    node = OffsetDiagnostic()
    
    try:
        rclpy.spin_once(node, timeout_sec=10.0)
        if rclpy.ok():
            node.get_logger().warn('\n10秒内未收到关节状态数据，请检查：')
            node.get_logger().warn('  1. 机器人系统是否已启动')
            node.get_logger().warn('  2. /joint_states 话题是否存在')
            node.get_logger().warn('  3. 关节名称是否正确')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


