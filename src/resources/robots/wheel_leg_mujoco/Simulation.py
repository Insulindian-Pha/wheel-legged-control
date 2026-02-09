import mujoco
import mujoco.viewer
import numpy as np
import time
import os
from datetime import datetime
from environment import *
from VMC import *
from keyboard import *
import math
from Controller import *

def print_detailed_results(vmc_r, vmc_l, GBC486, joint_torque):
    """打印详细的计算结果"""
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    
    # 获取关节位置（原始值，应用偏移前）
    right_front_raw = GBC486.joint_pos[0]
    right_rear_raw = GBC486.joint_pos[1]
    left_front_raw = GBC486.joint_pos[2]
    left_rear_raw = GBC486.joint_pos[3]
    
    # 获取应用偏移后的关节位置（Simulation.py中已经应用了偏移）
    right_front = GBC486.joint_pos[0] + math.pi  # 右前
    right_rear = GBC486.joint_pos[1]  # 右后
    left_front = GBC486.joint_pos[3] + math.pi  # 左前
    left_rear = GBC486.joint_pos[2]  # 左后
    
    # 获取最终力矩
    right_front_torque = joint_torque[0]  # vmc_r.torque_set[1]
    right_rear_torque = joint_torque[1]   # vmc_r.torque_set[0]
    left_front_torque = joint_torque[2]   # vmc_l.torque_set[0]
    left_rear_torque = joint_torque[3]    # vmc_l.torque_set[1]
    
    print("\n" + "="*80)
    print(f"时间戳: {timestamp}")
    print("="*80)
    
    # 输入数据摘要
    print("\n【输入数据】")
    print(f"  连杆长度 (原始):")
    print(f"    l1: {vmc_r.l1:8.4f} m")
    print(f"    l2: {vmc_r.l2:8.4f} m")
    print(f"    l3: {vmc_r.l3:8.4f} m")
    print(f"    l4: {vmc_r.l4:8.4f} m")
    print(f"    l5: {vmc_r.l5:8.4f} m")
    print(f"  关节位置 (原始):")
    print(f"    Right Front: {right_front_raw:8.4f} rad")
    print(f"    Right Rear:  {right_rear_raw:8.4f} rad")
    print(f"    Left Front:  {left_front_raw:8.4f} rad")
    print(f"    Left Rear:   {left_rear_raw:8.4f} rad")
    print(f"  关节位置 (应用偏移后):")
    print(f"    Right Front: {right_front:8.4f} rad (offset: {math.pi:8.4f})")
    print(f"    Right Rear:  {right_rear:8.4f} rad (offset: 0.0000)")
    print(f"    Left Front:  {left_front:8.4f} rad (offset: {math.pi:8.4f})")
    print(f"    Left Rear:   {left_rear:8.4f} rad (offset: 0.0000)")
    print(f"  Pitch: {GBC486.euler[1]:8.4f} rad ({math.degrees(GBC486.euler[1]):7.2f} deg)")
    print(f"  Pitch Gyro: {GBC486.gyro[1]:8.4f} rad/s")
    print(f"  力命令:")
    print(f"    Left:  F0={vmc_l.F0:8.3f} N,  Tp={vmc_l.Tp:8.3f} Nm")
    print(f"    Right: F0={vmc_r.F0:8.3f} N,  Tp={vmc_r.Tp:8.3f} Nm")
    
    # 中间计算值
    print("\n【中间计算值 - 左腿】")
    print(f"  L0:     {vmc_l.L0:8.4f} m")
    print(f"  phi0:   {vmc_l.phi0:8.4f} rad ({math.degrees(vmc_l.phi0):7.2f} deg)")
    print(f"  theta:  {vmc_l.theta:8.4f} rad ({math.degrees(vmc_l.theta):7.2f} deg)")
    print(f"  d_theta: {vmc_l.d_theta:8.4f} rad/s")
    print(f"  雅可比矩阵:")
    print(f"    j11: {vmc_l.j11:10.6f}")
    print(f"    j12: {vmc_l.j12:10.6f}")
    print(f"    j21: {vmc_l.j21:10.6f}")
    print(f"    j22: {vmc_l.j22:10.6f}")
    
    print("\n【中间计算值 - 右腿】")
    print(f"  L0:     {vmc_r.L0:8.4f} m")
    print(f"  phi0:   {vmc_r.phi0:8.4f} rad ({math.degrees(vmc_r.phi0):7.2f} deg)")
    print(f"  theta:  {vmc_r.theta:8.4f} rad ({math.degrees(vmc_r.theta):7.2f} deg)")
    print(f"  d_theta: {vmc_r.d_theta:8.4f} rad/s")
    print(f"  雅可比矩阵:")
    print(f"    j11: {vmc_r.j11:10.6f}")
    print(f"    j12: {vmc_r.j12:10.6f}")
    print(f"    j21: {vmc_r.j21:10.6f}")
    print(f"    j22: {vmc_r.j22:10.6f}")
    
    # 最终力矩
    print("\n【最终力矩计算结果】")
    print(f"  Left Front:  {left_front_torque:8.3f} Nm")
    print(f"  Left Rear:   {left_rear_torque:8.3f} Nm")
    print(f"  Right Front: {right_front_torque:8.3f} Nm")
    print(f"  Right Rear:  {right_rear_torque:8.3f} Nm")
    print("="*80 + "\n")

def main():
    
    TORQUE = 1  #为1时给力矩，为0是无力矩
    # 获取脚本所在目录，确保路径正确
    script_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(script_dir, 'MJCF', 'env.xml')
    GBC486 = LegWheelRobot(model_path)
    i = 0
    t1 = 1
    t2 = 4
    t3 = 20
    vmc_r = leg_VMC()
    vmc_l = leg_VMC()
    keyboard = KeyboardController()
    
    # 打印计数器，控制详细打印频率
    print_counter = 0
    print_frequency = 50  # 每50次VMC计算打印一次详细信息


    while True:
        i = i + 1
        
        # 执行仿真步
        GBC486.step()  # 仿真的timestep是1ms，意味着每执行一次step仿真世界时间过去1ms
        #传感器数据获取
        if i % t1 == 0: 
            GBC486.sensor_read_data()
        #vmc计算、观测器计算、LQR计算
        if i % t2 == 0:
            #正向运动学计算，获取状态
            vmc_r.vmc_calc_pos(phi1=GBC486.joint_pos[0]+math.pi,phi4=GBC486.joint_pos[1],pitch= GBC486.euler[1],gyro=GBC486.gyro[1])
            vmc_l.vmc_calc_pos(phi1=GBC486.joint_pos[3]+math.pi,phi4=GBC486.joint_pos[2],pitch=-GBC486.euler[1],gyro=-GBC486.gyro[1])
            vmc_r.F0 = 0
            vmc_l.F0 = 0
            vmc_r.Tp = 0
            vmc_l.Tp = 0
            
            vmc_l.vmc_calc_torque()
            vmc_r.vmc_calc_torque()
            # vmc.vmc_calc()
            w_r = 0
            w_l = 0
            GBC486.wheel_torque = [w_r,w_l]
            GBC486.joint_torque = [vmc_r.torque_set[1],vmc_r.torque_set[0],vmc_l.torque_set[0],vmc_l.torque_set[1]]
            
            # 打印详细结果（按频率控制）
            print_counter += 1
            if print_counter % print_frequency == 0:
                print_detailed_results(vmc_r, vmc_l, GBC486, GBC486.joint_torque)
            
            GBC486.actuator_set_torque()

        #键盘控制指令输入,以及打印数据;运行频率低以降低仿真延迟
        if i % t3 == 0:
            cmd = keyboard.get_command()
            # print(vmc_r.L0,vmc_l.L0)



if __name__ == '__main__':
    main()