#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import os
import time
import math

# 添加src到路径中
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

# 导入重构后的模块
from modules.ctrl_robot import CtrlRobot
from utils.helpers import read_conf, calc_dist, calc_target_speed

def imu_walk_dist(robot_base, target_dist):
    print(f"=== 基于里程计的行走{target_dist}米演示 ===")
    
    print("等待里程计数据...")
    timeout_cnt = 0
    while robot_base.get_current_position() is None and timeout_cnt < 100:
        time.sleep(0.1)
        timeout_cnt += 1
    
    if robot_base.get_current_position() is None:
        print("无法获取里程计数据，退出演示")
        return False
    
    print("里程计数据正常")
    
    try:
        print("\n1. 确保机器人站立...")
        robot_base.stand(duration=3000, sleep=3.0)

        if not robot_base.reset_odometry_start_position():
            print("⚠️ IMU里程计初始化失败")
            return False
        
        start_pos = robot_base._v
        print(f"✅ 起始位置: ({start_pos[0]:.3f}, {start_pos[1]:.3f})")

        print("\n2. 开始动态调速连续行走...")
        monitor_cnt = 0
        last_cmd_time = 0
        
        while True:
            monitor_cnt += 1
            
            distance = robot_base.get_traveled_distance()
            remaining = target_dist - distance
            progress = (distance / target_dist) * 100

            target_speed = calc_target_speed(remaining, 0)

            print(f"\r监控 #{monitor_cnt}: 距离 {distance:.3f}m ({progress:.1f}%) "
                  f"剩余 {remaining:.3f}m 目标速度 {target_speed:.2f}m/s", end="")
            
            if distance >= target_dist:
                print(f"\n🎯 到达目标距离! 最终距离: {distance:.3f}m")
                break
            
            current_time = time.time()
            if current_time - last_cmd_time > 0.2:
                if target_speed > 0.0:
                    robot_base._d.mode = 11
                    robot_base._d.gait_id = 27
                    robot_base._d.vel_des = [target_speed, 0, 0]
                    robot_base._d.step_height = [0.05,0.05]
                    robot_base._d.duration = 0
                    robot_base._d.life_count = (robot_base._d.life_count + 1) % 128
                    robot_base.send_cmd(robot_base._d)
                    last_cmd_time = current_time
                else:
                    break
            
            time.sleep(0.05)
            
        print("\n3. 停止机器人...")
        robot_base._d.mode = 12
        robot_base._d.gait_id = 0
        robot_base._d.vel_des = [0, 0, 0]
        robot_base._d.duration = 0
        robot_base._d.life_count = (robot_base._d.life_count + 1) % 128
        robot_base.send_cmd(robot_base._d)
        time.sleep(0.5)
        
        return True
        
    except KeyboardInterrupt:
        print("\n⚠️ 用户中断演示")
        return False
    except Exception as e:
        print(f"\n❌ 演示过程中发生错误: {e}")
        return False

def main():
    print("A库任务脚本")
    print("专门执行A库相关的操作：到达A库、装货、离开A库")
    
    config = read_conf()
    print(f"A库配置: {config['a_wh']}")
    print(f"B库配置: {config['b_wh']}")
    print(f"箭头配置: {config['arr']}")
    
    robot = CtrlRobot()
    robot.start()
    
    # 执行行走任务
    success = imu_walk_dist(robot, 1.0)
    
    if success:
        print("✅ 任务完成")
    else:
        print("❌ 任务失败")

if __name__ == "__main__":
    main()