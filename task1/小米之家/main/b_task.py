#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
B库任务脚本
"""

import sys
import os
import time
import math

# 添加src到路径中
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

# 导入重构后的模块
from modules.ctrl_robot import CtrlRobot
from utils.helpers import read_conf, calc_dist, calc_target_speed

def _a(robot_base, target_dist):
    """
    B库行走逻辑
    """
    print(f"=== B库行走任务 {target_dist}米 ===")
    
    print("等待里程计数据...")
    _b = 0
    while robot_base.get_current_position() is None and _b < 100:
        time.sleep(0.1)
        _b += 1
    
    if robot_base.get_current_position() is None:
        print("❌ 无法获取里程计数据")
        return False
    
    print("✅ 里程计数据正常")
    
    try:
        print("\n1. 机器人站立准备...")
        robot_base.stand(duration=3000, sleep=3.0)

        if not robot_base.reset_odometry_start_position():
            print("⚠️ 里程计初始化失败")
            return False
        
        _c = robot_base._v
        print(f"📍 起始位置: ({_c[0]:.3f}, {_c[1]:.3f})")

        print("\n2. 开始行走...")
        _d = 0
        _e = 0
        
        while True:
            _d += 1
            
            _f = robot_base.get_traveled_distance()
            _g = target_dist - _f
            _h = (_f / target_dist) * 100

            _i = calc_target_speed(_g, 0)

            print(f"\r监控 #{_d}: 距离 {_f:.3f}m ({_h:.1f}%) "
                  f"剩余 {_g:.3f}m 目标速度 {_i:.2f}m/s", end="")
            
            if _f >= target_dist:
                print(f"\n🎯 到达目标! 最终距离: {_f:.3f}m")
                break
            
            _j = time.time()
            if _j - _e > 0.2:
                if _i > 0.0:
                    robot_base._d.mode = 11
                    robot_base._d.gait_id = 27
                    robot_base._d.vel_des = [_i, 0, 0]
                    robot_base._d.step_height = [0.05, 0.05]
                    robot_base._d.duration = 0
                    robot_base._d.life_count = (robot_base._d.life_count + 1) % 128
                    robot_base.send_cmd(robot_base._d)
                    _e = _j
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
        print("\n⚠️ 用户中断")
        return False
    except Exception as _k:
        print(f"\n❌ 错误: {_k}")
        return False

def main():
    print("B库任务脚本")
    print("执行B库相关操作")
    
    _l = read_conf()
    print(f"A库配置: {_l['a_wh']}")
    print(f"B库配置: {_l['b_wh']}")
    print(f"箭头配置: {_l['arr']}")
    
    _m = CtrlRobot()
    _m.start()
    
    # 执行行走任务
    _n = _a(_m, 1.5)
    
    if _n:
        print("✅ B库任务完成")
    else:
        print("❌ B库任务失败")

if __name__ == "__main__":
    main()