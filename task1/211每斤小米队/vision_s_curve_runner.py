#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
纯视觉S弯道独立运行脚本 (双目视觉)
"""

import lcm
import sys
import os
import time
import rclpy
from threading import Thread, Lock

# 导入LCM消息
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from robot_control_response_lcmt import robot_control_response_lcmt

# 导入功能模块
from modules import RobotController, CameraController

class Robot_Ctrl:
    """底层LCM通信控制器 (来自main.py的简化版)"""
    def __init__(self):
        self.send_thread = Thread(target=self.send_publish)
        self.lc_s = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        self.cmd_msg = robot_control_cmd_lcmt()
        self.send_lock = Lock()
        self.delay_cnt = 0
        self.runing = 1

    def run(self):
        self.send_thread.start()

    def send_publish(self):
        while self.runing:
            self.send_lock.acquire()
            if self.delay_cnt > 20:
                self.lc_s.publish("robot_control_cmd", self.cmd_msg.encode())
                self.delay_cnt = 0
            self.delay_cnt += 1
            self.send_lock.release()
            time.sleep(0.005)

    def Send_cmd(self, msg):
        self.send_lock.acquire()
        self.delay_cnt = 50
        self.cmd_msg = msg
        self.send_lock.release()

    def stand(self, duration=1000, sleep=1.1):
        self.cmd_msg.mode = 12
        self.cmd_msg.gait_id = 0
        self.cmd_msg.duration = duration
        self.cmd_msg.life_count += 1
        if self.cmd_msg.life_count > 127: self.cmd_msg.life_count = 0
        self.Send_cmd(self.cmd_msg)
        time.sleep(sleep)
        
    def quit(self):
        self.runing = 0
        if self.send_thread.is_alive(): self.send_thread.join()
        print("✅ Robot_Ctrl 已停止")


def run_vision_s_curve_mission():
    """执行纯视觉S弯道任务"""
    robot_base = None
    robot_controller = None
    
    try:
        # 1. 初始化
        print("=== 初始化纯视觉S弯道任务 ===")
        robot_base = Robot_Ctrl()
        robot_controller = RobotController(robot_base)
        
        # 2. 启动系统
        print("=== 启动系统 ===")
        robot_base.run()
        robot_controller.start()
        time.sleep(2) # 等待相机和IMU启动
        
        # 3. 站立准备
        print("=== 机器人站立 ===")
        robot_controller.stand(duration=3000, sleep=3.0)

        # 4. 执行S弯道（BEV+Stanley 更稳方案）
        print("=== 开始执行纯视觉S弯道(BEV+Stanley) ===")
        TOTAL_S_CURVE_DISTANCE = 60.0
        success = robot_controller.navigate_s_curve_bev_stanley(
            distance=TOTAL_S_CURVE_DISTANCE,
            v_max=0.10,
            k_stanley=0.9,
            alpha_slow=1.6,
            lane_width_m=0.73,
            max_wz=0.4,
            cycle_hz=10.0,
            conf_thr=0.5
        )
        if not success:
            raise Exception("纯视觉S弯道导航失败")

        print("✅ S弯道完成")

        # 5. 任务结束，站立
        print("=== 任务结束，恢复站立 ===")
        robot_controller.stand(duration=3000, sleep=3.0)

        print("🎉 纯视觉S弯道任务成功完成！")

    except KeyboardInterrupt:
        print("\n📛 收到中断信号，正在停止...")
    except Exception as e:
        print(f"❌ 任务执行过程中出现错误: {e}")
    finally:
        if robot_controller:
            robot_controller.stop()
        if robot_base:
            robot_base.quit()
        print("程序已退出")


def main(args=None):
    """主函数"""
    try:
        rclpy.init(args=args)
        run_vision_s_curve_mission()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main() 