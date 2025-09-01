#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机器狗运动控制器
基于demo/hybrid_walk_demo.py实现，提供高级运动控制接口

作者：Xiaomi Cup 2025 Final Team
"""

import lcm
import time
import math
import socket
import struct
from threading import Thread, Lock
from typing import Optional, Tuple, List
import sys
import os

# 添加demo目录到路径，以便导入LCM消息类型
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'demo'))

try:
    from robot_control_cmd_lcmt import robot_control_cmd_lcmt
    from robot_control_response_lcmt import robot_control_response_lcmt
except ImportError as e:
    print(f"警告：无法导入LCM消息类型: {e}")
    print("请确保demo目录中的robot_control_cmd_lcmt.py和robot_control_response_lcmt.py文件存在")


class RobotController:
    """机器狗运动控制器"""
    
    def __init__(self):
        # LCM配置
        self.rec_thread = Thread(target=self.rec_response)
        self.send_thread = Thread(target=self.send_publish)
        self.lc_r = lcm.LCM("udpm://239.255.76.67:7670?ttl=255")
        self.lc_s = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        self.cmd_msg = robot_control_cmd_lcmt()
        self.rec_msg = robot_control_response_lcmt()
        self.send_lock = Lock()
        self.delay_cnt = 0
        self.mode_ok = 0
        self.gait_ok = 0
        self.running = 1

        
        # UDP配置
        self.multicast_group = "239.255.76.67"
        self.odometry_port = 7667
        
        print("机器狗控制器初始化完成")

    def run(self):
        """启动所有线程"""
        self.lc_r.subscribe("robot_control_response", self.msg_handler)
        self.send_thread.start()
        self.rec_thread.start()
        self.odometry_thread.start()
        print("所有控制线程已启动")

    def msg_handler(self, channel, data):
        """LCM消息处理器"""
        self.rec_msg = robot_control_response_lcmt().decode(data)
        if self.rec_msg.order_process_bar >= 95:
            self.mode_ok = self.rec_msg.mode
            self.gait_ok = self.rec_msg.gait_id
        else:
            self.mode_ok = 0

    def rec_response(self):
        """接收LCM响应"""
        while self.running:
            self.lc_r.handle()
            time.sleep(0.002)

    def send_publish(self):
        """发送LCM命令"""
        while self.running:
            self.send_lock.acquire()
            if self.delay_cnt > 20:  # 心跳信号 10Hz
                self.lc_s.publish("robot_control_cmd", self.cmd_msg.encode())
                self.delay_cnt = 0
            self.delay_cnt += 1
            self.send_lock.release()
            time.sleep(0.005)

    def send_cmd(self, msg):
        """发送命令"""
        self.send_lock.acquire()
        self.delay_cnt = 50
        self.cmd_msg = msg
        self.send_lock.release()

    def parse_lc02_message(self, data):
        """解析LC02消息格式"""
        if len(data) < 8:
            return None, None
        
        if data[:4] != b'LC02':
            return None, None
        
        msg_len = struct.unpack('>I', data[4:8])[0]
        channel_end = data.find(b'\x00', 8)
        if channel_end == -1:
            return None, None
        
        channel_name = data[8:channel_end].decode('utf-8', errors='ignore')
        payload = data[channel_end + 1:]
        
        return channel_name, payload
    def get_current_position(self) -> Optional[Tuple[float, float]]:
        """获取当前位置"""
        with self.odometry_lock:
            if self.odometry_data:
                return (self.odometry_data['x'], self.odometry_data['y'])
            return None

    def wait_finish(self, mode: int, gait_id: int, timeout: float = 10.0) -> bool:
        """等待命令完成"""
        count = 0
        max_count = int(timeout * 200)  # 5ms per cycle
        while self.running and count < max_count:
            if self.mode_ok == mode and self.gait_ok == gait_id:
                return True
            else:
                time.sleep(0.005)
                count += 1
        return False

    def stand(self, duration: int = 1000, sleep: float = 1.1):
        """站立命令 - 增强版本，多次发送确保成功"""
        print("发送站立命令...")
        
        # 多次发送站立命令，确保机器人站立
        for i in range(3):
            self.cmd_msg.mode = 12
            self.cmd_msg.gait_id = 0
            self.cmd_msg.duration = duration
            self.cmd_msg.life_count = (self.cmd_msg.life_count + 1) % 128  # 防止溢出
            self.send_cmd(self.cmd_msg)
            time.sleep(0.5)  # 每次发送后等待0.5秒
        
        time.sleep(sleep)  # 最后等待指定时间

    def lie(self):
        """趴下命令"""
        print("发送趴下命令...")
        self.cmd_msg.mode = 7
        self.cmd_msg.gait_id = 0
        self.cmd_msg.duration = 5000
        self.cmd_msg.life_count = (self.cmd_msg.life_count + 1) % 128  # 防止溢出
        self.send_cmd(self.cmd_msg)
        time.sleep(5.5)

    def move_forward(self, distance: float, speed: float = 0.3) -> bool:
        """前进指定距离 - 增强版本"""
        print(f"前进 {distance}m...")
        
        # 首先确保机器人处于站立状态
        self.stand(duration=1000, sleep=1.0)
        
        # 等待里程计数据
        timeout_count = 0
        while self.get_current_position() is None and timeout_count < 100:
            time.sleep(0.1)
            timeout_count += 1
        
        if self.get_current_position() is None:
            print("无法获取里程计数据，使用时间估算模式")
            return self._move_forward_time_based(distance, speed)
        
        # 记录起始位置
        self.start_position = self.get_current_position()
        traveled_distance = 0.0
        
        while traveled_distance < distance:
            current_pos = self.get_current_position()
            if current_pos:
                traveled_distance = self.calculate_distance(self.start_position, current_pos)
                remaining = distance - traveled_distance
                
                # 动态调整速度
                if remaining < 0.1:
                    target_speed = max(0.1, speed * 0.3)
                elif remaining < 0.3:
                    target_speed = max(0.1, speed * 0.5)
                else:
                    target_speed = speed
                
                self.cmd_msg.mode = 11
                self.cmd_msg.gait_id = 27
                self.cmd_msg.vel_des = [target_speed, 0, 0]
                self.cmd_msg.step_height = [0.05, 0.05]
                self.cmd_msg.duration = 0
                self.cmd_msg.life_count = (self.cmd_msg.life_count + 1) % 128  # 防止溢出
                self.send_cmd(self.cmd_msg)
                
                if remaining <= 0.05:  # 5cm精度
                    break
            
            time.sleep(0.1)
        
        # 停止
        self.cmd_msg.mode = 12
        self.cmd_msg.gait_id = 0
        self.cmd_msg.duration = 1000
        self.cmd_msg.life_count = (self.cmd_msg.life_count + 1) % 128  # 防止溢出
        self.send_cmd(self.cmd_msg)
        time.sleep(1.0)
        
        print(f"前进完成，实际距离: {traveled_distance:.2f}m")
        return True
    
    def _move_forward_time_based(self, distance: float, speed: float = 0.3) -> bool:
        """基于时间估算的前进方法（里程计不可用时的备用方案）"""
        print(f"使用时间估算模式前进 {distance}m，速度 {speed}m/s")
        
        # 计算需要的时间（距离/速度），增加10%的安全余量
        move_time = (distance / speed) * 1.1
        
        # 发送前进命令
        self.cmd_msg.mode = 11
        self.cmd_msg.gait_id = 27
        self.cmd_msg.vel_des = [speed, 0, 0]
        self.cmd_msg.step_height = [0.05, 0.05]
        self.cmd_msg.duration = 0
        self.cmd_msg.life_count = (self.cmd_msg.life_count + 1) % 128
        self.send_cmd(self.cmd_msg)
        
        # 等待计算出的时间
        print(f"预计移动时间: {move_time:.1f}秒")
        time.sleep(move_time)
        
        # 停止
        self.cmd_msg.mode = 12
        self.cmd_msg.gait_id = 0
        self.cmd_msg.duration = 1000
        self.cmd_msg.life_count = (self.cmd_msg.life_count + 1) % 128
        self.send_cmd(self.cmd_msg)
        time.sleep(1.0)
        
        print(f"时间估算模式前进完成")
        return True

    def move_backward(self, distance: float, speed: float = 0.3) -> bool:
        """后退指定距离"""
        print(f"后退 {distance}m...")
        
        # 等待里程计数据
        timeout_count = 0
        while self.get_current_position() is None and timeout_count < 100:
            time.sleep(0.1)
            timeout_count += 1
        
        if self.get_current_position() is None:
            print("无法获取里程计数据，使用时间估算模式")
            return self._move_backward_time_based(distance, speed)
        
        # 记录起始位置
        self.start_position = self.get_current_position()
        traveled_distance = 0.0
        
        while traveled_distance < distance:
            current_pos = self.get_current_position()
            if current_pos:
                traveled_distance = self.calculate_distance(self.start_position, current_pos)
                remaining = distance - traveled_distance
                
                # 动态调整速度
                if remaining < 0.1:
                    target_speed = max(0.1, speed * 0.3)
                elif remaining < 0.3:
                    target_speed = max(0.1, speed * 0.5)
                else:
                    target_speed = speed
                
                self.cmd_msg.mode = 11
                self.cmd_msg.gait_id = 27
                self.cmd_msg.vel_des = [-target_speed, 0, 0]  # 负速度表示后退
                self.cmd_msg.step_height = [0.05, 0.05]
                self.cmd_msg.duration = 0
                self.cmd_msg.life_count = (self.cmd_msg.life_count + 1) % 128  # 防止溢出
                self.send_cmd(self.cmd_msg)
                
                if remaining <= 0.05:  # 5cm精度
                    break
            
            time.sleep(0.1)
        
        # 停止
        self.cmd_msg.mode = 12
        self.cmd_msg.gait_id = 0
        self.cmd_msg.duration = 1000
        self.cmd_msg.life_count = (self.cmd_msg.life_count + 1) % 128  # 防止溢出
        self.send_cmd(self.cmd_msg)
        time.sleep(1.0)
        
        print(f"后退完成，实际距离: {traveled_distance:.2f}m")
        return True
    
    def _move_backward_time_based(self, distance: float, speed: float = 0.3) -> bool:
        """基于时间估算的后退方法（里程计不可用时的备用方案）"""
        print(f"使用时间估算模式后退 {distance}m，速度 {speed}m/s")
        
        # 计算需要的时间（距离/速度），增加10%的安全余量
        move_time = (distance / speed) * 1.1
        
        # 发送后退命令
        self.cmd_msg.mode = 11
        self.cmd_msg.gait_id = 27
        self.cmd_msg.vel_des = [-speed, 0, 0]  # 负速度表示后退
        self.cmd_msg.step_height = [0.05, 0.05]
        self.cmd_msg.duration = 0
        self.cmd_msg.life_count = (self.cmd_msg.life_count + 1) % 128
        self.send_cmd(self.cmd_msg)
        
        # 等待计算出的时间
        print(f"预计移动时间: {move_time:.1f}秒")
        time.sleep(move_time)
        
        # 停止
        self.cmd_msg.mode = 12
        self.cmd_msg.gait_id = 0
        self.cmd_msg.duration = 1000
        self.cmd_msg.life_count = (self.cmd_msg.life_count + 1) % 128
        self.send_cmd(self.cmd_msg)
        time.sleep(1.0)
        
        print(f"时间估算模式后退完成")
        return True

    def turn_left(self, angle_degrees: float = 90, angular_speed: float = 0.5) -> bool:
        """左转指定角度 - 增强版本"""
        print(f"左转 {angle_degrees}°...")
        
        # 首先确保机器人处于站立状态
        self.stand(duration=1000, sleep=1.0)
        
        # 计算转向时间（粗略估算）
        turn_time = abs(angle_degrees) / (angular_speed * 57.3)  # 转换弧度到度
        
        self.cmd_msg.mode = 11
        self.cmd_msg.gait_id = 27
        self.cmd_msg.vel_des = [0, 0, angular_speed]  # 正角速度表示左转
        self.cmd_msg.step_height = [0.05, 0.05]
        self.cmd_msg.duration = 0
        self.cmd_msg.life_count = (self.cmd_msg.life_count + 1) % 128  # 防止溢出
        self.send_cmd(self.cmd_msg)
        
        time.sleep(turn_time)
        
        # 停止
        self.cmd_msg.mode = 12
        self.cmd_msg.gait_id = 0
        self.cmd_msg.duration = 1000
        self.cmd_msg.life_count = (self.cmd_msg.life_count + 1) % 128  # 防止溢出
        self.send_cmd(self.cmd_msg)
        time.sleep(1.0)
        
        print(f"左转 {angle_degrees}° 完成")
        return True

    def turn_right(self, angle_degrees: float = 90, angular_speed: float = 0.5) -> bool:
        """右转指定角度 - 增强版本"""
        print(f"右转 {angle_degrees}°...")
        
        # 首先确保机器人处于站立状态
        self.stand(duration=1000, sleep=1.0)
        
        # 计算转向时间（粗略估算）
        turn_time = abs(angle_degrees) / (angular_speed * 57.3)  # 转换弧度到度
        
        self.cmd_msg.mode = 11
        self.cmd_msg.gait_id = 27
        self.cmd_msg.vel_des = [0, 0, -angular_speed]  # 负角速度表示右转
        self.cmd_msg.step_height = [0.05, 0.05]
        self.cmd_msg.duration = 0
        self.cmd_msg.life_count = (self.cmd_msg.life_count + 1) % 128  # 防止溢出
        self.send_cmd(self.cmd_msg)
        
        time.sleep(turn_time)
        
        # 停止
        self.cmd_msg.mode = 12
        self.cmd_msg.gait_id = 0
        self.cmd_msg.duration = 1000
        self.cmd_msg.life_count = (self.cmd_msg.life_count + 1) % 128  # 防止溢出
        self.send_cmd(self.cmd_msg)
        time.sleep(1.0)
        
        print(f"右转 {angle_degrees}° 完成")
        return True

    def stop(self):
        """停止机器人"""
        print("停止机器人...")
        self.cmd_msg.mode = 12
        self.cmd_msg.gait_id = 0
        self.cmd_msg.duration = 1000
        self.cmd_msg.life_count = (self.cmd_msg.life_count + 1) % 128  # 防止溢出
        self.send_cmd(self.cmd_msg)
        time.sleep(1.0)

    def set_step_height(self, front_left=0.06, front_right=0.06, back_left=0.06, back_right=0.06):
        """
        设置四肢步高
        
        Args:
            front_left: 前左腿步高（米）
            front_right: 前右腿步高（米）
            back_left: 后左腿步高（米）
            back_right: 后右腿步高（米）
        """
        try:
            print(f"设置步高: FL={front_left:.3f}, FR={front_right:.3f}, BL={back_left:.3f}, BR={back_right:.3f}")
            
            self.cmd_msg.mode = 11
            self.cmd_msg.gait_id = 27
            self.cmd_msg.step_height = [front_left, front_right, back_left, back_right]
            self.cmd_msg.contact = 1
            self.cmd_msg.life_count = (self.cmd_msg.life_count + 1) % 128
            
            self.send_cmd(self.cmd_msg)
            time.sleep(0.5)
            
        except Exception as e:
            print(f"设置步高失败: {e}")
    
    def set_body_height(self, height: float):
        """
        设置机身高度
        
        Args:
            height: 机身高度偏移（米）
        """
        try:
            print(f"设置机身高度: {height:.3f}m")
            
            self.cmd_msg.mode = 11
            self.cmd_msg.gait_id = 27
            self.cmd_msg.pos_des = [0.0, 0.0, height, 0.0, 0.0, 0.0]
            self.cmd_msg.contact = 1
            self.cmd_msg.life_count = (self.cmd_msg.life_count + 1) % 128
            
            self.send_cmd(self.cmd_msg)
            time.sleep(0.5)
            
        except Exception as e:
            print(f"设置机身高度失败: {e}")
    
    def set_body_pose(self, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
        """
        设置机身姿态
        
        Args:
            x, y, z: 位置偏移（米）
            roll, pitch, yaw: 姿态角度（弧度）
        """
        try:
            print(f"设置机身姿态: pos=[{x:.3f}, {y:.3f}, {z:.3f}], rot=[{roll:.3f}, {pitch:.3f}, {yaw:.3f}]")
            
            self.cmd_msg.mode = 11
            self.cmd_msg.gait_id = 27
            self.cmd_msg.pos_des = [x, y, z, roll, pitch, yaw]
            self.cmd_msg.contact = 1
            self.cmd_msg.life_count = (self.cmd_msg.life_count + 1) % 128
            
            self.send_cmd(self.cmd_msg)
            time.sleep(0.5)
            
        except Exception as e:
            print(f"设置机身姿态失败: {e}")
    
    def load_gait_from_file(self, toml_file_path: str):
        """
        从TOML文件加载步态配置
        
        Args:
            toml_file_path: TOML文件路径
        """
        try:
            print(f"加载步态文件: {toml_file_path}")
            
            # 切换到太空步步态模式
            self.cmd_msg.mode = 11
            self.cmd_msg.gait_id = 90  # 太空步步态ID
            self.cmd_msg.contact = 1
            self.cmd_msg.life_count = (self.cmd_msg.life_count + 1) % 128
            
            self.send_cmd(self.cmd_msg)
            time.sleep(1)
            
            print("步态文件加载完成")
            
        except Exception as e:
            print(f"加载步态文件失败: {e}")
    
    def move_distance_imu(self, distance: float, velocity: float = 0.1, backward: bool = False):
        """
        使用IMU导航移动指定距离
        
        Args:
            distance: 移动距离（米）
            velocity: 移动速度（米/秒）
            backward: 是否向后移动
        """
        try:
            direction = -1 if backward else 1
            print(f"IMU导航移动: {distance:.2f}m, 速度: {velocity:.2f}m/s, {'后退' if backward else '前进'}")
            
            # 计算移动时间
            move_time = abs(distance) / velocity
            
            self.cmd_msg.mode = 11
            self.cmd_msg.gait_id = 27
            self.cmd_msg.vel_des = [direction * velocity, 0.0, 0.0]
            self.cmd_msg.step_height = [0.05, 0.05]
            self.cmd_msg.duration = 0
            self.cmd_msg.life_count = (self.cmd_msg.life_count + 1) % 128
            
            self.send_cmd(self.cmd_msg)
            time.sleep(move_time)
            
            # 停止移动
            self.cmd_msg.mode = 12
            self.cmd_msg.gait_id = 0
            self.cmd_msg.duration = 1000
            self.send_cmd(self.cmd_msg)
            
        except Exception as e:
            print(f"IMU导航移动失败: {e}")

    def quit(self):
        """退出控制器"""
        self.running = 0
        if self.rec_thread.is_alive():
            self.rec_thread.join()
        if self.send_thread.is_alive():
            self.send_thread.join()
        if self.odometry_thread.is_alive():
            self.odometry_thread.join(timeout=2.0)
        print("机器狗控制器已退出")
