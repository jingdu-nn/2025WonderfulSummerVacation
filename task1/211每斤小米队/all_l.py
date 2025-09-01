#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ALL_L：单1111文件全流程（左向闭环）
- 决策与识别逻辑保持与 all.py 一致（绿箭头/黄框/二维码/黑框）
- 路线按用户描述改为"向左闭环"，并重排执行顺序
- 过限高杆优先使用 gaits/moonwalk2.toml；缺失或 toml 不可用时退化为低速后退1m模拟
"""

import os
import sys
import time
import math
import copy
import subprocess
import json
import socket
import struct
from threading import Thread, Lock
from typing import Optional, Tuple
from collections import deque

import cv2
import numpy as np
import lcm
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pyzbar.pyzbar import decode

# 语音与触摸接口（可选）
try:
    from protocol.srv import AudioTextPlay
    from protocol.msg import AudioPlay, TouchStatus
except Exception:
    AudioTextPlay = None
    AudioPlay = None
    TouchStatus = None

# LCM 消息（仅导入消息定义）
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from robot_control_response_lcmt import robot_control_response_lcmt

# toml 用于过限高杆步态
try:
    import toml
except Exception:
    toml = None


class AllConfig:
    """集中配置参数，便于调参（从all_r.py移植）"""

    # 识别与等待（秒/帧）
    WAIT_RGB_TIMEOUT_S: float = 5.0        # 等待首次RGB帧超时（s）
    GREEN_ARROW_TIMEOUT_S: float = 40.0    # 绿箭头识别超时（s）
    GREEN_ARROW_STABLE_FRAMES: int = 5     # 绿箭头需连续命中的帧数（帧）
    QR_TIMEOUT_S: float = 10.0             # 二维码识别超时（s）
    BLACK_BARRIER_TIMEOUT_S: float = 40.0  # 黑框（限高杆）识别超时（s）
    
    # 头部传感器碰撞检测配置（用于红色限高杆检测）
    HEAD_COLLISION_MAX_DISTANCE_M: float = 15.0    # 头部碰撞检测最大前进距离（米）
    HEAD_COLLISION_SPEED: float = 0.15             # 头部碰撞检测时的前进速度（m/s）
    HEAD_COLLISION_BACKUP_M: float = 0.0           # 检测到碰撞后的后退距离（米）
    
    # 限高杆过杆参数
    BLACK_APPROACH_SPEED: float = 0.15             # 行走中靠近的速度（m/s）
    BLACK_AFTER_TARGET_FWD_M: float = 0.8          # 达到目标距离后，向前再走的距离（米）
    BLACK_AFTER_TARGET_FWD_SPEED: float = 0.20     # 上述前进速度（m/s）
    BLACK_AFTER_TARGET_TURN_DEG: float = -180.0    # 随后原地掉头角度（度）- 右转180°
    POST_GAIT_TURN_DEG: float = -200.0             # 过杆步态完成后再次掉头角度（度）- 右转200°
    POST_GAIT_FORWARD_M: float = 3.0               # 限高杆阶段总距离（米）
    POST_GAIT_FORWARD_SPEED: float = 0.1           # 最后直行速度（m/s）
    POST_GAIT_HIGH_STEP: tuple = (0.1, 0.1)        # 高抬腿步高（m）
    
    # 黄色圆形检测配置（从new_all_r.py同步）
    YELLOW_DETECT_MAX_M: float = 2.5               # 黄色圆形识别段最大距离（m）
    YELLOW_DETECT_SPEED: float = 0.12              # 黄色圆形识别段速度（m/s）
    YELLOW_DETECT_CONTROL_FREQ_HZ: float = 10.0    # 控制频率（Hz）
    YELLOW_ROI_UPPER_RATIO: float = 0.3            # ROI上半部分比例
    YELLOW_ROI_MIN_AREA: float = 2500              # ROI区域最小面积
    YELLOW_ROI_MIN_CIRCULARITY: float = 0.3        # ROI区域最小圆度
    ENABLE_YELLOW_CENTERING: bool = True           # 启用黄线矫正
    YL_ROI_BOTTOM_PERCENT: float = 0.50            # 黄线ROI底部百分比

    # 箭头路径后续动作参数（单位：米/米每秒/度）
    ARROW_FWD1_M: float = 0.6          # 识别到箭头后：第一段直行距离（m）
    ARROW_FWD1_SPEED: float = 0.20     # 第一段直行速度（m/s）
    ARROW_FWD2_M: float = 0.85         # 第二段直行距离（m）
    ARROW_FWD2_SPEED: float = 0.22     # 第二段直行速度（m/s）

    # 石板路段参数
    STONE_ROAD_M: float = 5.0          # 石板路距离（m）
    STONE_ROAD_SPEED: float = 0.15     # 石板路速度（m/s）

    # 到二维码路径参数
    TO_QR_FWD1_M: float = 0.1          # 第一段直行距离（m）
    TO_QR_FWD1_SPEED: float = 0.22     # 第一段直行速度（m/s）
    TO_QR_FWD2_M: float = 0.7          # 第二段直行距离（m）
    TO_QR_FWD2_SPEED: float = 0.22     # 第二段直行速度（m/s）

    # B1路径（卸货段，单位：米/米每秒/度/秒）
    B1_FWD1_M: float = 1.0             # 直行（m）
    B1_FWD1_SPEED: float = 0.20        # 速度（m/s）
    B1_FWD2_M: float = 1.0             # 直行（m）
    B1_FWD2_SPEED: float = 0.18        # 速度（m/s）

    # B2装货段（单位：米/米每秒/度/秒）
    B2_BACK_M: float = -1.0            # 后退距离（m，负号为后退）
    B2_BACK_SPEED: float = 0.18        # 后退速度（m/s）
    B2_FWD1_M: float = 2.0             # 直行（m）
    B2_FWD1_SPEED: float = 0.20        # 速度（m/s）
    B2_FWD2_M: float = 1.0             # 直行（m）
    B2_FWD2_SPEED: float = 0.18        # 速度（m/s）

    # 黄灯后路线参数
    YELLOW_AFTER_FWD1_M: float = 1.5   # 黄灯后第一段直行（m）
    YELLOW_AFTER_FWD1_SPEED: float = 0.22  # 黄灯后第一段速度（m/s）
    YELLOW_AFTER_BACK_M: float = -2.0  # 后退上坡距离（m）
    YELLOW_AFTER_BACK_SPEED: float = 0.22  # 后退上坡速度（m/s）
    YELLOW_AFTER_FWD2_M: float = 0.0   # 坡间路距离（m）
    YELLOW_AFTER_FWD2_SPEED: float = 0.12  # 坡间路速度（m/s）
    YELLOW_AFTER_FWD3_M: float = 2.0   # 下坡距离（m）
    YELLOW_AFTER_FWD3_SPEED: float = 0.12  # 下坡速度（m/s）

    # 黄灯检测段参数
    YELLOW_DETECT_M: float = 4.0       # 黄灯检测段距离（m）
    YELLOW_DETECT_SPEED: float = 0.22  # 黄灯检测段速度（m/s）

    # 黄线"左右边线取中线"居中（参考 yellow_centering_module.py 逻辑）
    ENABLE_YELLOW_CENTERING: bool = True           # 是否启用该居中方案（优先级最高）
    YL_MIN_DIST_M: float = 0.3                     # 仅当直行距离≥该值时启用
    YL_ROI_BOTTOM_PERCENT: float = 0.5            # ROI为图像底部百分比高度
    YL_HSV_LOWER: tuple = (10, 40, 40)             # HSV下界（结合demo调参）
    YL_HSV_UPPER: tuple = (45, 255, 255)           # HSV上界（拉宽）
    YL_MIN_CONTOUR_AREA: int = 300                 # 最小轮廓面积
    YL_FALLBACK_OFFSET_PX: int = 200               # 单侧缺失时的像素偏移补偿
    YL_KP: float = 0.009                           # 比例增益（转向）
    YL_MAX_TURN: float = 0.6                       # 最大转向（rad/s）
    YL_SMOOTH_BUFFER: int = 5                      # 误差平滑窗口
    YL_SMOOTH_MAX_DELTA: float = 0.20              # 相邻转向的最大变化量
    YL_CTRL_TICK_S: float = 0.05                   # 控制周期
    # PID（方向控制）
    YL_PID_KP: float = 2.0                         # 比例增益（基于归一化偏差）
    YL_PID_KI: float = 0.30                        # 积分增益
    YL_PID_KD: float = 0.80                        # 微分增益
    YL_PID_INT_LIM: float = 0.40                   # 积分限幅

    # 绿色与黄色的HSV阈值（供通用检测使用，已拉宽）
    GREEN_HSV_LOWER: tuple = (25, 40, 40)
    GREEN_HSV_UPPER: tuple = (95, 255, 255)
    YELLOW_HSV_LOWER: tuple = (10, 40, 40)
    YELLOW_HSV_UPPER: tuple = (45, 255, 255)


class AllNodeL(Node):
    """ALL单文件节点（左向闭环）：LCM运动 + RGB识别（箭头/黄矩形/二维码/黑框）"""

    def __init__(self):
        super().__init__('all_node_l')

        # LCM 通信
        self.lc_r = lcm.LCM("udpm://239.255.76.67:7670?ttl=255")
        self.lc_s = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        self.cmd_msg = robot_control_cmd_lcmt()
        self.rec_msg = robot_control_response_lcmt()
        self.cmd_msg.life_count = 0
        self.send_lock = Lock()
        self.delay_cnt = 0
        self.mode_ok = 0
        self.running = 1
        self.rec_thread = Thread(target=self._rec_response_loop, daemon=True)
        self.send_thread = Thread(target=self._send_publish_loop, daemon=True)

        # 相机
        self.bridge = CvBridge()
        self.latest_rgb_image = None
        self.image_lock = Lock()
        self.rgb_sub = self.create_subscription(Image, '/image_rgb', self._rgb_callback, 10)
        print("✅ 已订阅RGB相机: /image_rgb")

        # 绿箭头参数（HSV，拉宽）
        self.green_lower = np.array(list(AllConfig.GREEN_HSV_LOWER), dtype=np.uint8)
        self.green_upper = np.array(list(AllConfig.GREEN_HSV_UPPER), dtype=np.uint8)

        # 黄矩形参数（HSV，拉宽）
        self.yellow_lower = np.array(list(AllConfig.YELLOW_HSV_LOWER), dtype=np.uint8)
        self.yellow_upper = np.array(list(AllConfig.YELLOW_HSV_UPPER), dtype=np.uint8)

        # 黑框参数（HSV）
        self.black_lower = np.array([0, 0, 0])
        self.black_upper = np.array([180, 255, 80])

        # 红色参数（HSV）- 收紧阈值避免误检黄色/黑色
        self.red_lower1 = np.array([0, 120, 100], dtype=np.uint8)    # 低红色范围：H=0-6, S≥120, V≥100
        self.red_upper1 = np.array([6, 255, 255], dtype=np.uint8)    # 进一步缩小H范围
        self.red_lower2 = np.array([174, 120, 100], dtype=np.uint8)  # 高红色范围：H=174-180, S≥120, V≥100
        self.red_upper2 = np.array([180, 255, 255], dtype=np.uint8)  # 收紧高端范围起点

        # IMU里程计数据（参考hybrid_walk_demo.py）
        self.odometry_data = None
        self.odometry_lock = Lock()
        self.odometry_thread = Thread(target=self._listen_odometry, daemon=True)
        self.start_position = None
        self.current_distance = 0.0
        self.last_odometry_xyz = None
        
        # UDP配置（里程计）
        self.multicast_group = "239.255.76.67"
        self.odometry_port = 7667
        
        # 启动里程计监听
        self.odometry_thread.start()
        print("✅ IMU里程计监听已启动")

        print("✅ ALL_L 节点初始化完成")

    # ----------------------------
    # 基础通信与相机
    # ----------------------------
    def start(self):
        self.lc_r.subscribe("robot_control_response", self._msg_handler)
        self.rec_thread.start()
        self.send_thread.start()
        print("✅ LCM 线程已启动，等待RGB图像...")
        # 初始化语音与触摸（若可用）
        try:
            self._init_speech_service()
        except Exception:
            pass
        try:
            self._init_touch_subscription()
        except Exception:
            pass

    def _msg_handler(self, channel, data):
        try:
            self.rec_msg = robot_control_response_lcmt().decode(data)
            if self.rec_msg.order_process_bar >= 95:
                self.mode_ok = self.rec_msg.mode
            else:
                self.mode_ok = 0
        except Exception:
            pass

    def _rec_response_loop(self):
        while self.running:
            try:
                self.lc_r.handle()
                time.sleep(0.002)
            except Exception:
                time.sleep(0.05)

    def _send_publish_loop(self):
        while self.running:
            self.send_lock.acquire()
            try:
                if self.delay_cnt > 20:  # 10Hz心跳
                    self.lc_s.publish("robot_control_cmd", self.cmd_msg.encode())
                    self.delay_cnt = 0
                self.delay_cnt += 1
            finally:
                self.send_lock.release()
            time.sleep(0.005)

    def _rgb_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.image_lock:
                self.latest_rgb_image = cv_image
        except Exception as e:
            self.get_logger().warn(f"RGB图像转换失败: {e}")

    def get_rgb_image(self) -> Optional[np.ndarray]:
        with self.image_lock:
            if self.latest_rgb_image is None:
                return None
            return self.latest_rgb_image.copy()

    def wait_for_rgb(self, timeout_s: float = 5.0) -> Optional[np.ndarray]:
        start = time.time()
        while time.time() - start < timeout_s:
            img = self.get_rgb_image()
            if img is not None:
                return img
            time.sleep(0.1)
        return None

    # ----------------------------
    # IMU里程计功能（参考hybrid_walk_demo.py）
    # ----------------------------
    def _parse_lc02_message(self, data):
        """解析LC02消息格式"""
        if len(data) < 8:
            return None, None
        
        # LC02消息格式: LC02 + 长度(4字节) + 频道名 + \x00 + 数据
        if data[:4] != b'LC02':
            return None, None
        
        # 获取消息长度
        msg_len = struct.unpack('>I', data[4:8])[0]
        
        # 查找频道名结束位置
        channel_end = data.find(b'\x00', 8)
        if channel_end == -1:
            return None, None
        
        channel_name = data[8:channel_end].decode('utf-8', errors='ignore')
        payload = data[channel_end + 1:]
        
        return channel_name, payload

    def _listen_odometry(self):
        """直接UDP监听里程计数据"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(('', self.odometry_port))
            
            mreq = socket.inet_aton(self.multicast_group) + socket.inet_aton('0.0.0.0')
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
            
            print(f"📡 开始监听IMU里程计端口 {self.odometry_port}")
            
            while self.running:
                try:
                    data, addr = sock.recvfrom(1024)
                    
                    # 解析LC02消息
                    channel_name, payload = self._parse_lc02_message(data)
                    
                    if channel_name == "global_to_robot" and payload:
                        try:
                            # 尝试不同的偏移量来找到正确的数据位置
                            for offset in range(0, min(20, len(payload))):
                                if len(payload) >= offset + 24:  # 至少需要24字节用于xyz+vxyz
                                    try:
                                        xyz = struct.unpack('>fff', payload[offset:offset+12])
                                        vxyz = struct.unpack('>fff', payload[offset+12:offset+24])
                                        
                                        # 检查数值是否合理，排除异常数据
                                        if (all(-1000 < x < 1000 for x in xyz) and 
                                            all(-100 < v < 100 for v in vxyz) and
                                            not (abs(xyz[0]) < 0.001 and abs(xyz[1]) < 0.001) and  # 排除接近(0,0)的数据
                                            abs(xyz[2]) < 50):  # 排除Z轴异常高的数据
                                            
                                            with self.odometry_lock:
                                                self.odometry_data = {
                                                    'x': xyz[0], 'y': xyz[1], 'z': xyz[2],
                                                    'vx': vxyz[0], 'vy': vxyz[1], 'vz': vxyz[2],
                                                    'timestamp': time.time()
                                                }
                                            break
                                        
                                    except Exception as e:
                                        continue
                            
                        except Exception as e:
                            # 不打印解析错误，避免日志过多
                            pass
                    
                except Exception as e:
                    # 不打印监听错误，避免日志过多
                    time.sleep(0.01)
                    
        except Exception as e:
            print(f"⚠️ IMU里程计监听设置失败: {e}")

    def get_current_position(self):
        """获取当前位置"""
        with self.odometry_lock:
            if self.odometry_data:
                return (self.odometry_data['x'], self.odometry_data['y'])
            return None

    def get_current_velocity(self):
        """获取当前速度（从里程计数据）"""
        with self.odometry_lock:
            if self.odometry_data:
                # 计算XY平面的合成速度
                vx = self.odometry_data['vx']
                vy = self.odometry_data['vy']
                return math.sqrt(vx*vx + vy*vy)
            return 0.0

    def calculate_distance(self, start_pos, current_pos):
        """计算行走距离"""
        if start_pos is None or current_pos is None:
            return 0.0
        
        dx = current_pos[0] - start_pos[0]
        dy = current_pos[1] - start_pos[1]
        return math.sqrt(dx*dx + dy*dy)

    def reset_odometry_start_position(self):
        """重置里程计起始位置"""
        print("📍 重置IMU里程计起始位置...")
        
        # 重置异常值过滤器
        if hasattr(self, '_last_valid_distance'):
            delattr(self, '_last_valid_distance')
        
        # 获取稳定的起始位置
        stable_positions = []
        for i in range(15):  # 增加采样次数
            pos = self.get_current_position()
            if pos and all(-1000 < x < 1000 for x in pos):
                # 🛡️ 过滤明显异常的初始数据
                if not (abs(pos[0]) < 0.001 or abs(pos[1]) < 0.001):  # 排除接近(0,0)的数据
                    stable_positions.append(pos)
            time.sleep(0.1)
        
        if len(stable_positions) < 3:  # 降低要求，但确保质量
            print("⚠️ 无法获取足够的稳定起始位置数据")
            return False
        
        # 🔍 进一步过滤异常值
        valid_positions = []
        for pos in stable_positions:
            # 排除坐标为0的异常数据
            if abs(pos[0]) > 0.01 and abs(pos[1]) > 0.01:
                # 排除坐标变化过大的异常数据
                if len(valid_positions) == 0:
                    valid_positions.append(pos)
                else:
                    last_pos = valid_positions[-1]
                    distance = math.sqrt((pos[0] - last_pos[0])**2 + (pos[1] - last_pos[1])**2)
                    if distance < 2.0:  # 相邻位置变化不应超过2米
                        valid_positions.append(pos)
        
        if not valid_positions:
            print("⚠️ 所有起始位置数据都异常")
            return False
        
        # 🔍 选择最稳定的位置作为起始位置
        if len(valid_positions) >= 3:
            # 选择中位数位置，更稳定
            x_coords = [pos[0] for pos in valid_positions[-5:]]  # 取最后5个
            y_coords = [pos[1] for pos in valid_positions[-5:]]
            x_coords.sort()
            y_coords.sort()
            median_x = x_coords[len(x_coords)//2]
            median_y = y_coords[len(y_coords)//2]
            self.start_position = (median_x, median_y)
        else:
            self.start_position = valid_positions[-1]
        
        print(f"📍 IMU起始位置: ({self.start_position[0]:.3f}, {self.start_position[1]:.3f})")
        print(f"🔍 采样统计: 总共{len(stable_positions)}个稳定位置, {len(valid_positions)}个有效位置")
        
        # 🛡️ 最终验证起始位置的合理性
        if abs(self.start_position[0]) < 0.01 or abs(self.start_position[1]) < 0.01:
            print(f"❌ 起始位置仍然异常，放弃IMU控制")
            return False
        
        return True

    def get_traveled_distance(self):
        """获取已行走距离（带异常值过滤）"""
        current_pos = self.get_current_position()
        if self.start_position and current_pos:
            distance = self.calculate_distance(self.start_position, current_pos)
            
            # 🛡️ 增强异常值过滤
            if hasattr(self, '_last_valid_distance'):
                distance_change = abs(distance - self._last_valid_distance)
                if distance_change > 2.0:  # 降低阈值，更严格过滤
                    print(f"⚠️ IMU异常值检测：距离跳变{distance_change:.2f}m，保持上次有效值")
                    return self._last_valid_distance
            
            # 🛡️ 初始距离异常值检测
            if not hasattr(self, '_last_valid_distance'):
                # 初始距离不应过大
                if distance > 5.0:
                    print(f"⚠️ IMU初始距离异常：{distance:.2f}m，重置为0")
                    self._last_valid_distance = 0.0
                    return 0.0
            
            # 记录有效距离
            if distance < 50.0:  # 防止异常值
                self._last_valid_distance = distance
            
            return distance
        return 0.0

    def _send_cmd_now(self):
        self.send_lock.acquire()
        try:
            self.delay_cnt = 50
        finally:
            self.send_lock.release()

    # ----------------------------
    # 基础运动
    # ----------------------------
    def stand_up(self, duration_ms: int = 3000, wait_s: float = 3.0):
        self.cmd_msg.mode = 12
        self.cmd_msg.gait_id = 0
        self.cmd_msg.duration = duration_ms
        self.cmd_msg.life_count = (self.cmd_msg.life_count + 1) % 128
        self._send_cmd_now()
        print("🧍 发送站立指令...")
        time.sleep(wait_s)

    def send_stop(self, hold_s: float = 0.2):
        self.cmd_msg.mode = 11
        self.cmd_msg.gait_id = 27
        self.cmd_msg.vel_des = [0.0, 0.0, 0.0]
        self.cmd_msg.step_height = [0.05, 0.05]
        self.cmd_msg.duration = 100
        self.cmd_msg.life_count = (self.cmd_msg.life_count + 1) % 128
        self._send_cmd_now()
        time.sleep(hold_s)

    def move_forward(self, distance_m: float, speed_mps: float = 0.2,
                     step_height: Tuple[float, float] = (0.05, 0.05), use_imu: bool = True):
        """基础前进函数，可选择IMU距离控制或时间控制"""
        if speed_mps == 0:
            return
        
        if use_imu and distance_m > 0:
            # 使用IMU距离控制
            self.move_forward_with_imu(distance_m, speed_mps, step_height)
        else:
            # 使用原来的时间控制
            duration_ms = max(200, int(abs(distance_m) / abs(speed_mps) * 1000))
            self.cmd_msg.mode = 11
            self.cmd_msg.gait_id = 27
            self.cmd_msg.vel_des = [float(np.sign(distance_m)) * abs(speed_mps), 0.0, 0.0]
            self.cmd_msg.step_height = [float(step_height[0]), float(step_height[1])]
            self.cmd_msg.duration = duration_ms
            self.cmd_msg.rpy_des = [0.0, 0.0, 0.0]
            self.cmd_msg.pos_des = [0.0, 0.0, 0.0]
            self.cmd_msg.acc_des = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            self.cmd_msg.life_count = (self.cmd_msg.life_count + 1) % 128
            self._send_cmd_now()
            time.sleep(duration_ms / 1000.0 + 0.2)
            self.send_stop()

    def move_forward_with_imu(self, distance_m: float, speed_mps: float = 0.2,
                              step_height: Tuple[float, float] = (0.05, 0.05)):
        """基于IMU距离统计的直线前进（无视觉矫正）"""
        if distance_m <= 0 or speed_mps <= 0:
            return
        
        print(f"🚶 基于IMU的直线前进 {distance_m}m @ {speed_mps}m/s...")
        
        # 重置IMU起始位置
        if not self.reset_odometry_start_position():
            print("⚠️ IMU里程计初始化失败，退化为时间控制")
            # 退化为时间控制
            duration_ms = max(200, int(abs(distance_m) / abs(speed_mps) * 1000))
            self.cmd_msg.mode = 11
            self.cmd_msg.gait_id = 27
            self.cmd_msg.vel_des = [float(np.sign(distance_m)) * abs(speed_mps), 0.0, 0.0]
            self.cmd_msg.step_height = [float(step_height[0]), float(step_height[1])]
            self.cmd_msg.duration = duration_ms
            self.cmd_msg.life_count = (self.cmd_msg.life_count + 1) % 128
            self._send_cmd_now()
            time.sleep(duration_ms / 1000.0 + 0.2)
            self.send_stop()
            return
        
        # 基于IMU的距离控制
        dt = 0.05  # 控制周期50ms
        monitor_count = 0
        last_log_time = 0.0
        
        while True:
            monitor_count += 1
            
            # 获取当前行走距离
            traveled_distance = self.get_traveled_distance()
            remaining_distance = distance_m - traveled_distance
            
            # 检查是否已到达目标距离
            if traveled_distance >= distance_m:
                print(f"\n🎯 到达目标距离! 最终距离: {traveled_distance:.3f}m")
                break
            
            # 发送控制指令（直线前进，无转向）
            self.cmd_msg.mode = 11
            self.cmd_msg.gait_id = 27
            self.cmd_msg.vel_des = [float(speed_mps), 0.0, 0.0]  # 直线前进
            self.cmd_msg.step_height = [float(step_height[0]), float(step_height[1])]
            self.cmd_msg.duration = int(dt * 1000)
            self.cmd_msg.life_count = (self.cmd_msg.life_count + 1) % 128
            self._send_cmd_now()
            
            # 定期日志输出
            current_time = time.time()
            if current_time - last_log_time > 0.5:
                progress = (traveled_distance / distance_m) * 100
                print(f"   ↳ IMU监控 #{monitor_count}: 距离 {traveled_distance:.3f}m ({progress:.1f}%) "
                      f"剩余 {remaining_distance:.3f}m")
                last_log_time = current_time
            
            time.sleep(dt)
            
            # 防止超时
            if monitor_count > 1000:  # 50秒超时
                print(f"\n⏰ IMU监控超时，停止行走")
                break
        
        self.send_stop()

    def turn_in_place(self, angle_deg: float, angular_speed_rad: float = 0.8,
                      step_height: Tuple[float, float] = (0.05, 0.05)):
        duration_ms = int(abs(angle_deg) * 26)  # ≈26ms/deg
        wz = angular_speed_rad if angle_deg > 0 else -angular_speed_rad
        self.cmd_msg.mode = 11
        self.cmd_msg.gait_id = 27
        self.cmd_msg.vel_des = [0.0, 0.0, float(wz)]
        self.cmd_msg.step_height = [float(step_height[0]), float(step_height[1])]
        self.cmd_msg.duration = duration_ms
        self.cmd_msg.life_count = (self.cmd_msg.life_count + 1) % 128
        self._send_cmd_now()
        time.sleep(duration_ms / 1000.0 + 1.0)
        self.send_stop()

    def lie_down(self, wait_s: float = 10.0):
        print(f"🤸 趴下等待 {wait_s}s...")
        end_time = time.time() + max(0.0, wait_s)
        while time.time() < end_time:
            self.cmd_msg.mode = 7  # 纯阻尼
            self.cmd_msg.gait_id = 0
            self.cmd_msg.duration = 0
            self.cmd_msg.life_count = (self.cmd_msg.life_count + 1) % 128
            self._send_cmd_now()
            time.sleep(0.3)

    # ----------------------------
    # 语音播报与触摸双击确认
    # ----------------------------
    def _init_speech_service(self, service_suffix: str = '/speech_text_play', timeout_sec: float = 5.0):
        self.speech_cli = None
        self.speech_service_name = ''
        if AudioTextPlay is None:
            self.get_logger().warn("未找到 AudioTextPlay 接口，语音播报不可用")
            return
        start_time = time.time()
        self.get_logger().info("正在查找语音服务...")
        while rclpy.ok() and time.time() - start_time < timeout_sec:
            for name, types in self.get_service_names_and_types():
                if name.endswith(service_suffix) and 'protocol/srv/AudioTextPlay' in types:
                    self.speech_service_name = name
                    self.get_logger().info(f"找到语音服务: '{self.speech_service_name}'")
                    break
            if self.speech_service_name:
                break
            time.sleep(0.3)
        if not self.speech_service_name:
            self.get_logger().warn("语音服务未找到，语音播报将跳过")
            return
        self.speech_cli = self.create_client(AudioTextPlay, self.speech_service_name)
        if not self.speech_cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(f"语音服务 '{self.speech_service_name}' 不可用")
            self.speech_cli = None

    def speak(self, text_to_speak: str):
        if AudioTextPlay is None or self.speech_cli is None:
            print(f"📢(模拟) {text_to_speak}")
            return
        try:
            req = AudioTextPlay.Request()
            req.module_name = 'all_l'
            req.is_online = True
            req.text = text_to_speak
            if AudioPlay is not None:
                req.speech = AudioPlay(module_name=req.module_name, play_id=0)
            future = self.speech_cli.call_async(req)
            # 不阻塞主流程，仅打印结果
            def _cb(fut):
                try:
                    resp = fut.result()
                    if getattr(resp, 'status', 1) == 0:
                        self.get_logger().info("语音播报成功")
                    else:
                        self.get_logger().warn("语音播报失败")
                except Exception as e:
                    self.get_logger().warn(f"语音播报回调异常: {e}")
            future.add_done_callback(_cb)
        except Exception as e:
            self.get_logger().warn(f"语音播报异常: {e}")

    def say_text(self, text: str) -> bool:
        """简化的语音播报接口，兼容all_r.py的调用方式"""
        self.speak(text)
        return True

    def _init_touch_subscription(self, topic_suffix: str = '/touch_status', timeout_sec: float = 5.0):
        self.touch_sub = None
        self.touch_topic_name = ''
        self.last_touch_state = 0
        self.last_double_tap_time = 0.0
        self.LPWG_DOUBLETAP_DETECTED = 0x03
        if TouchStatus is None:
            self.get_logger().warn("未找到 TouchStatus 接口，触摸交互不可用")
            return
        self.get_logger().info("正在查找触摸话题...")
        start_time = time.time()
        while rclpy.ok() and time.time() - start_time < timeout_sec:
            for name, types in self.get_topic_names_and_types():
                if name.endswith(topic_suffix) and 'protocol/msg/TouchStatus' in types:
                    self.touch_topic_name = name
                    break
            if self.touch_topic_name:
                break
            time.sleep(0.3)
        if not self.touch_topic_name:
            self.get_logger().warn("触摸话题未找到，触摸交互将跳过")
            return
        self.touch_sub = self.create_subscription(TouchStatus, self.touch_topic_name, self._touch_callback, 10)
        self.get_logger().info(f"已订阅触摸话题: {self.touch_topic_name}")

    def _touch_callback(self, msg):
        try:
            self.last_touch_state = getattr(msg, 'touch_state', 0)
            if self.last_touch_state == self.LPWG_DOUBLETAP_DETECTED:
                self.last_double_tap_time = time.time()
                self.get_logger().info("收到双击确认")
        except Exception:
            pass

    def wait_for_double_tap(self, timeout_s: float = 0.0, since_ts: float = None) -> bool:
        if self.touch_sub is None:
            try:
                self._init_touch_subscription()
            except Exception:
                pass
        base_ts = since_ts if since_ts is not None else time.time()
        start = time.time()
        print("⏳ 等待双击触摸确认... (超时0为不超时)")
        while True:
            if self.last_double_tap_time > base_ts:
                print("✅ 已收到双击，继续执行")
                return True
            if timeout_s > 0.0 and (time.time() - start) > timeout_s:
                print("⌛ 等待双击超时，继续执行")
                return False
            time.sleep(0.05)

    def lie_down_and_wait_touch(self, max_wait_s: float = 120.0):
        """趴下并等待触摸继续（参考all_r.py）"""
        print("🤸 趴下并等待触摸继续...")
        start = time.time()
        # 重置触摸状态
        self.last_double_tap_time = 0.0
        
        while True:
            # 维持趴下阻尼
            self.cmd_msg.mode = 7  # 纯阻尼
            self.cmd_msg.gait_id = 0
            self.cmd_msg.duration = 0
            self.cmd_msg.life_count = (self.cmd_msg.life_count + 1) % 128
            self._send_cmd_now()
            
            # 检查触摸（使用现有的双击检测）
            if self.last_double_tap_time > start:
                print("🖐️ 检测到触摸，继续后续流程")
                break
            if max_wait_s is not None and (time.time() - start) > max_wait_s:
                print("⏰ 等待触摸超时，继续后续流程")
                break
            time.sleep(0.2)

    def lie_down_announce_and_wait(self, arrive_text: str, max_wait_s: float = 120.0):
        """趴下，播报到达信息并等待触摸（参考all_r.py）"""
        print("🤸 趴下，播报到达信息并等待触摸...")
        start = time.time()
        # 重置触摸状态
        self.last_double_tap_time = 0.0
        announced = False
        
        while True:
            # 维持趴下阻尼
            self.cmd_msg.mode = 7  # 纯阻尼
            self.cmd_msg.gait_id = 0
            self.cmd_msg.duration = 0
            self.cmd_msg.life_count = (self.cmd_msg.life_count + 1) % 128
            self._send_cmd_now()
            
            # 播报到达信息（只播报一次）
            if not announced:
                self.say_text(arrive_text)
                announced = True
            
            # 检查触摸（使用现有的双击检测）
            if self.last_double_tap_time > start:
                print("🖐️ 检测到触摸，继续后续流程")
                break
            if max_wait_s is not None and (time.time() - start) > max_wait_s:
                print("⏰ 等待触摸超时，继续后续流程")
                break
            time.sleep(0.2)


    # ----------------------------
    # 绿箭头识别（重心偏移法）
    # ----------------------------
    def detect_green_arrow_direction(self, frame_bgr: np.ndarray,
                                     min_area: int = 300,
                                     max_area: int = 10000) -> Optional[str]:
        if frame_bgr is None:
            return None
        try:
            hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.green_lower, self.green_upper)
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

            ct_info = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = ct_info[1] if len(ct_info) == 3 else ct_info[0]
            if not contours:
                return None
            contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(contour)
            if not (min_area < area < max_area):
                return None
            x, y, w, h = cv2.boundingRect(contour)
            if not (w > h and (w / max(1, h)) > 1.2):
                return None
            M = cv2.moments(contour)
            if M["m00"] == 0:
                return None
            cX = int(M["m10"] / M["m00"])
            left_area = 0
            right_area = 0
            for pt in contour:
                px = pt[0][0]
                if px < cX:
                    left_area += 1
                elif px > cX:
                    right_area += 1
            if right_area > 1.2 * left_area:
                return "右箭头"
            if left_area > 1.2 * right_area:
                return "左箭头"
            return None
        except Exception:
            return None

    def wait_for_green_arrow_right(self, timeout_s: float = 30.0, stability_frames: int = 5) -> bool:
        print("=== 绿色箭头识别：目标=右箭头 ===")
        stable = 0
        start = time.time()
        while time.time() - start < timeout_s:
            img = self.get_rgb_image()
            if img is None:
                time.sleep(0.05)
                continue
            direction = self.detect_green_arrow_direction(img)
            if direction == "右箭头":
                stable += 1
                if stable >= stability_frames:
                    print("✅ 稳定识别：右箭头")
                    return True
            else:
                stable = 0
            time.sleep(0.05)
        print("⏰ 绿色箭头识别超时/失败")
        return False

    def wait_for_green_arrow_left(self, timeout_s: float = 30.0, stability_frames: int = 5) -> bool:
        print("=== 绿色箭头识别：目标=左箭头 ===")
        stable = 0
        start = time.time()
        while time.time() - start < timeout_s:
            img = self.get_rgb_image()
            if img is None:
                time.sleep(0.05)
                continue
            direction = self.detect_green_arrow_direction(img)
            if direction == "左箭头":
                stable += 1
                if stable >= stability_frames:
                    print("✅ 稳定识别：左箭头")
                    return True
            else:
                stable = 0
            time.sleep(0.05)
        print("⏰ 绿色箭头识别超时/失败")
        return False

    # ----------------------------
    # 黄色圆形检测（从new_all_r.py同步的先进逻辑）
    # ----------------------------
    def detect_yellow_circles(self, frame: np.ndarray, distance_traveled: float = None):
        """黄色圆形检测逻辑（V9: 与独立检测代码逻辑一致，检测所有符合条件的黄色圆形）"""
        if frame is None:
            return None, False
        
        # 获取图像尺寸
        img_height, img_width = frame.shape[:2]
        
        # 1. 预处理 (CLAHE + 高斯模糊)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        v_clahe = clahe.apply(v)
        hsv_normalized = cv2.merge([h, s, v_clahe])
        hsv_normalized = cv2.GaussianBlur(hsv_normalized, (7, 7), 0)
        
        # 2. 定义HSV阈值并创建掩码 - 优化黄色范围，避免检测红色
        lower_yellow = np.array([20, 100, 100])  # 更精确的黄色下限
        upper_yellow = np.array([35, 255, 255])  # 更精确的黄色上限
        mask = cv2.inRange(hsv_normalized, lower_yellow, upper_yellow)
        
        # 3. 形态学操作
        kernel_size = 9
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        mask_processed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=3)
        
        # 4. 定义并应用感兴趣区域 (ROI) - 上40%区域
        roi_mask = np.zeros(mask_processed.shape, dtype=np.uint8)
        cv2.rectangle(roi_mask, (0, 0), (img_width, img_height * 4 // 10), 255, -1)
        mask_roi = cv2.bitwise_and(mask_processed, mask_processed, mask=roi_mask)
        
        # 5. 在应用了ROI的最终掩码上寻找轮廓
        ct = cv2.findContours(mask_roi.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = ct[1] if len(ct) == 3 else ct[0]
        
        detection_count = 0
        detected_circles = []
        
        if contours:
            print(f"🔍 在ROI区域检测到 {len(contours)} 个黄色轮廓")
            
            # 面积和圆度阈值
            min_area = 1000  # 最小面积阈值
            max_area = 4000  # 最大面积阈值
            min_circularity = 0.4
            
            # 遍历所有轮廓
            for contour in contours:
                area = cv2.contourArea(contour)
                
                # 过滤面积不在范围内的轮廓
                if area <= min_area or area >= max_area:
                    continue
                
                (x, y), radius = cv2.minEnclosingCircle(contour)
                circle_area = np.pi * (radius ** 2)
                circularity = area / circle_area if circle_area > 0 else 0
                
                # 检查圆度
                if min_circularity < circularity < 1.4:
                    detection_count += 1
                    center = (int(x), int(y))
                    radius = int(radius)
                    detected_circles.append((center, radius, area, circularity))
                    
                    print(f"🔍 符合条件的黄色圆形 #{detection_count}: 中心{center}, 面积{area:.0f}, 圆度{circularity:.2f}")
            
            if detection_count > 0:
                # 选择面积最大的作为最佳检测结果
                best_circle = max(detected_circles, key=lambda x: x[2])  # 按面积排序
                center, radius, area, circularity = best_circle
                
                print(f"✅ 成功检测到黄色圆形！总数: {detection_count}, 最佳: 中心{center}, 面积{area:.0f}, 圆度{circularity:.2f}")
                
                # 返回检测结果：x, y, w, h, area, circularity, area/(w*h), score, strategy
                w = h = radius * 2  # 估算宽高
                score = int(area / 100)  # 简单的评分
                return (center[0], center[1], w, h, area, circularity, area/(w*h), score, "v9_optimized"), True
            else:
                print("⚠️ 未找到符合条件的黄色圆形（面积或圆度不满足）")
                return None, False
        else:
            print("🔍 在感兴趣区域(ROI)内未找到轮廓")
            return None, False

    def wait_for_yellow_frame(self, timeout_s: float = 30.0, initial_distance: float = 0.0) -> bool:
        print("=== 真正的边走边检测黄色圆形识别模式 ===")
        start = time.time()
        detection_count = 0
        last_save_time = 0.0  # 上次保存图片的时间
        save_interval = 0.5   # 保存间隔：0.5秒
        
        # 启动连续前进运动
        print(f"🚶 开始连续前进，速度 {AllConfig.YELLOW_DETECT_SPEED} m/s，同时进行黄灯检测...")
        
        # 重置IMU起始位置用于距离监控
        if not self.reset_odometry_start_position():
            print("⚠️ IMU里程计初始化失败，将基于时间估算距离")
            use_imu = False
        else:
            use_imu = True
        
        # 开始连续运动控制
        dt = 1.0 / AllConfig.YELLOW_DETECT_CONTROL_FREQ_HZ  # 控制周期基于配置频率
        last_distance_log = 0.0
        
        while time.time() - start < timeout_s:
            detection_count += 1
            current_time = time.time()
            
            # 获取当前行走距离
            if use_imu:
                traveled_distance = self.get_traveled_distance()
            else:
                # 基于时间估算距离
                traveled_distance = (current_time - start) * AllConfig.YELLOW_DETECT_SPEED
            
            # 检查是否已达到最大检测距离
            if traveled_distance >= AllConfig.YELLOW_DETECT_MAX_M:
                print(f"⏰ 已达到最大检测距离 {AllConfig.YELLOW_DETECT_MAX_M}m，停止前进")
                self.send_stop()
                break
            
            # 发送连续前进指令（使用黄线矫正）
            if AllConfig.ENABLE_YELLOW_CENTERING:
                # 使用黄线矫正的连续前进
                img_for_centering = self.get_rgb_image()
                steer = 0.0
                if img_for_centering is not None:
                    # 简化的黄线检测逻辑
                    h, w = img_for_centering.shape[:2]
                    roi_h = int(h * AllConfig.YL_ROI_BOTTOM_PERCENT)
                    roi = img_for_centering[-roi_h:, :]
                    
                    # 黄线检测和转向计算（简化版）
                    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                    yellow_mask = cv2.inRange(hsv_roi, self.yellow_lower, self.yellow_upper)
                    
                    # 寻找左右边线
                    left_x, right_x = None, None
                    roi_w = roi.shape[1]
                    left_half = yellow_mask[:, :roi_w//2]
                    right_half = yellow_mask[:, roi_w//2:]
                    
                    # 寻找左边线
                    left_contours = cv2.findContours(left_half, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    left_contours = left_contours[1] if len(left_contours) == 3 else left_contours[0]
                    if left_contours:
                        largest_left = max(left_contours, key=cv2.contourArea)
                        if cv2.contourArea(largest_left) > 100:
                            M = cv2.moments(largest_left)
                            if M["m00"] != 0:
                                left_x = int(M["m10"] / M["m00"])
                    
                    # 寻找右边线
                    right_contours = cv2.findContours(right_half, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    right_contours = right_contours[1] if len(right_contours) == 3 else right_contours[0]
                    if right_contours:
                        largest_right = max(right_contours, key=cv2.contourArea)
                        if cv2.contourArea(largest_right) > 100:
                            M = cv2.moments(largest_right)
                            if M["m00"] != 0:
                                right_x = int(M["m10"] / M["m00"]) + roi_w//2
                    
                    # 计算转向
                    if left_x is not None and right_x is not None:
                        line_center = (left_x + right_x) / 2
                        image_center = roi_w / 2
                        error = line_center - image_center
                        steer = -0.003 * error  # 简单P控制
                        steer = max(-0.3, min(0.3, steer))  # 限幅
                
                # 发送运动指令
                self.send_walk_cmd(AllConfig.YELLOW_DETECT_SPEED, 0.0, steer, duration_ms=int(dt*1000))
            else:
                # 不使用黄线矫正，直接前进
                self.send_walk_cmd(AllConfig.YELLOW_DETECT_SPEED, 0.0, 0.0, duration_ms=int(dt*1000))
            
            # 获取当前行走距离（从初始距离开始计算）
            current_distance = initial_distance + traveled_distance
            
            # 进行黄色圆形检测
            img = self.get_rgb_image()
            if img is not None:
                best, detected = self.detect_yellow_circles(img, current_distance)
                
                # 定期日志输出
                if current_time - last_distance_log > 0.5:
                    print(f"🚶 已行走: {traveled_distance:.2f}m (总距离: {current_distance:.2f}m) | 检测: {'✅' if detected else '❌'}")
                    last_distance_log = current_time
                
                if detected:
                    print(f"✅ 识别到黄色圆形: bbox={best[:4]}, area={best[4]:.0f}, circularity={best[5]:.2f}")
                    self.send_stop()
                    return True
            
            time.sleep(dt)
        
        print("⏰ 黄色圆形识别超时")
        self.send_stop()
        return False

    # ----------------------------
    # RGB黄线矫正系统（从all_r.py完整移植）
    # ----------------------------
    def follow_yellow_centering(self, distance_m: float, speed_mps: float) -> None:
        """基于 all_r.py 思路的双黄线居中行走（时间开环）

        - 底部ROI分左右，提取黄色连通域最大轮廓的中心；
        - 同时存在左右则取中点作为中线中心；单侧缺失则加固定像素偏移补偿；
        - error = line_center_x - image_center；steer = kp*avg_error，限幅并做变化约束；
        - 周期性发送 [vx, 0, -steer]；达到目标时间后停止。
        """
        if distance_m <= 0 or speed_mps <= 0:
            return
        print("🟨 双黄线居中直行...")
        duration_s = distance_m / speed_mps
        end_t = time.time() + duration_s
        last_tick = 0.0
        error_buf = deque(maxlen=AllConfig.YL_SMOOTH_BUFFER)
        prev_steer = 0.0
        # PID 状态
        integ = 0.0
        prev_err_norm = 0.0
        dt = AllConfig.YL_CTRL_TICK_S
        while time.time() < end_t:
            img = self.get_rgb_image()
            steer = 0.0
            detected = False
            if img is not None:
                h, w = img.shape[:2]
                roi_h = int(h * AllConfig.YL_ROI_BOTTOM_PERCENT)
                roi = img[h - roi_h: h, :, :]
                hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                lower = np.array(AllConfig.YL_HSV_LOWER, dtype=np.uint8)
                upper = np.array(AllConfig.YL_HSV_UPPER, dtype=np.uint8)
                mask = cv2.inRange(hsv, lower, upper)
                kernel = np.ones((3, 3), np.uint8)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

                mid = w // 2
                left_mask = mask[:, :mid]
                right_mask = mask[:, mid:]
                # 兼容 OpenCV 版本（有的返回2，有的返回3个返回值）
                ctL = cv2.findContours(left_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cntsL = ctL[0] if len(ctL) == 2 else ctL[1]
                ctR = cv2.findContours(right_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cntsR = ctR[0] if len(ctR) == 2 else ctR[1]

                cntsL = [c for c in cntsL if cv2.contourArea(c) > AllConfig.YL_MIN_CONTOUR_AREA]
                cntsR = [c for c in cntsR if cv2.contourArea(c) > AllConfig.YL_MIN_CONTOUR_AREA]

                left_cx = right_cx = None
                if len(cntsL) > 0:
                    largestL = max(cntsL, key=cv2.contourArea)
                    x, _, ww, _ = cv2.boundingRect(largestL)
                    left_cx = x + ww // 2
                if len(cntsR) > 0:
                    largestR = max(cntsR, key=cv2.contourArea)
                    x, _, ww, _ = cv2.boundingRect(largestR)
                    right_cx = mid + x + ww // 2

                if left_cx is not None and right_cx is not None:
                    line_center_x = (left_cx + right_cx) // 2
                    detected = True
                elif left_cx is not None:
                    line_center_x = left_cx + AllConfig.YL_FALLBACK_OFFSET_PX
                    detected = True
                elif right_cx is not None:
                    line_center_x = right_cx - AllConfig.YL_FALLBACK_OFFSET_PX
                    detected = True

                if detected:
                    image_center = w // 2
                    error = float(line_center_x - image_center)
                    error_buf.append(error)
                    avg_err = float(np.mean(error_buf)) if len(error_buf) > 0 else error
                    # 归一化误差（相对图宽），作为PID输入
                    err_norm = avg_err / float(max(1, w))
                    # PID
                    integ += err_norm * dt
                    integ = max(-AllConfig.YL_PID_INT_LIM, min(AllConfig.YL_PID_INT_LIM, integ))
                    deriv = (err_norm - prev_err_norm) / dt
                    prev_err_norm = err_norm
                    steer = AllConfig.YL_PID_KP * err_norm + AllConfig.YL_PID_KI * integ + AllConfig.YL_PID_KD * deriv
                    steer = max(-AllConfig.YL_MAX_TURN, min(AllConfig.YL_MAX_TURN, steer))
                    # 平滑变化
                    max_delta = AllConfig.YL_SMOOTH_MAX_DELTA
                    steer = max(prev_steer - max_delta, min(prev_steer + max_delta, steer))
                    prev_steer = steer
            # 发送控制
            self.cmd_msg.mode = 11
            self.cmd_msg.gait_id = 27
            yaw = -steer if detected else 0.0
            self.cmd_msg.vel_des = [float(speed_mps), 0.0, float(yaw)]
            self.cmd_msg.step_height = [0.05, 0.05]
            self.cmd_msg.duration = int(dt * 1000)
            self.cmd_msg.life_count = (self.cmd_msg.life_count + 1) % 128
            self._send_cmd_now()
            time.sleep(dt)
            if time.time() - last_tick > 0.6:
                print(f"   ↳ 黄线居中 detected={detected}, steer={steer:.3f}")
                last_tick = time.time()
        self.send_stop()

    def follow_yellow_centering_with_imu(self, distance_m: float, speed_mps: float) -> None:
        """基于IMU距离统计的双黄线居中行走（距离开环，替代时间开环）
        
        - 使用IMU里程计精确测量行走距离，而不是依赖时间估算
        - 实时监控黄线位置并调整转向，直到达到目标距离
        """
        if distance_m <= 0 or speed_mps <= 0:
            return
        
        print(f"🟨 基于IMU的双黄线居中直行 {distance_m}m @ {speed_mps}m/s...")
        
        # 重置IMU起始位置
        if not self.reset_odometry_start_position():
            print("⚠️ IMU里程计初始化失败，退化为时间开环")
            self.follow_yellow_centering(distance_m, speed_mps)
            return
        
        # 初始化控制参数
        last_tick = 0.0
        error_buf = deque(maxlen=AllConfig.YL_SMOOTH_BUFFER)
        prev_steer = 0.0
        integ = 0.0
        prev_err_norm = 0.0
        dt = AllConfig.YL_CTRL_TICK_S
        
        monitor_count = 0
        last_log_time = 0.0
        
        # 持续监控距离并控制
        while True:
            monitor_count += 1
            
            # 获取当前行走距离
            traveled_distance = self.get_traveled_distance()
            remaining_distance = distance_m - traveled_distance
            
            # 检查是否已到达目标距离
            if traveled_distance >= distance_m:
                print(f"\n🎯 到达目标距离! 最终距离: {traveled_distance:.3f}m")
                break
            
            # 检测黄线并计算转向
            img = self.get_rgb_image()
            steer = 0.0
            detected = False
            
            if img is not None:
                h, w = img.shape[:2]
                roi_h = int(h * AllConfig.YL_ROI_BOTTOM_PERCENT)
                roi = img[h - roi_h: h, :, :]
                hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                lower = np.array(AllConfig.YL_HSV_LOWER, dtype=np.uint8)
                upper = np.array(AllConfig.YL_HSV_UPPER, dtype=np.uint8)
                mask = cv2.inRange(hsv, lower, upper)
                kernel = np.ones((3, 3), np.uint8)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

                mid = w // 2
                left_mask = mask[:, :mid]
                right_mask = mask[:, mid:]
                
                # 兼容 OpenCV 版本
                ctL = cv2.findContours(left_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cntsL = ctL[0] if len(ctL) == 2 else ctL[1]
                ctR = cv2.findContours(right_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cntsR = ctR[0] if len(ctR) == 2 else ctR[1]

                cntsL = [c for c in cntsL if cv2.contourArea(c) > AllConfig.YL_MIN_CONTOUR_AREA]
                cntsR = [c for c in cntsR if cv2.contourArea(c) > AllConfig.YL_MIN_CONTOUR_AREA]

                left_cx = right_cx = None
                if len(cntsL) > 0:
                    largestL = max(cntsL, key=cv2.contourArea)
                    x, _, ww, _ = cv2.boundingRect(largestL)
                    left_cx = x + ww // 2
                if len(cntsR) > 0:
                    largestR = max(cntsR, key=cv2.contourArea)
                    x, _, ww, _ = cv2.boundingRect(largestR)
                    right_cx = mid + x + ww // 2

                if left_cx is not None and right_cx is not None:
                    line_center_x = (left_cx + right_cx) // 2
                    detected = True
                elif left_cx is not None:
                    line_center_x = left_cx + AllConfig.YL_FALLBACK_OFFSET_PX
                    detected = True
                elif right_cx is not None:
                    line_center_x = right_cx - AllConfig.YL_FALLBACK_OFFSET_PX
                    detected = True

                if detected:
                    image_center = w // 2
                    error = float(line_center_x - image_center)
                    error_buf.append(error)
                    avg_err = float(np.mean(error_buf)) if len(error_buf) > 0 else error
                    # 归一化误差（相对图宽），作为PID输入
                    err_norm = avg_err / float(max(1, w))
                    # PID
                    integ += err_norm * dt
                    integ = max(-AllConfig.YL_PID_INT_LIM, min(AllConfig.YL_PID_INT_LIM, integ))
                    deriv = (err_norm - prev_err_norm) / dt
                    prev_err_norm = err_norm
                    steer = AllConfig.YL_PID_KP * err_norm + AllConfig.YL_PID_KI * integ + AllConfig.YL_PID_KD * deriv
                    steer = max(-AllConfig.YL_MAX_TURN, min(AllConfig.YL_MAX_TURN, steer))
                    # 平滑变化
                    max_delta = AllConfig.YL_SMOOTH_MAX_DELTA
                    steer = max(prev_steer - max_delta, min(prev_steer + max_delta, steer))
                    prev_steer = steer
            
            # 发送控制指令
            self.cmd_msg.mode = 11
            self.cmd_msg.gait_id = 27
            yaw = -steer if detected else 0.0
            self.cmd_msg.vel_des = [float(speed_mps), 0.0, float(yaw)]
            self.cmd_msg.step_height = [0.05, 0.05]
            self.cmd_msg.duration = int(dt * 1000)
            self.cmd_msg.life_count = (self.cmd_msg.life_count + 1) % 128
            self._send_cmd_now()
            
            # 定期日志输出
            current_time = time.time()
            if current_time - last_log_time > 0.5:
                progress = (traveled_distance / distance_m) * 100
                print(f"   ↳ IMU监控 #{monitor_count}: 距离 {traveled_distance:.3f}m ({progress:.1f}%) "
                      f"剩余 {remaining_distance:.3f}m, 黄线检测={detected}, steer={steer:.3f}")
                last_log_time = current_time
            
            time.sleep(dt)
            
            # 防止超时
            if monitor_count > 1000:  # 50秒超时
                print(f"\n⏰ IMU监控超时，停止行走")
                break
        
        self.send_stop()

    def _walk_with_yellow_centering_and_detection(self, distance_m: float, speed_mps: float) -> None:
        """边走边检测黄灯的黄线居中行走（增强版）
        
        - 基于已有的IMU起始位置进行距离控制
        - 实时监控黄线位置并调整转向
        - 使用PID控制算法，参考all_r.py的实现
        """
        if distance_m <= 0 or speed_mps <= 0:
            return
        
        print(f"🟨 增强黄线矫正直行 {distance_m}m @ {speed_mps}m/s (边走边检测)...")
        
        # 获取当前IMU距离作为起始点
        start_imu_distance = self.get_traveled_distance() if hasattr(self, 'start_position') else 0.0
        target_distance = start_imu_distance + distance_m
        
        # 初始化控制参数（参考all_r.py）
        error_buf = deque(maxlen=AllConfig.YL_SMOOTH_BUFFER)
        prev_steer = 0.0
        integ = 0.0
        prev_err_norm = 0.0
        dt = AllConfig.YL_CTRL_TICK_S
        
        monitor_count = 0
        last_log_time = 0.0
        
        # 持续监控距离并控制
        while True:
            monitor_count += 1
            
            # 获取当前IMU距离
            current_imu_distance = self.get_traveled_distance() if hasattr(self, 'start_position') else start_imu_distance
            
            # 检查是否已到达目标距离
            if current_imu_distance >= target_distance:
                actual_traveled = current_imu_distance - start_imu_distance
                print(f"🎯 增强黄线矫正完成! 计划{distance_m:.2f}m, 实际{actual_traveled:.2f}m")
                break
            
            # 检测黄线并计算转向（增强版，参考all_r.py）
            img = self.get_rgb_image()
            steer = 0.0
            detected = False
            
            if img is not None:
                h, w = img.shape[:2]
                roi_h = int(h * AllConfig.YL_ROI_BOTTOM_PERCENT)
                roi = img[h - roi_h: h, :, :]
                hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                lower = np.array(AllConfig.YL_HSV_LOWER, dtype=np.uint8)
                upper = np.array(AllConfig.YL_HSV_UPPER, dtype=np.uint8)
                mask = cv2.inRange(hsv, lower, upper)
                kernel = np.ones((3, 3), np.uint8)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

                mid = w // 2
                left_mask = mask[:, :mid]
                right_mask = mask[:, mid:]
                
                # 兼容 OpenCV 版本的轮廓检测
                ctL = cv2.findContours(left_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cntsL = ctL[0] if len(ctL) == 2 else ctL[1]
                ctR = cv2.findContours(right_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cntsR = ctR[0] if len(ctR) == 2 else ctR[1]

                cntsL = [c for c in cntsL if cv2.contourArea(c) > AllConfig.YL_MIN_CONTOUR_AREA]
                cntsR = [c for c in cntsR if cv2.contourArea(c) > AllConfig.YL_MIN_CONTOUR_AREA]

                left_cx = right_cx = None
                if len(cntsL) > 0:
                    largestL = max(cntsL, key=cv2.contourArea)
                    x, _, ww, _ = cv2.boundingRect(largestL)
                    left_cx = x + ww // 2
                if len(cntsR) > 0:
                    largestR = max(cntsR, key=cv2.contourArea)
                    x, _, ww, _ = cv2.boundingRect(largestR)
                    right_cx = mid + x + ww // 2

                if left_cx is not None and right_cx is not None:
                    line_center_x = (left_cx + right_cx) // 2
                    detected = True
                elif left_cx is not None:
                    line_center_x = left_cx + AllConfig.YL_FALLBACK_OFFSET_PX
                    detected = True
                elif right_cx is not None:
                    line_center_x = right_cx - AllConfig.YL_FALLBACK_OFFSET_PX
                    detected = True

                if detected:
                    image_center = w // 2
                    error = float(line_center_x - image_center)
                    error_buf.append(error)
                    avg_err = float(np.mean(error_buf)) if len(error_buf) > 0 else error
                    # 归一化误差（相对图宽），作为PID输入
                    err_norm = avg_err / float(max(1, w))
                    # PID控制
                    integ += err_norm * dt
                    integ = max(-AllConfig.YL_PID_INT_LIM, min(AllConfig.YL_PID_INT_LIM, integ))
                    deriv = (err_norm - prev_err_norm) / dt
                    prev_err_norm = err_norm
                    steer = AllConfig.YL_PID_KP * err_norm + AllConfig.YL_PID_KI * integ + AllConfig.YL_PID_KD * deriv
                    steer = max(-AllConfig.YL_MAX_TURN, min(AllConfig.YL_MAX_TURN, steer))
                    # 平滑变化
                    max_delta = AllConfig.YL_SMOOTH_MAX_DELTA
                    steer = max(prev_steer - max_delta, min(prev_steer + max_delta, steer))
                    prev_steer = steer
            
            # 发送控制指令
            self.cmd_msg.mode = 11
            self.cmd_msg.gait_id = 27
            yaw = -steer if detected else 0.0
            self.cmd_msg.vel_des = [float(speed_mps), 0.0, float(yaw)]
            self.cmd_msg.step_height = [0.05, 0.05]
            self.cmd_msg.duration = int(dt * 1000)
            self.cmd_msg.life_count = (self.cmd_msg.life_count + 1) % 128
            self._send_cmd_now()
            
            # 定期日志输出
            current_time = time.time()
            if current_time - last_log_time > 0.5:
                remaining = target_distance - current_imu_distance
                progress = ((current_imu_distance - start_imu_distance) / distance_m) * 100
                print(f"   ↳ 增强矫正 #{monitor_count}: 距离 {current_imu_distance - start_imu_distance:.3f}m ({progress:.1f}%) "
                      f"剩余 {remaining:.3f}m, 黄线检测={detected}, steer={steer:.3f}")
                last_log_time = current_time
            
            time.sleep(dt)
            
            # 防止超时
            if monitor_count > 1000:  # 50秒超时
                print(f"\n⏰ 增强黄线矫正超时，停止行走")
                break
        
        self.send_stop()

    def _follow_yellow_centering_no_reset(self, distance_m: float, speed_mps: float) -> None:
        """黄线居中行走，不重置IMU起始位置（用于黄灯检测段）
        
        - 基于已有的IMU起始位置进行距离控制
        - 实时监控黄线位置并调整转向，直到达到目标距离
        """
        if distance_m <= 0 or speed_mps <= 0:
            return
        
        print(f"🟨 黄线矫正直行 {distance_m}m @ {speed_mps}m/s (不重置IMU)...")
        
        # 获取当前IMU距离作为起始点
        start_imu_distance = self.get_traveled_distance() if hasattr(self, 'start_position') else 0.0
        target_distance = start_imu_distance + distance_m
        
        # 初始化控制参数
        error_buf = deque(maxlen=AllConfig.YL_SMOOTH_BUFFER)
        prev_steer = 0.0
        integ = 0.0
        prev_err_norm = 0.0
        dt = AllConfig.YL_CTRL_TICK_S
        
        monitor_count = 0
        
        # 持续监控距离并控制
        while True:
            monitor_count += 1
            
            # 获取当前IMU距离
            current_imu_distance = self.get_traveled_distance() if hasattr(self, 'start_position') else start_imu_distance
            
            # 检查是否已到达目标距离
            if current_imu_distance >= target_distance:
                actual_traveled = current_imu_distance - start_imu_distance
                print(f"🎯 黄线矫正完成! 计划{distance_m:.2f}m, 实际{actual_traveled:.2f}m")
                break
            
            # 检测黄线并计算转向
            img = self.get_rgb_image()
            steer = 0.0
            detected = False
            
            if img is not None:
                h, w = img.shape[:2]
                roi_h = int(h * AllConfig.YL_ROI_BOTTOM_PERCENT)
                roi = img[h - roi_h: h, :, :]
                hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                lower = np.array(AllConfig.YL_HSV_LOWER, dtype=np.uint8)
                upper = np.array(AllConfig.YL_HSV_UPPER, dtype=np.uint8)
                mask = cv2.inRange(hsv, lower, upper)
                kernel = np.ones((3, 3), np.uint8)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

                mid = w // 2
                left_mask = mask[:, :mid]
                right_mask = mask[:, mid:]
                
                # 兼容 OpenCV 版本
                ctL = cv2.findContours(left_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cntsL = ctL[0] if len(ctL) == 2 else ctL[1]
                ctR = cv2.findContours(right_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cntsR = ctR[0] if len(ctR) == 2 else ctR[1]

                cntsL = [c for c in cntsL if cv2.contourArea(c) > AllConfig.YL_MIN_CONTOUR_AREA]
                cntsR = [c for c in cntsR if cv2.contourArea(c) > AllConfig.YL_MIN_CONTOUR_AREA]

                left_cx = right_cx = None
                if len(cntsL) > 0:
                    largestL = max(cntsL, key=cv2.contourArea)
                    x, _, ww, _ = cv2.boundingRect(largestL)
                    left_cx = x + ww // 2
                if len(cntsR) > 0:
                    largestR = max(cntsR, key=cv2.contourArea)
                    x, _, ww, _ = cv2.boundingRect(largestR)
                    right_cx = mid + x + ww // 2

                if left_cx is not None and right_cx is not None:
                    line_center_x = (left_cx + right_cx) // 2
                    detected = True
                elif left_cx is not None:
                    line_center_x = left_cx + AllConfig.YL_FALLBACK_OFFSET_PX
                    detected = True
                elif right_cx is not None:
                    line_center_x = right_cx - AllConfig.YL_FALLBACK_OFFSET_PX
                    detected = True

                if detected:
                    image_center = w // 2
                    error = float(line_center_x - image_center)
                    error_buf.append(error)
                    avg_err = float(np.mean(error_buf)) if len(error_buf) > 0 else error
                    # 归一化误差（相对图宽），作为PID输入
                    err_norm = avg_err / float(max(1, w))
                    # PID
                    integ += err_norm * dt
                    integ = max(-AllConfig.YL_PID_INT_LIM, min(AllConfig.YL_PID_INT_LIM, integ))
                    deriv = (err_norm - prev_err_norm) / dt
                    prev_err_norm = err_norm
                    steer = AllConfig.YL_PID_KP * err_norm + AllConfig.YL_PID_KI * integ + AllConfig.YL_PID_KD * deriv
                    steer = max(-AllConfig.YL_MAX_TURN, min(AllConfig.YL_MAX_TURN, steer))
                    # 平滑变化
                    max_delta = AllConfig.YL_SMOOTH_MAX_DELTA
                    steer = max(prev_steer - max_delta, min(prev_steer + max_delta, steer))
                    prev_steer = steer
            
            # 发送控制指令
            self.cmd_msg.mode = 11
            self.cmd_msg.gait_id = 27
            yaw = -steer if detected else 0.0
            self.cmd_msg.vel_des = [float(speed_mps), 0.0, float(yaw)]
            self.cmd_msg.step_height = [0.05, 0.05]
            self.cmd_msg.duration = int(dt * 1000)
            self.cmd_msg.life_count = (self.cmd_msg.life_count + 1) % 128
            self._send_cmd_now()
            
            time.sleep(dt)
            
            # 防止超时
            if monitor_count > 1000:  # 50秒超时
                print(f"\n⏰ 黄线矫正超时，停止行走")
                break
        
        self.send_stop()

    def follow_yellow_centering_with_step_height(self, distance_m: float, speed_mps: float, 
                                               step_height: Tuple[float, float] = (0.1, 0.1),
                                               value: int = 0) -> None:
        """基于双黄线居中行走（支持高抬腿步高和步态类型）
        
        - 与 follow_yellow_centering 相同的逻辑，但支持自定义步高和步态类型参数
        - 用于限高杆后的高抬腿前进
        - value: 步态类型，0=内八步态，2=垂直步态
        """
        if distance_m <= 0 or speed_mps <= 0:
            return
        gait_type = "内八步态" if value == 0 else "垂直步态" if value == 2 else f"步态{value}"
        print(f"🟨 双黄线居中直行（高抬腿 {step_height}，{gait_type}）...")
        duration_s = distance_m / speed_mps
        end_t = time.time() + duration_s
        last_tick = 0.0
        error_buf = deque(maxlen=AllConfig.YL_SMOOTH_BUFFER)
        prev_steer = 0.0
        # PID 状态
        integ = 0.0
        prev_err_norm = 0.0
        dt = AllConfig.YL_CTRL_TICK_S
        while time.time() < end_t:
            img = self.get_rgb_image()
            steer = 0.0
            detected = False
            if img is not None:
                h, w = img.shape[:2]
                roi_h = int(h * AllConfig.YL_ROI_BOTTOM_PERCENT)
                roi = img[h - roi_h: h, :, :]
                hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                lower = np.array(AllConfig.YL_HSV_LOWER, dtype=np.uint8)
                upper = np.array(AllConfig.YL_HSV_UPPER, dtype=np.uint8)
                mask = cv2.inRange(hsv, lower, upper)
                kernel = np.ones((3, 3), np.uint8)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

                mid = w // 2
                left_mask = mask[:, :mid]
                right_mask = mask[:, mid:]
                # 兼容 OpenCV 版本（有的返回2，有的返回3个返回值）
                ctL = cv2.findContours(left_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cntsL = ctL[0] if len(ctL) == 2 else ctL[1]
                ctR = cv2.findContours(right_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                cntsR = ctR[0] if len(ctR) == 2 else ctR[1]

                cntsL = [c for c in cntsL if cv2.contourArea(c) > AllConfig.YL_MIN_CONTOUR_AREA]
                cntsR = [c for c in cntsR if cv2.contourArea(c) > AllConfig.YL_MIN_CONTOUR_AREA]

                left_cx = right_cx = None
                if len(cntsL) > 0:
                    largestL = max(cntsL, key=cv2.contourArea)
                    x, _, ww, _ = cv2.boundingRect(largestL)
                    left_cx = x + ww // 2
                if len(cntsR) > 0:
                    largestR = max(cntsR, key=cv2.contourArea)
                    x, _, ww, _ = cv2.boundingRect(largestR)
                    right_cx = mid + x + ww // 2

                if left_cx is not None and right_cx is not None:
                    line_center_x = (left_cx + right_cx) // 2
                    detected = True
                elif left_cx is not None:
                    line_center_x = left_cx + AllConfig.YL_FALLBACK_OFFSET_PX
                    detected = True
                elif right_cx is not None:
                    line_center_x = right_cx - AllConfig.YL_FALLBACK_OFFSET_PX
                    detected = True

                if detected:
                    image_center = w // 2
                    error = float(line_center_x - image_center)
                    error_buf.append(error)
                    avg_err = float(np.mean(error_buf)) if len(error_buf) > 0 else error
                    # 归一化误差（相对图宽），作为PID输入
                    err_norm = avg_err / float(max(1, w))
                    # PID
                    integ += err_norm * dt
                    integ = max(-AllConfig.YL_PID_INT_LIM, min(AllConfig.YL_PID_INT_LIM, integ))
                    deriv = (err_norm - prev_err_norm) / dt
                    prev_err_norm = err_norm
                    steer = AllConfig.YL_PID_KP * err_norm + AllConfig.YL_PID_KI * integ + AllConfig.YL_PID_KD * deriv
                    steer = max(-AllConfig.YL_MAX_TURN, min(AllConfig.YL_MAX_TURN, steer))
                    # 平滑变化
                    max_delta = AllConfig.YL_SMOOTH_MAX_DELTA
                    steer = max(prev_steer - max_delta, min(prev_steer + max_delta, steer))
                    prev_steer = steer
            # 发送控制（使用自定义步高和步态类型）
            self.cmd_msg.mode = 11
            self.cmd_msg.gait_id = 27
            yaw = -steer if detected else 0.0
            self.cmd_msg.vel_des = [float(speed_mps), 0.0, float(yaw)]
            self.cmd_msg.step_height = [float(step_height[0]), float(step_height[1])]  # 使用自定义步高
            self.cmd_msg.value = value  # 设置步态类型
            self.cmd_msg.duration = int(dt * 1000)
            self.cmd_msg.life_count = (self.cmd_msg.life_count + 1) % 128
            self._send_cmd_now()
            time.sleep(dt)
            if time.time() - last_tick > 0.6:
                print(f"   ↳ 黄线居中（高抬腿） detected={detected}, steer={steer:.3f}")
                last_tick = time.time()
        self.send_stop()

    def move_forward_with_vision_correction(self, distance_m: float, speed_mps: float = 0.2,
                                             step_height: Tuple[float, float] = (0.05, 0.05), use_imu: bool = True):
        """带视觉校正的前进（使用黄线居中系统）"""
        if distance_m <= 0 or speed_mps <= 0:
            return
        print(f"🚶 视觉校正前进 {distance_m}m @ {speed_mps}m/s（使用黄线居中{'，启用IMU' if use_imu else ''}）")
        
        # 如果距离足够长，使用黄线居中系统
        if distance_m >= AllConfig.YL_MIN_DIST_M and AllConfig.ENABLE_YELLOW_CENTERING:
            if use_imu:
                # 优先使用IMU距离统计的黄线矫正
                self.follow_yellow_centering_with_imu(distance_m, speed_mps)
            else:
                # 使用时间开环的黄线矫正
                self.follow_yellow_centering(distance_m, speed_mps)
        else:
            # 距离太短，使用普通前进
            self.move_forward(distance_m, speed_mps=speed_mps, step_height=step_height)

    # ----------------------------
    # 二维码识别（pyzbar）
    # ----------------------------
    def detect_qr_code(self, image: np.ndarray):
        if image is None:
            return None
        try:
            corrected_img = image
            scales = [1.0, 0.8, 1.2, 0.6, 1.5]
            for scale in scales:
                if scale != 1.0:
                    resized = cv2.resize(corrected_img, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
                else:
                    resized = corrected_img.copy()
                gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)
                imgs = [
                    gray,
                    cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2),
                    cv2.GaussianBlur(gray, (5, 5), 0),
                ]
                for proc in imgs:
                    try:
                        dec = decode(proc)
                        if dec:
                            data = dec[0].data.decode('utf-8')
                            return data
                    except Exception:
                        pass
        except Exception:
            pass
        return None

    def wait_for_b_qr(self, timeout_s: float = 30.0) -> Optional[str]:
        print("=== B库二维码识别 ===")
        start = time.time()
        while time.time() - start < timeout_s:
            img = self.get_rgb_image()
            if img is None:
                time.sleep(0.05)
                continue
            data = self.detect_qr_code(img)
            if data:
                print(f"✅ 检测到二维码: {data}")
                return data
            time.sleep(0.05)
        print("⏰ 二维码识别超时")
        return None

    # ----------------------------
    # 黑框（限高杆）
    # ----------------------------
    def detect_black_frame(self, frame: np.ndarray):
        if frame is None:
            return False, None
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.black_lower, self.black_upper)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        ct = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = ct[1] if len(ct) == 3 else ct[0]
        if not contours:
            return False, None
        best = None
        best_score = 0.0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 800:
                continue
            x, y, w, h = cv2.boundingRect(cnt)
            if w < 30 or h < 20:
                continue
            rect_area = w * h
            recty = float(area) / float(rect_area) if rect_area > 0 else 0.0
            aspect = w / float(h)
            aspect_target = 90.0 / 40.0
            aspect_tol = 0.45
            aspect_score = max(0.0, 1.0 - (abs(aspect - aspect_target) / aspect_tol))
            recty_score = min(1.0, recty / 0.85)
            area_score = min(1.0, area / 20000.0)
            size_score = 0.7 if (w > 600 or h > 300) else 1.0
            score = aspect_score * 0.4 + recty_score * 0.3 + area_score * 0.2 + size_score * 0.1
            ar_ok = (aspect_target - aspect_tol) <= aspect <= (aspect_target + aspect_tol)
            rect_ok = recty >= 0.50
            geom_ok = (recty >= 0.9 and area >= 4000) or (area >= 4000 and 0.7 <= aspect <= 3.0)
            cond_ok = (score >= 0.50) or geom_ok or (ar_ok and rect_ok)
            if cond_ok and score > best_score:
                best_score = score
                best = (x, y, w, h, area, aspect, recty, score)
        return (best is not None), best

    def wait_for_black_barrier(self, timeout_s: float = 30.0) -> bool:
        print("=== 限高杆（黑框）识别 ===")
        start = time.time()
        while time.time() - start < timeout_s:
            img = self.get_rgb_image()
            if img is None:
                time.sleep(0.05)
                continue
            detected, info = self.detect_black_frame(img)
            if detected:
                print(f"✅ 识别到黑框：bbox={info[:4]}, area={info[4]:.0f}, ar={info[5]:.2f}, recty={info[6]:.2f}, s={info[7]:.2f}")
                return True
            time.sleep(0.03)
        print("⏰ 限高杆识别超时")
        return False

    # ----------------------------
    # 红色检测（从 all_r.py 移植）
    # ----------------------------
    def detect_red_frame(self, image_bgr: np.ndarray) -> bool:
        """检测图像中是否存在红色区域（纯颜色检测）
        
        Args:
            image_bgr: 输入的BGR图像
            
        Returns:
            bool: 是否检测到红色
        """
        if image_bgr is None:
            return False
        
        try:
            hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
            
            # 红色HSV阈值 - 收紧范围减少误检测
            mask1 = cv2.inRange(hsv, self.red_lower1, self.red_upper1)
            mask2 = cv2.inRange(hsv, self.red_lower2, self.red_upper2)
            mask = cv2.bitwise_or(mask1, mask2)
            
            # 形态学操作 - 加强去噪，减少误检测
            kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close)

            # 查找轮廓
            ct = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = ct[1] if len(ct) == 3 else ct[0]
            
            if contours:
                # 找到最大轮廓
                largest_contour = max(contours, key=cv2.contourArea)
                max_area = cv2.contourArea(largest_contour)
                
                # 纯颜色检测，无其他限制
                detected = len(contours) > 0 and max_area > 0
                
                # 调试信息（偶尔输出）
                if hasattr(self, '_red_debug_counter'):
                    self._red_debug_counter += 1
                else:
                    self._red_debug_counter = 1
                    
                if self._red_debug_counter % 20 == 0:  # 每20次检测输出一次详细信息
                    print(f"🔍 红色检测详情: 轮廓数={len(contours)}, 最大面积={max_area:.0f}px, 检测结果={'✅检测到' if detected else '❌未检测到'}")
                
                return detected
            else:
                return False
                
        except Exception as e:
            print(f"⚠️ 红色检测错误: {e}")
            return False

    def walk_until_red_lost(self, max_distance_m: float = 15.0, speed_mps: float = 0.15) -> bool:
        """边走边检测红色，直到检测不到红色或达到最大距离
        
        - max_distance_m: 最大前进距离（米）
        - speed_mps: 前进速度（m/s）
        返回：是否检测到红色消失（True=红色消失，应执行限高杆操作）
        """
        red_lost, _ = self.walk_until_red_lost_with_distance_tracking(max_distance_m, speed_mps)
        return red_lost

    def walk_until_red_lost_with_distance_tracking(self, max_distance_m: float = 15.0, speed_mps: float = 0.15) -> tuple:
        """边走边检测红色，直到检测不到红色或达到最大距离（带距离跟踪）
        
        - max_distance_m: 最大前进距离（米）
        - speed_mps: 前进速度（m/s）
        返回：(是否检测到红色消失, 实际行走距离)
        """
        print(f"🚶 开始边走边检测红色限高杆（最大距离 {max_distance_m}m @ {speed_mps} m/s）...")
        print("🔴 检测逻辑：识别到红色继续前进，识别不到红色则到达限高杆位置")
        
        step_size = 0.5  # 每次前进0.1米
        traveled = 0.0
        last_log_time = 0.0
        
        while traveled < max_distance_m:
            # 检测红色
            img = self.get_rgb_image()
            red_detected = False
            if img is not None:
                red_detected = self.detect_red_frame(img)
            
            # 实时日志输出检测状态
            current_time = time.time()
            if current_time - last_log_time > 1.0:  # 每1秒输出一次检测状态
                if red_detected:
                    print(f"   ↳ 已前进 {traveled:.1f}m，🔴 检测到红色，继续前进")
                else:
                    print(f"   ↳ 已前进 {traveled:.1f}m，⬛ 未检测到红色")
                last_log_time = current_time
            
            if not red_detected:
                print(f"✅ 红色消失！已前进 {traveled:.2f}m，到达限高杆位置")
                self.send_stop()  # 立即停止
                return True, traveled
            
            # 检测到红色，继续前进（启用黄线矫正）
            current_step = min(step_size, max_distance_m - traveled)
            if AllConfig.ENABLE_YELLOW_CENTERING and current_step >= AllConfig.YL_MIN_DIST_M:
                self.follow_yellow_centering_with_imu(current_step, speed_mps)
            else:
                self.move_forward(current_step, speed_mps=speed_mps)
            
            traveled += current_step
        
        print(f"⚠️ 达到最大距离 {max_distance_m}m，仍然检测到红色，可能未到达限高杆")
        self.send_stop()
        return False, traveled

    def walk_until_yellow_detected_with_distance_tracking(self, max_distance_m: float = 4.0, speed_mps: float = 0.22) -> tuple:
        """边走边检测黄灯，直到检测到黄灯或达到最大距离（带距离跟踪）
        
        - max_distance_m: 最大前进距离（米）
        - speed_mps: 前进速度（m/s）
        返回：(是否检测到黄灯, 实际行走距离)
        """
        print(f"🟡 开始边走边检测黄灯（最大距离 {max_distance_m}m @ {speed_mps} m/s）...")
        print("🟡 检测逻辑：边走边检测黄色矩形，识别到黄灯则停止")
        
        # 🔧 统一IMU起始位置管理
        if not self.reset_odometry_start_position():
            print("⚠️ IMU里程计初始化失败，退化为时间控制")
            # 使用时间控制作为备选方案
            duration_s = max_distance_m / speed_mps
            end_time = time.time() + duration_s
            while time.time() < end_time:
                img = self.get_rgb_image()
                if img is not None:
                    best, detected = self.detect_yellow_circles(img)
                    if detected:
                        elapsed = time.time() - (end_time - duration_s)
                        traveled_distance = elapsed * speed_mps
                        return True, min(traveled_distance, max_distance_m)
                time.sleep(0.1)
            return False, max_distance_m
        
        step_size = 0.5  # 每次前进0.5米
        traveled = 0.0
        last_log_time = 0.0
        
        while traveled < max_distance_m:
            # 检测黄灯
            img = self.get_rgb_image()
            yellow_detected = False
            if img is not None:
                best, detected = self.detect_yellow_circles(img)
                yellow_detected = detected
            
            # 实时日志输出检测状态
            current_time = time.time()
            if current_time - last_log_time > 1.0:  # 每1秒输出一次检测状态
                current_imu = self.get_traveled_distance() if hasattr(self, 'start_position') else -1
                if yellow_detected:
                    print(f"🟡 找到黄灯！位置: 计划{traveled:.1f}m | IMU{current_imu:.2f}m")
                else:
                    print(f"🔍 搜索中: 计划{traveled:.1f}m | IMU{current_imu:.2f}m | 剩余{max_distance_m-traveled:.1f}m")
                last_log_time = current_time
            
            if yellow_detected:
                print(f"✅ 检测到黄灯！已前进 {traveled:.2f}m")
                self.send_stop()  # 立即停止
                return True, traveled
            
            # 未检测到黄灯，继续前进（使用增强的黄线居中逻辑）
            current_step = min(step_size, max_distance_m - traveled)
            
            # 🔍 调试信息：记录移动前的IMU距离
            imu_before = self.get_traveled_distance() if hasattr(self, 'start_position') else -1
            
            # 🔧 使用增强的黄线居中前进（边走边检测黄灯+黄线矫正）
            print(f"🔍 条件判断: ENABLE_YELLOW_CENTERING={AllConfig.ENABLE_YELLOW_CENTERING}, current_step={current_step:.2f}m, YL_MIN_DIST_M={AllConfig.YL_MIN_DIST_M}m")
            if AllConfig.ENABLE_YELLOW_CENTERING and current_step >= AllConfig.YL_MIN_DIST_M:
                # 使用增强的黄线矫正（参考all_r.py的实现）
                print(f"✅ 使用增强的黄线矫正方法 (边走边检测)")
                self._walk_with_yellow_centering_and_detection(current_step, speed_mps)
            else:
                # 使用基础IMU移动（不重置起始位置）
                print(f"✅ 使用基础IMU移动（不重置起始位置）")
                self.move_forward(current_step, speed_mps=speed_mps, use_imu=False)
            
            # 🔍 调试信息：记录移动后的IMU距离
            imu_after = self.get_traveled_distance() if hasattr(self, 'start_position') else -1
            imu_delta = imu_after - imu_before if imu_before >= 0 and imu_after >= 0 else -1
            
            print(f"📏 第{int(traveled/step_size)+1}步: 计划{current_step:.1f}m | IMU: {imu_before:.2f}m→{imu_after:.2f}m (增量{imu_delta:.2f}m)")
            
            # 🔧 使用IMU实际距离更新traveled
            if imu_after >= 0:
                traveled = imu_after  # 使用IMU实际距离
            else:
                traveled += current_step  # 退化为计划距离
        
        # 📊 最终总结
        final_imu_distance = self.get_traveled_distance() if hasattr(self, 'start_position') else -1
        total_steps = int(traveled / step_size)
        print(f"⚠️ 黄灯检测完成：未在{max_distance_m}m内找到黄灯")
        print(f"📊 总结: 走了{total_steps}步 | 计划{traveled:.1f}m | IMU实测{final_imu_distance:.2f}m")
        if final_imu_distance > 0:
            error = abs(final_imu_distance - traveled)
            if error > 0.5:
                print(f"❌ 距离误差过大: {error:.2f}m (IMU可能异常)")
            else:
                print(f"✅ 距离误差正常: {error:.2f}m")
        self.send_stop()
        return False, traveled

    # ----------------------------
    # 过限高杆步态（正/倒）
    # ----------------------------
    def execute_custom_gait_front(self):
        self._execute_custom_gait(backward=False)

    def execute_custom_gait_backward(self):
        self._execute_custom_gait(backward=True)

    def _execute_custom_gait(self, backward=False):
        robot_cmd = {
            "mode": 0, "gait_id": 0, "contact": 0, "life_count": 0,
            "vel_des": [0.0, 0.0, 0.0],
            "rpy_des": [0.0, 0.0, 0.0],
            "pos_des": [0.0, 0.0, 0.0],
            "acc_des": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "ctrl_point": [0.0, 0.0, 0.0],
            "foot_pose": [0.0] * 12,
            "step_height": [0.0, 0.0],
            "value": 0, "duration": 0,
        }

        try:
            self.stand_up()

            gait_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "gaits", "moonwalk2.toml")
            if not os.path.exists(gait_path):
                legacy_path = "/home/mi/boqi/gaits/moonwalk2.toml"
                gait_path = legacy_path if os.path.exists(legacy_path) else gait_path
            if not os.path.exists(gait_path) or toml is None:
                if backward:
                    print("⚠️ 步态文件不存在或toml不可用，退化为低速倒车1.0m模拟过杆")
                    self.move_forward(-1.0, speed_mps=0.10, step_height=(0.12, 0.12))
                else:
                    print("⚠️ 步态文件不存在或toml不可用，退化为低速直行1.0m模拟过杆")
                    self.move_forward(1.0, speed_mps=0.10, step_height=(0.12, 0.12))
                return

            steps = toml.load(gait_path)
            full_steps = {"step": []}
            for i in steps.get("step", []):
                cmd = copy.deepcopy(robot_cmd)
                cmd["duration"] = i.get("duration", 0)
                t = i.get("type", "")
                if t == "usergait":
                    cmd["mode"] = 11
                    cmd["gait_id"] = i.get("gait_id", 90)
                    vel_des = i.get("body_vel_des", [0.0, 0.0, 0.0])
                    if backward and len(vel_des) >= 1:
                        vel_des[0] = -vel_des[0]
                    cmd["vel_des"] = vel_des
                    bpd = i.get("body_pos_des", [0.0]*6)
                    cmd["rpy_des"] = bpd[0:3]
                    cmd["pos_des"] = bpd[3:6]
                    cmd["foot_pose"] = i.get("landing_pos_des", [0.0]*12)
                    cmd["step_height"] = i.get("step_height", [0.0, 0.0])
                    cmd["acc_des"] = i.get("weight", [0.0]*6)
                    cmd["value"] = i.get("use_mpc_traj", 0)
                    cmd["contact"] = int(math.floor(i.get("landing_gain", 0.0) * 10))
                    cp = cmd.get("ctrl_point", [0.0, 0.0, 0.0])
                    if len(cp) >= 3:
                        cp[2] = i.get("mu", 0.0)
                        cmd["ctrl_point"] = cp
                elif t == "torctrlposture":
                    cmd["mode"] = 1
                    cmd["gait_id"] = 0
                elif t == "locomotion":
                    cmd["mode"] = 11
                    cmd["gait_id"] = i.get("gait_id", 31)
                    vel_des = i.get("vel_des", [0.0, 0.0, 0.0])
                    if backward and len(vel_des) >= 1:
                        vel_des[0] = -vel_des[0]
                    cmd["vel_des"] = vel_des
                elif t == "recoverystand":
                    cmd["mode"] = 12
                    cmd["gait_id"] = 0
                full_steps["step"].append(cmd)

            if backward:
                print("🚧 执行Moonwalk倒车过限高杆步态（最多10s）...")
            else:
                print("🚧 执行Moonwalk过限高杆步态（最多10s）...")
            start_time = time.time()
            for step in full_steps["step"]:
                if time.time() - start_time >= 10.0:
                    print("⏹️ 步态执行达到10秒上限，提前结束")
                    break
                self.cmd_msg.mode = step.get("mode", 11)
                self.cmd_msg.value = step.get("value", 0)
                self.cmd_msg.contact = step.get("contact", 0)
                self.cmd_msg.gait_id = step.get("gait_id", 31)
                self.cmd_msg.duration = step.get("duration", 100)
                self.cmd_msg.life_count = (self.cmd_msg.life_count + 1) % 128
                self.cmd_msg.vel_des = step.get("vel_des", [0.0, 0.0, 0.0])
                self.cmd_msg.rpy_des = step.get("rpy_des", [0.0, 0.0, 0.0])
                self.cmd_msg.pos_des = step.get("pos_des", [0.0, 0.0, 0.0])
                self.cmd_msg.acc_des = step.get("acc_des", [0.0]*6)
                self.cmd_msg.foot_pose = step.get("foot_pose", [0.0]*12)
                self.cmd_msg.ctrl_point = step.get("ctrl_point", [0.0, 0.0, 0.0])
                sh = step.get("step_height", [0.0, 0.0])
                if len(sh) >= 2:
                    self.cmd_msg.step_height = [float(sh[0]), float(sh[1])]
                else:
                    self.cmd_msg.step_height = [0.05, 0.05]
                self._send_cmd_now()
                time.sleep(0.1)

            print("🔄 步态完成后维持心跳 15s...")
            for _ in range(75):
                self._send_cmd_now()
                time.sleep(0.2)

        except KeyboardInterrupt:
            self.cmd_msg.mode = 7
            self.cmd_msg.gait_id = 0
            self.cmd_msg.duration = 0
            self.cmd_msg.life_count = (self.cmd_msg.life_count + 1) % 128
            self._send_cmd_now()
            print("⚠️ 中断，进入阻尼模式")
        except Exception as e:
            print(f"❌ 过限高杆步态执行错误: {e}")


def main():
    node: Optional[AllNodeL] = None
    spin_thread: Optional[Thread] = None
    try:
        rclpy.init(args=sys.argv)
        node = AllNodeL()
        node.start()
        spin_thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()

        if node.wait_for_rgb(timeout_s=5.0) is None:
            print("⚠️ RGB相机未就绪，仍继续流程（可能无法识别）")

        # 站立
        node.stand_up()

        node.say_text("左侧路线")

        node.move_forward(AllConfig.ARROW_FWD1_M, speed_mps=AllConfig.ARROW_FWD1_SPEED)
        node.turn_in_place(+90)
        node.move_forward(AllConfig.ARROW_FWD2_M, speed_mps=AllConfig.ARROW_FWD2_SPEED)
        node.turn_in_place(-90)
        print(f"🚶 石板路阶段（{AllConfig.STONE_ROAD_M}m，高抬腿，垂直步态，视觉校正）...")
        # 使用限高杆后的高抬腿配置 + 垂直步态
        if AllConfig.ENABLE_YELLOW_CENTERING and AllConfig.STONE_ROAD_M >= AllConfig.YL_MIN_DIST_M:
            node.follow_yellow_centering_with_step_height(AllConfig.STONE_ROAD_M, speed_mps=AllConfig.STONE_ROAD_SPEED, step_height=AllConfig.POST_GAIT_HIGH_STEP, value=1)
        else:
            node.move_forward(AllConfig.STONE_ROAD_M, speed_mps=AllConfig.STONE_ROAD_SPEED, step_height=AllConfig.POST_GAIT_HIGH_STEP, value=1)

        # 2) 限高杆：动态距离分配，总距离恒定3.0米
        print("🚶 限高杆阶段：总距离3.0米，动态分配识别前后距离...")
        
        # 记录动态分配的总距离
        TOTAL_BARRIER_DISTANCE = 3.0  # 总距离恒定3.0米
        
        red_lost, traveled_distance = node.walk_until_red_lost_with_distance_tracking(
            max_distance_m=TOTAL_BARRIER_DISTANCE,  # 最大搜索距离即为总距离
            speed_mps=AllConfig.HEAD_COLLISION_SPEED
        )
        
        if red_lost:
            print(f"✅ 红色消失，检测到限高杆位置！已前进{traveled_distance:.2f}米")
            node.say_text("识别到限高杆")
            
            # 计算剩余需要前进的距离
            remaining_distance = TOTAL_BARRIER_DISTANCE - traveled_distance
            print(f"📏 动态分配：识别前{traveled_distance:.2f}m + 过杆后{remaining_distance:.2f}m = 总计{TOTAL_BARRIER_DISTANCE}m")
            
            # 检测到限高杆后，后退一小段距离准备过杆
            print(f"🚶 检测到限高杆，后退{AllConfig.HEAD_COLLISION_BACKUP_M}米准备过杆...")
            node.move_forward(-AllConfig.HEAD_COLLISION_BACKUP_M, speed_mps=AllConfig.BLACK_APPROACH_SPEED)
            
            # 原地掉头
            node.turn_in_place(AllConfig.BLACK_AFTER_TARGET_TURN_DEG)
            # 执行过限高杆（倒车版本）
            node.execute_custom_gait_backward()
            # 过完后原地掉头
            node.turn_in_place(AllConfig.POST_GAIT_TURN_DEG)
            
            # 正常步态前进剩余距离（动态分配）
            print(f"🚶 正常步态前进{remaining_distance:.2f}米（动态分配，启用黄线矫正+IMU）...")
            if AllConfig.ENABLE_YELLOW_CENTERING and remaining_distance >= AllConfig.YL_MIN_DIST_M:
                node.follow_yellow_centering_with_imu(remaining_distance, AllConfig.POST_GAIT_FORWARD_SPEED)
            else:
                node.move_forward(remaining_distance, speed_mps=AllConfig.POST_GAIT_FORWARD_SPEED)
        else:
            print(f"⚠️ 达到总距离{TOTAL_BARRIER_DISTANCE}米仍检测到红色，未到达限高杆")
            print(f"📏 已完成限高杆阶段总距离{TOTAL_BARRIER_DISTANCE}米，跳过过杆步态")

        # 3) 到二维码路径：直走0 -> 右90 -> 直走1 -> 左90
        node.move_forward(AllConfig.TO_QR_FWD1_M, speed_mps=AllConfig.TO_QR_FWD1_SPEED)
        node.turn_in_place(-90)
        node.move_forward(AllConfig.TO_QR_FWD2_M, speed_mps=AllConfig.TO_QR_FWD2_SPEED)
        node.turn_in_place(+90)

        # 4) 二维码识别（B库），若失败则用 Doubao 文本识别辅助判断
        qr = node.wait_for_b_qr(timeout_s=10.0)
        decide_b1 = False
        decide_b2 = False
        if qr:
            norm = qr.strip().lower().replace('-', '').replace('_', '').replace(' ', '')
            decide_b1 = ('b1' in norm)
            decide_b2 = ('b2' in norm)
            # 播报二维码识别结果
            if decide_b1:
                print("✅ 二维码识别到B1")
                node.say_text("识别到B1")
            elif decide_b2:
                print("✅ 识别到B2")
                node.say_text("识别到B2")
        if not (decide_b1 or decide_b2):
            print("⚠️ 未识别到有效二维码，尝试使用 Doubao 文字识别辅助决策…")
            try:
                proc = subprocess.run([sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'doubao_seed_flash_ocr_qr.py')],
                                      cwd=os.path.dirname(os.path.abspath(__file__)), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                      universal_newlines=True, timeout=20)
                out = (proc.stdout or '').strip()
                if out:
                    obj = json.loads(out.splitlines()[-1])
                    key = str(obj.get('文字识别结果', '（）'))
                    if 'B' in key or '（B' in key or 'b' in key:
                        k2 = key.replace('（', '').replace('）', '').replace(' ', '').upper()
                        decide_b1 = ('B-1' in k2) or ('B1' in k2)
                        decide_b2 = ('B-2' in k2) or ('B2' in k2)
                        # 播报文字识别结果
                        if decide_b1:
                            print("✅ 文字识别到B1")
                            node.say_text("识别到B1")
                        elif decide_b2:
                            print("✅ 文字识别到B2")
                            node.say_text("识别到B2")
                        else:
                            print("⚠️ 文字识别无效内容")
                            node.say_text("识别到B1")
                    else:
                        print("⚠️ 文字识别无相关内容")
                        node.say_text("识别到B1")
                else:
                    print("⚠️ 文字识别无输出")
                    node.say_text("识别到B1")
            except Exception as _e:
                print(f"⚠️ Doubao 辅助识别失败: {_e}")
                node.say_text("识别到B1")
        is_b1 = decide_b1
        is_b2 = decide_b2
        if is_b2:
            print("=== 识别到B2二维码，进入B2路径 ===")
            # B2路径：右转90° -> 直行1m -> 左转90° -> 直行1m -> 卸货
            node.turn_in_place(AllConfig.B1_TURN1_DEG)  # 右转90°
            node.move_forward(AllConfig.B1_FWD1_M, speed_mps=AllConfig.B1_FWD1_SPEED)
            node.turn_in_place(AllConfig.B1_TURN2_DEG)  # 左转90°
            node.move_forward(AllConfig.B1_FWD2_M, speed_mps=AllConfig.B1_FWD2_SPEED)
            # 卸货：趴下后，先播报到达B2库，再等待触摸继续
            node.lie_down_announce_and_wait("已到达B2库", max_wait_s=120.0)

            node.stand_up()
            # B2卸货后到B1装货路径：后退1m -> 左转90° -> 直行2m -> 右转90° -> 直行1m -> 装货
            node.move_forward(AllConfig.B2_BACK_M, speed_mps=AllConfig.B2_BACK_SPEED)  # 后退
            node.turn_in_place(AllConfig.B2_TURN1_DEG)  # 左转90°
            node.move_forward(AllConfig.B2_FWD1_M, speed_mps=AllConfig.B2_FWD1_SPEED)
            node.turn_in_place(AllConfig.B2_TURN2_DEG)  # 右转90°
            node.move_forward(AllConfig.B2_FWD2_M, speed_mps=AllConfig.B2_FWD2_SPEED)
            # B1装货：趴下后，先播报到达B1库，再等待触摸继续
            node.lie_down_announce_and_wait("已到达B1库", max_wait_s=120.0)
            
            # B2装货后回程路径：后退1m -> 左转180° -> 直行1m
            node.stand_up()
            node.move_forward(AllConfig.B2_BACK_M, speed_mps=AllConfig.B2_BACK_SPEED)  # 后退1米
            node.turn_in_place(-90)     # 右转90°
            node.move_forward(AllConfig.B2_FWD1_M, speed_mps=AllConfig.B2_FWD1_SPEED)   # 直走2米
            node.turn_in_place(-90)     # 右转90°
            
            # 4米黄灯动态识别段（B2路径）
            print(f"🟡 B2路径：开始{AllConfig.YELLOW_DETECT_M}米黄灯动态识别段（总距离{AllConfig.YELLOW_DETECT_M}米）...")
            yellow_detected, traveled_distance = node.walk_until_yellow_detected_with_distance_tracking(
                max_distance_m=AllConfig.YELLOW_DETECT_M,  # 总距离4米
                speed_mps=AllConfig.YELLOW_DETECT_SPEED
            )
            
            if yellow_detected:
                print(f"✅ 识别到黄灯！已前进{traveled_distance:.2f}米")
                node.move_forward(0.3, speed_mps=AllConfig.YELLOW_DETECT_SPEED)
                node.say_text("识别到黄灯，倒计时5秒")
                print("🛑 停止并等待5秒...")
                node.send_stop()
                # 倒计时5秒
                for i in range(5, 0, -1):
                    print(f"⏰ 倒计时: {i}")
                    node.say_text(str(i))
                    time.sleep(1.0)
                
                # 计算剩余需要前进的距离
                remaining_distance = AllConfig.YELLOW_DETECT_M - traveled_distance
                print(f"📏 B2动态分配：识别前{traveled_distance:.2f}m + 识别后{remaining_distance:.2f}m = 总计{AllConfig.YELLOW_DETECT_M}m")
                
                # 完成剩余距离
                if remaining_distance > 0.01:  # 避免浮点误差
                    if AllConfig.ENABLE_YELLOW_CENTERING and remaining_distance >= AllConfig.YL_MIN_DIST_M:
                        node.follow_yellow_centering_with_imu(remaining_distance, speed_mps=AllConfig.YELLOW_DETECT_SPEED)
                    else:
                        node.move_forward(remaining_distance, speed_mps=AllConfig.YELLOW_DETECT_SPEED)
            else:
                print("⚠️ 4米内未识别到黄灯，继续后续流程")

            # 黄灯后的正确路线：1.5m前进 → 180°转弯 → 1.5m后退上坡 → 1m坡间路 → 1.5m下坡
            print("🚶 开始黄灯后正确路线（B2路径）...")
            
            # 1. 直行1.5米（使用RGB矫正，时间控制避免IMU重置）
            print(f"🚶 直行{AllConfig.YELLOW_AFTER_FWD1_M}米（使用RGB矫正，时间控制）...")
            if AllConfig.ENABLE_YELLOW_CENTERING and AllConfig.YELLOW_AFTER_FWD1_M >= AllConfig.YL_MIN_DIST_M:
                node.follow_yellow_centering(AllConfig.YELLOW_AFTER_FWD1_M, speed_mps=AllConfig.YELLOW_AFTER_FWD1_SPEED)
            else:
                node.move_forward(AllConfig.YELLOW_AFTER_FWD1_M, speed_mps=AllConfig.YELLOW_AFTER_FWD1_SPEED)
            
            # 2. 转弯180°
            print("🔄 转弯180°...")
            node.turn_in_place(180)     # 左转180°
            
            # 3. 后退1.5米（上坡）
            print(f"🚶 后退{abs(AllConfig.YELLOW_AFTER_BACK_M)}米（上坡）...")
            node.move_forward(AllConfig.YELLOW_AFTER_BACK_M, speed_mps=AllConfig.YELLOW_AFTER_BACK_SPEED, step_height=(0.1, 0.1))
            
            # 3.5. 转弯180°
            print("🔄 转弯180°...")
            node.turn_in_place(180)     # 左转180°
            
            # 4. 直走1米（坡间路，使用RGB矫正，时间控制）
            print(f"🚶 直走{AllConfig.YELLOW_AFTER_FWD2_M}米（坡间路，使用RGB矫正，时间控制）...")
            if AllConfig.ENABLE_YELLOW_CENTERING and AllConfig.YELLOW_AFTER_FWD2_M >= AllConfig.YL_MIN_DIST_M:
                node.follow_yellow_centering(AllConfig.YELLOW_AFTER_FWD2_M, speed_mps=AllConfig.YELLOW_AFTER_FWD2_SPEED)
            else:
                node.move_forward(AllConfig.YELLOW_AFTER_FWD2_M, speed_mps=AllConfig.YELLOW_AFTER_FWD2_SPEED)
            
            # 5. 下坡直走1.5米（使用RGB矫正，时间控制）
            print(f"🚶 下坡直走{AllConfig.YELLOW_AFTER_FWD3_M}米（使用RGB矫正，时间控制）...")
            if AllConfig.ENABLE_YELLOW_CENTERING and AllConfig.YELLOW_AFTER_FWD3_M >= AllConfig.YL_MIN_DIST_M:
                node.move_forward(AllConfig.YELLOW_AFTER_FWD3_M, speed_mps=AllConfig.YELLOW_AFTER_FWD3_SPEED, step_height=(0.02, 0.02))
            else:
                node.move_forward(AllConfig.YELLOW_AFTER_FWD3_M, speed_mps=AllConfig.YELLOW_AFTER_FWD3_SPEED, step_height=(0.02, 0.02))
            
            # 6. 停下
            print("🛑 到达终点，停止运动")

        else:
            if is_b1:
                print("=== 识别到B1二维码，进入B1路径 ===")
            else:
                print("⚠️ 未识别到二维码或非B1/B2，默认执行B1路径")

            node.turn_in_place(+90)     # 左90
            node.move_forward(AllConfig.B1_FWD1_M, speed_mps=AllConfig.B1_FWD1_SPEED)
            node.turn_in_place(-90)     # "有转"按右90处理
            node.move_forward(AllConfig.B1_FWD2_M, speed_mps=AllConfig.B1_FWD2_SPEED)
            # 到达B1库：趴下，播报并等待触摸
            node.lie_down_announce_and_wait("已到达B1库", max_wait_s=120.0)

            node.stand_up()
            node.move_forward(AllConfig.B2_BACK_M, speed_mps=AllConfig.B2_BACK_SPEED)
            node.turn_in_place(-90)     # 右90
            node.move_forward(AllConfig.B2_FWD1_M, speed_mps=AllConfig.B2_FWD1_SPEED)
            node.turn_in_place(+90)     # 左90
            node.move_forward(AllConfig.B2_FWD2_M, speed_mps=AllConfig.B2_FWD2_SPEED)
            # 到达B2库：趴下，播报并等待触摸
            node.lie_down_announce_and_wait("已到达B2库", max_wait_s=120.0)
            
            # B1路径：从B2出库到黄灯检测
            node.stand_up()
            node.move_forward(AllConfig.B2_BACK_M, speed_mps=AllConfig.B2_BACK_SPEED)  # 后退1米
            node.turn_in_place(-180)    # 右转180°
            
            # 4米黄灯动态识别段（B1路径）
            print(f"🟡 B1路径：开始{AllConfig.YELLOW_DETECT_M}米黄灯动态识别段（总距离{AllConfig.YELLOW_DETECT_M}米）...")
            yellow_detected, traveled_distance = node.walk_until_yellow_detected_with_distance_tracking(
                max_distance_m=AllConfig.YELLOW_DETECT_M,  # 总距离4米
                speed_mps=AllConfig.YELLOW_DETECT_SPEED
            )
            
            if yellow_detected:
                print(f"✅ 识别到黄灯！已前进{traveled_distance:.2f}米")
                node.move_forward(0.3, speed_mps=AllConfig.YELLOW_DETECT_SPEED)
                node.say_text("识别到黄灯，倒计时5秒")
                print("🛑 停止并等待5秒...")
                node.send_stop()
                # 倒计时5秒
                for i in range(5, 0, -1):
                    print(f"⏰ 倒计时: {i}")
                    node.say_text(str(i))
                    time.sleep(1.0)
                
                # 计算剩余需要前进的距离
                remaining_distance = AllConfig.YELLOW_DETECT_M - traveled_distance
                print(f"📏 B1动态分配：识别前{traveled_distance:.2f}m + 识别后{remaining_distance:.2f}m = 总计{AllConfig.YELLOW_DETECT_M}m")
                
                # 完成剩余距离
                if remaining_distance > 0.01:  # 避免浮点误差
                    if AllConfig.ENABLE_YELLOW_CENTERING and remaining_distance >= AllConfig.YL_MIN_DIST_M:
                        node.follow_yellow_centering_with_imu(remaining_distance, speed_mps=AllConfig.YELLOW_DETECT_SPEED)
                    else:
                        node.move_forward(remaining_distance, speed_mps=AllConfig.YELLOW_DETECT_SPEED)
            else:
                print("⚠️ 4米内未识别到黄灯，继续后续流程")

            # 黄灯后的正确路线：1.5m前进 → 180°转弯 → 1.5m后退上坡 → 1m坡间路 → 1.5m下坡
            print("🚶 开始黄灯后正确路线（B1路径）...")
            
            # 1. 直行1.5米（使用RGB矫正，时间控制避免IMU重置）
            print(f"🚶 直行{AllConfig.YELLOW_AFTER_FWD1_M}米（使用RGB矫正，时间控制）...")
            if AllConfig.ENABLE_YELLOW_CENTERING and AllConfig.YELLOW_AFTER_FWD1_M >= AllConfig.YL_MIN_DIST_M:
                node.follow_yellow_centering(AllConfig.YELLOW_AFTER_FWD1_M, speed_mps=AllConfig.YELLOW_AFTER_FWD1_SPEED)
            else:
                node.move_forward(AllConfig.YELLOW_AFTER_FWD1_M, speed_mps=AllConfig.YELLOW_AFTER_FWD1_SPEED)
            
            # 2. 转弯180°
            print("🔄 转弯180°...")
            node.turn_in_place(180)     # 左转180°
            
            # 3. 后退1.5米（上坡）
            print(f"🚶 后退{abs(AllConfig.YELLOW_AFTER_BACK_M)}米（上坡）...")
            node.move_forward(AllConfig.YELLOW_AFTER_BACK_M, speed_mps=AllConfig.YELLOW_AFTER_BACK_SPEED,step_height=0.1)
            
            # 3.5. 转弯180°
            print("🔄 转弯180°...")
            node.turn_in_place(180)     # 左转180°
            
            # 4. 直走1米（坡间路，使用RGB矫正，时间控制）
            print(f"🚶 直走{AllConfig.YELLOW_AFTER_FWD2_M}米（坡间路，使用RGB矫正，时间控制）...")
            if AllConfig.ENABLE_YELLOW_CENTERING and AllConfig.YELLOW_AFTER_FWD2_M >= AllConfig.YL_MIN_DIST_M:
                node.follow_yellow_centering(AllConfig.YELLOW_AFTER_FWD2_M, speed_mps=AllConfig.YELLOW_AFTER_FWD2_SPEED)
            else:
                node.move_forward(AllConfig.YELLOW_AFTER_FWD2_M, speed_mps=AllConfig.YELLOW_AFTER_FWD2_SPEED)
            
            # 5. 下坡直走1.5米（使用RGB矫正，时间控制）
            print(f"🚶 下坡直走{AllConfig.YELLOW_AFTER_FWD3_M}米（使用RGB矫正，时间控制）...")
            if AllConfig.ENABLE_YELLOW_CENTERING and AllConfig.YELLOW_AFTER_FWD3_M >= AllConfig.YL_MIN_DIST_M:
                node.move_forward(AllConfig.YELLOW_AFTER_FWD3_M, speed_mps=AllConfig.YELLOW_AFTER_FWD3_SPEED,step_height=(0.04, 0.04))
            else:
                node.move_forward(AllConfig.YELLOW_AFTER_FWD3_M, speed_mps=AllConfig.YELLOW_AFTER_FWD3_SPEED,step_height=(0.04, 0.04))
            
            # 6. 停下
            print("🛑 到达终点，停止运动")

        # 结束
        node.send_stop()
        print("✅ 任务完成")

        print("\n📛 中断，准备停止...")
    except Exception as e:
        print(f"❌ 任务执行错误: {e}")
    finally:
        try:
            if node is not None:
                node.running = 0
                node.send_stop()
                # 等待IMU里程计线程结束
                if hasattr(node, 'odometry_thread') and node.odometry_thread.is_alive():
                    node.odometry_thread.join(timeout=2.0)
        except Exception:
            pass
        if node is not None and spin_thread is not None and spin_thread.is_alive():
            try:
                rclpy.shutdown()
                spin_thread.join(timeout=1.5)
            except Exception:
                pass
        elif rclpy.ok():
            rclpy.shutdown()
        print("程序退出，资源已释放")


if __name__ == '__main__':
    main()
