#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ALL：单文件全流程（B库）

流程：
1) 站立 -> RGB识别绿色箭头（来自 green_arrow_detector 思路）
   - 若识别为右箭头：直行0.5m -> 右转90° -> 直行1m -> 左转90° -> 直行6m
2) 到达横向黄线区域 -> RGB检测空中横向黄线（来自 detect_yellow_in_2png 思路）
   - 检出后停止5秒
3) 直行4m -> 左转90° -> 直行1m -> 右转90°
4) 使用RGB识别B库二维码（来自 stand_and_qr_detect 思路）
   - 若识别为 b1：
     右转90° -> 直行1m -> 左转90° -> 直行1m -> 趴下10s（卸货）
     站立 -> 后退1m -> 左转90° -> 直行2m -> 右转90° -> 直行1m -> 趴下10s（装货）
     转180° -> 直行4.5m -> RGB识别黑框限高杆（来自 test_realtime_black_frame 思路）
     识别后执行过限高杆步态（来自 yellow_run 思路） -> 停止

说明：
- 不调用项目内其它脚本文件，所有识别与运动控制逻辑内联；
- 仅导入LCM消息定义（robot_control_cmd_lcmt / robot_control_response_lcmt）和三方库；
- 过限高杆步态尝试读取 gaits/moonwalk2.toml；若缺失或 toml 不可用，将退化为低速直行1m。
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
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
from pyzbar.pyzbar import decode

# LCM 消息（仅导入消息定义）
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from robot_control_response_lcmt import robot_control_response_lcmt

# toml 用于过限高杆步态
try:
    import toml
except Exception:
    toml = None

# 触摸与语音服务（可选）
try:
    from protocol.srv import AudioTextPlay
    from protocol.msg import AudioPlay, TouchStatus
    HAS_PROTOCOL = True
except Exception:
    AudioTextPlay = None
    AudioPlay = None
    TouchStatus = None
    HAS_PROTOCOL = False

# 可选导入：IMU里程计模块（用于里程兜底）
try:
    from modules.imu_controller import IMUController as _IMUController
    HAS_IMU_MODULE = True
except Exception:
    _IMUController = None
    HAS_IMU_MODULE = False


class AllConfig:
    """集中配置参数，便于调参

    说明：以下均为常用可调参数，单位在字段名或注释中给出；
    修改这些值可直接影响主流程的运动与识别阈值，无需改动逻辑代码。
    """

    # 识别与等待（秒/帧）
    WAIT_RGB_TIMEOUT_S: float = 5.0        # 等待首次RGB帧超时（s）

    QR_TIMEOUT_S: float = 20.0             # 二维码识别超时（s）
    BLACK_BARRIER_TIMEOUT_S: float = 40.0  # 黑框（限高杆）识别超时（s）

    # 识别到黑框后，基于雷达的距离触发
    LIDAR_FRONT_SECTOR_DEG: tuple = (-30.0, 30.0)  # 前向扇区角度范围（度），默认±30°
    LIDAR_THRESHOLD_M: float = 0.15                # 触发阈值：前向最近距离≤该值（米）
    LIDAR_WAIT_TIMEOUT_S: float = 25.0             # 等待距离触发的超时（s）
    LIDAR_STABLE_TICKS: int = 3                    # 连续满足阈值的检测次数（抗抖动）
    LIDAR_SMOOTH_WINDOW: int = 5                   # 距离滑窗中值窗口大小（点）
    # 新增：黑框阶段"一米就位"触发
    LIDAR_TARGET_RANGE_M: float = 1.0              # 目标距离（米）
    LIDAR_TARGET_TOL_M: float = 0.10               # 容差（米）→ 判定区间 [target-tol, target+tol]
    BLACK_APPROACH_SPEED: float = 0.15             # 行走中靠近的速度（m/s）
    BLACK_AFTER_TARGET_FWD_M: float = 0.8          # 达到目标距离后，向前再走的距离（米）
    BLACK_AFTER_TARGET_FWD_SPEED: float = 0.20     # 上述前进速度（m/s）
    BLACK_AFTER_TARGET_TURN_DEG: float = -180.0    # 随后原地掉头角度（度）- 右转180°
    POST_GAIT_TURN_DEG: float = +180.0             # 过杆步态完成后再次掉头角度（度）- 左转180°
    POST_GAIT_FORWARD_M: float = 5.0               # 最后高抬腿直行距离（米）
    POST_GAIT_FORWARD_SPEED: float = 0.15           # 最后直行速度（m/s）
    POST_GAIT_HIGH_STEP: tuple = (0.1, 0.1)        # 高抬腿步高（m）
    
    # 头部传感器碰撞检测配置（已改为动态分配，不再使用固定15米）
    BARRIER_DETECT_MAX_DISTANCE_M: float = 6.0     # 限高杆检测段最大前进距离（米）- 动态分配用
    BARRIER_DETECT_SPEED: float = 0.15             # 限高杆检测时的前进速度（m/s）
    HEAD_COLLISION_BACKUP_M: float = 0.0           # 检测到碰撞后的后退距离（米）

    # 箭头路径后续动作参数（单位：米/米每秒/度）
    ARROW_FWD1_M: float = 0.9         # 识别到右箭头后：第一段直行距离（m）
    ARROW_FWD1_SPEED: float = 0.20     # 第一段直行速度（m/s）
    ARROW_TURN1_DEG: float = -100.0     # 右转90°（负号表示右转）
    ARROW_FWD2_M: float = 1.00         # 第二段直行距离（m）
    ARROW_FWD2_SPEED: float = 0.22     # 第二段直行速度（m/s）
    ARROW_TURN2_DEG: float = -90.    # 右转96°（负号表示右转）
    # 倒车段参数（倒着上坡1.5米）
    ARROW_BACK_M: float = -2.5      # 倒车距离（m，负号表示倒车）
    ARROW_BACK_SPEED: float = 0.22     # 倒车速度（m/s）
    # 转弯后前进段参数
    ARROW_TURN3_DEG: float = +180.0    # 转弯180°
    
    # 坡上段参数（转弯后直行，启用黄线矫正）
    UPHILL_FWD_M: float = 0.0          # 坡上直行距离（m）
    UPHILL_FWD_SPEED: float = 0.12     # 坡上直行速度（m/s）
    UPHILL_STEP_HEIGHT: tuple = (0.09, 0.09)  # 坡上自定义步高（m）
    
    # 下坡段参数（降低速度）
    DOWNHILL_FWD_M: float = 2.5       # 下坡直行距离（m）
    DOWNHILL_FWD_SPEED: float = 0.15   # 下坡直行速度（m/s，降低速度）
    
    # 黄色圆形区域总距离配置（动态分配）
    TOTAL_YELLOW_SECTION_M: float = 3.5            # 黄色圆形区域总距离（m）- 识别段+后续段
    YELLOW_DETECT_MAX_M: float = 2.5               # 黄色圆形识别段最大距离（m）
    YELLOW_DETECT_SPEED: float = 0.14              # 黄色圆形识别段速度（m/s）- 降低速度以提高识别准确率
    YELLOW_AFTER_DETECT_SPEED: float = 0.25        # 识别到黄灯后的前进速度（m/s）- 可以走快点
    # 黄色圆形识别逻辑：边走边检测，可能提前结束
    YLAMP_VISUAL_TRIGGER_H_RATIO: float = 0.15     # 触发阈值：bbox高度/图像高度 ≥ 此比例（黄色圆形降低要求）
    YLAMP_TRIGGER_STABLE_FRAMES: int = 1           # 连续满足帧数（一次即触发）
    # 多策略检测的不同高度比例触发阈值（提高精度要求）
    YLAMP_STRATEGY_H_RATIOS: dict = {              # 不同策略的高度比例阈值
        "标准": 0.10,      # 提高标准策略要求，减少误检
        "宽松": 0.08,      # 适度提高宽松策略要求
        "极宽松": 0.06     # 极宽松策略仍保持一定要求
    }
    # 基于距离的策略选择配置（目标停车距离：50cm ± 15cm）
    YLAMP_DISTANCE_STRATEGY_MAP: dict = {          # 距离范围到策略的映射
        "远距离": {"min_dist": 0.8, "max_dist": float('inf'), "strategy": "标准", "desc": "远距离精确检测"},
        "接近距离": {"min_dist": 0.5, "max_dist": 0.8, "strategy": "宽松", "desc": "接近阶段平衡检测"},
        "目标距离": {"min_dist": 0.35, "max_dist": 0.65, "strategy": "标准", "desc": "理想停车区域"},
        "过近距离": {"min_dist": 0.0, "max_dist": 0.35, "strategy": "极宽松", "desc": "过近紧急检测"}
    }
    # 横向黄线后段参数（动态计算距离）
    SEG1_FWD1_SPEED: float = 0.22                  # 横向黄线后直行速度（m/s）
    SEG1_TURN1_DEG: float = +95.0      # 左转（度）
    SEG1_FWD2_M: float = 1.3           # 第二段直行（m）
    SEG1_FWD2_SPEED: float = 0.22      # 第二段速度（m/s）
    SEG1_TURN2_DEG: float = -95.0      # 右转（度）- 修正为标准95度

    # B1路径（卸货段，单位：米/米每秒/度/秒）
    B1_TURN1_DEG: float = -90.0        # 右转
    B1_FWD1_M: float = 1.0             # 直行（m）
    B1_FWD1_SPEED: float = 0.20        # 速度（m/s）
    B1_TURN2_DEG: float = +90.0        # 左转
    B1_FWD2_M: float = 1.0             # 直行（m）
    B1_FWD2_SPEED: float = 0.18        # 速度（m/s）

    # B2装货段（单位：米/米每秒/度/秒）
    B2_BACK_M: float = -1.0            # 后退距离（m，负号为后退）
    B2_BACK_SPEED: float = 0.18        # 后退速度（m/s）
    B2_TURN1_DEG: float = +90.0        # 左转
    B2_FWD1_M: float = 2.0             # 直行（m）
    B2_FWD1_SPEED: float = 0.20        # 速度（m/s）
    B2_TURN2_DEG: float = -90.0        # 右转
    B2_FWD2_M: float = 1.0             # 直行（m）
    B2_FWD2_SPEED: float = 0.18        # 速度（m/s）
    RETURN_TURN_DEG: float = +180.0       # 原地转向（度）
    RETURN_NO_VISION_M: float = 0.0       # 回程启用视觉前，先不启用视觉直行（m）
    
    # 限高杆长直道总距离配置（动态分配）
    TOTAL_BARRIER_SECTION_M: float = 9.0           # 限高杆长直道总距离（m）- 回程段+检测段
    RETURN_TO_BARRIER_MAX_M: float = 3.0           # 回程段最大距离（m）
    RETURN_FWD_SPEED: float = 0.22                 # 回程直行速度（m/s）

    # 视觉近距离触发（当雷达不稳定时作为可靠备选）
    VISION_BLACK_MIN_HEIGHT_RATIO: float = 0.22    # 黑框bbox高度占整幅高度比例阈值（0-1）
    VISION_BLACK_STABLE_FRAMES: int = 3            # 连续满足帧数
    VISION_BLACK_TIMEOUT_S: float = 12.0           # 等待视觉近距离触发超时（s）

    # 近距离前探（分段前进）
    APPROACH_STEP_M: float = 0.05                  # 每步前进距离（m）
    APPROACH_SPEED: float = 0.10                   # 前探速度（m/s）
    APPROACH_MAX_STEPS: int = 10                   # 最多步数（总距离≈ step*max_steps）

    # 视觉姿态对齐/边线守护/中线PID 已移除，避免冗余（保持文件简洁）

    # 黄线"左右边线取中线"居中（参考 yellow_centering_module.py 逻辑）
    ENABLE_YELLOW_CENTERING: bool = True           # 是否启用该居中方案（优先级最高）
    YL_MIN_DIST_M: float = 1.0                     # 仅当直行距离≥该值时启用
    YL_ROI_BOTTOM_PERCENT: float = 0.50            # ROI为图像底部百分比高度
    YL_HSV_LOWER: tuple = (10, 40, 40)             # HSV下界（结合demo调参）
    YL_HSV_UPPER: tuple = (45, 255, 255)           # HSV上界（拉宽）
    YL_MIN_CONTOUR_AREA: int = 300                 # 最小轮廓面积
    YL_FALLBACK_OFFSET_PX: int = 200               # 单侧缺失时的像素偏移补偿
    YL_KP: float = 0.009                           # 比例增益（转向）
    YL_MAX_TURN: float = 0.4                       # 最大转向（rad/s）
    YL_SMOOTH_BUFFER: int = 5                      # 误差平滑窗口
    YL_SMOOTH_MAX_DELTA: float = 0.20              # 相邻转向的最大变化量
    YL_CTRL_TICK_S: float = 0.05                   # 控制周期
    # PID（方向控制）
    YL_PID_KP: float = 2.0                         # 比例增益（基于归一化偏差）
    YL_PID_KI: float = 0.30                        # 积分增益
    YL_PID_KD: float = 0.40                        # 微分增益
    YL_PID_INT_LIM: float = 0.40                   # 积分限幅
    # 相机选择：黄线矫正使用RGB相机
    CAMERA_FOR_YELLOW: str = "rgb"
    CAMERA_WAIT_TIMEOUT_S: float = 5.0

    # 黄色的HSV阈值（基于V9优化算法精确调优）
    YELLOW_HSV_LOWER: tuple = (20, 100, 100)  # 更精确的黄色下限，避免误检红色
    YELLOW_HSV_UPPER: tuple = (35, 255, 255)  # 更精确的黄色上限
    
    # ROI（感兴趣区域）配置 - V9优化
    YELLOW_ROI_UPPER_RATIO: float = 0.4      # ROI上半部分比例（0.4表示图像上40%）
    YELLOW_ROI_MIN_AREA: int = 1000           # ROI区域内的最小面积阈值
    YELLOW_ROI_MAX_AREA: int = 4000           # ROI区域内的最大面积阈值
    YELLOW_ROI_MIN_CIRCULARITY: float = 0.4   # ROI区域内的最小圆度要求
    YELLOW_ROI_MAX_CIRCULARITY: float = 1.4   # ROI区域内的最大圆度要求
    
    # 真正边走边检测配置
    YELLOW_DETECT_CONTROL_FREQ_HZ: float = 10.0   # 控制频率（Hz），即每秒检测次数


    # （保留空位）


class AllNode(Node):
    """ALL单文件节点：LCM运动+RGB识别（箭头/黄矩形/二维码/黑框）"""

    def __init__(self):
        super().__init__('all_node')

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

        # 相机 (仅RGB，用于黄线矫正)
        self.bridge = CvBridge()
        self.latest_rgb_image = None
        self.image_lock = Lock()
        self.rgb_sub = self.create_subscription(Image, '/image_rgb', self._rgb_callback, 10)
        print("✅ 已订阅RGB相机: /image_rgb")

        # 触摸与语音（可选）
        self.last_touch_state = None
        self.last_touch_time = 0.0
        self.touch_triggered = False
        
        # 头部传感器（用于限高杆碰撞检测）
        self.head_collision_detected = False
        self.last_head_touch_time = 0.0
        self.audio_client = None
        self.audio_service_name = ""
        # 直接订阅固定话题；若需动态可扩展为遍历话题后再订阅
        self.touch_sub = None
        if HAS_PROTOCOL:
            try:
                # 优先动态查找以 /touch_status 结尾的话题
                self._try_subscribe_touch_suffix('/touch_status')
                if self.touch_sub is None:
                    # 回退到默认名称
                    self.touch_sub = self.create_subscription(TouchStatus, '/touch_status', self._touch_callback, 10)
                    print("ℹ️ 未发现动态触摸话题，已回退订阅 /touch_status")
            except Exception as e:
                print(f"⚠️ 触摸话题订阅失败: {e}")
                self.touch_sub = None
            
            # 尝试订阅头部传感器话题
            try:
                self.head_touch_sub = self.create_subscription(TouchStatus, '/head_touch_status', self._head_touch_callback, 10)
                print("✅ 已订阅头部传感器: /head_touch_status")
            except Exception as e:
                print(f"⚠️ 头部传感器订阅失败: {e}")
                self.head_touch_sub = None
        else:
            print("⚠️ 未检测到 protocol 包，触摸/语音交互不可用")
            self.head_touch_sub = None

        # 雷达（LaserScan）
        self.latest_scan = None
        self.scan_lock = Lock()
        self.lidar_angles_deg = None
        self.lidar_dist_window = deque(maxlen=AllConfig.LIDAR_SMOOTH_WINDOW)
        # 话题名参考 lidar_test_demo.py
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/mi_desktop_48_b0_2d_7a_fe_a2/scan',
            self._lidar_callback,
            10
        )



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

        # IMU里程计数据（参考hybrid_walk_demo.py和all_l.py）
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

        print("✅ ALL 节点初始化完成")

    # ----------------------------
    # 基础通信与相机
    # ----------------------------
    def start(self):
        self.lc_r.subscribe("robot_control_response", self._msg_handler)
        self.rec_thread.start()
        self.send_thread.start()
        print("✅ LCM 线程已启动，等待RGB图像...")

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



    # ----------------------------
    # 触摸与语音
    # ----------------------------
    def _touch_callback(self, msg):
        try:
            # 参考 touch_interaction: 双击/长按触发
            if msg.touch_state in (0x03, 0x07):
                self.last_touch_state = msg.touch_state
                self.last_touch_time = time.time()
                self.touch_triggered = True
        except Exception:
            pass

    def _head_touch_callback(self, msg):
        try:
            # 头部传感器碰撞检测：任何接触状态都认为是碰撞
            if msg.touch_state > 0:
                self.head_collision_detected = True
                self.last_head_touch_time = time.time()
                print(f"🚨 头部传感器触发！状态: 0x{msg.touch_state:02x}")
        except Exception:
            pass

    def _try_subscribe_touch_suffix(self, suffix: str, timeout_s: float = 3.0):
        try:
            start = time.time()
            while time.time() - start < timeout_s and self.touch_sub is None:
                names_types = self.get_topic_names_and_types()
                for name, types in names_types:
                    if name.endswith(suffix) and 'protocol/msg/TouchStatus' in types:
                        self.touch_sub = self.create_subscription(TouchStatus, name, self._touch_callback, 10)
                        print(f"✅ 已订阅触摸话题: {name}")
                        return
                time.sleep(0.3)
        except Exception:
            pass

    def _ensure_audio_client(self) -> bool:
        if not HAS_PROTOCOL:
            return False
        try:
            if self.audio_client is not None and self.audio_client.service_is_ready():
                return True
            # 动态查找以 /speech_text_play 结尾的服务
            names_types = self.get_service_names_and_types()
            found = None
            for name, types in names_types:
                if name.endswith('/speech_text_play') and 'protocol/srv/AudioTextPlay' in types:
                    found = name
                    break
            if not found:
                return False
            self.audio_service_name = found
            self.audio_client = self.create_client(AudioTextPlay, found)
            return self.audio_client.wait_for_service(timeout_sec=2.0)
        except Exception:
            return False

    def say_ciallo(self):
        if not self._ensure_audio_client():
            print("⚠️ 语音服务不可用，跳过播报")
            return False
        try:
            req = AudioTextPlay.Request()
            req.module_name = 'all_node'
            req.is_online = True
            req.text = 'ciallo'
            req.speech = AudioPlay(module_name=req.module_name, play_id=0)
            future = self.audio_client.call_async(req)
            # 非阻塞等待，给一点时间让服务处理
            timeout = time.time() + 3.0
            while rclpy.ok() and time.time() < timeout and not future.done():
                time.sleep(0.05)
            if future.done():
                try:
                    resp = future.result()
                    print(f"🗣️ 播报结果: status={getattr(resp,'status',-1)}")
                except Exception:
                    pass
            return True
        except Exception:
            return False

    def say_text(self, text: str) -> bool:
        if not self._ensure_audio_client():
            print("⚠️ 语音服务不可用，跳过播报")
            return False
        try:
            req = AudioTextPlay.Request()
            req.module_name = 'all_node'
            req.is_online = True
            req.text = text
            req.speech = AudioPlay(module_name=req.module_name, play_id=0)
            future = self.audio_client.call_async(req)
            timeout = time.time() + 3.0
            while rclpy.ok() and time.time() < timeout and not future.done():
                time.sleep(0.05)
            return True
        except Exception:
            return False

    def get_rgb_image(self) -> Optional[np.ndarray]:
        with self.image_lock:
            if self.latest_rgb_image is None:
                return None
            return self.latest_rgb_image.copy()

    def wait_for_image(self, camera_type: str = "rgb", timeout_s: float = 5.0) -> Optional[np.ndarray]:
        start = time.time()
        while time.time() - start < timeout_s:
            img = self.get_rgb_image()
            if img is not None:
                return img
            time.sleep(0.1)
        return None

    def _lidar_callback(self, msg: LaserScan):
        try:
            # 计算每个测量点角度（度）
            num_points = len(msg.ranges)
            if num_points == 0:
                return
            angles = np.linspace(msg.angle_min, msg.angle_max, num_points)
            angles_deg = (np.degrees(angles) + 360.0) % 360.0
            with self.scan_lock:
                self.lidar_angles_deg = angles_deg
                self.latest_scan = msg
        except Exception:
            pass

    def get_front_min_distance(self,
                               sector_deg: Tuple[float, float] = (-30.0, 30.0),
                               min_valid: float = 0.1,
                               max_valid: float = 10.0) -> Optional[float]:
        with self.scan_lock:
            if self.latest_scan is None or self.lidar_angles_deg is None:
                return None
            ranges = np.array(self.latest_scan.ranges)
            angles = self.lidar_angles_deg
        if ranges.size == 0 or angles.size != ranges.size:
            return None
        # 有效距离过滤
        valid = np.isfinite(ranges) & (ranges >= max(min_valid, self.latest_scan.range_min)) \
                & (ranges <= min(max_valid, self.latest_scan.range_max))
        ranges = ranges[valid]
        angles = angles[valid]
        if ranges.size == 0:
            return None
        # 扇区过滤（处理跨0度情况）
        start_deg, end_deg = sector_deg
        if start_deg > end_deg:
            mask = (angles >= start_deg) | (angles <= end_deg)
        else:
            mask = (angles >= start_deg) & (angles <= end_deg)
        sector_ranges = ranges[mask]
        if sector_ranges.size == 0:
            return None
        return float(np.min(sector_ranges))

    def wait_until_front_distance_below(self, threshold_m: float = 0.15, timeout_s: float = 20.0,
                                        sector_deg: Tuple[float, float] = (-30.0, 30.0)) -> bool:
        print(f"📡 等待雷达前方距离 ≤ {threshold_m:.2f}m（含抗抖/平滑）...")
        start = time.time()
        last_log = 0.0
        stable_cnt = 0
        self.lidar_dist_window.clear()
        while time.time() - start < timeout_s:
            dist = self.get_front_min_distance(sector_deg=sector_deg)
            if dist is not None:
                self.lidar_dist_window.append(dist)
                # 中值平滑
                smooth_dist = float(np.median(self.lidar_dist_window))
                if time.time() - last_log > 0.5:
                    print(f"   ↳ 当前前方最近距离: {smooth_dist:.3f} m（原始: {dist:.3f}）")
                    last_log = time.time()
                if smooth_dist <= threshold_m:
                    stable_cnt += 1
                    if stable_cnt >= AllConfig.LIDAR_STABLE_TICKS:
                        print("✅ 雷达距离达标（稳定），准备执行倒车步态")
                        return True
                else:
                    stable_cnt = 0
            time.sleep(0.05)
        print("⏰ 等待雷达距离达标超时")
        return False

    def wait_for_visual_black_near(self, timeout_s: float,
                                   min_height_ratio: float,
                                   stable_frames: int) -> bool:
        """基于视觉黑框尺寸的近距离触发：当bbox高度比超过阈值时认为接近

        - timeout_s: 超时（秒）
        - min_height_ratio: 高度占比阈值（0-1）
        - stable_frames: 连续满足帧数
        返回：是否在超时前达到近距离判据
        """
        print("👁️ 尝试视觉近距离触发（基于黑框高度占比）...")
        start = time.time()
        stable = 0
        while time.time() - start < timeout_s:
            # 使用RGB相机
            img = self.get_rgb_image()
            if img is None:
                time.sleep(0.1)
                continue
            detected, info = self.detect_black_frame(img)
            if detected and info is not None:
                x, y, w, h = info[:4]
                ratio = h / max(1.0, img.shape[0])
                if ratio >= min_height_ratio:
                    stable += 1
                    if stable >= stable_frames:
                        print(f"✅ 视觉近距离触发：黑框高度占比 {ratio:.2f} ≥ {min_height_ratio:.2f}")
                        return True
                else:
                    stable = 0
            else:
                stable = 0
            time.sleep(0.03)
        print("⏰ 视觉近距离触发超时")
        return False

    def approach_in_small_steps_until(self, max_steps: int,
                                      step_m: float,
                                      speed_mps: float,
                                      check_fn) -> bool:
        """分段小步前探，逐步靠近直到 check_fn 返回True或用尽步数

        - max_steps: 最大步数
        - step_m: 每步位移（正向前）
        - speed_mps: 速度
        - check_fn: 可调用对象，无参，返回True则停止
        返回：是否在步数内满足条件
        """
        for i in range(max_steps):
            if check_fn():
                return True
            print(f"🚶 前探第 {i+1}/{max_steps} 步：前进 {step_m:.2f} m @ {speed_mps:.2f} m/s")
            self.move_forward(step_m, speed_mps=speed_mps)
        return check_fn()

    def wait_for_rgb(self, timeout_s: float = 5.0) -> Optional[np.ndarray]:
        # 保留旧接口，默认等待RGB一帧
        return self.wait_for_image("rgb", timeout_s)

    def _send_cmd_now(self):
        self.send_lock.acquire()
        try:
            self.delay_cnt = 50
        finally:
            self.send_lock.release()

    # ----------------------------
    # IMU里程计功能（从all_l.py移植）
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

    # ----------------------------
    # 基础运动
    # ----------------------------
    def stand_up(self, duration_ms: int = 3000, wait_s: float = 3.0, retries: int = 3):
        """站立指令（带重试机制）"""
        for attempt in range(retries):
            try:
                self.cmd_msg.mode = 12
                self.cmd_msg.gait_id = 0
                self.cmd_msg.duration = duration_ms
                self.cmd_msg.life_count = (self.cmd_msg.life_count + 1) % 128
                self._send_cmd_now()
                if attempt == 0:
                    print("🧍 发送站立指令...")
                else:
                    print(f"🧍 重试站立指令 ({attempt + 1}/{retries})...")
                time.sleep(wait_s)
                break  # 成功执行，退出重试循环
            except Exception as e:
                print(f"⚠️ 站立指令发送失败 (尝试 {attempt + 1}/{retries}): {e}")
                if attempt < retries - 1:
                    time.sleep(1.0)  # 重试前等待
                else:
                    print("❌ 站立指令多次重试失败")

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

    def lie_down_and_wait_touch(self, max_wait_s: float = 120.0):
        print("🤸 趴下并等待触摸继续...")
        start = time.time()
        self.touch_triggered = False
        while True:
            # 维持趴下阻尼
            self.cmd_msg.mode = 7
            self.cmd_msg.gait_id = 0
            self.cmd_msg.duration = 0
            self.cmd_msg.life_count = (self.cmd_msg.life_count + 1) % 128
            self._send_cmd_now()
            # 检查触摸
            if self.touch_triggered:
                print("🖐️ 检测到触摸，继续后续流程")
                break
            if max_wait_s is not None and (time.time() - start) > max_wait_s:
                print("⏰ 等待触摸超时，继续后续流程")
                break
            time.sleep(0.2)

    def lie_down_announce_and_wait(self, arrive_text: str, max_wait_s: float = 120.0):
        print("🤸 趴下，播报到达信息并等待触摸...")
        start = time.time()
        self.touch_triggered = False
        announced = False
        while True:
            # 维持趴下阻尼
            self.cmd_msg.mode = 7
            self.cmd_msg.gait_id = 0
            self.cmd_msg.duration = 0
            self.cmd_msg.life_count = (self.cmd_msg.life_count + 1) % 128
            self._send_cmd_now()
            if not announced:
                self.say_text(arrive_text)
                announced = True
            if self.touch_triggered:
                print("🖐️ 检测到触摸，继续后续流程")
                break
            if max_wait_s is not None and (time.time() - start) > max_wait_s:
                print("⏰ 等待触摸超时，继续后续流程")
                break
            time.sleep(0.2)



    # ----------------------------
    # 黄色圆形检测（专门检测黄色圆形物体）
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

    def _get_strategy_by_distance(self, distance_traveled: float) -> str:
        """根据行走距离选择检测策略（已废弃，保留兼容性）"""
        return "test"

    def _validate_yellow_detection(self, best_bbox, distance_traveled: float) -> bool:
        """额外验证检测结果，减少误检（已废弃，保留兼容性）"""
        return True

    def _save_yellow_lamp_detection_image(self, best_bbox, distance_traveled: float, h_ratio: float = None):
        """保存检测到黄灯时的图片，用于调试和分析
        
        Args:
            best_bbox: 检测到的边界框信息
            distance_traveled: 已行走距离
            h_ratio: 高度比例（如果为None表示正常检测通过）
        """
        try:
            # 获取当前RGB图像
            img = self.get_rgb_image()
            if img is None:
                print("⚠️ 无法获取图像，跳过保存")
                return
            
            # 创建保存目录
            save_dir = "yellow_lamp_detections"
            if not os.path.exists(save_dir):
                os.makedirs(save_dir)
            
            # 生成文件名：时间戳_距离_状态
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            status = "height_ratio_trigger" if h_ratio is not None else "normal_detection"
            filename = f"yellow_lamp_{timestamp}_dist{distance_traveled:.2f}m_{status}.jpg"
            filepath = os.path.join(save_dir, filename)
            
            # 在图像上绘制检测信息
            img_with_info = img.copy()
            x, y, w, h, area, circularity = best_bbox[:6]
            
            # 绘制边界框
            cv2.rectangle(img_with_info, (x, y), (x + w, y + h), (0, 255, 255), 2)
            
            # 绘制检测信息文本
            info_text = [
                f"Distance: {distance_traveled:.2f}m",
                f"BBox: ({x},{y},{w},{h})",
                f"Area: {area}",
                f"Circularity: {circularity:.2f}"
            ]
            if h_ratio is not None:
                info_text.append(f"Height Ratio: {h_ratio:.3f}")
                info_text.append("Status: Height Ratio Trigger")
            else:
                info_text.append("Status: Normal Detection")
            
            # 在图像上添加文本
            for i, text in enumerate(info_text):
                y_pos = 30 + i * 25
                cv2.putText(img_with_info, text, (10, y_pos), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            # 保存图片
            cv2.imwrite(filepath, img_with_info)
            print(f"📸 黄灯检测图片已保存: {filepath}")
            
        except Exception as e:
            print(f"⚠️ 保存黄灯检测图片失败: {e}")

    def _save_no_detection_image(self, img, distance_traveled: float, detection_count: int):
        """保存未检测到黄灯时的图片，用于调试和分析
        
        Args:
            img: 当前RGB图像
            distance_traveled: 已行走距离
            detection_count: 检测次数
        """
        try:
            if img is None:
                return
            
            # 创建保存目录
            save_dir = "yellow_lamp_detections"
            if not os.path.exists(save_dir):
                os.makedirs(save_dir)
            
            # 生成文件名：时间戳_距离_未检测状态
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"yellow_lamp_{timestamp}_dist{distance_traveled:.2f}m_no_detection_{detection_count:04d}.jpg"
            filepath = os.path.join(save_dir, filename)
            
            # 在图像上绘制调试信息
            img_with_info = img.copy()
            
            # 绘制调试信息文本
            info_text = [
                f"Distance: {distance_traveled:.2f}m",
                f"Detection Count: {detection_count}",
                f"Status: No Yellow Circle Detected",
                f"Image Size: {img.shape[1]}x{img.shape[0]}",
                f"Time: {timestamp}"
            ]
            
            # 在图像上添加文本
            for i, text in enumerate(info_text):
                y_pos = 30 + i * 25
                cv2.putText(img_with_info, text, (10, y_pos), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)  # 红色文字表示未检测到
            
            # 保存图片
            cv2.imwrite(filepath, img_with_info)
            print(f"📸 未检测到黄灯图片已保存: {filepath}")
            print(f"📁 完整保存路径: {os.path.abspath(filepath)}")
            
        except Exception as e:
            print(f"⚠️ 保存未检测到黄灯图片失败: {e}")

    def _detect_circle_strategy_standard(self, hsv, img_height, img_width):
        """标准圆形检测策略（已废弃，保留兼容性）"""
        return None, False

    def _detect_circle_strategy_relaxed(self, hsv, img_height, img_width):
        """宽松圆形检测策略（已废弃，保留兼容性）"""
        return None, False

    def _detect_circle_strategy_minimal(self, hsv, img_height, img_width):
        """极宽松圆形检测策略（已废弃，保留兼容性）"""
        return None, False

    def _find_best_yellow_circle(self, mask_processed, img_height, img_width, 
                                min_area, min_size, max_aspect_ratio, min_circularity, min_score, strategy):
        """查找最佳黄色圆形（已废弃，保留兼容性）"""
        return None, False

    def wait_for_yellow_frame(self, timeout_s: float = 30.0, initial_distance: float = 0.0):
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
            
            # 发送连续前进指令（使用增强的黄线矫正）
            if AllConfig.ENABLE_YELLOW_CENTERING:
                # 使用增强的黄线矫正的连续前进（参考follow_yellow_centering_with_imu）
                img_for_centering = self.get_rgb_image()
                steer = 0.0
                detected = False
                if img_for_centering is not None:
                    # 增强的黄线检测逻辑（参考follow_yellow_centering_with_imu）
                    h, w = img_for_centering.shape[:2]
                    roi_h = int(h * AllConfig.YL_ROI_BOTTOM_PERCENT)
                    roi = img_for_centering[h - roi_h: h, :, :]
                    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                    lower = np.array(AllConfig.YL_HSV_LOWER, dtype=np.uint8)
                    upper = np.array(AllConfig.YL_HSV_UPPER, dtype=np.uint8)
                    mask = cv2.inRange(hsv, lower, upper)
                    kernel = np.ones((3, 3), np.uint8)
                    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

                    mid = w // 3
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
                        # 使用PID控制（简化版）
                        err_norm = error / float(max(1, w))
                        steer = AllConfig.YL_PID_KP * err_norm
                        steer = max(-AllConfig.YL_MAX_TURN, min(AllConfig.YL_MAX_TURN, steer))
                
                # 发送带转向的前进指令
                self.cmd_msg.mode = 11
                self.cmd_msg.gait_id = 27
                self.cmd_msg.vel_des = [float(AllConfig.YELLOW_DETECT_SPEED), 0.0, float(steer)]
                self.cmd_msg.step_height = [0.05, 0.05]
                self.cmd_msg.duration = int(dt * 1000)
                self.cmd_msg.life_count = (self.cmd_msg.life_count + 1) % 128
                self._send_cmd_now()
            else:
                # 普通直线前进
                self.cmd_msg.mode = 11
                self.cmd_msg.gait_id = 27
                self.cmd_msg.vel_des = [float(AllConfig.YELLOW_DETECT_SPEED), 0.0, 0.0]
                self.cmd_msg.step_height = [0.05, 0.05]
                self.cmd_msg.duration = int(dt * 1000)
                self.cmd_msg.life_count = (self.cmd_msg.life_count + 1) % 128
                self._send_cmd_now()
            
            # 同时进行黄灯检测
            img = self.get_rgb_image()
            if img is None:
                continue
            
            # 获取当前行走距离（从初始距离开始计算）
            current_distance = initial_distance + traveled_distance
            best, detected = self.detect_yellow_circles(img, current_distance)
            
            # 定期日志输出
            if current_time - last_distance_log > 0.5:
                print(f"📍 连续前进中... 距离: {traveled_distance:.2f}m, 检测次数: {detection_count}")
                last_distance_log = current_time
            
            # 每0.5秒保存一次图片（无论是否检测到目标）
            if current_time - last_save_time >= save_interval:
                if detected and best is not None:
                    print(f"✅ 边走边检测到黄灯，保存图片：距离={traveled_distance:.2f}m")
                    self._save_yellow_lamp_detection_image(best, current_distance, None)
                else:
                    # 偶尔保存未检测到的图片用于调试（减少频率）
                    if detection_count % 10 == 0:  # 每10次检测保存一次
                        self._save_no_detection_image(img, current_distance, detection_count)
                last_save_time = current_time
            
            # 检测到黄灯时立即停止并返回
            if detected and best is not None:
                area = best[4]  # 面积在第5个位置
                print(f"🔍 边走边检测到黄色圆形，面积: {area:.2f}px")
                
                if area > AllConfig.YELLOW_ROI_MIN_AREA:
                    print(f"🎯 黄色圆形面积满足要求 ({area:.2f}px > {AllConfig.YELLOW_ROI_MIN_AREA}px)，立即停止！")
                    self.send_stop()  # 立即停止
                    return True, traveled_distance
                else:
                    print(f"🔍 黄色圆形面积不足 ({area:.2f}px < {AllConfig.YELLOW_ROI_MIN_AREA}px)，继续前进")
            
            # 移除sleep，保持连续运动
        # 超时时停止并返回实际走过的距离
        self.send_stop()
        if use_imu:
            final_distance = self.get_traveled_distance()
        else:
            final_distance = (time.time() - start) * AllConfig.YELLOW_DETECT_SPEED
        print(f"⏰ 黄色圆形识别超时，实际走过距离: {final_distance:.2f}m")
        return False, final_distance

    # ----------------------------
    # 直道黄线PD校正（参考 s_curve_runner_last.py 思路）
    # ----------------------------


    # ----------------------------
    # 二维码识别（pyzbar，多尺度+多预处理）
    # ----------------------------
    def detect_qr_code(self, image: np.ndarray):
        if image is None:
            return None
        try:
            corrected_img = image  # 简化：不做矫正
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

    def wait_for_b_qr(self, timeout_s: float = 40.0) -> Optional[str]:
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
            # 评分（与 test_realtime_black_frame 类似）
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
                self.say_text("识别到限高杆")
                return True
            time.sleep(0.03)
        print("⏰ 限高杆识别超时")
        return False

    # ----------------------------
    # 红色检测（从 red_follow_runner(1).py 移植）
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
        print(f"🚶 开始边走边检测红色限高杆（最大距离 {max_distance_m}m @ {speed_mps} m/s）...")
        print("🔴 检测逻辑：识别到红色继续前进，识别不到红色则到达限高杆位置")
        
        step_size = 0.1  # 每次前进0.1米
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
                return True
            
            # 检测到红色，继续前进（启用黄线矫正）
            current_step = min(step_size, max_distance_m - traveled)
            if AllConfig.ENABLE_YELLOW_CENTERING and current_step >= AllConfig.YL_MIN_DIST_M:
                self.follow_yellow_centering_with_imu(current_step, speed_mps)
            else:
                self.move_forward(current_step, speed_mps=speed_mps)
            
            traveled += current_step
        
        print(f"⚠️ 达到最大距离 {max_distance_m}m，仍然检测到红色，可能未到达限高杆")
        self.send_stop()
        return False


    def follow_yellow_centering(self, distance_m: float, speed_mps: float) -> None:
        """基于 all_correct.py 思路的双黄线居中行走（时间开环）

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

    def walk_until_head_collision(self, max_distance_m: float = 10.0, speed_mps: float = 0.15) -> bool:
        """边走边检测头部碰撞，直到检测到碰撞或达到最大距离
        
        - max_distance_m: 最大前进距离（米）
        - speed_mps: 前进速度（m/s）
        返回：是否检测到头部碰撞
        """
        print(f"🚶 开始边走边检测头部碰撞（最大距离 {max_distance_m}m @ {speed_mps} m/s）...")
        
        # 重置头部碰撞状态
        self.head_collision_detected = False
        
        step_size = 0.1  # 每次前进0.1米
        traveled = 0.0
        
        while traveled < max_distance_m:
            # 检查是否检测到头部碰撞
            if self.head_collision_detected:
                print(f"✅ 检测到头部碰撞！已前进 {traveled:.2f}m")
                self.send_stop()  # 立即停止
                return True
            
            # 小步前进（启用黄线矫正）
            current_step = min(step_size, max_distance_m - traveled)
            if AllConfig.ENABLE_YELLOW_CENTERING and current_step >= AllConfig.YL_MIN_DIST_M:
                self.follow_yellow_centering_with_imu(current_step, speed_mps)
            else:
                self.move_forward(current_step, speed_mps=speed_mps)
            
            traveled += current_step
            
            # 日志输出
            if int(traveled * 10) % 5 == 0:  # 每0.5米输出一次
                print(f"   ↳ 已前进 {traveled:.1f}m，继续检测...")
        
        print(f"⚠️ 达到最大距离 {max_distance_m}m，未检测到头部碰撞")
        self.send_stop()
        return False

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

    def follow_yellow_centering_with_step_height(self, distance_m: float, speed_mps: float, 
                                               step_height: Tuple[float, float] = (0.1, 0.1)) -> None:
        """基于双黄线居中行走（支持高抬腿步高）
        
        - 与 follow_yellow_centering 相同的逻辑，但支持自定义步高参数
        - 用于限高杆后的高抬腿前进
        """
        if distance_m <= 0 or speed_mps <= 0:
            return
        print(f"🟨 双黄线居中直行（高抬腿 {step_height}）...")
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
            # 发送控制（使用自定义步高）
            self.cmd_msg.mode = 11
            self.cmd_msg.gait_id = 27
            yaw = -steer if detected else 0.0
            self.cmd_msg.vel_des = [float(speed_mps), 0.0, float(yaw)]
            self.cmd_msg.step_height = [float(step_height[0]), float(step_height[1])]  # 使用自定义步高
            self.cmd_msg.duration = int(dt * 1000)
            self.cmd_msg.life_count = (self.cmd_msg.life_count + 1) % 128
            self._send_cmd_now()
            time.sleep(dt)
            if time.time() - last_tick > 0.6:
                print(f"   ↳ 黄线居中（高抬腿） detected={detected}, steer={steer:.3f}")
                last_tick = time.time()
        self.send_stop()

    def edge_guard_straight(self, distance_m: float, speed_mps: float) -> None:
        """沿道路边线守护直行：保持与边线平行，边线投影位于目标x比例位置

        - 仅控制角速度，线速度恒定；
        - 控制律：wz = -kp_offset*(cx_ratio - target_x_ratio) - kp_angle*(angle_err_deg/90)
        - 置信度不足时退化为直行；
        - 时间开环估算距离。
        """
        if distance_m <= 0 or speed_mps <= 0:
            return
        print(f"🟨 边线守护直行（侧={AllConfig.EDGE_SIDE}）...")
        duration_s = distance_m / speed_mps
        end_t = time.time() + duration_s
        last_log = 0.0
        while time.time() < end_t:
            img = self.get_rgb_image()
            cx_ratio, angle_err_deg, conf = self._detect_edge_line(img) if img is not None else (None, 0.0, 0.0)
            if cx_ratio is not None and conf >= AllConfig.EDGE_CONF_MIN:
                offset = cx_ratio - AllConfig.EDGE_TARGET_X_RATIO
                wz = -AllConfig.EDGE_KP_OFFSET * float(offset) - AllConfig.EDGE_KP_ANGLE * (float(angle_err_deg) / 90.0)
                wz = max(-AllConfig.EDGE_MAX_WZ, min(AllConfig.EDGE_MAX_WZ, wz))
            else:
                wz = 0.0
            # 发送短周期速度指令
            self.cmd_msg.mode = 11
            self.cmd_msg.gait_id = 27
            self.cmd_msg.vel_des = [float(speed_mps), 0.0, float(wz)]
            self.cmd_msg.step_height = [0.05, 0.05]
            self.cmd_msg.duration = int(AllConfig.EDGE_CTRL_TICK_S * 1000)
            self.cmd_msg.life_count = (self.cmd_msg.life_count + 1) % 128
            self._send_cmd_now()
            time.sleep(AllConfig.EDGE_CTRL_TICK_S)
            if time.time() - last_log > 0.6:
                print(f"   ↳ 边线 conf={conf:.2f}, cx_ratio={cx_ratio if cx_ratio is not None else 'None'}, angle_err={angle_err_deg:.1f}°")
                last_log = time.time()
        self.send_stop()

    # ----------------------------
    # 过限高杆步态（复刻 yellow_run.execute_custom_gait_front 核心）
    # ----------------------------
    def execute_custom_gait_front(self):
        """正向过限高杆步态"""
        self._execute_custom_gait(backward=False)

    def execute_custom_gait_backward(self):
        """倒车过限高杆步态"""
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
            # 1. 站立
            self.stand_up()

            # 2. 加载步态
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
                    # 如果是倒车模式，反转x方向速度
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
                    # 如果是倒车模式，反转x方向速度
                    if backward and len(vel_des) >= 1:
                        vel_des[0] = -vel_des[0]
                    cmd["vel_des"] = vel_des
                elif t == "recoverystand":
                    cmd["mode"] = 12
                    cmd["gait_id"] = 0
                full_steps["step"].append(cmd)

            # 3. 发送控制（最长10s）
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

            # 4. 心跳维持
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
    node: Optional[AllNode] = None
    spin_thread: Optional[Thread] = None
    try:
        rclpy.init(args=sys.argv)
        node = AllNode()
        node.start()
        # 在后台spin以处理相机回调
        spin_thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
        spin_thread.start()

        # 等待RGB相机就绪
        print("📷 等待RGB相机就绪...")
        if node.wait_for_image("rgb", timeout_s=AllConfig.WAIT_RGB_TIMEOUT_S) is None:
            print("⚠️ RGB相机未就绪，仍继续流程（可能无法识别）")
        else:
            print("✅ RGB相机已就绪")

        # 站立（增强版本，确保执行）
        print("🧍 准备站立...")
        time.sleep(1.0)  # 给LCM通信一点时间
        node.stand_up(duration_ms=3000, wait_s=4.0)  # 增加等待时间
        print("✅ 站立完成，开始任务流程")
        # 跳过箭头识别，直接执行右向路径
        print("▶️ 按默认右向路径执行（跳过箭头识别）")
        
        node.say_text("右侧路线")

        node.move_forward(AllConfig.ARROW_FWD1_M, speed_mps=AllConfig.ARROW_FWD1_SPEED)
        node.turn_in_place(AllConfig.ARROW_TURN1_DEG)   # 右转90°
        node.move_forward(AllConfig.ARROW_FWD2_M, speed_mps=AllConfig.ARROW_FWD2_SPEED)
        node.turn_in_place(AllConfig.ARROW_TURN2_DEG)   # 右转90°
        # 倒着上坡1.5米，然后转弯180°调整方向
        print("🚶 开始倒着上坡1.5米...")
        node.move_forward(AllConfig.ARROW_BACK_M, speed_mps=AllConfig.ARROW_BACK_SPEED, step_height=(0.1, 0.1))  # 倒车1.5米
        print("🔄 转弯180°调整方向...")
        node.turn_in_place(AllConfig.ARROW_TURN3_DEG)  # 转弯180°

        # 新路线：坡上直行1米（启用黄线矫正，垂直步态）
        print("🚶 坡上直行1米（启用黄线矫正，垂直步态，可自定义步高）...")
        if AllConfig.ENABLE_YELLOW_CENTERING and AllConfig.UPHILL_FWD_M >= AllConfig.YL_MIN_DIST_M:
            node.follow_yellow_centering_with_step_height(
                AllConfig.UPHILL_FWD_M,
                AllConfig.UPHILL_FWD_SPEED,
                AllConfig.UPHILL_STEP_HEIGHT,
                value=1
            )
        else:
            node.move_forward(
                AllConfig.UPHILL_FWD_M,
                speed_mps=AllConfig.UPHILL_FWD_SPEED,
                step_height=AllConfig.UPHILL_STEP_HEIGHT
            )

        # 下坡直行1.5米（降低速度，启用黄线矫正）
        print("🚶 下坡直行1.5米（降低速度，启用黄线矫正）...")
        if AllConfig.ENABLE_YELLOW_CENTERING and AllConfig.DOWNHILL_FWD_M >= AllConfig.YL_MIN_DIST_M:
            node.move_forward(AllConfig.DOWNHILL_FWD_M, AllConfig.DOWNHILL_FWD_SPEED,step_height=(0.04, 0.04))
        else:
            node.move_forward(AllConfig.DOWNHILL_FWD_M, speed_mps=AllConfig.DOWNHILL_FWD_SPEED,step_height=(0.04, 0.04))
        
        # 黄色圆形区域：动态分配距离，总距离3.5米
        print(f"🟡 黄色圆形区域开始：总距离{AllConfig.TOTAL_YELLOW_SECTION_M}米（动态分配识别段+后续段）")
        detected_yellow = False
        stable_cnt = 0
        yellow_detect_traveled = 0.0  # 黄色圆形识别段实际走的距离
        step_size = 0.5  # 小步前进，便于检测
        
        # 黄色圆形区域：边走边检测，总距离固定3.5米
        print(f"🔍 黄色圆形识别段：使用边走边检测模式...")
        detected_yellow, yellow_detect_traveled = node.wait_for_yellow_frame(timeout_s=45.0, initial_distance=0.0)
        
        if detected_yellow:
            print(f"✅ 识别到黄色圆形！已走{yellow_detect_traveled:.2f}米")
            
            # 识别到黄灯后，再走0.4米到达播报位置
            final_approach_distance = 0.4
            print("🎯 继续前进0.4米到达播报位置...")
            if AllConfig.ENABLE_YELLOW_CENTERING and final_approach_distance >= AllConfig.YL_MIN_DIST_M:
                node.follow_yellow_centering_with_imu(final_approach_distance, AllConfig.YELLOW_AFTER_DETECT_SPEED)
            else:
                node.move_forward(final_approach_distance, speed_mps=AllConfig.YELLOW_AFTER_DETECT_SPEED)
            
            # 到达播报位置，播报并站5秒
            print("🎉 到达黄灯播报位置！开始播报并等待5秒...")
            node.say_text("已到达黄灯位置，开始倒计时5秒")
            
            # 倒计时5秒
            for i in range(5, 0, -1):
                print(f"⏰ 倒计时: {i}")
                node.say_text(str(i))
                time.sleep(1.0)
            
            print("✅ 黄灯播报完成，继续后续流程")
            
            # 计算剩余距离，补足总距离3.5米
            total_traveled = yellow_detect_traveled + final_approach_distance
            remaining_distance = AllConfig.TOTAL_YELLOW_SECTION_M - total_traveled
        else:
            print(f"⚠️ 未识别到黄色圆形，已走{yellow_detect_traveled:.2f}米，继续流程")

            # 计算剩余距离，补足总距离3.5米
            remaining_distance = AllConfig.TOTAL_YELLOW_SECTION_M - yellow_detect_traveled
        
        # 补足剩余距离，确保总距离为3.5米
        if remaining_distance > 0.05:  # 最小阈值0.05米
            print(f"🚶 补足剩余距离{remaining_distance:.2f}米（总距离{AllConfig.TOTAL_YELLOW_SECTION_M}米）")
            if AllConfig.ENABLE_YELLOW_CENTERING and remaining_distance >= AllConfig.YL_MIN_DIST_M:
                node.follow_yellow_centering_with_imu(remaining_distance, AllConfig.SEG1_FWD1_SPEED)
            else:
                node.move_forward(remaining_distance, speed_mps=AllConfig.SEG1_FWD1_SPEED)
        else:
            print(f"🎯 黄色圆形区域已完成，总距离{AllConfig.TOTAL_YELLOW_SECTION_M}米")
      
      
        print("🔄 左转95°...")
        node.turn_in_place(AllConfig.SEG1_TURN1_DEG)
        
        print("🚶 直行1.3米（启用黄线矫正）...")
        if AllConfig.ENABLE_YELLOW_CENTERING and AllConfig.SEG1_FWD2_M >= AllConfig.YL_MIN_DIST_M:
            node.follow_yellow_centering_with_imu(AllConfig.SEG1_FWD2_M, AllConfig.SEG1_FWD2_SPEED)
        else:
            node.move_forward(AllConfig.SEG1_FWD2_M, speed_mps=AllConfig.SEG1_FWD2_SPEED)
            
        print("🔄 右转90°...")
        node.turn_in_place(AllConfig.SEG1_TURN2_DEG)

        # B库二维码识别（10秒超时，未识别则默认执行B1）
        qr = node.wait_for_b_qr(timeout_s=AllConfig.QR_TIMEOUT_S)
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
                print("✅ 二维码识别到B2")
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
            except Exception as _e:
                print(f"⚠️ Doubao 辅助识别失败: {_e}")
        # 根据识别结果执行对应路径
        if decide_b1:
            print("=== 识别到B1（二维码或文字），执行B1路径 ===")
            # B1路径：左转90° -> 直行1m -> 右转90° -> 直行1m -> 卸货
            node.turn_in_place(-AllConfig.B1_TURN1_DEG)  # 左转90°
            node.move_forward(AllConfig.B1_FWD1_M, speed_mps=AllConfig.B1_FWD1_SPEED)
            node.turn_in_place(-AllConfig.B1_TURN2_DEG)  # 右转90°
            node.move_forward(AllConfig.B1_FWD2_M, speed_mps=AllConfig.B1_FWD2_SPEED)
            # 卸货：趴下后，先播报到达B1库，再等待触摸继续
            node.lie_down_announce_and_wait("已到达B1库", max_wait_s=120.0)

            node.stand_up()
            # B1卸货后到B2装货路径：后退1m -> 右转90° -> 直行2m -> 左转90° -> 直行1m -> 装货
            node.move_forward(AllConfig.B2_BACK_M, speed_mps=AllConfig.B2_BACK_SPEED)  # 后退
            node.turn_in_place(-AllConfig.B2_TURN1_DEG)  # 右转90°
            node.move_forward(AllConfig.B2_FWD1_M, speed_mps=AllConfig.B2_FWD1_SPEED)
            node.turn_in_place(-AllConfig.B2_TURN2_DEG)  # 左转90°
            node.move_forward(AllConfig.B2_FWD2_M, speed_mps=AllConfig.B2_FWD2_SPEED)
            # B2装货：趴下后，先播报到达B2库，再等待触摸继续
            node.lie_down_announce_and_wait("已到达B2库", max_wait_s=120.0)

            # B1装货后回程路径：后退1m -> 左转90° -> 直行2m -> 左转90° -> 直行1m
            node.stand_up()
            print("=== B1装货完成，开始回程到限高杆 ===")
            node.move_forward(-1.0, speed_mps=AllConfig.B2_BACK_SPEED)  # 后退1m
            node.turn_in_place(+90.0)  # 左转90°
            node.move_forward(2.0, speed_mps=AllConfig.RETURN_FWD_SPEED)  # 直行2m
            node.turn_in_place(+90.0)  # 左转90°
            # 直行1m（启用黄线矫正）- 这是B1路径的最后一段
            if AllConfig.ENABLE_YELLOW_CENTERING and 1.0 >= AllConfig.YL_MIN_DIST_M:
                node.follow_yellow_centering_with_imu(1.0, AllConfig.RETURN_FWD_SPEED)
            else:
                node.move_forward(1.0, speed_mps=AllConfig.RETURN_FWD_SPEED)

        elif decide_b2:
            print("=== 识别到B2（二维码或文字），执行B2路径 ===")
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
            print("=== B2装货完成，开始回程到限高杆 ===")
            node.move_forward(-1.0, speed_mps=AllConfig.B2_BACK_SPEED)  # 后退1m
            node.turn_in_place(+180.0)  # 左转180°
            # 直行1m（启用黄线矫正）
            if AllConfig.ENABLE_YELLOW_CENTERING and 1.0 >= AllConfig.YL_MIN_DIST_M:
                node.follow_yellow_centering_with_imu(1.0, AllConfig.RETURN_FWD_SPEED)
            else:
                node.move_forward(1.0, speed_mps=AllConfig.RETURN_FWD_SPEED)

        else:
            print("⚠️ 未识别到B1/B2（二维码/文字均失败），默认执行B1路径")
            # 默认B1路径：左转90° -> 直行1m -> 右转90° -> 直行1m -> 卸货
            node.turn_in_place(-AllConfig.B1_TURN1_DEG)  # 左转90°
            node.move_forward(AllConfig.B1_FWD1_M, speed_mps=AllConfig.B1_FWD1_SPEED)
            node.turn_in_place(-AllConfig.B1_TURN2_DEG)  # 右转90°
            node.move_forward(AllConfig.B1_FWD2_M, speed_mps=AllConfig.B1_FWD2_SPEED)
            # 卸货：趴下后，先播报到达B1库，再等待触摸继续
            node.lie_down_announce_and_wait("已到达B1库", max_wait_s=120.0)

            node.stand_up()
            # B1卸货后到B2装货路径：后退1m -> 右转90° -> 直行2m -> 左转90° -> 直行1m -> 装货
            node.move_forward(AllConfig.B2_BACK_M, speed_mps=AllConfig.B2_BACK_SPEED)  # 后退
            node.turn_in_place(-AllConfig.B2_TURN1_DEG)  # 右转90°
            node.move_forward(AllConfig.B2_FWD1_M, speed_mps=AllConfig.B2_FWD1_SPEED)
            node.turn_in_place(-AllConfig.B2_TURN2_DEG)  # 左转90°
            node.move_forward(AllConfig.B2_FWD2_M, speed_mps=AllConfig.B2_FWD2_SPEED)
            # B2装货：趴下后，先播报到达B2库，再等待触摸继续
            node.lie_down_announce_and_wait("已到达B2库", max_wait_s=120.0)

            # 默认B1装货后回程路径：后退1m -> 左转90° -> 直行2m -> 左转90° -> 直行1m
            node.stand_up()
            print("=== 默认B1装货完成，开始回程到限高杆 ===")
            node.move_forward(-1.0, speed_mps=AllConfig.B2_BACK_SPEED)  # 后退1m
            node.turn_in_place(+90.0)  # 左转90°
            node.move_forward(2.0, speed_mps=AllConfig.RETURN_FWD_SPEED)  # 直行2m
            node.turn_in_place(+90.0)  # 左转90°
            # 直行1m（启用黄线矫正）
            if AllConfig.ENABLE_YELLOW_CENTERING and 1.0 >= AllConfig.YL_MIN_DIST_M:
                node.follow_yellow_centering_with_imu(1.0, AllConfig.RETURN_FWD_SPEED)
            else:
                node.move_forward(1.0, speed_mps=AllConfig.RETURN_FWD_SPEED)

        # 限高杆长直道：动态分配总距离9米
        print(f"🚶 限高杆长直道开始：总距离{AllConfig.TOTAL_BARRIER_SECTION_M}米（动态分配回程段+检测段）")
        barrier_return_traveled = 0.0  # 回程段实际走的距离
        step_size = 0.2  # 小步前进，便于记录距离
        
        # 第一阶段：回程段（最多走3米，可能提前检测到红色）
        print(f"🚶 回程段：最多{AllConfig.RETURN_TO_BARRIER_MAX_M}米，边走边检测红色...")
        red_detected_in_return = False
        while barrier_return_traveled < AllConfig.RETURN_TO_BARRIER_MAX_M:
            step = min(step_size, AllConfig.RETURN_TO_BARRIER_MAX_M - barrier_return_traveled)
            
            # 边走边检测红色
            if AllConfig.ENABLE_YELLOW_CENTERING and step >= AllConfig.YL_MIN_DIST_M:
                node.follow_yellow_centering_with_imu(step, AllConfig.RETURN_FWD_SPEED)
            else:
                node.move_forward(step, speed_mps=AllConfig.RETURN_FWD_SPEED)
            barrier_return_traveled += step
            
            # 检测红色
            img = node.get_rgb_image()
            if img is not None and node.detect_red_frame(img):
                print(f"🔴 回程段检测到红色！已走{barrier_return_traveled:.2f}米，提前进入检测段")
                red_detected_in_return = True
                break
        
        if not red_detected_in_return:
            print(f"⚪ 回程段未检测到红色，已走完{barrier_return_traveled:.2f}米")
        
        # 第二阶段：限高杆检测段（动态计算剩余距离）
        remaining_distance = AllConfig.TOTAL_BARRIER_SECTION_M - barrier_return_traveled
        if remaining_distance > 0.05:  # 最小阈值0.05米
            print(f"🔍 限高杆检测段：剩余{remaining_distance:.2f}米，边走边检测红色消失...")
            red_lost = node.walk_until_red_lost(
                max_distance_m=remaining_distance,
                speed_mps=AllConfig.BARRIER_DETECT_SPEED
            )
        else:
            print(f"🎯 长直道已完成，总距离{barrier_return_traveled:.2f}米")
            red_lost = False
        
        if red_lost:
            print("✅ 红色消失，检测到限高杆位置！开始限高杆操作...")
            node.say_text("识别到限高杆")
            
            # 检测到限高杆后，后退一小段距离准备过杆
            print(f"🚶 检测到限高杆，后退{AllConfig.HEAD_COLLISION_BACKUP_M}米准备过杆...")
            node.move_forward(-AllConfig.HEAD_COLLISION_BACKUP_M, speed_mps=0.15)
            
            # 原地掉头
            node.turn_in_place(AllConfig.BLACK_AFTER_TARGET_TURN_DEG)
            # 执行过限高杆（倒车版本）
            node.execute_custom_gait_backward()
            # 过完后原地掉头
            node.turn_in_place(AllConfig.POST_GAIT_TURN_DEG)
            # 高抬腿向前走6m（启用黄线矫正，垂直步态）
            print("🚶 高抬腿前进6米（启用黄线矫正，垂直步态）...")
            if AllConfig.ENABLE_YELLOW_CENTERING and AllConfig.POST_GAIT_FORWARD_M >= AllConfig.YL_MIN_DIST_M:
                node.follow_yellow_centering_with_step_height(AllConfig.POST_GAIT_FORWARD_M, 
                                                            AllConfig.POST_GAIT_FORWARD_SPEED,
                                                            AllConfig.POST_GAIT_HIGH_STEP,
                                                            value=1)
            else:
                node.move_forward(AllConfig.POST_GAIT_FORWARD_M, speed_mps=AllConfig.POST_GAIT_FORWARD_SPEED,
                                  step_height=AllConfig.POST_GAIT_HIGH_STEP)
        else:
            print("⚠️ 达到最大距离仍检测到红色，可能未到达限高杆，跳过过杆步态")

        # 最终停止
        node.send_stop()
        print("✅ 任务完成")

    except KeyboardInterrupt:
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


