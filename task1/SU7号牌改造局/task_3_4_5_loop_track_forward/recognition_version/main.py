#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
任务3-4-5：回形赛道前进方向 - 识别版
包括箭头识别、路径选择和四个路段的执行

作者：Xiaomi Cup 2025 Final Team
"""

import os
import sys
import time
import cv2
from typing import Optional

# 添加路径
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(current_dir, '..', '..', 'shared_modules'))
sys.path.append(os.path.join(current_dir, '..', '..', '..', 'demo'))

try:
    # 添加shared_modules到路径
    shared_modules_path = os.path.join(current_dir, '..', '..', 'shared_modules')
    if shared_modules_path not in sys.path:
        sys.path.insert(0, shared_modules_path)
    
    from loop_track_common import (
        LoopTrackState, ArrowDetector, SlopeSection, 
        YellowLightSection, StoneRoadSection, HeightLimitSection
    )
    from motion_control.robot_controller import RobotController
    from utils.voice_announcer import VoiceAnnouncer
    from utils.logger import Logger
    
except ImportError as e:
    print(f"第一次导入失败: {e}")
    try:
        # 备用导入方案
        import importlib.util
        
        # 动态导入loop_track_common
        spec = importlib.util.spec_from_file_location(
            "loop_track_common", 
            os.path.join(shared_modules_path, "loop_track_common.py")
        )
        loop_track_common = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(loop_track_common)
        
        # 从动态导入的模块获取类
        LoopTrackState = loop_track_common.LoopTrackState
        ArrowDetector = loop_track_common.ArrowDetector
        SlopeSection = loop_track_common.SlopeSection
        YellowLightSection = loop_track_common.YellowLightSection
        StoneRoadSection = loop_track_common.StoneRoadSection
        HeightLimitSection = loop_track_common.HeightLimitSection
        
        # 简化的Mock类
        class SimpleRobotController:
            def __init__(self): print("SimpleRobotController初始化")
            def run(self): print("机器人控制器启动")
            def stop(self): print("机器人控制器停止")
            def turn_right(self, angle): print(f"右转 {angle}°")
            def turn_left(self, angle): print(f"左转 {angle}°")
            def move_distance_imu(self, distance, **kwargs): print(f"移动 {distance}m")
            
        class SimpleVoiceAnnouncer:
            def speak_text(self, text): print(f"🔊 语音播报: {text}")
            
        class SimpleLogger:
            def __init__(self, name): self.name = name
            def info(self, msg): print(f"ℹ️ [{self.name}] {msg}")
            def warning(self, msg): print(f"⚠️ [{self.name}] {msg}")
            def error(self, msg): print(f"❌ [{self.name}] {msg}")
        
        RobotController = SimpleRobotController
        VoiceAnnouncer = SimpleVoiceAnnouncer
        Logger = SimpleLogger
        
    except Exception as e2:
        print(f"备用导入也失败: {e2}")
        print("将使用最小化的Mock实现")
        
        # 最小化Mock实现
        class MockClass:
            def __init__(self, *args, **kwargs): pass
            def __getattr__(self, name): 
                return lambda *args, **kwargs: print(f"Mock调用: {name}({args}, {kwargs})")
        
        LoopTrackState = MockClass
        ArrowDetector = MockClass
        SlopeSection = MockClass
        YellowLightSection = MockClass
        StoneRoadSection = MockClass
        HeightLimitSection = MockClass
        RobotController = MockClass
        VoiceAnnouncer = MockClass
        Logger = MockClass


class LoopTrackForwardController:
    """回形赛道前进方向控制器"""
    
    def __init__(self):
        # 初始化组件
        self.robot = RobotController()
        self.voice = VoiceAnnouncer()
        self.logger = Logger("LoopTrackForward")
        self.state_manager = LoopTrackState()
        self.arrow_detector = ArrowDetector()
        
        # 初始化各路段控制器
        self.slope_section = SlopeSection(self.robot, self.voice)
        self.yellow_light_section = YellowLightSection(self.robot, self.voice)
        self.stone_road_section = StoneRoadSection(self.robot, self.voice)
        self.height_limit_section = HeightLimitSection(self.robot, self.voice)
        
        print("回形赛道前进控制器初始化完成")
    
    def capture_arrow_image(self) -> str:
        """捕获箭头图像"""
        try:
            # 初始化摄像头
            cap = cv2.VideoCapture(0)
            if not cap.isOpened():
                raise Exception("无法打开摄像头")
            
            # 设置摄像头参数
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            
            # 预热摄像头
            for _ in range(5):
                ret, frame = cap.read()
                if not ret:
                    continue
                time.sleep(0.1)
            
            # 捕获图像
            ret, frame = cap.read()
            if not ret:
                raise Exception("无法捕获图像")
            
            # 保存图像
            image_path = os.path.join(current_dir, "arrow_capture.jpg")
            cv2.imwrite(image_path, frame)
            
            cap.release()
            return image_path
            
        except Exception as e:
            print(f"图像捕获失败: {e}")
            return None
    
    def detect_arrow_direction(self) -> str:
        """检测箭头方向"""
        self.logger.info("开始箭头方向识别")
        self.voice.speak_text("开始识别前方箭头")
        
        try:
            # 捕获图像
            image_path = self.capture_arrow_image()
            if not image_path:
                self.logger.warning("图像捕获失败，使用默认方向")
                return "L"
            
            # 识别箭头方向
            direction = self.arrow_detector.detect_arrow_direction(image_path)
            self.logger.info(f"识别到箭头方向: {direction}")
            
            # 清理临时文件
            if os.path.exists(image_path):
                os.remove(image_path)
            
            return direction
            
        except Exception as e:
            self.logger.error(f"箭头识别失败: {e}")
            return "L"  # 默认返回左
    
    def execute_initial_turn(self, direction: str):
        """执行初始转向"""
        self.logger.info(f"执行初始转向，方向: {direction}")
        
        # 先站立准备
        print("机器人站立准备")
        self.robot.stand(duration=1000, sleep=1.0)
        
        if direction == "L":
            # 左赛道：先站立，播报"检测到左箭头，左转"，右转110°，前进90cm，然后左转
            self.voice.speak_text("检测到左箭头，左转")
            time.sleep(1)
            
            self.robot.turn_right(110)  # 右转110°
            time.sleep(1)
            
            self.robot.move_distance_imu(0.9, velocity=0.1)  # 前进90cm（修改）
            time.sleep(1)
            
            self.robot.turn_left(90)  # 左转进入左赛道
            
        else:  # direction == "R"
            # 右赛道：先站立，播报"检测到右箭头，右转"，左转110°，前进90cm，然后右转
            self.voice.speak_text("检测到右箭头，右转")
            time.sleep(1)
            
            self.robot.turn_left(110)  # 左转110°
            time.sleep(1)
            
            self.robot.move_distance_imu(0.9, velocity=0.1)  # 前进90cm
            time.sleep(1)
            
            self.robot.turn_right(90)  # 右转进入右赛道（修正）
        
        self.logger.info("初始转向完成")
    
    def execute_left_track_go(self):
        """执行左赛道去程：上下坡 -> 黄灯"""
        self.logger.info("开始执行左赛道去程")
        self.voice.speak_text("进入左赛道，开始通过上下坡")
        
        # 1. 执行上下坡路段（400cm）
        self.slope_section.execute_slope_section()
        
        # 2. 执行黄灯识别路段（450cm）
        self.voice.speak_text("开始黄灯识别路段")
        self.yellow_light_section.execute_yellow_light_section(use_sensors=True, sensor_config=0, is_dead_reckoning=False)
        
        self.logger.info("左赛道去程完成")
    
    def execute_right_track_go(self):
        """执行右赛道去程：石板路 -> 限高杆"""
        self.logger.info("开始执行右赛道去程")
        self.voice.speak_text("进入右赛道，开始通过石板路")
        
        # 1. 执行石板路路段（400cm）
        self.stone_road_section.execute_stone_road_section()
        
        # 2. 执行限高杆路段（450cm）
        self.voice.speak_text("开始限高杆路段")
        self.height_limit_section.execute_height_limit_section(use_vision=True)
        
        self.logger.info("右赛道去程完成")
    
    def run(self):
        """主要执行流程"""
        try:
            self.logger.info("=" * 50)
            self.logger.info("开始任务3-4-5：回形赛道前进方向")
            self.logger.info("=" * 50)
            
            # 启动机器人控制器
            self.robot.run()
            time.sleep(2)
            
            # 等待机器人准备就绪
            self.voice.speak_text("机器人系统启动完成，准备开始任务")
            time.sleep(1)
            
            # 步骤1：箭头方向识别
            direction = self.detect_arrow_direction()
            
            # 步骤2：保存状态
            self.state_manager.save_state(direction)
            
            # 步骤3：执行初始转向
            self.execute_initial_turn(direction)
            
            # 步骤4：根据方向执行对应赛道
            if direction == "L":
                self.execute_left_track_go()
            else:
                self.execute_right_track_go()
            
            # 任务完成
            self.voice.speak_text("回形赛道前进方向任务完成")
            self.logger.info("任务3-4-5完成")
            
        except KeyboardInterrupt:
            self.logger.info("用户中断任务")
            self.voice.speak_text("任务已中断")
        except Exception as e:
            self.logger.error(f"任务执行失败: {e}")
            self.voice.speak_text("任务执行出现错误")
        finally:
            # 停止机器人
            self.robot.stop()
            self.logger.info("机器人已停止")


def main():
    """主函数"""
    controller = LoopTrackForwardController()
    controller.run()


if __name__ == "__main__":
    main()