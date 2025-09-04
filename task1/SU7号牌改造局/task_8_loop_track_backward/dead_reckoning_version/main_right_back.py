#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
任务8：回形赛道返程方向 - 写死版（右返程）
固定执行右赛道返程：限高杆 -> 石板路

作者：Xiaomi Cup 2025 Final Team
"""

import os
import sys
import time

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
        StoneRoadSection, HeightLimitSection
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
        
        StoneRoadSection = MockClass
        HeightLimitSection = MockClass
        RobotController = MockClass
        VoiceAnnouncer = MockClass
        Logger = MockClass


class RightBackTrackController:
    """右赛道返程控制器（写死版）"""
    
    def __init__(self):
        # 初始化组件
        self.robot = RobotController()
        self.voice = VoiceAnnouncer()
        self.logger = Logger("RightBackTrack")
        
        # 初始化路段控制器
        self.stone_road_section = StoneRoadSection(self.robot, self.voice)
        self.height_limit_section = HeightLimitSection(self.robot, self.voice)
        
        print("右赛道返程控制器初始化完成")
    
    def execute_initial_turn_right_back(self):
        """执行右返程的初始转向"""
        self.logger.info("执行右返程初始转向")
        
        # 先站立准备
        print("机器人站立准备")
        self.robot.stand(duration=1000, sleep=1.0)
        
        # 右返程固定路径：先站立，右转110°，前进90cm，然后右转
        self.voice.speak_text("开始右赛道返程")
        time.sleep(1)
        
        self.robot.turn_right(110)  # 右转110°
        time.sleep(1)
        
        self.robot.move_distance_imu(0.9, velocity=0.1)  # 前进90cm
        time.sleep(1)
        
        self.robot.turn_right(90)  # 右转进入右返程赛道（修正）
        
        self.logger.info("右返程初始转向完成")
    
    def execute_height_limit_and_stone_road(self):
        """执行限高杆和石板路路段"""
        self.logger.info("开始执行限高杆和石板路路段")
        
        # 1. 限高杆路段（450cm）- 使用太空步方案
        self.voice.speak_text("开始限高杆路段")
        self.height_limit_section.execute_height_limit_section(use_vision=False)
        
        time.sleep(1)
        
        # 2. 石板路路段（400cm）
        self.voice.speak_text("开始通过石板路路段")
        self.stone_road_section.execute_stone_road_section()
        
        self.logger.info("限高杆和石板路路段完成")
    
    def run(self):
        """主要执行流程"""
        try:
            self.logger.info("=" * 50)
            self.logger.info("开始任务8：右赛道返程（写死版）")
            self.logger.info("=" * 50)
            
            # 启动机器人控制器
            self.robot.run()
            time.sleep(2)
            
            # 等待机器人准备就绪
            self.voice.speak_text("机器人系统启动完成，准备执行右赛道返程")
            time.sleep(1)
            
            # 步骤1：执行初始转向
            self.execute_initial_turn_right_back()
            
            # 步骤2：执行限高杆和石板路路段
            self.execute_height_limit_and_stone_road()
            
            # 任务完成
            self.voice.speak_text("右赛道返程任务完成")
            self.logger.info("右赛道返程任务完成")
            
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
    controller = RightBackTrackController()
    controller.run()


if __name__ == "__main__":
    main()