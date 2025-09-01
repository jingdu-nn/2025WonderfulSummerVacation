#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
任务8：回形赛道返程方向 - 识别版
根据loop_track.json读取返程方向并执行对应路段

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
        LoopTrackState, SlopeSection, YellowLightSection,
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
        LoopTrackState = loop_track_common.LoopTrackState
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
        SlopeSection = MockClass
        YellowLightSection = MockClass
        StoneRoadSection = MockClass
        HeightLimitSection = MockClass
        RobotController = MockClass
        VoiceAnnouncer = MockClass
        Logger = MockClass


class LoopTrackBackwardController:
    """回形赛道返程方向控制器"""
    
    def __init__(self):
        # 初始化组件
        self.robot = RobotController()
        self.voice = VoiceAnnouncer()
        self.logger = Logger("LoopTrackBackward")
        self.state_manager = LoopTrackState()
        
        # 初始化各路段控制器
        self.slope_section = SlopeSection(self.robot, self.voice)
        self.yellow_light_section = YellowLightSection(self.robot, self.voice)
        self.stone_road_section = StoneRoadSection(self.robot, self.voice)
        self.height_limit_section = HeightLimitSection(self.robot, self.voice)
        
        print("回形赛道返程控制器初始化完成")
    
    def get_back_direction(self) -> str:
        """获取返程方向"""
        try:
            state = self.state_manager.load_state()
            if state and 'back' in state:
                direction = state['back']
                self.logger.info(f"从状态文件读取返程方向: {direction}")
                return direction
            else:
                self.logger.warning("无法读取状态文件，使用默认返程方向")
                return "R"  # 默认右返程
        except Exception as e:
            self.logger.error(f"读取返程方向失败: {e}")
            return "R"
    
    def execute_initial_turn_for_back(self, back_direction: str):
        """执行返程的初始转向"""
        self.logger.info(f"执行返程初始转向，方向: {back_direction}")
        
        # 先站立准备
        print("机器人站立准备")
        self.robot.stand(duration=1000, sleep=1.0)
        
        if back_direction == "L":
            # 左返程：先站立，左转110°，前进90cm，然后右转
            self.voice.speak_text("开始左赛道返程")
            time.sleep(1)
            
            self.robot.turn_left(110)  # 左转110°
            time.sleep(1)
            
            self.robot.move_distance_imu(0.9, velocity=0.1)  # 前进90cm（修改）
            time.sleep(1)
            
            self.robot.turn_right(90)  # 右转进入左返程赛道
            
        else:  # back_direction == "R"
            # 右返程：先站立，右转110°，前进90cm，然后右转
            self.voice.speak_text("开始右赛道返程")
            time.sleep(1)
            
            self.robot.turn_right(110)  # 右转110°
            time.sleep(1)
            
            self.robot.move_distance_imu(0.9, velocity=0.1)  # 前进90cm
            time.sleep(1)
            
            self.robot.turn_right(90)  # 右转进入右返程赛道（修正）
        
        self.logger.info("返程初始转向完成")
    
    def execute_left_track_back(self):
        """执行左赛道返程：黄灯 -> 上下坡"""
        self.logger.info("开始执行左赛道返程")
        self.voice.speak_text("进入左赛道返程，先通过黄灯识别")
        
        # 1. 执行黄灯识别路段（450cm）
        self.yellow_light_section.execute_yellow_light_section(use_sensors=True, sensor_config=0, is_dead_reckoning=False)
        
        # 2. 执行上下坡路段（400cm）
        self.voice.speak_text("开始通过上下坡路段")
        self.slope_section.execute_slope_section()
        
        self.logger.info("左赛道返程完成")
    
    def execute_right_track_back(self):
        """执行右赛道返程：限高杆 -> 石板路"""
        self.logger.info("开始执行右赛道返程")
        self.voice.speak_text("进入右赛道返程，先通过限高杆")
        
        # 1. 执行限高杆路段（450cm）
        self.height_limit_section.execute_height_limit_section(use_vision=True)
        
        # 2. 执行石板路路段（400cm）
        self.voice.speak_text("开始通过石板路路段")
        self.stone_road_section.execute_stone_road_section()
        
        self.logger.info("右赛道返程完成")
    
    def run(self):
        """主要执行流程"""
        try:
            self.logger.info("=" * 50)
            self.logger.info("开始任务8：回形赛道返程方向")
            self.logger.info("=" * 50)
            
            # 启动机器人控制器
            self.robot.run()
            time.sleep(2)
            
            # 等待机器人准备就绪
            self.voice.speak_text("机器人系统启动完成，准备开始返程任务")
            time.sleep(1)
            
            # 步骤1：读取返程方向
            back_direction = self.get_back_direction()
            
            # 步骤2：执行返程初始转向
            self.execute_initial_turn_for_back(back_direction)
            
            # 步骤3：根据方向执行对应返程赛道
            if back_direction == "L":
                self.execute_left_track_back()
            else:
                self.execute_right_track_back()
            
            # 任务完成
            self.voice.speak_text("回形赛道返程任务完成")
            self.logger.info("任务8完成")
            
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
    controller = LoopTrackBackwardController()
    controller.run()


if __name__ == "__main__":
    main()