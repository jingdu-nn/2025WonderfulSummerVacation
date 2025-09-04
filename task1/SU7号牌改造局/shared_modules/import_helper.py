#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
导入助手模块
提供鲁棒的模块导入功能，处理cyberdog2环境中的路径问题

作者：Xiaomi Cup 2025 Final Team
"""

import sys
import os
from typing import Any, Optional

def setup_paths(file_path: str):
    """
    设置模块搜索路径
    
    Args:
        file_path: 调用文件的__file__路径
    """
    # 获取调用文件的目录
    caller_dir = os.path.dirname(os.path.abspath(file_path))
    
    # 计算shared_modules路径
    if 'shared_modules' in caller_dir:
        # 如果调用者在shared_modules内部
        shared_modules_path = os.path.dirname(caller_dir)
        while not os.path.basename(shared_modules_path) == 'shared_modules':
            shared_modules_path = os.path.dirname(shared_modules_path)
    else:
        # 如果调用者在任务目录中
        shared_modules_path = os.path.join(caller_dir, '..', '..', 'shared_modules')
    
    shared_modules_path = os.path.abspath(shared_modules_path)
    
    # 添加路径到sys.path
    if shared_modules_path not in sys.path:
        sys.path.insert(0, shared_modules_path)
    
    # 计算visual路径
    visual_path = os.path.join(os.path.dirname(shared_modules_path), 'visual')
    visual_path = os.path.abspath(visual_path)
    
    if os.path.exists(visual_path) and visual_path not in sys.path:
        sys.path.insert(0, visual_path)
    
    return shared_modules_path, visual_path

def safe_import_utils():
    """
    安全导入utils模块
    
    Returns:
        tuple: (get_logger, TouchSensorMonitor, TouchSensorType, wait_for_touch, TouchMode)
    """
    get_logger = None
    TouchSensorMonitor = None
    TouchSensorType = None
    wait_for_touch = None
    TouchMode = None
    
    try:
        # 方法1：直接从utils包导入
        from utils import get_logger, TouchSensorMonitor, TouchSensorType
        from utils.enhanced_touch_sensor import wait_for_touch, TouchMode
        return get_logger, TouchSensorMonitor, TouchSensorType, wait_for_touch, TouchMode
    except ImportError:
        pass
    
    try:
        # 方法2：分别导入
        from utils.logger import get_logger
        from utils.touch_sensor import TouchSensorMonitor, TouchSensorType
        from utils.enhanced_touch_sensor import wait_for_touch, TouchMode
        return get_logger, TouchSensorMonitor, TouchSensorType, wait_for_touch, TouchMode
    except ImportError:
        pass
    
    try:
        # 方法3：直接导入模块文件
        import importlib.util
        
        # 导入logger
        current_dir = os.path.dirname(__file__)
        logger_path = os.path.join(current_dir, 'utils', 'logger.py')
        if os.path.exists(logger_path):
            spec = importlib.util.spec_from_file_location("logger", logger_path)
            logger_module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(logger_module)
            get_logger = logger_module.get_logger
        
        # 导入touch_sensor
        touch_sensor_path = os.path.join(current_dir, 'utils', 'touch_sensor.py')
        if os.path.exists(touch_sensor_path):
            spec = importlib.util.spec_from_file_location("touch_sensor", touch_sensor_path)
            touch_sensor_module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(touch_sensor_module)
            TouchSensorMonitor = touch_sensor_module.TouchSensorMonitor
            TouchSensorType = touch_sensor_module.TouchSensorType
        
        return get_logger, TouchSensorMonitor, TouchSensorType
    except Exception:
        pass
    
    # 如果所有方法都失败，创建替代品
    if get_logger is None:
        def get_logger(name):
            import logging
            return logging.getLogger(name)
    
    if TouchSensorType is None:
        from enum import Enum
        class TouchSensorType(Enum):
            HEAD_BACK = "head_back"
            HEAD_CHIN = "head_chin"
    
    if TouchSensorMonitor is None:
        class TouchSensorMonitor:
            def __init__(self):
                self.callbacks = {}
                self.last_trigger_time = {}
                print("使用TouchSensorMonitor替代实现")
            
            def register_callback(self, sensor_type, callback):
                self.callbacks[sensor_type] = callback
                print(f"已注册 {sensor_type} 传感器回调")
            
            def start_monitoring(self):
                print("开始监控触摸传感器（模拟）")
            
            def stop_monitoring(self):
                print("停止监控触摸传感器")
            
            def trigger_sensor(self, sensor_type):
                if sensor_type in self.callbacks:
                    self.callbacks[sensor_type]()
            
            def wait_for_touch(self, sensor_type, timeout=60.0):
                print(f"等待触摸 {sensor_type} 传感器（模拟）")
                print("按Enter键继续...")
                try:
                    input()
                    return True
                except:
                    return False
    
    return get_logger, TouchSensorMonitor, TouchSensorType

def safe_import_motion_control():
    """
    安全导入运动控制模块
    
    Returns:
        tuple: (RobotController, ActionSequence)
    """
    try:
        from motion_control import RobotController, ActionSequence
        return RobotController, ActionSequence
    except ImportError:
        pass
    
    try:
        from motion_control.robot_controller import RobotController
        from motion_control.action_sequence import ActionSequence
        return RobotController, ActionSequence
    except ImportError:
        pass
    
    # 创建替代品
    class RobotController:
        def __init__(self):
            print("使用RobotController替代实现")
        
        def run(self):
            print("机器人控制器启动（模拟）")
        
        def quit(self):
            print("机器人控制器退出（模拟）")
        
        def stand(self, duration=3000, sleep=3.0):
            print(f"机器人站立 {duration}ms（模拟）")
            import time
            time.sleep(sleep)
        
        def lie(self):
            print("机器人趴下（模拟）")
        
        def move_forward(self, distance):
            print(f"机器人前进 {distance}m（模拟）")
            import time
            time.sleep(2)
            return True
        
        def turn_left(self, angle):
            print(f"机器人左转 {angle}度（模拟）")
            import time
            time.sleep(1)
            return True
        
        def turn_right(self, angle):
            print(f"机器人右转 {angle}度（模拟）")
            import time
            time.sleep(1)
            return True
    
    class ActionSequence:
        def __init__(self, robot):
            self.robot = robot
            print("使用ActionSequence替代实现")
    
    return RobotController, ActionSequence