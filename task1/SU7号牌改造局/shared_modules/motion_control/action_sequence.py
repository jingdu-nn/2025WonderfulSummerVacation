#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
动作序列控制器
封装复杂的动作序列，如入库、出库、装卸货等

作者：Xiaomi Cup 2025 Final Team  
"""

import time
from typing import List, Callable, Any
from .robot_controller import RobotController


class ActionSequence:
    """动作序列执行器"""
    
    def __init__(self, robot_controller: RobotController):
        self.robot = robot_controller
        self.action_queue = []
        
    def add_action(self, action_func: Callable, *args, **kwargs):
        """添加动作到队列"""
        self.action_queue.append((action_func, args, kwargs))
    
    def clear_queue(self):
        """清空动作队列"""
        self.action_queue.clear()
    
    def execute_queue(self) -> bool:
        """执行队列中的所有动作"""
        print(f"开始执行动作序列，共 {len(self.action_queue)} 个动作")
        
        for i, (action_func, args, kwargs) in enumerate(self.action_queue):
            print(f"执行动作 {i+1}/{len(self.action_queue)}: {action_func.__name__}")
            
            try:
                result = action_func(*args, **kwargs)
                if result is False:
                    print(f"动作 {action_func.__name__} 执行失败")
                    return False
                    
                # 动作间短暂暂停
                time.sleep(0.5)
                
            except Exception as e:
                print(f"动作 {action_func.__name__} 执行异常: {e}")
                return False
        
        print("动作序列执行完成")
        self.clear_queue()
        return True
    
    def enter_warehouse_a1(self):
        """进入A-1库的标准动作序列"""
        print("执行进入A-1库动作序列")
        self.clear_queue()
        
        # A-1库进入序列：向右转115°，向前走85cm，再向右转115°，向前走100cm，趴下
        self.add_action(self.robot.turn_right, 90)
        self.add_action(self.robot.move_forward, 0.85)
        self.add_action(self.robot.turn_right, 90)
        self.add_action(self.robot.move_forward, 1.0)
        self.add_action(self.robot.lie)
        
        return self.execute_queue()
    
    def enter_warehouse_a2(self):
        """进入A-2库的标准动作序列"""
        print("执行进入A-2库动作序列")
        self.clear_queue()
        
        # A-2库进入序列：向左转115°，向前走85cm，再向左转115°，向前走100cm，趴下
        self.add_action(self.robot.turn_left, 90)
        self.add_action(self.robot.move_forward, 0.85)
        self.add_action(self.robot.turn_left, 90)
        self.add_action(self.robot.move_forward, 1.0)
        self.add_action(self.robot.lie)
        
        return self.execute_queue()
    
    def exit_warehouse_a1(self):
        """退出A-1库的标准动作序列"""
        print("执行退出A-1库动作序列")
        self.clear_queue()
        
        # 站立，后退100cm，向右转115°，向前走85cm，再向左转115°
        self.add_action(self.robot.stand)
        self.add_action(self.robot.move_backward, 1.0)
        self.add_action(self.robot.turn_right, 90)
        self.add_action(self.robot.move_forward, 0.85)
        self.add_action(self.robot.turn_left, 90)
        
        return self.execute_queue()
    
    def exit_warehouse_a2(self):
        """退出A-2库的标准动作序列"""
        print("执行退出A-2库动作序列")
        self.clear_queue()
        
        # 站立，后退100cm，向左转115°，向前走85cm，再向右转115°
        self.add_action(self.robot.stand)
        self.add_action(self.robot.move_backward, 1.0)
        self.add_action(self.robot.turn_left, 90)
        self.add_action(self.robot.move_forward, 0.85)
        self.add_action(self.robot.turn_right, 90)
        
        return self.execute_queue()
    
    def enter_warehouse_b1(self):
        """进入B-1库的标准动作序列"""
        print("执行进入B-1库动作序列")
        self.clear_queue()
        
        # B-1库进入序列：向左转115°，向前走100cm，再向右转115°，向前走100cm，趴下
        self.add_action(self.robot.turn_left, 90)
        self.add_action(self.robot.move_forward, 1.0)
        self.add_action(self.robot.turn_right, 90)
        self.add_action(self.robot.move_forward, 1.0)
        self.add_action(self.robot.lie)
        
        return self.execute_queue()
    
    def enter_warehouse_b2(self):
        """进入B-2库的标准动作序列"""
        print("执行进入B-2库动作序列")
        self.clear_queue()
        
        # B-2库进入序列：向右转115°，向前走100cm，再向左转115°，向前走100cm，趴下
        self.add_action(self.robot.turn_right, 90)
        self.add_action(self.robot.move_forward, 1.0)
        self.add_action(self.robot.turn_left, 90)
        self.add_action(self.robot.move_forward, 1.0)
        self.add_action(self.robot.lie)
        
        return self.execute_queue()
    
    def exit_warehouse_b1(self):
        """退出B-1库的标准动作序列"""
        print("执行退出B-1库动作序列")
        self.clear_queue()
        
        # 站立，后退100cm，向右转115°，向前走100cm，再向左转115°
        self.add_action(self.robot.stand)
        self.add_action(self.robot.move_backward, 1.0)
        self.add_action(self.robot.turn_right, 90)
        self.add_action(self.robot.move_forward, 1.0)
        self.add_action(self.robot.turn_left, 90)
        
        return self.execute_queue()
    
    def exit_warehouse_b2(self):
        """退出B-2库的标准动作序列"""
        print("执行退出B-2库动作序列")
        self.clear_queue()
        
        # 站立，后退100cm，向左转115°，向前走100cm，再向右转115°
        self.add_action(self.robot.stand)
        self.add_action(self.robot.move_backward, 1.0)
        self.add_action(self.robot.turn_left, 90)
        self.add_action(self.robot.move_forward, 1.0)
        self.add_action(self.robot.turn_right, 90)
        
        return self.execute_queue()
    
    def move_b1_to_b2(self):
        """从B-1库移动到B-2库"""
        print("执行从B-1库移动到B-2库动作序列")
        self.clear_queue()
        
        # 向右转115°，向前走300cm，再向左转115°，向前走100cm，趴下
        self.add_action(self.robot.turn_right, 90)
        self.add_action(self.robot.move_forward, 3.0)
        self.add_action(self.robot.turn_left, 90)
        self.add_action(self.robot.move_forward, 1.0)
        self.add_action(self.robot.lie)
        
        return self.execute_queue()
    
    def move_b2_to_b1(self):
        """从B-2库移动到B-1库"""
        print("执行从B-2库移动到B-1库动作序列")
        self.clear_queue()
        
        # 向左转115°，向前走300cm，再向右转115°，向前走100cm，趴下
        self.add_action(self.robot.turn_left, 90)
        self.add_action(self.robot.move_forward, 3.0)
        self.add_action(self.robot.turn_right, 90)
        self.add_action(self.robot.move_forward, 1.0)
        self.add_action(self.robot.lie)
        
        return self.execute_queue()
