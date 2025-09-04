"""
运动控制模块
提供机器狗的基础运动控制功能
"""

from .robot_controller import RobotController
from .action_sequence import ActionSequence

__all__ = ['RobotController', 'ActionSequence']
