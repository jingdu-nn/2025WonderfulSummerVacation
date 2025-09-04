#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
综合任务执行器
"""

import sys
import os
import time

# 添加src到路径中
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from modules.ctrl_robot import CtrlRobot
from utils.helpers import read_conf
from main.a_task import imu_walk_dist as _a_task
from main.b_task import _a as _b_task

class _A:
    def __init__(self):
        self._a = read_conf()
        self._b = CtrlRobot()
        self._b.start()
        
    def _c(self):
        """
        执行完整任务流程
        """
        print("🚀 启动任务执行器")
        print(f"📋 配置信息:")
        print(f"   A库: {self._a['a_wh']}")
        print(f"   B库: {self._a['b_wh']}")
        print(f"   箭头: {self._a['arr']}")
        
        # 执行A库任务
        print("\n📍执行A库任务...")
        _d = _a_task(self._b, 1.0)
        if not _d:
            print("❌ A库任务失败")
            return False
            
        print("✅ A库任务完成")
        time.sleep(1)
        
        # 执行B库任务
        print("\n📍执行B库任务...")
        _e = _b_task(self._b, 1.5)
        if not _e:
            print("❌ B库任务失败")
            return False
            
        print("✅ B库任务完成")
        return True

def main():
    print("🤖 小米之家机器人任务执行器")
    print("=" * 30)
    
    _f = _A()
    _g = _f._c()
    
    if _g:
        print("\n🎉 所有任务成功完成!")
    else:
        print("\n💥 任务执行中出现问题!")

if __name__ == "__main__":
    main()