#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
小米之家机器人任务执行主程序
"""

import sys
import os
import time

# 添加src到路径中
sys.path.append(os.path.dirname(__file__))

from main.executor import main as _a

def _b():
    print("🤖 小米之家机器人任务执行系统")
    print("=" * 40)
    print("开始执行任务...")
    time.sleep(1)
    
    # 执行主任务
    _a()

if __name__ == "__main__":
    _b()