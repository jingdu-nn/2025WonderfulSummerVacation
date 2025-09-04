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

from main.a_task import main as a_task_main

def _a():
    print("小米之家机器人任务执行系统")
    print("=" * 30)
    
    # 执行A库任务
    a_task_main()

if __name__ == "__main__":
    _a()