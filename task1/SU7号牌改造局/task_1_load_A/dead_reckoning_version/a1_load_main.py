#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
任务1写死版主程序：A-1库装载

任务流程：
1. 初始状态是趴下
2. 站立起来，向前走100cm
3. 向右转115°
4. 向前走150cm
5. 向右转115°，向前走95cm，再向右转115°，向前走100cm，趴下，语音播报A-1

作者：Xiaomi Cup 2025 Final Team
"""

import sys
import os
import signal
import time

# 添加共享模块到路径
shared_modules_path = os.path.join(os.path.dirname(__file__), '..', '..', 'shared_modules')
sys.path.insert(0, os.path.abspath(shared_modules_path))

# 添加missions根目录到路径，用于导入warehouse_state等
missions_root = os.path.join(os.path.dirname(__file__), '..', '..')
sys.path.insert(0, os.path.abspath(missions_root))

from motion_control import RobotController, ActionSequence
from utils import get_logger

# 导入共享模块功能
try:
    from utils.voice_announcer import say
    print("✅ 语音模块导入成功")
except ImportError as e:
    print(f"❌ 语音模块导入失败: {e}")
    def say(text, wait_time=0.5):
        print(f"[语音播报] {text}")
        import time
        time.sleep(wait_time)


class Task1A1Load:
    """任务1 A-1库装载控制器"""
    
    def __init__(self):
        self.logger = get_logger("Task1A1Load")
        self.robot = RobotController()
        self.action_seq = ActionSequence(self.robot)
        
        # 注册信号处理器
        signal.signal(signal.SIGINT, self._signal_handler)
        
        self.logger.info("任务1 A-1库装载控制器初始化完成")
    
    def _signal_handler(self, signum, frame):
        """信号处理器"""
        self.logger.info("收到中断信号，正在安全退出...")
        self.cleanup()
        sys.exit(0)
    
    def run(self):
        """执行任务1 A-1库装载"""
        self.logger.info("=== 开始执行任务1：A-1库装载（写死版） ===")
        
        try:
            # 启动机器人控制器
            self.robot.run()
            time.sleep(2)  # 等待系统初始化
            
            # 执行任务步骤
            if not self._execute_approach_phase():
                return False
            
            if not self._execute_a1_entry():
                return False
            
            self.logger.info("=== 任务1 A-1库装载执行完成 ===")
            return True
            
        except Exception as e:
            self.logger.error(f"任务执行异常: {e}")
            return False
        finally:
            self.cleanup()
    
    def _execute_approach_phase(self) -> bool:
        """执行接近阶段（通用部分）"""
        self.logger.info("--- 第一阶段：接近仓库区域 ---")
        
        try:
            # 1. 从趴下状态站立起来
            self.logger.info("步骤1：站立")
            self.robot.stand(duration=3000, sleep=3.0)
            
            # 2. 向前走100cm
            self.logger.info("步骤2：向前走100cm")
            if not self.robot.move_forward(1.0):
                self.logger.error("向前走100cm失败")
                return False
            
            # 3. 向右转115°
            self.logger.info("步骤3：向右转115°")
            if not self.robot.turn_right(115):
                self.logger.error("向右转115°失败")
                return False
            
            # 4. 向前走180cm
            self.logger.info("步骤4：向前走180cm")
            if not self.robot.move_forward(1.8):
                self.logger.error("向前走180cm失败")
                return False
            
            self.logger.info("接近阶段完成")
            return True
            
        except Exception as e:
            self.logger.error(f"接近阶段异常: {e}")
            return False
    
    def _execute_a1_entry(self) -> bool:
        """执行A-1库入库"""
        self.logger.info("--- 第二阶段：进入A-1仓库 ---")
        
        try:
            # 语音播报A-1（在转弯前）
            say("A-1")
            
            # A-1库序列：向右转115°，向前走95cm，再向右转115°，向前走100cm，趴下
            
            # 1. 向右转115°
            self.logger.info("步骤1：向右转115°")
            if not self.robot.turn_right(115):
                return False
            
            # 2. 向前走95cm
            self.logger.info("步骤2：向前走95cm")
            if not self.robot.move_forward(0.95):
                return False
            
            # 3. 再向右转115°
            self.logger.info("步骤3：再向右转115°")
            if not self.robot.turn_right(115):
                return False
            
            # 4. 语音播报A-1
            self.logger.info("步骤4：语音播报A-1")
            say("A-1")
            
            # 5. 向前走100cm
            self.logger.info("步骤5：向前走100cm")
            if not self.robot.move_forward(1.0):
                return False
            
            # 6. 趴下
            self.logger.info("步骤6：趴下")
            self.robot.lie()
            
            self.logger.info("A-1仓库入库完成")
            return True
            
        except Exception as e:
            self.logger.error(f"A-1仓库入库异常: {e}")
            return False
    
    def cleanup(self):
        """清理资源"""
        self.logger.info("清理资源...")
        if hasattr(self, 'robot'):
            self.robot.quit()


def main():
    """主函数"""
    print("启动任务1写死版：A-1库装载")
    
    try:
        task = Task1A1Load()
        success = task.run()
        
        if success:
            print("任务1 A-1库装载执行成功！")
        else:
            print("任务1 A-1库装载执行失败！")
            
    except KeyboardInterrupt:
        print("\n任务被用户中断")
    except Exception as e:
        print(f"任务执行异常: {e}")


if __name__ == "__main__":
    main()
