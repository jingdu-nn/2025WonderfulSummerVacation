#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
任务10写死版主程序：A-1库卸货

任务流程：
1. 从逆向S弯道走出，向前走100cm
2. 向左转115°
3. 向前走170cm
4. 向右转115°，向前走95cm，再向右转115°，向前走100cm，趴下

作者：Xiaomi Cup 2025 Final Team
"""

import sys
import os
import signal
import time

# 添加共享模块到路径
shared_modules_path = os.path.join(os.path.dirname(__file__), '..', '..', 'shared_modules')
sys.path.insert(0, os.path.abspath(shared_modules_path))

# 添加missions根目录到路径
missions_root = os.path.join(os.path.dirname(__file__), '..', '..')
sys.path.insert(0, os.path.abspath(missions_root))

from motion_control import RobotController, ActionSequence
from utils import get_logger


class Task10A1Unload:
    """任务10 A-1库卸货控制器"""
    
    def __init__(self):
        self.logger = get_logger("Task10A1Unload")
        self.robot = RobotController()
        self.action_seq = ActionSequence(self.robot)
        
        # 注册信号处理器
        signal.signal(signal.SIGINT, self._signal_handler)
        
        self.logger.info("任务10 A-1库卸货控制器初始化完成")
    
    def _signal_handler(self, signum, frame):
        """信号处理器"""
        self.logger.info("收到中断信号，正在安全退出...")
        self.cleanup()
        sys.exit(0)
    
    def run(self):
        """执行任务10 A-1库卸货"""
        self.logger.info("=== 开始执行任务10：A-1库卸货（写死版） ===")
        
        try:
            # 启动机器人控制器
            self.robot.run()
            time.sleep(2)  # 等待系统初始化
            
            # 确保机器人站立
            self.logger.info("机器人站立")
            self.robot.stand(duration=3000, sleep=3.0)
            
            # 执行任务步骤
            if not self._execute_approach_phase():
                return False
            
            if not self._execute_a1_unload():
                return False
            
            self.logger.info("=== 任务10 A-1库卸货执行完成 ===")
            return True
            
        except Exception as e:
            self.logger.error(f"任务执行异常: {e}")
            return False
        finally:
            self.cleanup()
    
    def _execute_approach_phase(self) -> bool:
        """执行接近阶段（从S弯道到A区）"""
        self.logger.info("--- 第一阶段：从S弯道接近A区 ---")
        
        try:
            # 1. 从逆向S弯道走出，向前走110cm
            self.logger.info("步骤1：向前走110cm（从S弯道走出）")
            if not self.robot.move_forward(1.1):
                self.logger.error("向前走110cm失败")
                return False
            
            # 2. 向左转115°
            self.logger.info("步骤2：向左转115°")
            if not self.robot.turn_left(115):
                self.logger.error("向左转115°失败")
                return False
            
            # 3. 向前走240cm（比task1多60cm）
            self.logger.info("步骤3：向前走240cm（比task1多60cm）")
            if not self.robot.move_forward(2.4):
                self.logger.error("向前走240cm失败")
                return False
            
            self.logger.info("接近阶段完成")
            return True
            
        except Exception as e:
            self.logger.error(f"接近阶段异常: {e}")
            return False
    
    def _execute_a1_unload(self) -> bool:
        """执行A-1库卸货"""
        self.logger.info("--- 第二阶段：进入A-1仓库卸货 ---")
        
        try:
            # A-1卸货序列：向右转115°，向前走95cm，再向右转115°，向前走100cm，趴下
            
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
            
            # 4. 向前走120cm
            self.logger.info("步骤4：向前走120cm")
            if not self.robot.move_forward(1.2):
                return False
            
            # 5. 趴下卸载
            self.logger.info("步骤5：趴下卸载")
            # 语音播报抵达库区
            try:
                from utils.voice_announcer import say
                say("抵达A1库卸货")
            except Exception as e:
                print(f"语音播报失败: {e}")
            
            self.robot.lie()
            
            self.logger.info("A-1仓库卸货完成")
            return True
            
        except Exception as e:
            self.logger.error(f"A-1仓库卸货异常: {e}")
            return False
    
    def cleanup(self):
        """清理资源"""
        self.logger.info("清理资源...")
        if hasattr(self, 'robot'):
            self.robot.quit()


def main():
    """主函数"""
    print("启动任务10写死版：A-1库卸货")
    
    try:
        task = Task10A1Unload()
        success = task.run()
        
        if success:
            print("任务10 A-1库卸货执行成功！")
        else:
            print("任务10 A-1库卸货执行失败！")
            
    except KeyboardInterrupt:
        print("\n任务被用户中断")
    except Exception as e:
        print(f"任务执行异常: {e}")


if __name__ == "__main__":
    main()
