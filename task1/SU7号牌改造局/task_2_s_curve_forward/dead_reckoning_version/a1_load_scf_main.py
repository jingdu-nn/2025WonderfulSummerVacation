#!/usr/bin/env python3
"""
任务2写死版主程序：A-1装载后的正向S弯

任务流程：
1. 等待触摸传感器触发（三击后脑勺）
2. 站立
3. 向后退100cm
4. A-1路径：向右转115°，向前走85cm，再向左转115°，向前走270cm，向右转115°完成

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

# 导入共享模块功能
try:
    from utils.enhanced_touch_sensor import wait_for_touch, TouchMode
    from utils.voice_announcer import say
    print("✅ 增强模块导入成功")
except ImportError as e:
    print(f"❌ 增强模块导入失败: {e}")
    def say(text, wait_time=0.5):
        print(f"[语音播报] {text}")
        import time
        time.sleep(wait_time)
    def wait_for_touch(mode=None, timeout=15):
        print(f"[等待触摸] 15秒内按Enter继续，或等待自动进入下一环节...")
        import threading
        import time
        
        def wait_input():
            try:
                input()
                return True
            except:
                return False
        
        start_time = time.time()
        input_thread = threading.Thread(target=wait_input, daemon=True)
        input_thread.start()
        
        while time.time() - start_time < timeout:
            if not input_thread.is_alive():
                print("手动触摸确认，继续执行...")
                return True
            time.sleep(0.1)
        
        print("15秒超时，自动进入下一环节...")
        return True
    
    class TouchMode:
        TRIPLE_TOUCH = "triple"
        SIX_TOUCHES = "six_touches"


class Task2A1LoadSCF:
    """任务2 A-1装载后正向S弯控制器"""
    
    def __init__(self):
        self.logger = get_logger("Task2A1LoadSCF")
        self.robot = RobotController()
        self.action_seq = ActionSequence(self.robot)
        
        # 注册信号处理器
        signal.signal(signal.SIGINT, self._signal_handler)
        
        self.logger.info("任务2 A-1装载后正向S弯控制器初始化完成")
    
    def _signal_handler(self, signum, frame):
        """信号处理器"""
        self.logger.info("收到中断信号，正在安全退出...")
        self.cleanup()
        sys.exit(0)
    

    
    def run(self):
        """执行任务2 A-1装载后正向S弯"""
        self.logger.info("=== 开始执行任务2：A-1装载后正向S弯（写死版） ===")
        
        try:
            # 启动机器人控制器
            self.robot.run()
            time.sleep(2)  # 等待系统初始化
            
            # 执行任务步骤
            if not self._wait_for_touch_trigger():
                return False
            
            if not self._execute_exit_warehouse():
                return False
            
            if not self._execute_a1_to_s_curve():
                return False
            
            self.logger.info("=== 任务2 A-1装载后正向S弯执行完成 ===")
            return True
            
        except Exception as e:
            self.logger.error(f"任务执行异常: {e}")
            return False
        finally:
            self.cleanup()
    
    def _wait_for_touch_trigger(self) -> bool:
        """等待触摸传感器触发"""
        self.logger.info("--- 第一阶段：等待触摸传感器触发 ---")
        
        try:
            self.logger.info("等待触摸触发任务（15秒超时自动继续）...")
            
            # 使用15秒超时的触摸等待
            if not wait_for_touch(TouchMode.SIX_TOUCHES, timeout=15):
                self.logger.error("等待触摸传感器超时")
                return False
            
            self.logger.info("触摸传感器触发成功")
            say("开始执行S弯任务")
            return True
            
        except Exception as e:
            self.logger.error(f"等待触摸传感器异常: {e}")
            return False
    
    def _execute_exit_warehouse(self) -> bool:
        """执行出库阶段"""
        self.logger.info("--- 第二阶段：出库 ---")
        
        try:
            # 1. 站立
            self.logger.info("步骤1：站立")
            self.robot.stand(duration=3000, sleep=3.0)
            
            # 2. 向后退100cm
            self.logger.info("步骤2：向后退100cm")
            if not self.robot.move_backward(1.0):
                self.logger.error("向后退100cm失败")
                return False
            
            self.logger.info("出库阶段完成")
            return True
            
        except Exception as e:
            self.logger.error(f"出库阶段异常: {e}")
            return False
    
    def _execute_a1_to_s_curve(self) -> bool:
        """执行A-1到S弯道路径"""
        self.logger.info("--- 第三阶段：A-1到S弯道路径 ---")
        
        try:
            # A-1路径：向右转115°，向前走85cm，再向左转115°，向前走180cm，向右转准备进入S弯道
            
            # 1. 向右转115°
            self.logger.info("步骤1：向右转115°")
            if not self.robot.turn_right(115):
                return False
            
            # 2. 向前走95cm
            self.logger.info("步骤2：向前走95cm")
            if not self.robot.move_forward(0.95):
                return False
            
            # 3. 向左转115°
            self.logger.info("步骤3：向左转115°")
            if not self.robot.turn_left(115):
                return False
            
                    # 4. 向前走220cm
        self.logger.info("步骤4：向前走220cm")
        if not self.robot.move_forward(2.2):
            return False
            
            # 5. 向右转115°完成路径
            self.logger.info("步骤5：向右转115°完成路径")
            if not self.robot.turn_right(115):
                return False
            
            # 6. 向前走50cm
            self.logger.info("步骤6：向前走50cm")
            if not self.robot.move_forward(0.5):
                return False
            
            self.logger.info("A-1到S弯道路径完成")
            say("S弯路径执行完成")
            return True
            
        except Exception as e:
            self.logger.error(f"A-1到S弯道路径异常: {e}")
            return False
    
    def cleanup(self):
        """清理资源"""
        self.logger.info("清理资源...")
        if hasattr(self, 'robot'):
            self.robot.quit()


def main():
    """主函数"""
    print("启动任务2写死版：A-1装载后正向S弯")
    
    try:
        task = Task2A1LoadSCF()
        success = task.run()
        
        if success:
            print("任务2 A-1装载后正向S弯执行成功！")
        else:
            print("任务2 A-1装载后正向S弯执行失败！")
            
    except KeyboardInterrupt:
        print("\n任务被用户中断")
    except Exception as e:
        print(f"任务执行异常: {e}")


if __name__ == "__main__":
    main()
