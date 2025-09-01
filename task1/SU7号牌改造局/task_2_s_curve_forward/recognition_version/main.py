#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
任务2识别版主程序：正向S弯（基于前序任务识别结果）

任务流程：
1. 等待触摸传感器触发（三击后脑勺）
2. 站立
3. 向后退100cm
4. 根据前序任务的装载库区（A-1或A-2）选择路径：
   - A-1：向右转115°，向前走85cm，再向左转115°，向前走270cm，向右转115°完成
   - A-2：向左转115°，向前走85cm，再向右转115°，向前走270cm，向右转115°完成

作者：Xiaomi Cup 2025 Final Team
"""

import sys
import os
import signal
import time

# 添加共享模块到路径
shared_modules_path = os.path.join(os.path.dirname(__file__), '..', '..', 'shared_modules')
sys.path.append(os.path.abspath(shared_modules_path))

from motion_control import RobotController, ActionSequence
from utils import get_logger
from utils.touch_sensor import wait_for_touch, TouchMode
from utils.voice_announcer import say


class Task2Recognition:
    """任务2识别版控制器"""
    
    def __init__(self, loaded_warehouse: str = None):
        """
        初始化任务2识别版控制器
        
        Args:
            loaded_warehouse: 前序任务的装载库区（'A-1'或'A-2'）
        """
        self.logger = get_logger("Task2Recognition")
        self.robot = RobotController()
        self.action_seq = ActionSequence(self.robot)
        self.loaded_warehouse = loaded_warehouse
        
        # 注册信号处理器
        signal.signal(signal.SIGINT, self._signal_handler)
        
        self.logger.info(f"任务2识别版控制器初始化完成，装载库区: {loaded_warehouse}")
    
    def _signal_handler(self, signum, frame):
        """信号处理器"""
        self.logger.info("收到中断信号，正在安全退出...")
        self.cleanup()
        sys.exit(0)
    

    
    def run(self):
        """执行任务2识别版"""
        self.logger.info("=== 开始执行任务2：正向S弯（识别版） ===")
        
        try:
            # 启动机器人控制器
            self.robot.run()
            time.sleep(2)  # 等待系统初始化
            
            # 检查装载库区参数
            if not self.loaded_warehouse:
                self.logger.warning("未指定装载库区，尝试从warehouse.json获取")
                try:
                    sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
                    from warehouse_state import get_task1_result
                    self.loaded_warehouse = get_task1_result()
                    if self.loaded_warehouse:
                        self.logger.info(f"从warehouse.json获取到装载库区: {self.loaded_warehouse}")
                    else:
                        self.logger.warning("warehouse.json中没有找到装载库区信息")
                except Exception as e:
                    self.logger.warning(f"读取warehouse.json失败: {e}")
                
            if not self.loaded_warehouse:
                self.logger.warning("尝试从环境变量获取")
                self.loaded_warehouse = os.getenv('LOADED_WAREHOUSE')
                
            if not self.loaded_warehouse:
                self.logger.error("无法确定装载库区，请指定A-1或A-2")
                return False
            
            # 执行任务步骤
            if not self._wait_for_touch_trigger():
                return False
            
            if not self._execute_exit_warehouse():
                return False
            
            if not self._execute_path_to_s_curve():
                return False
            
            self.logger.info("=== 任务2识别版执行完成 ===")
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
            self.logger.info("等待三击后脑勺触发任务...")
            
            # 使用增强触摸传感器，三击模式避免与电量播报冲突
            if not wait_for_touch(TouchMode.TRIPLE_TOUCH, timeout=60):
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
    
    def _execute_path_to_s_curve(self) -> bool:
        """执行到S弯道的路径"""
        self.logger.info("--- 第三阶段：前往S弯道 ---")
        
        try:
            if self.loaded_warehouse == "A-1":
                return self._execute_a1_to_s_curve()
            elif self.loaded_warehouse == "A-2":
                return self._execute_a2_to_s_curve()
            else:
                self.logger.error(f"未知的装载库区: {self.loaded_warehouse}")
                return False
                
        except Exception as e:
            self.logger.error(f"前往S弯道路径异常: {e}")
            return False
    
    def _execute_a1_to_s_curve(self) -> bool:
        """从A-1库前往S弯道"""
        self.logger.info("执行A-1到S弯道路径")
        
        try:
            # A-1路径：向右转115°，向前走85cm，再向左转115°，向前走180cm，向右转准备进入S弯道
            
            # 1. 向右转115°
            self.logger.info("步骤1：向右转115°")
            if not self.robot.turn_right(115):
                return False
            
            # 2. 向前走85cm
            self.logger.info("步骤2：向前走85cm")
            if not self.robot.move_forward(0.85):
                return False
            
            # 3. 向左转115°
            self.logger.info("步骤3：向左转115°")
            if not self.robot.turn_left(115):
                return False
            
            # 4. 向前走270cm
            self.logger.info("步骤4：向前走270cm")
            if not self.robot.move_forward(2.7):
                return False
            
            # 5. 向右转115°完成路径
            self.logger.info("步骤5：向右转115°完成路径")
            if not self.robot.turn_right(115):
                return False
            
            self.logger.info("A-1到S弯道路径完成")
            say("S弯路径执行完成")
            return True
            
        except Exception as e:
            self.logger.error(f"A-1到S弯道路径异常: {e}")
            return False
    
    def _execute_a2_to_s_curve(self) -> bool:
        """从A-2库前往S弯道"""
        self.logger.info("执行A-2到S弯道路径")
        
        try:
            # A-2路径：向左转115°，向前走85cm，再向右转115°，向前走180cm，向右转准备进入S弯道
            
            # 1. 向左转115°
            self.logger.info("步骤1：向左转115°")
            if not self.robot.turn_left(115):
                return False
            
            # 2. 向前走85cm
            self.logger.info("步骤2：向前走85cm")
            if not self.robot.move_forward(0.85):
                return False
            
            # 3. 向右转115°
            self.logger.info("步骤3：向右转115°")
            if not self.robot.turn_right(115):
                return False
            
            # 4. 向前走270cm
            self.logger.info("步骤4：向前走270cm")
            if not self.robot.move_forward(2.7):
                return False
            
            # 5. 向右转115°完成路径
            self.logger.info("步骤5：向右转115°完成路径")
            if not self.robot.turn_right(115):
                return False
            
            self.logger.info("A-2到S弯道路径完成")
            say("S弯路径执行完成")
            return True
            
        except Exception as e:
            self.logger.error(f"A-2到S弯道路径异常: {e}")
            return False
    
    def cleanup(self):
        """清理资源"""
        self.logger.info("清理资源...")
        if hasattr(self, 'robot'):
            self.robot.quit()


def main():
    """主函数"""
    print("启动任务2识别版：正向S弯")
    
    # 从命令行参数或环境变量获取装载库区
    loaded_warehouse = None
    if len(sys.argv) > 1:
        loaded_warehouse = sys.argv[1]
    else:
        loaded_warehouse = os.getenv('LOADED_WAREHOUSE')
    
    if not loaded_warehouse:
        print("请指定装载库区：")
        print("方式1：python main.py A-1")
        print("方式2：export LOADED_WAREHOUSE=A-1")
        return
    
    try:
        task = Task2Recognition(loaded_warehouse=loaded_warehouse)
        success = task.run()
        
        if success:
            print("任务2识别版执行成功！")
        else:
            print("任务2识别版执行失败！")
            
    except KeyboardInterrupt:
        print("\n任务被用户中断")
    except Exception as e:
        print(f"任务执行异常: {e}")


if __name__ == "__main__":
    main()
