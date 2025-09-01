#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
任务11写死版主程序：A-2卸货后回到充电站

任务流程：
1. 等待触摸传感器触发（后脑勺或下巴）
2. 站立
3. 向后退100cm
4. 向左转115°，向前走85cm，再向右转115°
5. 向前走180cm
6. 向右转115°
7. 向后退100cm（进入充电站）
8. 趴下

作者：Xiaomi Cup 2025 Final Team
"""

import sys
import os
import signal
import time

# 添加共享模块到路径
shared_modules_path = os.path.join(os.path.dirname(__file__), '..', '..', 'shared_modules')
sys.path.append(os.path.abspath(shared_modules_path))

# 使用鲁棒的导入机制
try:
    from import_helper import setup_paths, safe_import_utils, safe_import_motion_control
    
    # 设置路径
    setup_paths(__file__)
    
    # 安全导入模块
    get_logger, TouchSensorMonitor, TouchSensorType, wait_for_touch, TouchMode = safe_import_utils()
    RobotController, ActionSequence = safe_import_motion_control()
    
    print("✅ 所有模块导入成功")
except ImportError:
    # 降级到传统导入方式
    print("⚠️ 使用传统导入方式")
    try:
        from motion_control import RobotController, ActionSequence
        from utils import get_logger, TouchSensorMonitor, TouchSensorType
        from utils.enhanced_touch_sensor import wait_for_touch, TouchMode
        print("✅ 传统导入成功")
    except ImportError as e:
        print(f"❌ 导入失败: {e}")
        # 创建最基本的替代品
        def get_logger(name):
            import logging
            return logging.getLogger(name)
        
        from enum import Enum
        class TouchSensorType(Enum):
            HEAD_BACK = "head_back"
            HEAD_CHIN = "head_chin"
        
        class TouchSensorMonitor:
            def __init__(self):
                print("使用基础TouchSensorMonitor")
            def register_callback(self, sensor_type, callback):
                pass
            def start_monitoring(self):
                pass
            def stop_monitoring(self):
                pass
        
        class RobotController:
            def __init__(self):
                print("使用基础RobotController")
            def run(self): pass
            def quit(self): pass
            def stand(self, duration=3000, sleep=3.0): 
                import time; time.sleep(sleep)
            def lie(self): pass
            def move_forward(self, distance): 
                import time; time.sleep(2); return True
            def turn_left(self, angle): 
                import time; time.sleep(1); return True
            def turn_right(self, angle): 
                import time; time.sleep(1); return True
        
        class ActionSequence:
            def __init__(self, robot): pass


class Task11A2Unload:
    """任务11 A-2卸货后回充电站控制器"""
    
    def __init__(self):
        self.logger = get_logger("Task11A2Unload")
        self.robot = RobotController()
        self.action_seq = ActionSequence(self.robot)
        
        # 注册信号处理器
        signal.signal(signal.SIGINT, self._signal_handler)
        
        # 移除触摸传感器初始化（使用新的触摸系统）
        self.touch_detected = False
        
        self.logger.info("任务11 A-2卸货后回充电站控制器初始化完成")
    
    def _signal_handler(self, signum, frame):
        """信号处理器"""
        self.logger.info("收到中断信号，正在安全退出...")
        self.cleanup()
        sys.exit(0)
    
    def _on_touch_detected(self):
        """触摸传感器回调"""
        self.logger.info("检测到触摸传感器触发")
        self.touch_detected = True
    
    def run(self):
        """执行任务11 A-2卸货后回充电站"""
        self.logger.info("=== 开始执行任务11：A-2卸货后回充电站（写死版） ===")
        
        try:
            # 启动机器人控制器
            self.robot.run()
            time.sleep(2)  # 等待系统初始化
            
            # 执行任务步骤
            if not self._wait_for_touch_trigger():
                return False
            
            if not self._execute_exit_warehouse():
                return False
            
            if not self._execute_return_to_charge():
                return False
            
            self.logger.info("=== 任务11 A-2卸货后回充电站执行完成 ===")
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
            say("开始执行回充电站任务")
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
    
    def _execute_return_to_charge(self) -> bool:
        """执行返回充电站"""
        self.logger.info("--- 第三阶段：A-2库返回充电站 ---")
        
        try:
            # A-2返回充电站序列：向左转115°，向前走85cm，再向右转115°，向前走180cm，向右转115°，向后退100cm，趴下
            
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
            
            # 4. 向前走180cm
            self.logger.info("步骤4：向前走180cm")
            if not self.robot.move_forward(1.8):
                return False
            
            # 5. 向右转115°
            self.logger.info("步骤5：向右转115°")
            if not self.robot.turn_right(115):
                return False
            
            # 6. 向后退100cm（进入充电站）
            self.logger.info("步骤6：向后退100cm（进入充电站）")
            if not self.robot.move_backward(1.0):
                return False
            
            # 7. 趴下
            self.logger.info("步骤7：趴下（充电状态）")
            self.robot.lie()
            
            self.logger.info("A-2库返回充电站完成")
            return True
            
        except Exception as e:
            self.logger.error(f"A-2库返回充电站异常: {e}")
            return False
    
    def cleanup(self):
        """清理资源"""
        self.logger.info("清理资源...")
        if hasattr(self, 'robot'):
            self.robot.quit()


def main():
    """主函数"""
    print("启动任务11写死版：A-2卸货后回充电站")
    
    try:
        task = Task11A2Unload()
        success = task.run()
        
        if success:
            print("任务11 A-2卸货后回充电站执行成功！")
        else:
            print("任务11 A-2卸货后回充电站执行失败！")
            
    except KeyboardInterrupt:
        print("\n任务被用户中断")
    except Exception as e:
        print(f"任务执行异常: {e}")


if __name__ == "__main__":
    main()
