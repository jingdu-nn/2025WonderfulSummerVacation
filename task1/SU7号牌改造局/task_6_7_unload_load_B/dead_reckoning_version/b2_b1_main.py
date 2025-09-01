#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
任务6-7写死版主程序：B-2卸货 -> B-1装载

任务流程：
第一阶段 - B-2卸货：
1. 向右转115°，向前走100cm，再向左转115°，向前走100cm，趴下
2. 等待触摸传感器触发，站立并后退100cm

第二阶段 - B-1装载：
3. 向左转115°，向前走300cm，再向右转115°，向前走100cm，趴下
4. 等待触摸传感器触发，站立并后退100cm

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
    get_logger, TouchSensorMonitor, TouchSensorType = safe_import_utils()
    RobotController, ActionSequence = safe_import_motion_control()
    
    # 导入语音模块
    try:
        from utils.voice_announcer import say
    except ImportError:
        def say(text, wait_time=0.5):
            print(f"[语音播报] {text}")
            time.sleep(wait_time)
    
    print("✅ 所有模块导入成功")
except ImportError:
    # 降级到传统导入方式
    print("⚠️ 使用传统导入方式")
    try:
        from motion_control import RobotController, ActionSequence
        from utils import get_logger, TouchSensorMonitor, TouchSensorType
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


class Task67B2B1:
    """任务6-7 B-2卸货B-1装载控制器"""
    
    def __init__(self):
        self.logger = get_logger("Task67B2B1")
        self.robot = RobotController()
        self.action_seq = ActionSequence(self.robot)
        self.touch_sensor = TouchSensorMonitor()
        
        # 注册信号处理器
        signal.signal(signal.SIGINT, self._signal_handler)
        
        # 注册触摸传感器回调
        self.touch_sensor.register_callback(TouchSensorType.HEAD_BACK, self._on_touch_detected)
        self.touch_sensor.register_callback(TouchSensorType.HEAD_CHIN, self._on_touch_detected)
        
        self.touch_detected = False
        
        self.logger.info("任务6-7 B-2卸货B-1装载控制器初始化完成")
    
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
        """执行任务6-7 B-2卸货B-1装载"""
        self.logger.info("=== 开始执行任务6-7：B-2卸货B-1装载（写死版） ===")
        
        try:
            # 启动机器人控制器
            self.robot.run()
            time.sleep(2)  # 等待系统初始化
            
            # 第一阶段：B-2卸货
            if not self._execute_b2_unload():
                return False
            
            # 第二阶段：B-1装载
            if not self._execute_b1_load():
                return False
            
            self.logger.info("=== 任务6-7 B-2卸货B-1装载执行完成 ===")
            return True
            
        except Exception as e:
            self.logger.error(f"任务执行异常: {e}")
            return False
        finally:
            self.cleanup()
    
    def _execute_b2_unload(self) -> bool:
        """执行B-2卸货"""
        self.logger.info("=== 第一阶段：B-2仓库卸货 ===")
        
        try:
            # B-2卸货序列：向右转115°，向前走100cm，再向左转115°，向前走100cm，趴下
            
            # 语音播报到达B-2库
            try:
                say("已经抵达B2库")
            except Exception as e:
                print(f"语音播报失败: {e}")
            
            # 1. 向右转115°
            self.logger.info("步骤1：向右转115°")
            if not self.robot.turn_right(115):
                return False
            
            # 2. 向前走100cm
            self.logger.info("步骤2：向前走100cm")
            if not self.robot.move_forward(1.0):
                return False
            
            # 3. 向左转115°
            self.logger.info("步骤3：向左转115°")
            if not self.robot.turn_left(115):
                return False
            
            # 4. 向前走100cm
            self.logger.info("步骤4：向前走100cm")
            if not self.robot.move_forward(1.0):
                return False
            
            # 5. 趴下卸货
            self.logger.info("步骤5：趴下卸货")
            self.robot.lie()
            
            # 6. 等待触摸传感器并退出
            if not self._wait_and_exit_warehouse("B-2卸货"):
                return False
            
            self.logger.info("B-2仓库卸货完成")
            return True
            
        except Exception as e:
            self.logger.error(f"B-2卸货异常: {e}")
            return False
    
    def _execute_b1_load(self) -> bool:
        """执行B-1装载"""
        self.logger.info("=== 第二阶段：移动到B-1仓库装载 ===")
        
        try:
            # B-2到B-1装载序列：向左转115°，向前走300cm，再向右转115°，向前走100cm，趴下
            
            # 语音播报到达B-1库
            try:
                say("已经抵达B1库")
            except Exception as e:
                print(f"语音播报失败: {e}")
            
            # 1. 向左转115°
            self.logger.info("步骤1：向左转115°")
            if not self.robot.turn_left(115):
                return False
            
            # 2. 向前走300cm
            self.logger.info("步骤2：向前走300cm")
            if not self.robot.move_forward(3.0):
                return False
            
            # 3. 向右转115°
            self.logger.info("步骤3：向右转115°")
            if not self.robot.turn_right(115):
                return False
            
            # 4. 向前走100cm
            self.logger.info("步骤4：向前走100cm")
            if not self.robot.move_forward(1.0):
                return False
            
            # 5. 趴下装载
            self.logger.info("步骤5：趴下装载")
            self.robot.lie()
            
            # 6. 等待触摸传感器并退出
            if not self._wait_and_exit_warehouse("B-1装载"):
                return False
            
            self.logger.info("B-1仓库装载完成")
            return True
            
        except Exception as e:
            self.logger.error(f"B-1装载异常: {e}")
            return False
    
    def _wait_and_exit_warehouse(self, phase_name: str) -> bool:
        """等待触摸传感器并退出仓库"""
        self.logger.info(f"--- 等待触摸传感器触发并退出仓库 ({phase_name}) ---")
        
        try:
            # 1. 等待触摸传感器
            self.logger.info("等待触摸传感器触发...")
            self.touch_detected = False
            self.touch_sensor.start_monitoring()
            
            if not self.touch_sensor.wait_for_touch(TouchSensorType.HEAD_BACK, timeout=60):
                self.logger.error("等待触摸传感器超时")
                return False
            
            # 2. 站立
            self.logger.info("触摸检测到，站立")
            self.robot.stand(duration=3000, sleep=3.0)
            
            # 站立后等待1s并语音播报检测到库区
            time.sleep(1.0)
            try:
                say("检测到B2库")
            except Exception as e:
                print(f"语音播报失败: {e}")
            
            # 3. 后退100cm
            self.logger.info("后退100cm")
            if not self.robot.move_backward(1.0):
                return False
            
            self.logger.info(f"退出仓库完成 ({phase_name})")
            return True
            
        except Exception as e:
            self.logger.error(f"退出仓库异常 ({phase_name}): {e}")
            return False
    
    def cleanup(self):
        """清理资源"""
        self.logger.info("清理资源...")
        if hasattr(self, 'touch_sensor'):
            self.touch_sensor.stop_monitoring()
        if hasattr(self, 'robot'):
            self.robot.quit()


def main():
    """主函数"""
    print("启动任务6-7写死版：B-2卸货B-1装载")
    
    try:
        task = Task67B2B1()
        success = task.run()
        
        if success:
            print("任务6-7 B-2卸货B-1装载执行成功！")
        else:
            print("任务6-7 B-2卸货B-1装载执行失败！")
            
    except KeyboardInterrupt:
        print("\n任务被用户中断")
    except Exception as e:
        print(f"任务执行异常: {e}")


if __name__ == "__main__":
    main()
