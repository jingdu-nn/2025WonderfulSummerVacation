#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
任务11识别版主程序：回到充电站（基于前序任务识别结果）

任务流程：
1. 等待触摸传感器触发（后脑勺或下巴）
2. 站立
3. 向后退100cm
4. 根据前序任务的卸载A库区选择路径：
   - 卸载A-1：向右转115°，向前走85cm，再向左转115°
   - 卸载A-2：向左转115°，向前走85cm，再向右转115°
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
    get_logger, TouchSensorMonitor, TouchSensorType = safe_import_utils()
    RobotController, ActionSequence = safe_import_motion_control()
    
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


class Task11Recognition:
    """任务11识别版控制器"""
    
    def __init__(self, unload_warehouse: str = None):
        """
        初始化任务11识别版控制器
        
        Args:
            unload_warehouse: 前序任务的卸载A库区（'A-1'或'A-2'）
        """
        self.logger = get_logger("Task11Recognition")
        self.robot = RobotController()
        self.action_seq = ActionSequence(self.robot)
        self.touch_sensor = TouchSensorMonitor()
        self.unload_warehouse = unload_warehouse
        
        # 注册信号处理器
        signal.signal(signal.SIGINT, self._signal_handler)
        
        # 注册触摸传感器回调
        self.touch_sensor.register_callback(TouchSensorType.HEAD_BACK, self._on_touch_detected)
        self.touch_sensor.register_callback(TouchSensorType.HEAD_CHIN, self._on_touch_detected)
        
        self.touch_detected = False
        
        self.logger.info(f"任务11识别版控制器初始化完成，卸载库区: {unload_warehouse}")
    
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
        """执行任务11识别版"""
        self.logger.info("=== 开始执行任务11：回到充电站（识别版） ===")
        
        try:
            # 启动机器人控制器
            self.robot.run()
            time.sleep(2)  # 等待系统初始化
            
            # 检查卸载库区参数
            if not self.unload_warehouse:
                self.logger.warning("未指定卸载库区，尝试从warehouse.json获取")
                try:
                    sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
                    from warehouse_state import get_task10_unload_warehouse
                    self.unload_warehouse = get_task10_unload_warehouse()
                    if self.unload_warehouse:
                        self.logger.info(f"从warehouse.json获取到卸载库区: {self.unload_warehouse}")
                    else:
                        self.logger.warning("warehouse.json中没有找到卸载库区信息")
                except Exception as e:
                    self.logger.warning(f"读取warehouse.json失败: {e}")
                
            if not self.unload_warehouse:
                self.logger.warning("尝试从环境变量获取")
                self.unload_warehouse = os.getenv('UNLOAD_WAREHOUSE')
                
            if not self.unload_warehouse:
                self.logger.error("无法确定卸载库区，请指定A-1或A-2")
                return False
            
            # 执行任务步骤
            if not self._wait_for_touch_trigger():
                return False
            
            if not self._execute_exit_warehouse():
                return False
            
            if not self._execute_path_to_charge_area():
                return False
            
            if not self._execute_enter_charge_area():
                return False
            
            self.logger.info("=== 任务11识别版执行完成 ===")
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
            self.logger.info("等待触摸传感器触发（后脑勺或下巴）...")
            self.logger.info("提示：调用 touch_sensor.trigger_sensor() 来模拟触摸")
            
            # 启动传感器监控
            self.touch_sensor.start_monitoring()
            
            # 等待触摸
            if not self.touch_sensor.wait_for_touch(TouchSensorType.HEAD_BACK, timeout=60):
                self.logger.error("等待触摸传感器超时")
                return False
            
            self.logger.info("触摸传感器触发成功")
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
    
    def _execute_path_to_charge_area(self) -> bool:
        """执行到充电区的路径"""
        self.logger.info("--- 第三阶段：前往充电区 ---")
        
        try:
            if self.unload_warehouse == "A-1":
                return self._execute_a1_to_charge_path()
            elif self.unload_warehouse == "A-2":
                return self._execute_a2_to_charge_path()
            else:
                self.logger.error(f"未知的卸载库区: {self.unload_warehouse}")
                return False
                
        except Exception as e:
            self.logger.error(f"前往充电区路径异常: {e}")
            return False
    
    def _execute_a1_to_charge_path(self) -> bool:
        """从A-1库前往充电区"""
        self.logger.info("执行A-1到充电区路径")
        
        try:
            # A-1路径：向右转115°，向前走85cm，再向左转115°
            
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
            
            self.logger.info("A-1到充电区路径完成")
            return True
            
        except Exception as e:
            self.logger.error(f"A-1到充电区路径异常: {e}")
            return False
    
    def _execute_a2_to_charge_path(self) -> bool:
        """从A-2库前往充电区"""
        self.logger.info("执行A-2到充电区路径")
        
        try:
            # A-2路径：向左转115°，向前走85cm，再向右转115°
            
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
            
            self.logger.info("A-2到充电区路径完成")
            return True
            
        except Exception as e:
            self.logger.error(f"A-2到充电区路径异常: {e}")
            return False
    
    def _execute_enter_charge_area(self) -> bool:
        """执行进入充电区"""
        self.logger.info("--- 第四阶段：进入充电区 ---")
        
        try:
            # 1. 向前走180cm
            self.logger.info("步骤1：向前走180cm")
            if not self.robot.move_forward(1.8):
                return False
            
            # 2. 向右转115°
            self.logger.info("步骤2：向右转115°")
            if not self.robot.turn_right(115):
                return False
            
            # 3. 向后退100cm（进入充电站）
            self.logger.info("步骤3：向后退100cm（进入充电站）")
            if not self.robot.move_backward(1.0):
                return False
            
            # 4. 趴下
            self.logger.info("步骤4：趴下（充电状态）")
            self.robot.lie()
            
            self.logger.info("进入充电区完成")
            return True
            
        except Exception as e:
            self.logger.error(f"进入充电区异常: {e}")
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
    print("启动任务11识别版：回到充电站")
    
    # 从命令行参数或环境变量获取卸载库区
    unload_warehouse = None
    if len(sys.argv) > 1:
        unload_warehouse = sys.argv[1]
    else:
        unload_warehouse = os.getenv('UNLOAD_WAREHOUSE')
    
    if not unload_warehouse:
        print("请指定卸载库区：")
        print("方式1：python main.py A-1")
        print("方式2：export UNLOAD_WAREHOUSE=A-1")
        return
    
    try:
        task = Task11Recognition(unload_warehouse=unload_warehouse)
        success = task.run()
        
        if success:
            print("任务11识别版执行成功！")
        else:
            print("任务11识别版执行失败！")
            
    except KeyboardInterrupt:
        print("\n任务被用户中断")
    except Exception as e:
        print(f"任务执行异常: {e}")


if __name__ == "__main__":
    main()
