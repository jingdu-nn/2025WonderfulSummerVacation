#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
任务10识别版主程序：A区卸货（基于前序任务识别结果）

任务流程：
1. 从逆向S弯道走出，向前走100cm
2. 向左转115°
3. 向前走180cm
4. 根据前序任务的装载A库区确定卸载A库区（相反库区）：
   - 装载A-1 -> 卸载A-2：向左转115°，向前走85cm，再向左转115°，向前走100cm，趴下
   - 装载A-2 -> 卸载A-1：向右转115°，向前走85cm，再向右转115°，向前走100cm，趴下

作者：Xiaomi Cup 2025 Final Team
"""

import sys
import os
import signal
import time
import json

# 添加共享模块到路径
shared_modules_path = os.path.join(os.path.dirname(__file__), '..', '..', 'shared_modules')
sys.path.append(os.path.abspath(shared_modules_path))

from motion_control import RobotController, ActionSequence
from utils import get_logger


class Task10Recognition:
    """任务10识别版控制器"""
    
    def __init__(self, loaded_warehouse: str = None):
        """
        初始化任务10识别版控制器
        
        Args:
            loaded_warehouse: 前序任务的装载A库区（'A-1'或'A-2'）
        """
        self.logger = get_logger("Task10Recognition")
        self.robot = RobotController()
        self.action_seq = ActionSequence(self.robot)
        self.loaded_warehouse = loaded_warehouse
        self.unload_warehouse = None  # 卸载库区（与装载库区相反）
        
        # 注册信号处理器
        signal.signal(signal.SIGINT, self._signal_handler)
        
        self.logger.info(f"任务10识别版控制器初始化完成，装载库区: {loaded_warehouse}")
    
    def _signal_handler(self, signum, frame):
        """信号处理器"""
        self.logger.info("收到中断信号，正在安全退出...")
        self.cleanup()
        sys.exit(0)
    
    def _read_warehouse_result(self):
        """读取前序任务的识别结果"""
        try:
            warehouse_file = os.path.join(os.path.dirname(__file__), '..', '..', 'warehouse.json')
            if os.path.exists(warehouse_file):
                with open(warehouse_file, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    task1_result = data.get('task1_result', 'A-1')  # 默认A-1
                    self.logger.info(f"读取到前序任务1结果：{task1_result}")
                    return task1_result
            else:
                self.logger.warning("warehouse.json文件不存在，使用默认值A-1")
                return 'A-1'
        except Exception as e:
            self.logger.error(f"读取warehouse.json失败：{e}，使用默认值A-1")
            return 'A-1'
    
    def _determine_unload_warehouse(self):
        """根据前序装载结果确定卸载库区（相反库区）"""
        if self.previous_warehouse == 'A-1':
            return 'A-2'  # 装载A-1 -> 卸载A-2
        else:
            return 'A-1'  # 装载A-2 -> 卸载A-1
    
    def run(self):
        """执行任务10识别版"""
        self.logger.info("=== 开始执行任务10：A区卸货（识别版） ===")
        
        try:
            # 启动机器人控制器
            self.robot.run()
            time.sleep(2)  # 等待系统初始化
            
            # 检查装载库区参数
            if not self.loaded_warehouse:
                self.logger.warning("未指定装载库区，尝试从warehouse.json获取")
                try:
                    sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
                    from warehouse_state import get_task1_warehouses
                    warehouses = get_task1_warehouses()
                    self.loaded_warehouse = warehouses['load_warehouse']
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
            
            # 确定卸载库区
            self._determine_unload_warehouse()
            
            # 执行任务步骤
            if not self._execute_approach_phase():
                return False
            
            if not self._execute_unload_phase():
                return False
            
            self.logger.info("=== 任务10识别版执行完成 ===")
            return True
            
        except Exception as e:
            self.logger.error(f"任务执行异常: {e}")
            return False
        finally:
            self.cleanup()
    
    def _determine_unload_warehouse(self):
        """确定卸载仓库（与装载仓库相反）"""
        if self.loaded_warehouse == "A-1":
            self.unload_warehouse = "A-2"
        elif self.loaded_warehouse == "A-2":
            self.unload_warehouse = "A-1"
        else:
            self.unload_warehouse = "A-1"  # 默认值
        
        self.logger.info(f"确定卸载仓库：{self.unload_warehouse}（装载仓库：{self.loaded_warehouse}）")
    
    def _execute_approach_phase(self) -> bool:
        """执行接近阶段（从S弯道到A区）"""
        self.logger.info("--- 第一阶段：从S弯道接近A区 ---")
        
        try:
            # 1. 从逆向S弯道走出，向前走100cm
            self.logger.info("步骤1：向前走100cm（从S弯道走出）")
            if not self.robot.move_forward(1.0):
                self.logger.error("向前走100cm失败")
                return False
            
            # 2. 向左转115°
            self.logger.info("步骤2：向左转115°")
            if not self.robot.turn_left(115):
                self.logger.error("向左转115°失败")
                return False
            
            # 3. 向前走180cm
            self.logger.info("步骤3：向前走180cm")
            if not self.robot.move_forward(1.8):
                self.logger.error("向前走180cm失败")
                return False
            
            self.logger.info("接近阶段完成")
            return True
            
        except Exception as e:
            self.logger.error(f"接近阶段异常: {e}")
            return False
    
    def _execute_unload_phase(self) -> bool:
        """执行卸载阶段"""
        self.logger.info("--- 第二阶段：进入A区卸载仓库 ---")
        
        try:
            if self.unload_warehouse == "A-1":
                return self._unload_warehouse_a1()
            elif self.unload_warehouse == "A-2":
                return self._unload_warehouse_a2()
            else:
                self.logger.error(f"未知的卸载仓库代码: {self.unload_warehouse}")
                return False
                
        except Exception as e:
            self.logger.error(f"卸载阶段异常: {e}")
            return False
    
    def _unload_warehouse_a1(self) -> bool:
        """卸载A-1仓库"""
        self.logger.info("执行A-1仓库卸载序列")
        
        try:
            # A-1卸载序列：向右转115°，向前走85cm，再向右转115°，向前走100cm，趴下
            
            # 1. 向右转115°
            self.logger.info("步骤1：向右转115°")
            if not self.robot.turn_right(115):
                return False
            
            # 2. 向前走85cm
            self.logger.info("步骤2：向前走85cm")
            if not self.robot.move_forward(0.85):
                return False
            
            # 3. 再向右转115°
            self.logger.info("步骤3：再向右转115°")
            if not self.robot.turn_right(115):
                return False
            
            # 4. 向前走100cm
            self.logger.info("步骤4：向前走100cm")
            if not self.robot.move_forward(1.0):
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
            
            self.logger.info("A-1仓库卸载完成")
            return True
            
        except Exception as e:
            self.logger.error(f"A-1仓库卸载异常: {e}")
            return False
    
    def _unload_warehouse_a2(self) -> bool:
        """卸载A-2仓库"""
        self.logger.info("执行A-2仓库卸载序列")
        
        try:
            # A-2卸载序列：向左转115°，向前走85cm，再向左转115°，向前走100cm，趴下
            
            # 1. 向左转115°
            self.logger.info("步骤1：向左转115°")
            if not self.robot.turn_left(115):
                return False
            
            # 2. 向前走85cm
            self.logger.info("步骤2：向前走85cm")
            if not self.robot.move_forward(0.85):
                return False
            
            # 3. 再向左转115°
            self.logger.info("步骤3：再向左转115°")
            if not self.robot.turn_left(115):
                return False
            
            # 4. 向前走100cm
            self.logger.info("步骤4：向前走100cm")
            if not self.robot.move_forward(1.0):
                return False
            
            # 5. 趴下卸载
            self.logger.info("步骤5：趴下卸载")
            # 语音播报抵达库区
            try:
                from utils.voice_announcer import say
                say("抵达A2库卸货")
            except Exception as e:
                print(f"语音播报失败: {e}")
            
            self.robot.lie()
            
            self.logger.info("A-2仓库卸载完成")
            return True
            
        except Exception as e:
            self.logger.error(f"A-2仓库卸载异常: {e}")
            return False
    
    def cleanup(self):
        """清理资源"""
        self.logger.info("清理资源...")
        if hasattr(self, 'robot'):
            self.robot.quit()


def main():
    """主函数"""
    print("启动任务10识别版：A区卸货")
    
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
        task = Task10Recognition(loaded_warehouse=loaded_warehouse)
        success = task.run()
        
        if success:
            print("任务10识别版执行成功！")
        else:
            print("任务10识别版执行失败！")
            
    except KeyboardInterrupt:
        print("\n任务被用户中断")
    except Exception as e:
        print(f"任务执行异常: {e}")


if __name__ == "__main__":
    main()
