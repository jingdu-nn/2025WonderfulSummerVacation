#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
任务6-7识别版主程序：B区卸货与装载（基于智能识别）

任务流程：
第一阶段 - 卸货：
1. 识别B区二维码确定库区（B-1或B-2）
2. 根据识别结果进入相应库区卸货
3. 趴下完成卸货
4. 等待触摸传感器触发，站立并退出

第二阶段 - 装载：
5. 移动到另一个B库进行装载
6. 趴下完成装载
7. 等待触摸传感器触发，站立并退出

作者：Xiaomi Cup 2025 Final Team
"""

import sys
import os
import signal
import time

# 添加共享模块到路径
shared_modules_path = os.path.join(os.path.dirname(__file__), '..', '..', 'shared_modules')
sys.path.insert(0, os.path.abspath(shared_modules_path))

# 添加visual模块到路径
visual_path = os.path.join(os.path.dirname(__file__), '..', '..', 'visual')
sys.path.insert(0, os.path.abspath(visual_path))

try:
    from motion_control import RobotController, ActionSequence
    from utils import get_logger
    from utils.enhanced_touch_sensor import wait_for_touch, TouchMode
    from utils.voice_announcer import say
    print("✅ 基础模块导入成功")
except ImportError as e:
    print(f"❌ 基础模块导入失败: {e}")
    print(f"当前Python路径: {sys.path}")
    print(f"共享模块路径: {os.path.abspath(shared_modules_path)}")
    raise

# 尝试导入识别模块 - 优先使用简化版本避免依赖问题
print("🔄 正在导入识别模块...")

# 定义手动识别类作为最终回退
class ManualRecognition:
    def __init__(self, debug=False):
        self.debug = debug
        print("使用手动识别模式")
    
    def recognize_from_camera(self, timeout=10.0):
        print("请手动输入识别到的仓库代码 (B-1 或 B-2):")
        try:
            code = input().strip()
            if code in ['B-1', 'B-2']:
                return {'success': True, 'code': code, 'method': 'manual', 'confidence': 1.0}
            else:
                return {'success': False, 'message': f'无效代码: {code}'}
        except:
            return {'success': False, 'message': '输入错误'}

# 尝试导入顺序：简化识别 → 完整识别 → 手动输入
UnifiedRecognition = None
USE_SIMPLE_RECOGNITION = True

try:
    # 优先尝试简化识别模块
    from vision.unified_recognition_simple import SimpleUnifiedRecognition as UnifiedRecognition
    print("✅ 简化识别模块导入成功")
except ImportError as e:
    print(f"⚠️ 简化识别模块导入失败: {e}")
    try:
        # 尝试完整识别模块
        from unified_recognition import UnifiedRecognition
        print("✅ 完整识别模块导入成功")
        USE_SIMPLE_RECOGNITION = False
    except ImportError as e2:
        print(f"⚠️ 完整识别模块也失败: {e2}")
        print("💡 将使用手动输入模式")
        UnifiedRecognition = ManualRecognition


class Task67Recognition:
    """任务6-7识别版控制器"""
    
    def __init__(self):
        self.logger = get_logger("Task67Recognition")
        self.robot = RobotController()
        self.action_seq = ActionSequence(self.robot)
        self.recognition_module = UnifiedRecognition(debug=True)
        
        self.unload_warehouse = None  # 卸货库区
        self.load_warehouse = None    # 装货库区
        
        # 注册信号处理器
        signal.signal(signal.SIGINT, self._signal_handler)
        
        self.logger.info("任务6-7识别版控制器初始化完成")
    
    def _signal_handler(self, signum, frame):
        """信号处理器"""
        self.logger.info("收到中断信号，正在安全退出...")
        self.cleanup()
        sys.exit(0)
    

    
    def run(self):
        """执行任务6-7识别版"""
        self.logger.info("=== 开始执行任务6-7：B区卸货与装载（识别版） ===")
        
        try:
            # 启动机器人控制器
            self.robot.run()
            time.sleep(2)  # 等待系统初始化
            
            # 第一阶段：卸货
            if not self._execute_unload_phase():
                return False
            
            # 第二阶段：装载
            if not self._execute_load_phase():
                return False
            
            self.logger.info("=== 任务6-7识别版执行完成 ===")
            return True
            
        except Exception as e:
            self.logger.error(f"任务执行异常: {e}")
            return False
        finally:
            self.cleanup()
    
    def _execute_unload_phase(self) -> bool:
        """执行卸货阶段"""
        self.logger.info("=== 第一阶段：B区卸货 ===")
        
        try:
            # 1. 识别B区仓库二维码
            if not self._recognize_unload_warehouse():
                return False
            
            # 2. 进入卸货库区
            if not self._enter_unload_warehouse():
                return False
            
            # 3. 等待触摸传感器触发并退出
            if not self._wait_and_exit_unload_warehouse():
                return False
            
            return True
            
        except Exception as e:
            self.logger.error(f"卸货阶段异常: {e}")
            return False
    
    def _execute_load_phase(self) -> bool:
        """执行装载阶段"""
        self.logger.info("=== 第二阶段：B区装载 ===")
        
        try:
            # 1. 确定装载库区（与卸货库区相反）
            self._determine_load_warehouse()
            
            # 2. 移动到装载库区
            if not self._move_to_load_warehouse():
                return False
            
            # 3. 等待触摸传感器触发并退出
            if not self._wait_and_exit_load_warehouse():
                return False
            
            return True
            
        except Exception as e:
            self.logger.error(f"装载阶段异常: {e}")
            return False
    
    def _recognize_unload_warehouse(self) -> bool:
        """识别卸货仓库"""
        self.logger.info("--- 智能识别B区卸货仓库标识 ---")
        
        try:
            # 尝试最多3次识别
            max_attempts = 3
            for attempt in range(1, max_attempts + 1):
                self.logger.info(f"第{attempt}次识别尝试")
                
                # 从摄像头获取图像并识别
                result = self.recognition_module.recognize_from_camera(timeout=10.0)
                
                if result['success']:
                    code = result['code']
                    method = result['method']
                    confidence = result['confidence']
                    
                    # 验证是否为B区仓库代码
                    if code in ['B-1', 'B-2']:
                        self.unload_warehouse = code
                        # 确定装载库区（与卸载库区相反）
                        if code == "B-1":
                            self.load_warehouse = "B-2"
                        else:
                            self.load_warehouse = "B-1"
                        
                        self.logger.info(f"识别成功，卸货仓库：{code} (方法: {method}, 置信度: {confidence:.2f})")
                        self.logger.info(f"装载仓库：{self.load_warehouse}")
                        
                        # 语音播报识别结果
                        try:
                            say(f"识别到{code}")
                        except Exception as e:
                            print(f"语音播报失败: {e}")
                        
                        # 保存识别结果到状态文件
                        try:
                            sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
                            from warehouse_state import save_task67_result
                            save_task67_result(self.unload_warehouse, self.load_warehouse, method)
                            self.logger.info(f"B区识别结果已保存到warehouse.json")
                        except Exception as e:
                            self.logger.warning(f"保存B区识别结果失败: {e}")
                        
                        return True
                    else:
                        self.logger.warning(f"识别到非B区代码: {code}，继续尝试...")
                        time.sleep(2)  # 等待2秒后重试
                        continue
                else:
                    self.logger.warning(f"第{attempt}次识别失败: {result.get('message', '未知错误')}")
                    if attempt < max_attempts:
                        self.logger.info("等待3秒后重试...")
                        time.sleep(3)
                    continue
            
            self.logger.error("所有识别尝试均失败")
            return False
                
        except Exception as e:
            self.logger.error(f"识别卸货仓库异常: {e}")
            return False
    
    def _enter_unload_warehouse(self) -> bool:
        """进入卸货仓库"""
        self.logger.info(f"--- 进入{self.unload_warehouse}仓库卸货 ---")
        
        try:
            if self.unload_warehouse == "B-1":
                return self._enter_warehouse_b1_unload()
            elif self.unload_warehouse == "B-2":
                return self._enter_warehouse_b2_unload()
            else:
                self.logger.error(f"未知的卸货仓库代码: {self.unload_warehouse}")
                return False
                
        except Exception as e:
            self.logger.error(f"进入卸货仓库异常: {e}")
            return False
    
    def _enter_warehouse_b1_unload(self) -> bool:
        """进入B-1仓库卸货"""
        self.logger.info("执行B-1仓库卸货入库序列")
        
        try:
            # B-1卸货序列：向左转115°，向前走100cm，再向右转115°，向前走100cm，趴下
            
            # 语音播报开始B-1卸货
            say("B-1")
            
            # 1. 向左转115°
            self.logger.info("步骤1：向左转115°")
            if not self.robot.turn_left(115):
                return False
            
            # 2. 向前走100cm
            self.logger.info("步骤2：向前走100cm")
            if not self.robot.move_forward(1.0):
                return False
            
            # 3. 向右转115°
            self.logger.info("步骤3：向右转115°")
            if not self.robot.turn_right(115):
                return False
            
            # 4. 向前走100cm
            self.logger.info("步骤4：向前走100cm")
            if not self.robot.move_forward(1.0):
                return False
            
            # 5. 趴下卸货
            self.logger.info("步骤5：趴下卸货")
            self.robot.lie()
            
            self.logger.info("B-1仓库卸货完成")
            return True
            
        except Exception as e:
            self.logger.error(f"B-1仓库卸货异常: {e}")
            return False
    
    def _enter_warehouse_b2_unload(self) -> bool:
        """进入B-2仓库卸货"""
        self.logger.info("执行B-2仓库卸货入库序列")
        
        try:
            # B-2卸货序列：向右转115°，向前走100cm，再向左转115°，向前走100cm，趴下
            
            # 语音播报开始B-2卸货
            say("B-2")
            
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
            
            self.logger.info("B-2仓库卸货完成")
            return True
            
        except Exception as e:
            self.logger.error(f"B-2仓库卸货异常: {e}")
            return False
    
    def _wait_and_exit_unload_warehouse(self) -> bool:
        """等待触摸传感器并退出卸货仓库"""
        self.logger.info("--- 等待触摸传感器触发并退出卸货仓库 ---")
        
        try:
            # 1. 等待触摸传感器
            self.logger.info("等待三击后脑勺触发退出...")
            
            if not wait_for_touch(TouchMode.SIX_TOUCHES, timeout=60):
                self.logger.error("等待触摸传感器超时")
                return False
            
            # 2. 站立
            self.logger.info("触摸检测到，站立")
            say("开始退出")
            self.robot.stand(duration=3000, sleep=3.0)
            
            # 3. 后退100cm
            self.logger.info("后退100cm")
            if not self.robot.move_backward(1.0):
                return False
            
            self.logger.info("退出卸货仓库完成")
            return True
            
        except Exception as e:
            self.logger.error(f"退出卸货仓库异常: {e}")
            return False
    
    def _determine_load_warehouse(self):
        """确定装载仓库（与卸货仓库相反）"""
        if self.unload_warehouse == "B-1":
            self.load_warehouse = "B-2"
        elif self.unload_warehouse == "B-2":
            self.load_warehouse = "B-1"
        else:
            self.load_warehouse = "B-1"  # 默认值
        
        self.logger.info(f"确定装载仓库：{self.load_warehouse}")
    
    def _move_to_load_warehouse(self) -> bool:
        """移动到装载仓库"""
        self.logger.info(f"--- 移动到{self.load_warehouse}仓库装载 ---")
        
        try:
            if self.unload_warehouse == "B-1" and self.load_warehouse == "B-2":
                return self._move_b1_to_b2()
            elif self.unload_warehouse == "B-2" and self.load_warehouse == "B-1":
                return self._move_b2_to_b1()
            else:
                self.logger.error(f"无效的移动路径: {self.unload_warehouse} -> {self.load_warehouse}")
                return False
                
        except Exception as e:
            self.logger.error(f"移动到装载仓库异常: {e}")
            return False
    
    def _move_b1_to_b2(self) -> bool:
        """从B-1移动到B-2"""
        self.logger.info("执行B-1到B-2移动序列")
        
        try:
            # B-1到B-2：向右转115°，向前走300cm，再向左转115°，向前走100cm，趴下
            
            # 1. 向右转115°
            self.logger.info("步骤1：向右转115°")
            if not self.robot.turn_right(115):
                return False
            
            # 2. 向前走300cm
            self.logger.info("步骤2：向前走300cm")
            if not self.robot.move_forward(3.0):
                return False
            
            # 3. 向左转115°
            self.logger.info("步骤3：向左转115°")
            if not self.robot.turn_left(115):
                return False
            
            # 4. 向前走100cm
            self.logger.info("步骤4：向前走100cm")
            if not self.robot.move_forward(1.0):
                return False
            
            # 5. 趴下装载
            self.logger.info("步骤5：趴下装载")
            self.robot.lie()
            
            self.logger.info("B-1到B-2移动并装载完成")
            return True
            
        except Exception as e:
            self.logger.error(f"B-1到B-2移动异常: {e}")
            return False
    
    def _move_b2_to_b1(self) -> bool:
        """从B-2移动到B-1"""
        self.logger.info("执行B-2到B-1移动序列")
        
        try:
            # B-2到B-1：向左转115°，向前走300cm，再向右转115°，向前走100cm，趴下
            
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
            
            self.logger.info("B-2到B-1移动并装载完成")
            return True
            
        except Exception as e:
            self.logger.error(f"B-2到B-1移动异常: {e}")
            return False
    
    def _wait_and_exit_load_warehouse(self) -> bool:
        """等待触摸传感器并退出装载仓库"""
        self.logger.info("--- 等待触摸传感器触发并退出装载仓库 ---")
        
        try:
            # 1. 等待触摸传感器
            self.logger.info("等待三击后脑勺触发退出...")
            
            if not wait_for_touch(TouchMode.SIX_TOUCHES, timeout=60):
                self.logger.error("等待触摸传感器超时")
                return False
            
            # 2. 站立
            self.logger.info("触摸检测到，站立")
            say("开始退出")
            self.robot.stand(duration=3000, sleep=3.0)
            
            # 3. 后退100cm
            self.logger.info("后退100cm")
            if not self.robot.move_backward(1.0):
                return False
            
            self.logger.info("退出装载仓库完成")
            return True
            
        except Exception as e:
            self.logger.error(f"退出装载仓库异常: {e}")
            return False
    
    def cleanup(self):
        """清理资源"""
        self.logger.info("清理资源...")
        if hasattr(self, 'recognition_module'):
            # 统一识别模块不需要特殊清理
            pass
        if hasattr(self, 'robot'):
            self.robot.quit()


def main():
    """主函数"""
    print("启动任务6-7识别版：B区卸货与装载")
    
    try:
        task = Task67Recognition()
        success = task.run()
        
        if success:
            print("任务6-7识别版执行成功！")
        else:
            print("任务6-7识别版执行失败！")
            
    except KeyboardInterrupt:
        print("\n任务被用户中断")
    except Exception as e:
        print(f"任务执行异常: {e}")


if __name__ == "__main__":
    main()
