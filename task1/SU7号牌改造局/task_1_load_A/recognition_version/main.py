#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
任务1识别版主程序：A区装载（基于智能识别）

任务流程：
1. 初始状态是趴下
2. 站立起来，向前走100cm
3. 向右转115°
4. 向前走180cm
5. 智能识别库区标识（A-1或A-2），支持文字和二维码
6. 根据识别结果进入相应库区
7. 趴下完成装载

作者：Xiaomi Cup 2025 Final Team
"""

import sys
import os
import signal
import time

# 添加共享模块到路径
shared_modules_path = os.path.join(os.path.dirname(__file__), '..', '..', 'shared_modules')
sys.path.append(os.path.abspath(shared_modules_path))

# 添加visual模块到路径
visual_path = os.path.join(os.path.dirname(__file__), '..', '..', 'visual')
sys.path.append(os.path.abspath(visual_path))

try:
    from motion_control import RobotController, ActionSequence
    from utils import get_logger
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
        print("请手动输入识别到的仓库代码 (A-1 或 A-2):")
        try:
            code = input().strip()
            if code in ['A-1', 'A-2']:
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
        from vision.unified_recognition import UnifiedRecognition
        print("✅ 完整识别模块导入成功")
        USE_SIMPLE_RECOGNITION = False
    except ImportError as e2:
        print(f"⚠️ 完整识别模块也失败: {e2}")
        print("💡 将使用手动输入模式")
        UnifiedRecognition = ManualRecognition


class Task1Recognition:
    """任务1识别版控制器"""
    
    def __init__(self):
        self.logger = get_logger("Task1Recognition")
        self.robot = RobotController()
        self.action_seq = ActionSequence(self.robot)
        self.recognition_module = UnifiedRecognition(debug=True)
        self.warehouse_code = None
        
        # 注册信号处理器
        signal.signal(signal.SIGINT, self._signal_handler)
        
        self.logger.info("任务1识别版控制器初始化完成")
    
    def _signal_handler(self, signum, frame):
        """信号处理器"""
        self.logger.info("收到中断信号，正在安全退出...")
        self.cleanup()
        sys.exit(0)
    
    def run(self):
        """执行任务1识别版"""
        self.logger.info("=== 开始执行任务1：A区装载（识别版） ===")
        
        try:
            # 启动机器人控制器
            self.robot.run()
            time.sleep(2)  # 等待系统初始化
            
            # 执行任务步骤
            if not self._execute_approach_phase():
                return False
            
            if not self._execute_recognition_phase():
                return False
            
            if not self._execute_warehouse_entry():
                return False
            
            self.logger.info("=== 任务1识别版执行完成 ===")
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
    
    def _execute_recognition_phase(self) -> bool:
        """执行识别阶段"""
        self.logger.info("--- 第二阶段：智能识别A区仓库标识 ---")
        
        try:
            # 使用统一识别模块识别A区仓库标识
            self.logger.info("正在识别A区仓库标识（QR码+文字识别）...")
            
            # 尝试最多3次识别
            max_attempts = 3
            for attempt in range(1, max_attempts + 1):
                self.logger.info(f"第{attempt}次识别尝试")
                
                # 从摄像头获取图像并识别
                result = self.recognition_module.recognize_from_camera(timeout=15.0)
                
                if result['success']:
                    code = result['code']
                    method = result['method']
                    confidence = result['confidence']
                    
                    # 验证是否为A区仓库代码
                    if code in ['A-1', 'A-2']:
                        self.warehouse_code = code
                        self.logger.info(f"识别成功：{code} (方法: {method}, 置信度: {confidence:.2f})")
                        
                        # 语音播报识别结果
                        try:
                            say(code)
                        except Exception as e:
                            self.logger.warning(f"语音播报失败: {e}")
                                
                        # 保存识别结果到状态文件
                        try:
                            sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
                            from warehouse_state import save_task1_result
                            save_task1_result(code, method)
                            self.logger.info(f"识别结果已保存到warehouse.json")
                        except Exception as e:
                            self.logger.warning(f"保存识别结果失败: {e}")
                                
                        return True
                    else:
                        self.logger.warning(f"识别到非A区代码: {code}，继续尝试...")
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
            self.logger.error(f"识别阶段异常: {e}")
            return False
    
    def _execute_warehouse_entry(self) -> bool:
        """执行入库阶段"""
        self.logger.info("--- 第三阶段：进入仓库 ---")
        
        try:
            if self.warehouse_code == "A-1":
                self.logger.info("进入A-1仓库")
                return self._enter_warehouse_a1()
            elif self.warehouse_code == "A-2":
                self.logger.info("进入A-2仓库")
                return self._enter_warehouse_a2()
            else:
                self.logger.error(f"未知的仓库代码: {self.warehouse_code}")
                return False
                
        except Exception as e:
            self.logger.error(f"入库阶段异常: {e}")
            return False
    
    def _enter_warehouse_a1(self) -> bool:
        """进入A-1仓库"""
        self.logger.info("执行A-1仓库入库序列")
        
        try:
            # A-1库序列：向右转115°，向前走85cm，再向右转115°，向前走100cm，趴下
            
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
            
            # 5. 趴下
            self.logger.info("步骤5：趴下")
            self.robot.lie()
            
            self.logger.info("A-1仓库入库完成")
            return True
            
        except Exception as e:
            self.logger.error(f"A-1仓库入库异常: {e}")
            return False
    
    def _enter_warehouse_a2(self) -> bool:
        """进入A-2仓库"""
        self.logger.info("执行A-2仓库入库序列")
        
        try:
            # A-2库序列：向左转115°，向前走85cm，再向左转115°，向前走100cm，趴下
            
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
            
            # 5. 趴下
            self.logger.info("步骤5：趴下")
            self.robot.lie()
            
            self.logger.info("A-2仓库入库完成")
            return True
            
        except Exception as e:
            self.logger.error(f"A-2仓库入库异常: {e}")
            return False
    
    def cleanup(self):
        """清理资源"""
        self.logger.info("清理资源...")
        if hasattr(self, 'recognition_module') and hasattr(self.recognition_module, 'cleanup'):
            try:
                self.recognition_module.cleanup()
            except Exception as e:
                self.logger.warning(f"识别模块清理失败: {e}")
        if hasattr(self, 'robot'):
            self.robot.quit()


def main():
    """主函数"""
    print("启动任务1识别版：A区装载")
    
    try:
        task = Task1Recognition()
        success = task.run()
        
        if success:
            print("任务1识别版执行成功！")
        else:
            print("任务1识别版执行失败！")
            
    except KeyboardInterrupt:
        print("\n任务被用户中断")
    except Exception as e:
        print(f"任务执行异常: {e}")


if __name__ == "__main__":
    main()
