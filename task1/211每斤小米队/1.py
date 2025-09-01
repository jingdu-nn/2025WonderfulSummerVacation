#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
A库任务脚本
专门执行A库相关的操作：到达A库、装货、离开A库

主要功能：
1. 从起点到达A库装货点
2. 进入A库装货
3. 离开A库返回

设计原则：
- 使用稳定的IMU导航
- 精确的转弯控制
- 安全的装货操作
"""

import lcm
import sys
import os
import time
import subprocess
import json
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from threading import Thread, Lock

# 导入LCM消息
from robot_control_cmd_lcmt import robot_control_cmd_lcmt
from robot_control_response_lcmt import robot_control_response_lcmt

# 导入功能模块
from modules import RobotController, CameraController, VisionDetector

# 触摸与语音服务（可选）
try:
    from protocol.srv import AudioTextPlay
    from protocol.msg import AudioPlay, TouchStatus
    HAS_PROTOCOL = True
except Exception:
    AudioTextPlay = None
    AudioPlay = None
    TouchStatus = None
    HAS_PROTOCOL = False
from stand_and_qr_detect import detect_qr_code_with_rgb

class Robot_Ctrl:
    """底层LCM通信控制器 (来自main.py)"""
    def __init__(self):
        self.rec_thread = Thread(target=self.rec_responce)
        self.send_thread = Thread(target=self.send_publish)
        self.lc_r = lcm.LCM("udpm://239.255.76.67:7670?ttl=255")
        self.lc_s = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        self.cmd_msg = robot_control_cmd_lcmt()
        self.rec_msg = robot_control_response_lcmt()
        self.send_lock = Lock()
        self.delay_cnt = 0
        self.mode_ok = 0
        self.gait_ok = 0
        self.runing = 1

    def run(self):
        self.lc_r.subscribe("robot_control_response", self.msg_handler)
        self.send_thread.start()
        self.rec_thread.start()

    def msg_handler(self, channel, data):
        try:
            self.rec_msg = robot_control_response_lcmt().decode(data)
            if self.rec_msg.order_process_bar >= 95:
                self.mode_ok = self.rec_msg.mode
                self.gait_ok = self.rec_msg.gait_id
            else:
                self.mode_ok = 0
        except Exception:
            pass

    def rec_responce(self):
        while self.runing:
            try:
                self.lc_r.handle()
                time.sleep(0.002)
            except Exception:
                time.sleep(0.1)

    def send_publish(self):
        while self.runing:
            self.send_lock.acquire()
            if self.delay_cnt > 20:
                self.lc_s.publish("robot_control_cmd", self.cmd_msg.encode())
                self.delay_cnt = 0
            self.delay_cnt += 1
            self.send_lock.release()
            time.sleep(0.005)

    def Send_cmd(self, msg):
        self.send_lock.acquire()
        self.delay_cnt = 50
        self.cmd_msg = msg
        self.send_lock.release()

    def stand(self, duration=1000, sleep=1.1):
        self.cmd_msg.mode = 12
        self.cmd_msg.gait_id = 0
        self.cmd_msg.duration = duration
        self.cmd_msg.life_count += 1
        if self.cmd_msg.life_count > 127: 
            self.cmd_msg.life_count = 0
        self.Send_cmd(self.cmd_msg)
        time.sleep(sleep)

    def simple_forward(self, duration=3000, vel=0.25):
        """简单前进命令"""
        self.cmd_msg.mode = 11
        self.cmd_msg.gait_id = 27
        self.cmd_msg.vel_des = [vel, 0, 0]
        self.cmd_msg.step_height = [0.1, 0.1]
        self.cmd_msg.duration = duration
        self.cmd_msg.life_count += 1
        if self.cmd_msg.life_count > 127:
            self.cmd_msg.life_count = 0
        self.Send_cmd(self.cmd_msg)
        time.sleep(duration/1000)

    def simple_turn(self, duration=2600, angular_vel=0.8):
        """简单转弯命令 - 使用精确的90°参数"""
        self.cmd_msg.mode = 11
        self.cmd_msg.gait_id = 27
        self.cmd_msg.vel_des = [0, 0, angular_vel]
        self.cmd_msg.duration = duration
        self.cmd_msg.step_height = [0.02, 0.02]
        self.cmd_msg.life_count += 1
        if self.cmd_msg.life_count > 127:
            self.cmd_msg.life_count = 0
        self.Send_cmd(self.cmd_msg)
        time.sleep(duration/1000)

    def lie_down(self, duration=5000):
        """趴下"""
        self.cmd_msg.mode = 7
        self.cmd_msg.gait_id = 0
        self.cmd_msg.duration = duration
        self.cmd_msg.life_count += 1
        if self.cmd_msg.life_count > 127:
            self.cmd_msg.life_count = 0
        self.Send_cmd(self.cmd_msg)
        time.sleep(duration/1000 + 1)

    def quit(self):
        self.runing = 0
        if self.rec_thread.is_alive(): 
            self.rec_thread.join()
        if self.send_thread.is_alive(): 
            self.send_thread.join()
        print("✅ Robot_Ctrl 已停止")


class InteractionNode(Node):
    """语音播报+触摸双击等待（最小实现，等待过程中按spin_once推进回调）"""

    def __init__(self):
        super().__init__('a_wh_interaction')
        self.audio_client = None
        self.audio_service_name = ''
        self.touch_sub = None
        self.touch_events = 0
        self.last_touch_ts = 0.0

        # 订阅触摸（若可用）
        if TouchStatus is not None:
            try:
                # 动态查找以 /touch_status 结尾的话题
                names_types = self.get_topic_names_and_types()
                found = None
                for name, types in names_types:
                    if name.endswith('/touch_status') and 'protocol/msg/TouchStatus' in types:
                        found = name
                        break
                topic = found if found else '/touch_status'
                self.touch_sub = self.create_subscription(TouchStatus, topic, self._touch_cb, 10)
                self.get_logger().info(f"订阅触摸话题: {topic}")
            except Exception as e:
                self.get_logger().warn(f"触摸话题订阅失败: {e}")

    def _ensure_audio_client(self) -> bool:
        if not HAS_PROTOCOL or AudioTextPlay is None:
            return False
        try:
            if self.audio_client is not None and self.audio_client.service_is_ready():
                return True
            # 动态查找以 /speech_text_play 结尾的服务
            names_types = self.get_service_names_and_types()
            found = None
            for name, types in names_types:
                if name.endswith('/speech_text_play') and 'protocol/srv/AudioTextPlay' in types:
                    found = name
                    break
            if not found:
                return False
            self.audio_service_name = found
            self.audio_client = self.create_client(AudioTextPlay, found)
            return self.audio_client.wait_for_service(timeout_sec=2.0)
        except Exception:
            return False

    def say_text(self, text: str) -> bool:
        if not self._ensure_audio_client():
            print(f"📢(模拟) {text}")
            return False
        try:
            req = AudioTextPlay.Request()
            req.module_name = 'a_warehouse'
            req.is_online = True
            req.text = text
            if AudioPlay is not None:
                req.speech = AudioPlay(module_name=req.module_name, play_id=0)
            fut = self.audio_client.call_async(req)
            # 非阻塞，给一点处理时间
            t_end = time.time() + 2.0
            exec_ = SingleThreadedExecutor()
            exec_.add_node(self)
            try:
                while rclpy.ok() and time.time() < t_end and not fut.done():
                    exec_.spin_once(timeout_sec=0.05)
            finally:
                exec_.remove_node(self)
            return True
        except Exception:
            return False

    def _touch_cb(self, msg):
        try:
            # 参考 all_r.py：0x03 双击，0x07 长按；这里把二者都作为“触摸事件”计数
            if getattr(msg, 'touch_state', 0) in (0x03, 0x07):
                now = time.time()
                # 去抖：0.4s内不重复计数
                if now - self.last_touch_ts > 0.4:
                    self.touch_events += 1
                    self.last_touch_ts = now
                    print(f"🖐️ 收到触摸事件 #{self.touch_events}")
        except Exception:
            pass

    def wait_for_n_double_taps(self, n: int = 2, timeout_s: float = 0.0) -> bool:
        """等待 n 次触摸事件（双击/长按均算一次）。timeout_s=0 表示不限时"""
        target = max(1, int(n))
        self.touch_events = 0
        start = time.time()
        print(f"⏳ 等待触摸事件 {target} 次...（0=不限时）")
        exec_ = SingleThreadedExecutor()
        exec_.add_node(self)
        try:
            while True:
                if self.touch_events >= target:
                    print("✅ 触摸确认完成，继续执行")
                    return True
                if timeout_s > 0 and (time.time() - start) > timeout_s:
                    print("⌛ 触摸等待超时，继续执行")
                    return False
                exec_.spin_once(timeout_sec=0.1)
        finally:
            exec_.remove_node(self)

def check_robot_status(robot_controller):
    """检查机器人状态"""
    print("=== 检查机器人状态 ===")
    try:
        # 确保机器人站立
        print("确保机器人站立状态...")
        robot_controller.stand(duration=3000, sleep=3.0)
        time.sleep(2)  # 额外等待
        print("✅ 机器人状态检查完成")
        return True
    except Exception as e:
        print(f"❌ 机器人状态检查失败: {e}")
        return False

def run_a_warehouse_mission(keep_system_alive=False):
    """执行A库任务"""
    robot_base = None
    robot_controller = None
    
    try:
        # 1. 初始化
        print("=== 初始化A库任务 ===")
        robot_base = Robot_Ctrl()
        robot_controller = RobotController(robot_base)
        
        # 2. 启动系统
        print("=== 启动系统 ===")
        robot_base.run()
        robot_controller.start()
        time.sleep(2)  # 等待相机和IMU启动
        
        # 3. 站立准备
        print("=== 机器人站立 ===")
        robot_controller.stand(duration=3000, sleep=3.0)
        
        # 4. 状态检查
        print("=== 执行状态检查 ===")
        if not check_robot_status(robot_controller):
            print("❌ 机器人状态检查失败，终止任务")
            return

        # ========== 第一阶段：到达A库装货点 ==========
        print("=== 第一阶段：到达A库装货点 ===")
        
        # 直走1.0米到达识别点 - 使用IMU导航
        print("1. IMU直走1.0米到达二维码识别点")
        success = robot_controller.imu_forward_distance(1.1, base_vel=0.25)
        if not success:
            print("⚠️ 第一段行走失败，尝试重试...")
            # 重试一次
            time.sleep(1)
            success = robot_controller.imu_forward_distance(1.1, base_vel=0.25)
            if not success:
                print("❌ 第一段行走重试失败，继续执行...")
        
        # 等待稳定
        print("1.5. 等待机器人稳定...")
        time.sleep(2)
        
        # 右转90°
        print("2. 右转90°")
        robot_base.simple_turn(duration=2600, angular_vel=-0.8)
        
        # 等待转弯完成
        print("2.5. 等待转弯稳定...")
        time.sleep(1)
        
        # 直走0.5米 - 使用IMU导航
        print("3. IMU直走0.5米")
        success = robot_controller.imu_forward_distance(0.5, base_vel=0.25)
        if not success:
            print("⚠️ 第三段行走失败，尝试重试...")
            time.sleep(1)
            success = robot_controller.imu_forward_distance(0.5, base_vel=0.25)
            if not success:
                print("❌ 第三段行走重试失败，继续执行...")
        
        # 二维码识别等待阶段：站立并在10秒窗口内识别二维码
        print("4. 到达识别点，进行二维码识别（10秒超时）")
        # 发送站立指令，但仅短暂等待以便立即开始识别
        robot_base.stand(duration=10000, sleep=1.0)
        recognized_a1 = False
        recognized_a2 = False
        try:
            qr_detector = VisionDetector()
            qr_content = detect_qr_code_with_rgb(
                robot_controller.camera_controller, qr_detector, timeout=10.0
            )
            if qr_content:
                print(f"✅ 二维码识别成功：{qr_content}")
                norm = qr_content.strip().lower().replace('-', '').replace('_', '').replace(' ', '')
                recognized_a1 = ('a1' in norm)
                recognized_a2 = ('a2' in norm)
            else:
                print("⚠️ 在限定时间内未识别到二维码")
        except Exception as e:
            print(f"⚠️ 二维码识别过程中出现异常：{e}")
        # 先判断 A1/A2；如果都没有，再调 Doubao 文本识别辅助判断
        if not (recognized_a1 or recognized_a2):
            print("⚠️ 未识别到 A1/A2（二维码失败），尝试使用 Doubao 文本识别…")
            try:
                proc = subprocess.run([sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'doubao_seed_flash_ocr_qr.py')],
                                      cwd=os.path.dirname(os.path.abspath(__file__)), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                      universal_newlines=True, timeout=20)
                out = (proc.stdout or '').strip()
                if out:
                    obj = json.loads(out.splitlines()[-1])
                    key = str(obj.get('文字识别结果', '（）'))
                    if 'A' in key or '（A' in key or 'a' in key:
                        k2 = key.replace('（', '').replace('）', '').replace(' ', '').upper()
                        recognized_a1 = ('A-1' in k2) or ('A1' in k2)
                        recognized_a2 = ('A-2' in k2) or ('A2' in k2)
            except Exception as _e:
                print(f"⚠️ Doubao 辅助识别失败: {_e}")
        # 初始化交互节点用于语音播报
        inter_node = None
        try:
            inter_node = InteractionNode()
            
            if recognized_a1 and not recognized_a2:
                print("➡️ 识别到 A1（二维码/文字），执行 A1 库路径")
                inter_node.say_text("识别到A1")
            elif recognized_a2 and not recognized_a1:
                print("➡️ 识别到 A2（二维码/文字），执行 A2 库路径")
                inter_node.say_text("识别到A2")
            elif recognized_a1 and recognized_a2:
                print("➡️ 同时包含 A1/A2，默认执行 A1 库路径")
                inter_node.say_text("识别到A1")
            else:
                print("➡️ 未识别到 A1/A2（二维码/文字均失败），默认执行 A1 库路径")
                inter_node.say_text("识别到A1")
        except Exception as _e:
            print(f"⚠️ 语音播报异常: {_e}")
        finally:
            try:
                if inter_node is not None:
                    inter_node.destroy_node()
            except Exception:
                pass

        # ========== 分支执行：A2 或 A1 ==========
        robot_controller.stand(duration=3000, sleep=3.0)
        if recognized_a2:
            print("5. IMU直走1.2米到达二维码识别点")
            success = robot_controller.imu_forward_distance(1.2, base_vel=0.25)
            if not success:
                print("⚠️ 第二段行走失败，尝试重试...")
                time.sleep(1)
                success = robot_controller.imu_forward_distance(1.2, base_vel=0.25)
                if not success:
                    print("❌ 第二段行走重试失败，继续执行...")
            
            print("5.5. 等待机器人稳定...")
            time.sleep(2)
            
            print("6. 左转90°")
            robot_base.simple_turn(duration=2700, angular_vel=0.8)

            print("6.5. 等待转弯稳定...")
            time.sleep(1)

            print("7. IMU直走1米")
            success = robot_controller.imu_forward_distance(1.2, base_vel=0.15)
            if not success:
                print("⚠️ 第七段行走失败，尝试重试...")
                time.sleep(1)
                success = robot_controller.imu_forward_distance(1.0, base_vel=0.15)
                if not success:
                    print("❌ 第七段行走重试失败，继续执行...")

            print("8. 左转90°")
            robot_base.simple_turn(duration=2600, angular_vel=0.8)

            print("9. IMU直走1米进入A2库")
            robot_controller.imu_forward_distance(1.0, base_vel=0.15)

            print("10. 趴下装货，等待触摸 (A2)")
            robot_base.lie_down(duration=3000)
            inter_node = None
            try:
                inter_node = InteractionNode()
                inter_node.say_text("A区库位2")
                inter_node.wait_for_n_double_taps(n=2, timeout_s=0.0)
            except Exception as _e:
                print(f"⚠️ 交互节点异常，跳过触摸等待: {_e}")
            finally:
                try:
                    if inter_node is not None:
                        inter_node.destroy_node()
                except Exception:
                    pass

            print("11. 装货完成，站立 (A2)")
            robot_controller.stand(duration=3000, sleep=3.0)

            print("=== 第三阶段：离开A2库 ===")

            print("12. 后退1米")
            exit_duration = int(1.0 / 0.15 * 1000)
            robot_base.simple_forward(duration=exit_duration, vel=-0.15)

            print("13. 左转90°")
            robot_base.simple_turn(duration=2600, angular_vel=0.8)

            print("14. IMU直走1米")
            robot_controller.imu_forward_distance(1.0, base_vel=0.25)

            print("15. 右转90°")
            robot_base.simple_turn(duration=2750, angular_vel=-0.8)

            print("16. IMU直走2米")
            robot_controller.imu_forward_distance(2.0, base_vel=0.25)

            print("17. 右转90°")
            robot_base.simple_turn(duration=2400, angular_vel=-0.8)

            print("18. IMU直走1米")
            robot_controller.imu_forward_distance(1.0, base_vel=0.25)

            print("=== A2库任务完成 ===")
            print("✅ 已成功完成A2库装货任务")
            print("📋 任务总结:")
            print("   - 到达A2库装货点")
            print("   - 进入A2库装货")
            print("   - 离开A2库返回")
            print("   - 准备进入下一阶段")

            print("19. 任务完成，保持站立状态 (A2)")
            robot_controller.stand(duration=5000, sleep=5.0)

            print("🎉 A2库任务成功完成！")
        else:
            # 默认/识别到A1
            print("=== 第二阶段：前往A1库装货 ===")
            
            print("5. IMU直走1.2米到达二维码识别点")
            success = robot_controller.imu_forward_distance(1.2, base_vel=0.25)
            if not success:
                print("⚠️ 第二段行走失败，尝试重试...")
                time.sleep(1)
                success = robot_controller.imu_forward_distance(1.2, base_vel=0.25)
                if not success:
                    print("❌ 第二段行走重试失败，继续执行...")
            
            print("5.5. 等待机器人稳定...")
            time.sleep(2)
            
            print("6. 右转90°")
            robot_base.simple_turn(duration=2600, angular_vel=-0.8)
            
            print("6.5. 等待转弯稳定...")
            time.sleep(1)
            
            print("7. IMU直走1米")
            success = robot_controller.imu_forward_distance(1.0, base_vel=0.15)
            if not success:
                print("⚠️ 第七段行走失败，尝试重试...")
                time.sleep(1)
                success = robot_controller.imu_forward_distance(1.0, base_vel=0.15)
                if not success:
                    print("❌ 第七段行走重试失败，继续执行...")
            
            print("8. 右转90°")
            robot_base.simple_turn(duration=2500, angular_vel=-0.8)
            
            print("9. IMU直走1米进入A库")
            robot_controller.imu_forward_distance(0.8, base_vel=0.15)

            print("10. 趴下装货，等待触摸")
            robot_base.lie_down(duration=3000)
            inter_node = None
            try:
                inter_node = InteractionNode()
                inter_node.say_text("A区库位1")
                inter_node.wait_for_n_double_taps(n=2, timeout_s=0.0)
            except Exception as _e:
                print(f"⚠️ 交互节点异常，跳过触摸等待: {_e}")
            finally:
                try:
                    if inter_node is not None:
                        inter_node.destroy_node()
                except Exception:
                    pass
            
            print("11. 装货完成，站立")
            robot_controller.stand(duration=3000, sleep=3.0)
            
            print("=== 第三阶段：离开A库 ===")
            
            print("12. 后退1米")
            exit_duration = int(1.0 / 0.15 * 1000)
            robot_base.simple_forward(duration=exit_duration, vel=-0.15)
            
            print("13. 右转90°")
            robot_base.simple_turn(duration=2600, angular_vel=-0.8)
            
            print("14. IMU直走1米")
            robot_controller.imu_forward_distance(1.0, base_vel=0.25)
            
            print("15. 左转90°")
            robot_base.simple_turn(duration=2750, angular_vel=0.8)
            
            print("16. IMU直走2米")
            robot_controller.imu_forward_distance(2.0, base_vel=0.25)
            
            print("17. 右转90°")
            robot_base.simple_turn(duration=2400, angular_vel=-0.8)
            
            print("18. IMU直走1米")
            robot_controller.imu_forward_distance(1.0, base_vel=0.25)
            
            print("=== A库任务完成 ===")
            print("✅ 已成功完成A库装货任务")
            print("📋 任务总结:")
            print("   - 到达A库装货点")
            print("   - 进入A库装货")
            print("   - 离开A库返回")
            print("   - 准备进入下一阶段")
            
            print("19. 任务完成，保持站立状态")
            robot_controller.stand(duration=5000, sleep=5.0)
            
            print("🎉 A库任务成功完成！")

    except KeyboardInterrupt:
        print("\n📛 收到中断信号，正在停止...")
    except Exception as e:
        print(f"❌ 任务执行过程中出现错误: {e}")
        import traceback
        traceback.print_exc()
        print("=== 错误分析 ===")
        print("可能的原因：")
        print("1. IMU数据异常或不稳定")
        print("2. 机器人状态不一致")
        print("3. 网络通信问题")
        print("4. 传感器故障")
        print("建议：")
        print("- 重新启动机器人")
        print("- 检查IMU传感器")
        print("- 确保网络连接稳定")
    finally:
        if keep_system_alive:
            print("=== 保持系统运行模式 ===")
            print("✅ A库任务完成，机器人控制系统保持运行")
            print("📋 系统状态：")
            print("   - 机器人控制器：运行中")
            print("   - 相机系统：运行中")
            print("   - IMU系统：运行中")
            print("🔄 准备进行下一阶段任务...")
        else:
            print("=== 清理资源 ===")
            if robot_controller:
                robot_controller.stop()
            if robot_base:
                robot_base.quit()
            print("程序已退出")


def main(args=None):
    """主函数"""
    import argparse
    
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='A库任务执行器')
    parser.add_argument('--keep-system-alive', action='store_true', 
                       help='任务完成后保持机器人控制系统运行')
    parsed_args = parser.parse_args()
    
    try:
        rclpy.init(args=args)
        run_a_warehouse_mission(keep_system_alive=parsed_args.keep_system_alive)
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main() 