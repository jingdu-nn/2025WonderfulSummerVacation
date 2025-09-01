#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简化的语音播报模块
完全基于say_ciallo.py的实现，确保TTS正常工作

作者：Xiaomi Cup 2025 Final Team
"""

import rclpy
from rclpy.node import Node
import sys
import time

# 尝试导入由ROS2提供的服务和消息定义
try:
    from protocol.srv import AudioTextPlay
    from protocol.msg import AudioPlay
    HAS_PROTOCOL = True
except ImportError:
    print("警告：无法导入 'protocol' 包，将使用模拟播报")
    HAS_PROTOCOL = False


class CyberDogVoiceAnnouncer(Node):
    """
    CyberDog语音播报节点 - 完全基于say_ciallo.py
    """
    def __init__(self):
        super().__init__('cyberdog_voice_announcer')
        self.cli = None
        self.service_name = ""
        self.ready = False
        
        if HAS_PROTOCOL:
            if self.find_service('/speech_text_play'):
                if self.setup_client():
                    self.ready = True

    def find_service(self, service_suffix, timeout_sec=5.0):
        """
        动态查找服务，因为服务名称中可能包含动态部分。
        """
        self.get_logger().info(f"正在查找以 '{service_suffix}' 结尾的服务...")
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            service_names_and_types = self.get_service_names_and_types()
            for name, types in service_names_and_types:
                if name.endswith(service_suffix) and 'protocol/srv/AudioTextPlay' in types:
                    self.service_name = name
                    self.get_logger().info(f"找到语音服务: '{self.service_name}'")
                    return True
            time.sleep(0.5)
        
        self.get_logger().error(f"查找服务超时: 未找到匹配的服务。")
        return False

    def setup_client(self):
        """
        根据找到的服务名称设置客户端。
        """
        if not self.service_name:
            self.get_logger().error("服务名称为空，无法创建客户端。")
            return False
            
        self.cli = self.create_client(AudioTextPlay, self.service_name)
        self.get_logger().info("正在等待服务可用...")
        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"服务 '{self.service_name}' 不可用。")
            return False
        
        self.get_logger().info("语音服务客户端已就绪。")
        return True

    def say(self, text_to_speak):
        """
        发送文本到语音服务进行播报。
        """
        if not self.ready or not self.cli or not self.cli.service_is_ready():
            self.get_logger().error("服务客户端未就绪。")
            return False

        # 根据 say_ciallo.py 的定义构造请求
        req = AudioTextPlay.Request()
        req.module_name = "mission_control"
        req.is_online = True
        req.text = text_to_speak
        
        # 'speech' 字段是 AudioPlay 类型，即使不使用也创建一个空对象
        req.speech = AudioPlay()
        req.speech.module_name = "mission_control"
        req.speech.play_id = 0  # 0 表示不播放特定的离线语音

        self.get_logger().info(f'正在请求播报: "{text_to_speak}"')
        
        future = self.cli.call_async(req)
        
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.done():
            try:
                response = future.result()
                if response.status == 0:
                    self.get_logger().info(f"播报成功 (status={response.status}, code={response.code})")
                    return True
                else:
                    self.get_logger().error(f"播报失败 (status={response.status}, code={response.code})")
                    return False
            except Exception as e:
                self.get_logger().error(f'服务调用时发生异常: {e}')
                return False
        else:
            self.get_logger().error('服务调用超时。')
            return False


class SimpleVoiceAnnouncer:
    """
    简化的语音播报器 - 封装CyberDogVoiceAnnouncer
    """
    
    def __init__(self):
        self.node = None
        self.initialized = False
        self._init_ros2()
    
    def _init_ros2(self):
        """初始化ROS2"""
        try:
            if not rclpy.ok():
                rclpy.init()
            
            if HAS_PROTOCOL:
                self.node = CyberDogVoiceAnnouncer()
                self.initialized = self.node.ready
                if self.initialized:
                    print("✓ CyberDog语音服务初始化成功")
                else:
                    print("⚠️ CyberDog语音服务初始化失败，使用模拟播报")
            else:
                print("⚠️ 缺少protocol包，使用模拟播报")
                
        except Exception as e:
            print(f"ROS2初始化失败: {e}")
            self.initialized = False
    
    def speak_text(self, text: str, wait_time: float = 0.5) -> bool:
        """
        说出指定文本（主要接口）
        """
        if self.initialized and self.node:
            try:
                result = self.node.say(text)
                time.sleep(wait_time)
                return result
            except Exception as e:
                print(f"语音播报失败: {e}")
        
        # 备用方案：控制台输出
        print(f"🔊 语音播报: {text}")
        time.sleep(wait_time)
        return True
    
    def say(self, text: str, wait_time: float = 0.5) -> bool:
        """
        说出指定文本（兼容接口）
        """
        return self.speak_text(text, wait_time)
    
    def cleanup(self):
        """清理资源"""
        if self.node:
            self.node.destroy_node()


class FallbackVoiceAnnouncer:
    """
    备用语音播报器 - 当ROS2不可用时使用
    """
    
    def __init__(self):
        print("FallbackVoiceAnnouncer初始化")
    
    def speak_text(self, text: str, wait_time: float = 0.5) -> bool:
        """模拟语音播报"""
        print(f"🔊 语音播报: {text}")
        time.sleep(wait_time)
        return True
    
    def say(self, text: str, wait_time: float = 0.5) -> bool:
        """兼容接口"""
        return self.speak_text(text, wait_time)
    
    def cleanup(self):
        """无需清理"""
        pass


def create_voice_announcer():
    """创建合适的语音播报器"""
    if HAS_PROTOCOL:
        try:
            announcer = SimpleVoiceAnnouncer()
            return announcer
        except Exception as e:
            print(f"创建SimpleVoiceAnnouncer失败: {e}")
    
    # 返回备用实现
    return FallbackVoiceAnnouncer()


# 测试代码
if __name__ == "__main__":
    print("测试CyberDog语音播报模块...")
    
    announcer = create_voice_announcer()
    
    if announcer.speak_text("测试语音播报功能"):
        print("✓ 语音播报测试成功")
    else:
        print("✗ 语音播报测试失败")
    
    announcer.cleanup()
    print("测试完成")