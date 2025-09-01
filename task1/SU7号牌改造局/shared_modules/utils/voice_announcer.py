#!/usr/bin/env python3
"""
语音播报模块 (voice_announcer.py)

这个模块封装了机器狗的语音播报功能，提供简单易用的接口来进行TTS播报。
完全基于 say_ciallo.py 的服务调用模式重新实现。

作者：Xiaomi Cup 2025 Final Team
"""

import rclpy
from rclpy.node import Node
import time
import threading
from typing import Optional
import sys

# 尝试导入ROS2服务和消息定义
try:
    from protocol.srv import AudioTextPlay
    from protocol.msg import AudioPlay
    HAS_PROTOCOL = True
except ImportError:
    print("警告：无法导入 'protocol' 包，使用模拟模式")
    HAS_PROTOCOL = False
    
    # Mock类定义
    class AudioTextPlay:
        class Request:
            def __init__(self):
                self.module_name = ""
                self.is_online = True
                self.text = ""
                self.speech = None
    
    class AudioPlay:
        def __init__(self):
            self.module_name = ""
            self.play_id = 0


class CyberDogVoiceAnnouncer(Node):
    """
    CyberDog语音播报节点 - 完全基于say_ciallo.py
    """
    def __init__(self, node_name='cyberdog_voice_announcer'):
        super().__init__(node_name)
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
        if not HAS_PROTOCOL:
            print(f"模拟TTS播报: {text_to_speak}")
            return True
            
        if not self.ready or not self.cli or not self.cli.service_is_ready():
            self.get_logger().error("服务客户端未就绪。")
            return False

        # 根据 say_ciallo.py 构造请求
        req = AudioTextPlay.Request()
        req.module_name = "mission_control"
        req.is_online = True
        req.text = text_to_speak
        
        # 'speech' 字段是 AudioPlay 类型，即使不使用也创建一个空对象
        req.speech = AudioPlay()
        req.speech.module_name = "mission_control"
        req.speech.play_id = 0  # 0 表示不播放特定的离线语音

        self.get_logger().info(f'正在请求播报: "{text_to_speak}"')
        
        try:
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
        except Exception as e:
            self.get_logger().error(f'播报请求失败: {e}')
            return False


class VoiceAnnouncer:
    """
    语音播报器 - 基于 CyberDogVoiceAnnouncer 的简单接口包装
    """
    
    def __init__(self):
        self.node: Optional[CyberDogVoiceAnnouncer] = None
        self.executor = None
        self.executor_thread = None
        self._initialized = False
    
    def initialize(self) -> bool:
        """
        初始化语音播报器
        
        Returns:
            bool: 初始化是否成功
        """
        try:
            if not rclpy.ok():
                rclpy.init()
            
            self.node = CyberDogVoiceAnnouncer()
            
            # 使用单独的线程运行executor
            from rclpy.executors import SingleThreadedExecutor
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(self.node)
            self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
            self.executor_thread.start()
            
            self._initialized = True
            time.sleep(0.1)  # 确保节点完全启动
            
            return True
            
        except Exception as e:
            print(f"语音播报器初始化失败: {e}")
            return False
    
    def cleanup(self):
        """清理资源"""
        if self.executor:
            self.executor.remove_node(self.node)
            self.executor.shutdown()
        
        if self.node:
            self.node.destroy_node()
        
        self._initialized = False
    
    def say(self, text: str, wait_time: float = 0.5) -> bool:
        """
        说出指定文本（使用服务调用方式）
        
        Args:
            text (str): 要播报的文本
            wait_time (float): 播报后的等待时间（秒）
            
        Returns:
            bool: 播报是否成功
        """
        if not self._initialized:
            if not self.initialize():
                return False
        
        try:
            result = self.node.say(text)
            time.sleep(wait_time)
            return result
        except Exception as e:
            print(f"语音播报失败: {e}")
            return False
    
    def speak_text(self, text: str, wait_time: float = 0.5) -> bool:
        """
        说出指定文本（兼容接口）
        
        Args:
            text (str): 要播报的文本
            wait_time (float): 播报后的等待时间（秒）
            
        Returns:
            bool: 播报是否成功
        """
        return self.say(text, wait_time)


# 全局语音播报器实例
_global_announcer = None


def get_voice_announcer() -> VoiceAnnouncer:
    """获取全局语音播报器实例"""
    global _global_announcer
    if _global_announcer is None:
        _global_announcer = VoiceAnnouncer()
    return _global_announcer


def say(text: str, wait_time: float = 0.5) -> bool:
    """
    快速语音播报函数
    
    Args:
        text (str): 要播报的文本
        wait_time (float): 播报后的等待时间（秒）
        
    Returns:
        bool: 播报是否成功
        
    Example:
        >>> from shared_modules.utils.voice_announcer import say
        >>> say("A-1")
        >>> say("开始执行任务")
    """
    announcer = get_voice_announcer()
    return announcer.say(text, wait_time)





def cleanup_voice_announcer():
    """清理全局语音播报器"""
    global _global_announcer
    if _global_announcer:
        _global_announcer.cleanup()
        _global_announcer = None


if __name__ == "__main__":
    """测试代码"""
    print("测试语音播报模块...")
    
    # 测试基本功能
    if say("测试语音播报功能"):
        print("✓ 语音播报测试成功")
    else:
        print("✗ 语音播报测试失败")
    
    # 清理
    cleanup_voice_announcer()
    print("测试完成")