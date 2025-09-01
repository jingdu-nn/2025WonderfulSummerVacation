"""
共享模块包初始化文件
"""

__version__ = "1.0.0"
__author__ = "Xiaomi Cup 2025 Final Team"

# 导入核心功能模块
from .utils.voice_announcer import say, play_audio, VoiceAnnouncer
from .utils.enhanced_touch_sensor import wait_for_touch, TouchMode, EnhancedTouchSensor

__all__ = [
    'say', 'play_audio', 'VoiceAnnouncer',
    'wait_for_touch', 'TouchMode', 'EnhancedTouchSensor'
]
