"""
工具模块
提供日志、配置加载、相机管理等通用功能
"""

from .logger import get_logger
from .touch_sensor import TouchSensorMonitor, TouchSensorType

# 相机相关模块
try:
    from .camera_manager import get_camera_controller, start_camera_system, stop_camera_system, get_camera_image
    from .camera_checker import CameraChecker, check_cameras_quick, print_camera_status
    _camera_available = True
except ImportError as e:
    print(f"相机模块导入失败: {e}")
    _camera_available = False

if _camera_available:
    __all__ = [
        'get_logger', 
        'TouchSensorMonitor', 
        'TouchSensorType',
        'get_camera_controller',
        'start_camera_system',
        'stop_camera_system', 
        'get_camera_image',
        'CameraChecker',
        'check_cameras_quick',
        'print_camera_status'
    ]
else:
    __all__ = ['get_logger', 'TouchSensorMonitor', 'TouchSensorType']
