#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
二维码识别模块集成
整合visual/qr_code模块，提供任务专用的二维码识别接口

作者：Xiaomi Cup 2025 Final Team
"""

import sys
import os
from typing import List, Optional, Tuple
import cv2

# 添加visual/qr_code目录到路径
qr_code_path = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'visual', 'qr_code')
sys.path.append(qr_code_path)

try:
    from qr_detector import QRCodeDetector, QRCodeResult
    QR_DETECTOR_AVAILABLE = True
except ImportError as e:
    print(f"警告：无法导入二维码识别模块: {e}")
    print(f"请确保 visual/qr_code/qr_detector.py 文件存在")
    QR_DETECTOR_AVAILABLE = False
    
    # 创建兼容的备用类
    class QRCodeDetector:
        def __init__(self, **kwargs):
            print("警告：使用备用QRCodeDetector，功能受限")
            
        def detect_qr_codes(self, image):
            print("警告：二维码检测功能不可用")
            return []
            
        def detect_from_file(self, image_path):
            print("警告：二维码检测功能不可用")
            return []


class QRRecognitionModule:
    """二维码识别模块"""
    
    def __init__(self, use_camera: bool = True, camera_id: int = 0):
        """
        初始化二维码识别模块
        
        Args:
            use_camera: 是否使用摄像头
            camera_id: 摄像头ID
        """
        self.use_camera = use_camera
        self.camera_id = camera_id
        self.qr_available = QR_DETECTOR_AVAILABLE
        
        try:
            self.detector = QRCodeDetector(use_pyzbar=True, debug=True)
        except Exception as e:
            print(f"QRCodeDetector初始化失败: {e}")
            self.detector = QRCodeDetector()  # 使用备用版本
            self.qr_available = False
            
        self.cap = None
        
        # 打印初始化状态
        if self.qr_available:
            print("✓ 二维码识别模块初始化成功")
        else:
            print("⚠ 二维码识别模块初始化失败，功能受限")
            print("  建议安装：pip install pyzbar 或升级OpenCV版本")
        
        if use_camera:
            self._init_camera()
    
    def _init_camera(self):
        """初始化摄像头"""
        try:
            self.cap = cv2.VideoCapture(self.camera_id)
            if not self.cap.isOpened():
                print(f"警告：无法打开摄像头 {self.camera_id}")
                self.cap = None
        except Exception as e:
            print(f"摄像头初始化失败: {e}")
            self.cap = None
    
    def capture_and_recognize(self, max_attempts: int = 10, 
                            target_codes: List[str] = None) -> Optional[str]:
        """
        捕获图像并识别二维码
        
        Args:
            max_attempts: 最大尝试次数
            target_codes: 目标二维码列表，如['A-1', 'A-2', 'B-1', 'B-2']
            
        Returns:
            识别到的二维码内容，失败返回None
        """
        if not self.qr_available:
            print("警告：二维码检测功能不可用，返回None")
            return None
            
        if not self.cap:
            print("摄像头未初始化")
            return None
        
        print(f"开始识别二维码，最大尝试次数: {max_attempts}")
        if target_codes:
            print(f"目标二维码: {target_codes}")
        
        for attempt in range(max_attempts):
            try:
                ret, frame = self.cap.read()
                if not ret:
                    print(f"第 {attempt + 1} 次：无法读取摄像头画面")
                    continue
                
                # 检测二维码
                results = self.detector.detect_qr_codes(frame)
                
                if results:
                    for result in results:
                        qr_data = result.data.strip()
                        print(f"第 {attempt + 1} 次：检测到二维码 '{qr_data}'")
                        
                        # 如果指定了目标码，检查是否匹配
                        if target_codes:
                            if qr_data in target_codes:
                                print(f"找到目标二维码: {qr_data}")
                                return qr_data
                            else:
                                print(f"二维码 '{qr_data}' 不在目标列表中")
                        else:
                            return qr_data
                else:
                    print(f"第 {attempt + 1} 次：未检测到二维码")
                
            except Exception as e:
                print(f"第 {attempt + 1} 次识别失败: {e}")
        
        print("二维码识别失败")
        return None
    
    def recognize_warehouse_code(self, warehouse_type: str = 'A') -> Optional[str]:
        """
        识别仓库二维码
        
        Args:
            warehouse_type: 仓库类型，'A' 或 'B'
            
        Returns:
            识别到的仓库编号，如 'A-1', 'A-2', 'B-1', 'B-2'
        """
        if warehouse_type == 'A':
            target_codes = ['A-1', 'A-2']
        elif warehouse_type == 'B':
            target_codes = ['B-1', 'B-2']
        else:
            target_codes = ['A-1', 'A-2', 'B-1', 'B-2']
        
        print(f"识别 {warehouse_type} 类仓库二维码...")
        return self.capture_and_recognize(max_attempts=15, target_codes=target_codes)
    
    def recognize_from_file(self, image_path: str) -> Optional[str]:
        """
        从图像文件识别二维码
        
        Args:
            image_path: 图像文件路径
            
        Returns:
            识别到的二维码内容
        """
        if not self.qr_available:
            print("警告：二维码检测功能不可用，返回None")
            return None
            
        try:
            results = self.detector.detect_from_file(image_path)
            if results:
                return results[0].data.strip()
            return None
        except Exception as e:
            print(f"从文件识别二维码失败: {e}")
            return None
    
    def close(self):
        """关闭摄像头"""
        if self.cap:
            self.cap.release()
            cv2.destroyAllWindows()
            print("摄像头已关闭")
