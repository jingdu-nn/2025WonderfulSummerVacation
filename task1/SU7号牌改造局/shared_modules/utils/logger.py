#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
日志记录工具

作者：Xiaomi Cup 2025 Final Team
"""

import logging
import sys
from datetime import datetime
from typing import Optional


def get_logger(name: str = None, level: int = logging.INFO, 
               log_to_file: bool = True, log_file: Optional[str] = None) -> logging.Logger:
    """
    获取配置好的日志记录器
    
    Args:
        name: 日志记录器名称
        level: 日志级别
        log_to_file: 是否写入文件
        log_file: 日志文件路径，None则自动生成
        
    Returns:
        配置好的日志记录器
    """
    if name is None:
        name = "MissionLogger"
    
    logger = logging.getLogger(name)
    
    # 避免重复添加处理器
    if logger.handlers:
        return logger
    
    logger.setLevel(level)
    
    # 创建格式器
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    
    # 控制台处理器
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(level)
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)
    
    # 文件处理器
    if log_to_file:
        if log_file is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            log_file = f"mission_log_{timestamp}.log"
        
        file_handler = logging.FileHandler(log_file, mode='a', encoding='utf-8')
        file_handler.setLevel(level)
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)
    
    return logger


class Logger:
    """
    简单的日志记录器类，封装logging功能
    """
    
    def __init__(self, name: str = None, level: int = logging.INFO):
        """
        初始化Logger
        
        Args:
            name: 日志记录器名称
            level: 日志级别
        """
        self.logger = get_logger(name, level)
        self.name = name or "MissionLogger"
    
    def debug(self, message: str):
        """调试信息"""
        self.logger.debug(f"[{self.name}] {message}")
    
    def info(self, message: str):
        """一般信息"""
        self.logger.info(f"[{self.name}] {message}")
    
    def warning(self, message: str):
        """警告信息"""
        self.logger.warning(f"[{self.name}] {message}")
    
    def error(self, message: str):
        """错误信息"""
        self.logger.error(f"[{self.name}] {message}")
    
    def critical(self, message: str):
        """严重错误信息"""
        self.logger.critical(f"[{self.name}] {message}")


# 导出类和函数
__all__ = ['Logger', 'get_logger']
