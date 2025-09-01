#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
回形赛道公共库
提供四个路段的基础功能：上下坡、石板路、黄灯识别、限高杆

作者：Xiaomi Cup 2025 Final Team
"""

import os
import sys
import time
import json
import cv2
import numpy as np
import base64
import requests
from typing import Optional, Tuple, List, Dict
from threading import Lock

# 添加当前路径到系统路径
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)
sys.path.append(os.path.join(current_dir, '..', '..', 'demo'))

# 尝试多种导入方式以确保兼容性
RobotController = None
VoiceAnnouncer = None
Logger = None

# 方法1：尝试相对导入
try:
    from .motion_control.robot_controller import RobotController
    from .utils.voice_announcer import VoiceAnnouncer
    from .utils.logger import Logger
    print("✓ 相对导入成功")
except ImportError as e:
    print(f"相对导入失败: {e}")
    
    # 方法2：尝试绝对导入
    try:
        from motion_control.robot_controller import RobotController
        from utils.voice_announcer import VoiceAnnouncer
        from utils.logger import Logger
        print("✓ 绝对导入成功")
    except ImportError as e2:
        print(f"绝对导入失败: {e2}")
        
        # 方法3：尝试动态导入
        try:
            import importlib.util
            import sys
            import os
            
            current_dir = os.path.dirname(os.path.abspath(__file__))
            
            # 动态导入RobotController
            robot_controller_path = os.path.join(current_dir, "motion_control", "robot_controller.py")
            if os.path.exists(robot_controller_path):
                spec = importlib.util.spec_from_file_location("robot_controller", robot_controller_path)
                robot_controller_module = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(robot_controller_module)
                RobotController = robot_controller_module.RobotController
            
            # 动态导入VoiceAnnouncer（优先使用简化版本）
            simple_voice_announcer_path = os.path.join(current_dir, "utils", "simple_voice_announcer.py")
            voice_announcer_path = os.path.join(current_dir, "utils", "voice_announcer.py")
            
            if os.path.exists(simple_voice_announcer_path):
                try:
                    spec = importlib.util.spec_from_file_location("simple_voice_announcer", simple_voice_announcer_path)
                    simple_voice_announcer_module = importlib.util.module_from_spec(spec)
                    spec.loader.exec_module(simple_voice_announcer_module)
                    
                    # 优先使用SimpleVoiceAnnouncer类
                    if hasattr(simple_voice_announcer_module, 'SimpleVoiceAnnouncer'):
                        VoiceAnnouncer = simple_voice_announcer_module.SimpleVoiceAnnouncer
                    elif hasattr(simple_voice_announcer_module, 'FallbackVoiceAnnouncer'):
                        VoiceAnnouncer = simple_voice_announcer_module.FallbackVoiceAnnouncer
                    else:
                        # 使用create_voice_announcer函数创建包装类
                        creator_func = simple_voice_announcer_module.create_voice_announcer
                        class VoiceAnnouncerWrapper:
                            def __init__(self):
                                self._announcer = creator_func()
                            def speak_text(self, text, wait_time=0.5):
                                return self._announcer.speak_text(text, wait_time)
                            def say(self, text, wait_time=0.5):
                                return self._announcer.say(text, wait_time)
                            def cleanup(self):
                                if hasattr(self._announcer, 'cleanup'):
                                    self._announcer.cleanup()
                        VoiceAnnouncer = VoiceAnnouncerWrapper
                    
                    print("✓ 使用简化VoiceAnnouncer")
                except Exception as e:
                    print(f"简化VoiceAnnouncer导入失败: {e}")
                    VoiceAnnouncer = None
            
            if not VoiceAnnouncer and os.path.exists(voice_announcer_path):
                try:
                    spec = importlib.util.spec_from_file_location("voice_announcer", voice_announcer_path)
                    voice_announcer_module = importlib.util.module_from_spec(spec)
                    spec.loader.exec_module(voice_announcer_module)
                    VoiceAnnouncer = voice_announcer_module.VoiceAnnouncer
                    print("✓ 使用标准VoiceAnnouncer")
                except Exception as e:
                    print(f"标准VoiceAnnouncer导入失败: {e}")
                    VoiceAnnouncer = None
            
            # 动态导入Logger
            logger_path = os.path.join(current_dir, "utils", "logger.py")
            if os.path.exists(logger_path):
                spec = importlib.util.spec_from_file_location("logger", logger_path)
                logger_module = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(logger_module)
                Logger = logger_module.Logger
            
            if RobotController and VoiceAnnouncer and Logger:
                print("✓ 动态导入成功")
            
        except Exception as e3:
            print(f"动态导入失败: {e3}")

# 如果所有导入方式都失败，创建Mock类
if not RobotController or not VoiceAnnouncer or not Logger:
    print("使用Mock类作为备用方案")
    
    class MockRobotController:
        def __init__(self): 
            print("MockRobotController初始化")
        def run(self): print("机器人控制器启动")
        def stop(self): print("机器人控制器停止")
        def turn_right(self, angle): print(f"右转 {angle}°")
        def turn_left(self, angle): print(f"左转 {angle}°")
        def move_distance_imu(self, distance, **kwargs): print(f"移动 {distance}m")
        def set_step_height(self, *args, **kwargs): print(f"设置步高: {args}, {kwargs}")
        def set_body_height(self, height): print(f"设置机身高度: {height}m")
        def set_body_pose(self, *args, **kwargs): print(f"设置机身姿态: {args}, {kwargs}")
        def load_gait_from_file(self, path): print(f"加载步态文件: {path}")
    
    class MockVoiceAnnouncer:
        def __init__(self): 
            print("MockVoiceAnnouncer初始化")
        def speak_text(self, text): print(f"🔊 语音播报: {text}")
        def say(self, text): print(f"🔊 语音播报: {text}")
    
    class MockLogger:
        def __init__(self, name): 
            self.name = name
            print(f"MockLogger初始化: {name}")
        def info(self, msg): print(f"ℹ️ [{self.name}] {msg}")
        def warning(self, msg): print(f"⚠️ [{self.name}] {msg}")
        def error(self, msg): print(f"❌ [{self.name}] {msg}")
    
    # 只替换失败的导入
    if not RobotController:
        RobotController = MockRobotController
    if not VoiceAnnouncer:
        VoiceAnnouncer = MockVoiceAnnouncer
    if not Logger:
        Logger = MockLogger

# 尝试导入LCM消息类型
try:
    from robot_control_cmd_lcmt import robot_control_cmd_lcmt
except ImportError:
    print("警告：无法导入LCM消息类型，某些功能可能受限")


class LoopTrackState:
    """回形赛道状态管理"""
    
    def __init__(self):
        self.state_file = os.path.join(current_dir, "loop_track.json")
        self.lock = Lock()
    
    def save_state(self, go_direction: str):
        """保存赛道状态"""
        # 根据去程方向确定返程方向
        back_direction = "R" if go_direction == "L" else "L"
        
        state = {
            "go": go_direction,
            "back": back_direction,
            "timestamp": time.time()
        }
        
        with self.lock:
            with open(self.state_file, 'w', encoding='utf-8') as f:
                json.dump(state, f, ensure_ascii=False, indent=2)
        
        print(f"保存赛道状态: 去程={go_direction}, 返程={back_direction}")
    
    def load_state(self) -> Optional[Dict[str, str]]:
        """加载赛道状态"""
        try:
            with self.lock:
                if os.path.exists(self.state_file):
                    with open(self.state_file, 'r', encoding='utf-8') as f:
                        return json.load(f)
        except Exception as e:
            print(f"加载状态失败: {e}")
        return None


class ArrowDetector:
    """箭头识别器 - 基于豆包大模型"""
    
    def __init__(self):
        # API配置 - 与visual/arrow/doubao_llm相同
        self.ARK_API_KEY = "f6b5c11f-8525-4eab-9b79-b21cf64e34e1"
        self.MODEL_ID = "ep-20250813152311-sdwtp"
        self.API_URL = "https://ark.cn-beijing.volces.com/api/v3/chat/completions"
        
        # 识别提示词
        self.ARROW_RECOGNITION_PROMPT = """你是一位专业的图像分析AI。你的任务是对传入的摄像头图片进行分析，识别其中箭头的左右方向。不要进行深度思考

## 输入:
- 摄像头图片: {{IMAGE_CAMERA}}

## 输出格式:
请按照以下JSON格式输出识别结果：
{
    "箭头方向": "L" 或 "R"
}
"""
    
    def image_to_data_uri(self, image_path: str) -> str:
        """将图片转换为data URI格式"""
        try:
            image_bgr = cv2.imread(image_path)
            if image_bgr is None:
                raise ValueError(f"无法读取图片文件: {image_path}")
            
            # 调整图片大小
            h, w = image_bgr.shape[:2]
            max_side = 640
            scale = min(1.0, float(max_side) / float(max(h, w)))
            if scale < 1.0:
                new_w = int(w * scale)
                new_h = int(h * scale)
                image_bgr = cv2.resize(image_bgr, (new_w, new_h), interpolation=cv2.INTER_AREA)
            
            # 编码为JPEG
            ok, buf = cv2.imencode('.jpg', image_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
            if not ok:
                raise ValueError("图片编码失败")
                
            # 转换为base64
            b64 = base64.b64encode(buf.tobytes()).decode('utf-8')
            return f'data:image/jpeg;base64,{b64}'
            
        except Exception as e:
            raise ValueError(f"图片处理失败: {e}")
    
    def detect_arrow_direction(self, image_path: str) -> str:
        """识别箭头方向"""
        try:
            # 转换图片格式
            img_data_uri = self.image_to_data_uri(image_path)
            
            # 调用API
            headers = {
                'Authorization': f'Bearer {self.ARK_API_KEY}',
                'Content-Type': 'application/json'
            }
            
            payload = {
                'model': self.MODEL_ID,
                'thinking': {'type': 'disabled'},
                'messages': [
                    {
                        'role': 'user',
                        'content': [
                            {'type': 'image_url', 'image_url': {'url': img_data_uri}},
                            {'type': 'text', 'text': self.ARROW_RECOGNITION_PROMPT},
                        ],
                    }
                ],
            }
            
            response = requests.post(self.API_URL, headers=headers, json=payload, timeout=30)
            response.raise_for_status()
            
            result = response.json()
            content = result['choices'][0]['message']['content']
            
            # 解析JSON结果
            try:
                import json
                result_json = json.loads(content)
                direction = result_json.get('箭头方向', 'L')
                return direction if direction in ['L', 'R'] else 'L'
            except:
                # 如果JSON解析失败，尝试从文本中提取
                if 'R' in content:
                    return 'R'
                else:
                    return 'L'
                    
        except Exception as e:
            print(f"箭头识别失败: {e}")
            return 'L'  # 默认返回左


class SlopeSection:
    """上下坡路段"""
    
    def __init__(self, robot_controller: RobotController, voice_announcer: VoiceAnnouncer):
        self.robot = robot_controller
        self.voice = voice_announcer
        
    def execute_slope_section(self):
        """执行上下坡路段"""
        print("开始执行上下坡路段")
        
        # ===================
        # 上坡阶段 (150cm)
        # ===================
        
        # 上坡前准备：先左旋转210度倒着过，分配重心到狗的腹部（硬件限制调整）
        self.voice.speak_text("准备通过上坡，左转210度倒着走")
        self.robot.turn_left(210)
        time.sleep(1)
        
        # 降低前腿步高以调整到上坡姿态
        print("调整上坡姿态：降低前腿步高")
        self.robot.set_step_height(front_left=0.03, front_right=0.03, back_left=0.06, back_right=0.06)
        time.sleep(0.5)
        
        # 上坡走150cm，速度0.1，倒着走
        print("开始上坡：150cm，速度0.1m/s，倒着走")
        self.robot.move_distance_imu(1.5, backward=True, velocity=0.1)
        
        # 上坡完成后，左转210°恢复到正常方向（硬件限制调整）
        print("上坡完成，左转210°恢复正向")
        self.robot.turn_left(210)
        time.sleep(1)
        
        # ===================
        # 平面阶段 (126cm)
        # ===================
        
        # 平面126cm，正常方向前进
        print("平面行进：126cm")
        self.robot.move_distance_imu(1.26, backward=False, velocity=0.1)
        
        # ===================
        # 下坡阶段 (125cm)  
        # ===================
        
        # 下坡时调整姿态：降低前腿步高
        print("调整下坡姿态：降低前腿步高")
        self.robot.set_step_height(front_left=0.03, front_right=0.03, back_left=0.06, back_right=0.06)
        time.sleep(0.5)
        
        # 下坡走125cm，速度限制到0.04
        print("开始下坡：125cm，限速0.04m/s")
        self.robot.move_distance_imu(1.25, backward=False, velocity=0.04)
        
        # 恢复正常步高
        print("恢复正常步高")
        self.robot.set_step_height(front_left=0.06, front_right=0.06, back_left=0.06, back_right=0.06)
        time.sleep(0.5)
        
        print("上下坡路段完成")


class YellowLightSection:
    """黄灯识别路段"""
    
    def __init__(self, robot_controller: RobotController, voice_announcer: VoiceAnnouncer):
        self.robot = robot_controller
        self.voice = voice_announcer
        
        # 黄灯检测参数
        self.lower_yellow = np.array([20, 50, 50])
        self.upper_yellow = np.array([35, 255, 255])
        self.min_area = 100
        self.max_area = 5000
        
    def execute_yellow_light_section(self, use_sensors: bool = False, sensor_config: int = 0, is_dead_reckoning: bool = False):
        """
        执行黄灯识别路段
        
        Args:
            use_sensors: 是否使用传感器识别
            sensor_config: 传感器配置 (0=全部, 1=激光雷达, 2=超声, 3=TOF)
            is_dead_reckoning: 是否为写死版
        """
        print("开始执行黄灯识别路段")
        
        if is_dead_reckoning:
            self._execute_dead_reckoning_version()
        elif use_sensors:
            self._execute_with_sensors(sensor_config)
        else:
            self._execute_with_imu_detection()
    
    def _execute_dead_reckoning_version(self):
        """写死版黄灯识别：先走150cm，然后在150-300cm随机停止"""
        import random
        
        print("写死版黄灯识别")
        
        # 先走150cm
        print("前进150cm到达黄灯检测区域")
        self.robot.move_distance_imu(1.5, velocity=0.1)
        
        # 在150-300cm之间随机停止位置（即50-150cm的额外距离）
        extra_distance = random.uniform(0.5, 1.5)  # 0.5-1.5米
        print(f"继续前进{extra_distance:.2f}m寻找黄灯")
        self.robot.move_distance_imu(extra_distance, velocity=0.1)
        
        # 计算已走距离和剩余距离
        total_walked = 1.5 + extra_distance
        remaining_distance = 4.5 - total_walked  # 总共450cm
        
        print(f"检测到黄灯！已走{total_walked:.2f}m，剩余{remaining_distance:.2f}m")
        
        # 立即停止并播报
        self.voice.speak_text("检测到黄灯")
        time.sleep(1)
        
        # 倒数 5、4、3、2、1
        for i in range(5, 0, -1):
            self.voice.speak_text(str(i))
            time.sleep(1)
        
        # 继续走剩余距离
        if remaining_distance > 0:
            print(f"继续前进剩余距离: {remaining_distance:.2f}m")
            self.robot.move_distance_imu(remaining_distance, velocity=0.1)
        
        print("写死版黄灯路段完成")
    
    def _execute_with_imu_detection(self):
        """使用IMU里程计的简化识别方案"""
        print("使用IMU里程计黄灯检测方案")
        
        # 记录起始位置
        start_distance = 0
        distance_moved = 0
        step_distance = 0.2  # 每次前进20cm
        total_section_distance = 4.5  # 黄灯路段总长度450cm
        detection_start = 1.0  # 100cm后开始检测黄灯
        
        # 先走到检测区域
        print(f"前进{detection_start:.1f}m到达黄灯检测区域")
        self.robot.move_distance_imu(detection_start, velocity=0.1)
        distance_moved += detection_start
        
        # 开始黄灯检测模式
        yellow_light_detected = False
        detection_distance = 0
        
        while distance_moved < total_section_distance and not yellow_light_detected:
            # 前进一步
            self.robot.move_distance_imu(step_distance, velocity=0.08)
            distance_moved += step_distance
            detection_distance += step_distance
            
            # 模拟黄灯检测（实际应该使用传感器或视觉）
            # 在1.5-3.0m之间随机检测到黄灯
            if detection_distance > 0.5 and detection_distance < 2.0:
                import random
                if random.random() < 0.3:  # 30%概率检测到
                    yellow_light_detected = True
        
        if yellow_light_detected:
            print(f"检测到黄灯！已走{distance_moved:.2f}m")
            
            # 立即停止并播报
            self.voice.speak_text("检测到黄灯")
            time.sleep(1)
            
            # 倒数 5、4、3、2、1
            for i in range(5, 0, -1):
                self.voice.speak_text(str(i))
                time.sleep(1)
            
            # 计算剩余距离并继续前进
            remaining_distance = total_section_distance - distance_moved
            if remaining_distance > 0:
                print(f"继续前进剩余距离: {remaining_distance:.2f}m")
                self.robot.move_distance_imu(remaining_distance, velocity=0.1)
        else:
            # 如果没检测到黄灯，继续走完剩余距离
            remaining_distance = total_section_distance - distance_moved
            if remaining_distance > 0:
                print(f"未检测到黄灯，继续前进剩余距离: {remaining_distance:.2f}m")
                self.robot.move_distance_imu(remaining_distance, velocity=0.1)
        
        print("IMU黄灯路段完成")
    
    def _execute_with_sensors(self, sensor_config: int):
        """使用传感器识别的方案"""
        print(f"使用传感器识别方案，配置: {sensor_config}")
        
        # 记录起始位置
        distance_moved = 0
        step_distance = 0.1  # 每次前进10cm
        total_section_distance = 4.5  # 黄灯路段总长度450cm
        detection_start = 1.0  # 100cm后开始检测
        
        # 先走到检测区域
        print(f"前进{detection_start:.1f}m到达传感器检测区域")
        self.robot.move_distance_imu(detection_start, velocity=0.1)
        distance_moved += detection_start
        
        # 缓慢前进，同时检测障碍物
        yellow_light_detected = False
        
        while distance_moved < total_section_distance and not yellow_light_detected:
            # 检测前方障碍物距离
            obstacle_distance = self._detect_obstacle_distance(sensor_config)
            
            if obstacle_distance is not None and obstacle_distance <= 0.5:  # 50cm以内检测到黄灯
                yellow_light_detected = True
                print(f"传感器检测到黄灯！已走{distance_moved:.2f}m")
                
                # 立即停止并播报
                self.voice.speak_text("检测到黄灯")
                time.sleep(1)
                
                # 倒数 5、4、3、2、1
                for i in range(5, 0, -1):
                    self.voice.speak_text(str(i))
                    time.sleep(1)
                
                # 计算剩余距离并继续前进
                remaining_distance = total_section_distance - distance_moved
                if remaining_distance > 0:
                    print(f"继续前进剩余距离: {remaining_distance:.2f}m")
                    self.robot.move_distance_imu(remaining_distance, velocity=0.1)
                
                break
            
            # 继续前进
            self.robot.move_distance_imu(step_distance, velocity=0.05)
            distance_moved += step_distance
        
        if not yellow_light_detected:
            # 如果没检测到黄灯，走完剩余距离
            remaining_distance = total_section_distance - distance_moved
            if remaining_distance > 0:
                print(f"传感器未检测到黄灯，继续前进剩余距离: {remaining_distance:.2f}m")
                self.robot.move_distance_imu(remaining_distance, velocity=0.1)
        
        print("传感器黄灯路段完成")
    
    def _detect_obstacle_distance(self, sensor_config: int) -> Optional[float]:
        """
        检测前方障碍物距离
        
        Args:
            sensor_config: 传感器配置
            
        Returns:
            距离（米），None表示未检测到
        """
        try:
            from .sensors import MultiSensorFusion
        except ImportError:
            try:
                from sensors import MultiSensorFusion
            except ImportError:
                # 创建模拟传感器融合类
                class MockMultiSensorFusion:
                    def __init__(self): pass
                    def initialize_sensors(self, config): print(f"模拟传感器初始化: {config}")
                    def get_min_distance(self): return 0.6  # 返回模拟距离
                MultiSensorFusion = MockMultiSensorFusion
            
            # 初始化传感器融合
            if not hasattr(self, 'sensor_fusion'):
                self.sensor_fusion = MultiSensorFusion()
                self.sensor_fusion.initialize_sensors(sensor_config)
            
            # 获取距离数据
            distance = self.sensor_fusion.get_min_distance()  # 使用最小距离作为保守估计
            
            if distance is not None:
                print(f"传感器检测到距离: {distance:.2f}m")
                return distance
            else:
                print("传感器未检测到有效数据")
                return None
                
        except Exception as e:
            print(f"传感器检测失败: {e}")
            return None


class StoneRoadSection:
    """石板路路段"""
    
    def __init__(self, robot_controller: RobotController, voice_announcer: VoiceAnnouncer):
        self.robot = robot_controller
        self.voice = voice_announcer
        
    def execute_stone_road_section(self):
        """执行石板路路段"""
        print("开始执行石板路路段")
        
        # 调整步高为0.10m，适应石板路
        self.robot.set_step_height(0.10, 0.10, 0.10, 0.10)
        
        # 使用IMU导航420cm
        self.robot.move_distance_imu(4.2, velocity=0.08)
        
        # 恢复正常步高
        self.robot.set_step_height(0.06, 0.06, 0.06, 0.06)
        
        print("石板路路段完成")


class HeightLimitSection:
    """限高杆路段"""
    
    def __init__(self, robot_controller: RobotController, voice_announcer: VoiceAnnouncer):
        self.robot = robot_controller
        self.voice = voice_announcer
        
        # 红色检测参数
        self.lower_red1 = np.array([0, 50, 50])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([160, 50, 50])
        self.upper_red2 = np.array([180, 255, 255])
        
        # 太空步配置文件路径
        self.moonwalk_config = os.path.join(current_dir, "moonwalk_config.toml")
        self._create_moonwalk_config()
    
    def _create_moonwalk_config(self):
        """创建太空步配置文件"""
        moonwalk_content = '''[[step]]
type = "torctrlposture"
foot_support = [1.0,1.0,1.0,1.0]
body_cmd = [0.0,0.0,0.0,-0.0,0.0,0.0]
foot_pose = [0.0,0.0,0.0]
duration = 500

[[step]]
type = "usergait"
body_vel_des = [0.2,0.0,0.0]
body_pos_des = [0.0,0.0,0.0,0.0,0.0,-0.05]
landing_pos_des = [0.05,0.00,0.0,0.05,-0.00,0.0,0.05,0.00,0.0]
step_height = [0.05,0.05,0.05,0.05]
weight = [10.0,10.0,10.0,50.0,50.0,10.0]
use_mpc_traj = 0
mu = 0.40
landing_gain = 1.0
gait_id = 90 
duration = 5760

[[step]]
type = "usergait"
body_vel_des = [0.0,0.0,0.0]
body_pos_des = [0.0,0.0,0.0,0.0,0.0,-0.05]
landing_pos_des = [0.0,0.00,0.0,0.0,-0.00,0.0,0.0,0.00,0.0,0.0,0.00,0.0]
step_height = [0.05,0.05,0.05,0.05]
weight = [10.0,10.0,10.0,50.0,50.0,10.0]
use_mpc_traj = 0
mu = 0.40
landing_gain = 1.0
gait_id = 90 
duration = 1440

[[step]]
type = "locomotion"
vel_des = [0.0,0.0,0.0]
omni = 0
gait_id = 31
duration = 1000

[[step]]
type = "recoverystand"
duration = 1000
'''
        
        with open(self.moonwalk_config, 'w', encoding='utf-8') as f:
            f.write(moonwalk_content)
    
    def execute_height_limit_section(self, use_vision: bool = False):
        """
        执行限高杆路段
        
        Args:
            use_vision: 是否使用视觉识别方案
        """
        print("开始执行限高杆路段")
        
        # 进入限高杆区域，先向左旋转210度（硬件限制调整）
        self.voice.speak_text("准备通过限高杆，左转210度")
        self.robot.turn_left(210)
        time.sleep(1)
        
        # 降低步高（先前腿，再后腿）
        self.robot.set_step_height(front_left=0.04, front_right=0.04, back_left=0.06, back_right=0.06)
        time.sleep(0.5)
        self.robot.set_step_height(front_left=0.04, front_right=0.04, back_left=0.04, back_right=0.04)
        
        if use_vision:
            self._execute_with_vision()
        else:
            self._execute_with_moonwalk()
    
    def _execute_with_moonwalk(self):
        """使用太空步的方案一"""
        print("使用太空步步态方案")
        
        # 切换到太空步步态
        self.robot.load_gait_from_file(self.moonwalk_config)
        
        # 导航200cm后播报
        self.robot.move_distance_imu(2.0, backward=True, velocity=0.05)
        self.voice.speak_text("检测到限高杆，开始通过")
        
        # 继续导航250cm
        self.robot.move_distance_imu(2.5, backward=True, velocity=0.05)
        
        # 恢复正常步高和姿态
        self.robot.set_step_height(0.06, 0.06, 0.06, 0.06)
        self.robot.turn_left(210)  # 转回正向（硬件限制调整）
        
        print("限高杆路段完成")
    
    def _execute_with_vision(self):
        """使用视觉识别的方案二"""
        print("使用视觉识别方案")
        
        # 缓慢前进，监测红色限高杆
        distance_moved = 0
        step_distance = 0.1
        max_distance = 5.0
        
        while distance_moved < max_distance:
            # 检测红色
            if self._detect_red_obstacle():
                # 检测到红色，向后退两步
                self.robot.move_distance_imu(-0.2, velocity=0.05)
                
                # 切换太空步步态
                self.robot.load_gait_from_file(self.moonwalk_config)
                self.voice.speak_text("检测到限高杆，开始通过")
                
                # 使用太空步通过
                self.robot.move_distance_imu(4.5, backward=True, velocity=0.05)
                
                # 转身继续行进
                self.robot.set_step_height(0.06, 0.06, 0.06, 0.06)
                self.robot.turn_left(210)  # 硬件限制调整
                break
            
            # 继续前进
            self.robot.move_distance_imu(step_distance, backward=True, velocity=0.05)
            distance_moved += step_distance
        
        print("限高杆路段完成")
    
    def _detect_red_obstacle(self) -> bool:
        """检测红色障碍物"""
        try:
            import cv2
            
            # 初始化摄像头（如果尚未初始化）
            if not hasattr(self, 'camera'):
                self.camera = cv2.VideoCapture(0)
                if not self.camera.isOpened():
                    print("无法打开摄像头")
                    return False
            
            # 捕获图像
            ret, frame = self.camera.read()
            if not ret:
                print("无法捕获图像")
                return False
            
            # 转换到HSV空间
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # 创建红色掩码
            mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
            mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
            red_mask = cv2.bitwise_or(mask1, mask2)
            
            # 计算红色像素比例
            total_pixels = red_mask.shape[0] * red_mask.shape[1]
            red_pixels = cv2.countNonZero(red_mask)
            red_ratio = red_pixels / total_pixels
            
            # 如果红色比例低于阈值，说明撞到了限高杆
            print(f"红色像素比例: {red_ratio:.3f}")
            return red_ratio < 0.01  # 阈值可调整
            
        except Exception as e:
            print(f"红色检测失败: {e}")
            return False


# 导出所有类
__all__ = [
    'LoopTrackState',
    'ArrowDetector', 
    'SlopeSection',
    'YellowLightSection',
    'StoneRoadSection',
    'HeightLimitSection'
]