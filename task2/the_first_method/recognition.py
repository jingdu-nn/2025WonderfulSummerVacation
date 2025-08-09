import streamlit as st
import requests
import os
import base64
import cv2
import tempfile
import uuid
from PIL import Image, ImageDraw, ImageFont
import numpy as np
import time
import json

# 豆包 API 配置
ARK_API_KEY = "YOUR_API_KEY_HERE"
API_URL = "https://ark.cn-beijing.volces.com/api/v3/chat/completions"

st.set_page_config(page_title="增强车牌识别助手", page_icon="🚗", layout="centered")
st.title("🚗 增强车牌识别助手")

option = st.radio("请选择识别类型：", ["上传图片", "上传视频（实时识别）"])

def send_image_to_doubao_with_detection(image_bytes, mime_type="image/jpeg"):
    """调用豆包API进行车牌检测和识别"""
    base64_image = base64.b64encode(image_bytes).decode("utf-8")
    image_data_url = f"data:{mime_type};base64,{base64_image}"
    
    payload = {
        "model": "doubao-1-5-vision-pro-32k-250115",
        "messages": [
            {
                "role": "user",
                "content": [
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": image_data_url
                        }
                    },
                    {
                        "type": "text",
                        "text": "请仔细识别图片中的车牌号。车牌通常是蓝色或绿色的矩形区域，请告诉我：1.车牌号码 2.车牌在图片中的精确位置（左上角和右下角坐标）。格式：车牌号：XXX，位置：(x1,y1)-(x2,y2)。特别注意区分字母D和数字0。"
                    }
                ]
            }
        ]
    }
    
    headers = {
        "Content-Type": "application/json",
        "Authorization": f"Bearer {ARK_API_KEY}"
    }
    
    response = requests.post(API_URL, headers=headers, json=payload)
    response.raise_for_status()
    result = response.json()
    
    if "choices" in result and len(result["choices"]) > 0:
        content = result["choices"][0]["message"]["content"].strip()
        
        # 尝试解析位置信息
        bbox = None
        plate_number = ""
        
        # 提取车牌号
        if "车牌号：" in content:
            plate_part = content.split("车牌号：")[1].split("，")[0]
            plate_number = ''.join(filter(str.isalnum, plate_part))
        
        # 尝试提取坐标信息
        if "位置：" in content:
            try:
                pos_part = content.split("位置：")[1]
                # 查找坐标数字，支持多种格式
                import re
                # 尝试匹配 (x1,y1)-(x2,y2) 格式
                coord_pattern = r'\((\d+),(\d+)\)-\((\d+),(\d+)\)'
                match = re.search(coord_pattern, pos_part)
                if match:
                    x1, y1, x2, y2 = int(match.group(1)), int(match.group(2)), int(match.group(3)), int(match.group(4))
                    bbox = [x1, y1, x2, y2]
                else:
                    # 尝试匹配其他格式的数字
                    coords = re.findall(r'\d+', pos_part)
                    if len(coords) >= 4:
                        x1, y1, x2, y2 = int(coords[0]), int(coords[1]), int(coords[2]), int(coords[3])
                        bbox = [x1, y1, x2, y2]
            except:
                pass
        
        # 如果没有找到位置信息，尝试从整个文本中提取车牌号
        if not plate_number:
            plate_number = ''.join(filter(str.isalnum, content))
        
        return {"plate_number": plate_number, "bbox": bbox}
    else:
        return None

def draw_plate_on_image(image, detection_results):
    """在图片上绘制车牌检测结果"""
    if isinstance(image, np.ndarray):
        # OpenCV格式转换为PIL
        image = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    
    draw = ImageDraw.Draw(image)
    
    # 尝试加载中文字体
    try:
        font = ImageFont.truetype("simhei.ttf", 24)  # Windows
    except:
        try:
            font = ImageFont.truetype("/System/Library/Fonts/PingFang.ttc", 24)  # macOS
        except:
            try:
                font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 24)  # Linux
            except:
                font = ImageFont.load_default()
    
    if isinstance(detection_results, dict):
        detection_results = [detection_results]
    
    for result in detection_results:
        if isinstance(result, dict) and "plate_number" in result:
            plate_number = result["plate_number"]
            bbox = result.get("bbox")
            
            if bbox and len(bbox) == 4:
                # 绘制边界框
                x1, y1, x2, y2 = bbox
                draw.rectangle([x1, y1, x2, y2], outline="red", width=3)
                
                # 在车牌上方显示车牌号
                text_bbox = draw.textbbox((x1, y1 - 30), plate_number, font=font)
                draw.rectangle([text_bbox[0]-5, text_bbox[1]-5, text_bbox[2]+5, text_bbox[3]+5], 
                             fill="red", outline="red")
                draw.text((x1, y1 - 30), plate_number, fill="white", font=font)
            else:
                # 如果没有位置信息，在图片中央显示车牌号
                img_width, img_height = image.size
                text_bbox = draw.textbbox((0, 0), plate_number, font=font)
                text_width = text_bbox[2] - text_bbox[0]
                text_height = text_bbox[3] - text_bbox[1]
                
                x = (img_width - text_width) // 2
                y = 20
                
                # 绘制背景
                draw.rectangle([x-10, y-10, x+text_width+10, y+text_height+10], 
                             fill="red", outline="red")
                draw.text((x, y), plate_number, fill="white", font=font)
        elif isinstance(result, str):
            # 如果直接是字符串，在中央显示
            img_width, img_height = image.size
            text_bbox = draw.textbbox((0, 0), result, font=font)
            text_width = text_bbox[2] - text_bbox[0]
            text_height = text_bbox[3] - text_bbox[1]
            
            x = (img_width - text_width) // 2
            y = 20
            
            # 绘制背景
            draw.rectangle([x-10, y-10, x+text_width+10, y+text_height+10], 
                         fill="red", outline="red")
            draw.text((x, y), result, fill="white", font=font)
    
    return image

def process_image_with_detection(image_bytes):
    """处理图片并返回检测结果"""
    # 将图片字节转换为PIL Image并确保尺寸足够大
    with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as temp_file:
        temp_file.write(image_bytes)
        temp_file_path = temp_file.name
    
    image = Image.open(temp_file_path)
    
    # 确保图片尺寸足够大（至少14x14像素）
    if image.size[0] < 14 or image.size[1] < 14:
        # 如果图片太小，放大到至少100x100
        new_size = (max(100, image.size[0]), max(100, image.size[1]))
        image = image.resize(new_size, Image.Resampling.LANCZOS)
        
        # 重新保存放大后的图片
        with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as temp_file_resized:
            image.save(temp_file_resized.name, "JPEG", quality=95)
            temp_file_resized_path = temp_file_resized.name
        
        # 读取放大后的图片字节
        with open(temp_file_resized_path, "rb") as f:
            image_bytes = f.read()
        
        # 清理临时文件
        try:
            os.remove(temp_file_path)
            os.remove(temp_file_resized_path)
        except:
            pass  # 忽略删除失败
    else:
        # 清理临时文件
        try:
            os.remove(temp_file_path)
        except:
            pass  # 忽略删除失败
    
    # 调用API进行检测
    detection_results = send_image_to_doubao_with_detection(image_bytes)
    
    if not detection_results:
        return None, None
    
    # 绘制检测结果
    annotated_image = draw_plate_on_image(image, detection_results)
    
    return detection_results, annotated_image

# 图片识别流程
if option == "上传图片":
    uploaded_file = st.file_uploader("请选择一张车辆图片", type=["jpg", "jpeg", "png"])
    if uploaded_file is not None:
        # 显示原图
        col1, col2 = st.columns(2)
        with col1:
            st.image(uploaded_file, caption="原始图片", use_column_width=True)
        
        if st.button("开始识别"):
            with st.spinner("识别中..."):
                try:
                    # 处理图片
                    detection_results, annotated_image = process_image_with_detection(uploaded_file.read())
                    
                    if detection_results:
                        with col2:
                            st.image(annotated_image, caption="检测结果", use_column_width=True)
                        
                        # 显示检测结果
                        st.subheader("检测结果")
                        if isinstance(detection_results, list):
                            for i, result in enumerate(detection_results):
                                st.write(f"**车牌 {i+1}**: {result.get('plate_number', '未知')}")
                                if result.get('bbox'):
                                    st.write(f"**位置**: {result['bbox']}")
                        else:
                            st.write(f"**车牌号**: {detection_results.get('plate_number', '未知')}")
                            if detection_results.get('bbox'):
                                st.write(f"**位置**: {detection_results['bbox']}")
                    else:
                        st.warning("未能识别车牌号")
                except Exception as e:
                    st.error(f"识别失败：{e}")

# 视频实时识别流程
elif option == "上传视频（实时识别）":
    video_file = st.file_uploader("上传一段车辆视频", type=["mp4", "mov", "avi"])
    if video_file is not None:
        st.video(video_file)
        if st.button("实时识别并展示"):
            with st.spinner("实时识别中..."):
                try:
                    temp_video = tempfile.NamedTemporaryFile(delete=False, suffix=".mp4")
                    temp_video.write(video_file.read())
                    temp_video.close()
                    
                    cap = cv2.VideoCapture(temp_video.name)
                    frame_rate = cap.get(cv2.CAP_PROP_FPS)
                    interval = max(1, int(frame_rate))  # 每秒识别一帧
                    frame_count = 0
                    last_detection = None
                    placeholder = st.empty()
                    
                    while True:
                        ret, frame = cap.read()
                        if not ret:
                            break
                        
                        if frame_count % interval == 0:
                            # 取当前帧做识别
                            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                            image_pil = Image.fromarray(frame_rgb)
                            
                            # 压缩图片以加快API响应
                            with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as temp_img:
                                image_pil.save(temp_img.name, quality=80)
                                with open(temp_img.name, "rb") as f:
                                    img_bytes = f.read()
                            
                            try:
                                detection_results = send_image_to_doubao_with_detection(img_bytes)
                                if detection_results:
                                    last_detection = detection_results
                            except Exception as e:
                                pass  # 忽略单帧识别失败
                            
                            # 删除临时图片
                            os.remove(temp_img.name)
                        
                        # 在帧上绘制检测结果
                        display_frame = frame_rgb.copy()
                        if last_detection:
                            annotated_frame = draw_plate_on_image(display_frame, last_detection)
                            display_frame = np.array(annotated_frame)
                        
                        # 展示当前帧
                        placeholder.image(display_frame, channels="RGB")
                        frame_count += 1
                        
                        # 控制播放速度
                        time.sleep(1.0 / max(10, frame_rate))
                    
                    cap.release()
                    os.remove(temp_video.name)
                except Exception as e:
                    st.error(f"视频处理或识别失败：{e}")

# 添加说明
st.markdown("---")
st.markdown("""
### 功能说明
- **车牌检测**: 自动检测图片中的车牌位置
- **车牌识别**: 识别车牌号码
- **可视化**: 在图片上绘制边界框和车牌号
- **实时处理**: 支持视频实时识别

### 注意事项
- 检测结果包含车牌位置信息（边界框坐标）
- 支持多车牌检测
- 车牌号显示在车牌上方
""") 