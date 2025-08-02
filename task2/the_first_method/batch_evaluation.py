import streamlit as st
import requests
import os
import base64
import time
import random
import json
from datetime import datetime
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

# 豆包 API 配置
ARK_API_KEY = "YOUR_API_KEY_HERE"
API_URL = "https://ark.cn-beijing.volces.com/api/v3/chat/completions"

# 车牌字符映射表
provinces = ["皖", "沪", "津", "渝", "冀", "晋", "蒙", "辽", "吉", "黑", "苏", "浙", "京", "闽", "赣", "鲁", "豫", "鄂", "湘", "粤", "桂", "琼", "川", "贵", "云", "藏", "陕", "甘", "青", "宁", "新", "警", "学", "O"]
alphabets = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'J', 'K', 'L', 'M', 'N', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', 'O']
ads = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'J', 'K', 'L', 'M', 'N', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'O']

def parse_filename(filename):
    """解析文件名获取真实车牌号"""
    name = filename.split('.')[0]
    parts = name.split('-')
    
    # 查找包含车牌字符索引的部分
    for part in parts:
        if '_' in part and len(part.split('_')) >= 7:
            subparts = part.split('_')
            if len(subparts) >= 7:
                try:
                    # 尝试解析所有可能的数字作为车牌字符索引
                    numbers = []
                    for subpart in subparts:
                        try:
                            num = int(subpart)
                            numbers.append(num)
                        except ValueError:
                            break
                    
                    # 如果找到足够的数字，尝试解析车牌
                    if len(numbers) >= 7:
                        # 尝试8个字符的情况（如果有8个数字）
                        if len(numbers) >= 8:
                            try:
                                province = provinces[numbers[0]] if numbers[0] < len(provinces) else "O"
                                alphabet = alphabets[numbers[1]] if numbers[1] < len(alphabets) else "O"
                                ad_chars = [ads[idx] if idx < len(ads) else "O" for idx in numbers[2:8]]
                                plate_number = province + alphabet + ''.join(ad_chars)
                                return plate_number
                            except (IndexError, ValueError):
                                pass
                        
                        # 尝试7个字符的情况
                        try:
                            province = provinces[numbers[0]] if numbers[0] < len(provinces) else "O"
                            alphabet = alphabets[numbers[1]] if numbers[1] < len(alphabets) else "O"
                            ad_chars = [ads[idx] if idx < len(ads) else "O" for idx in numbers[2:7]]
                            plate_number = province + alphabet + ''.join(ad_chars)
                            return plate_number
                        except (IndexError, ValueError):
                            pass
                            
                except (ValueError, IndexError):
                    continue
    return None

def send_image_to_doubao(image_bytes, mime_type="image/jpeg"):
    """调用豆包API识别车牌"""
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
                        "text": "请识别图片中的车牌号，只返回车牌号码的字符，不要任何分隔符、空格、标点符号或其他内容。特别注意区分字母D和数字0，字母D有横线，数字0是圆形。例如：皖AD41988，不要写成皖A-D41988或皖A·D02108。"
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
        return result["choices"][0]["message"]["content"].strip()
    else:
        return None

def calculate_accuracy(predicted, actual):
    """计算车牌识别准确率"""
    if not predicted or not actual:
        return 0.0
    
    # 去除所有非字母数字字符（包括分隔符、空格、标点等）
    predicted_clean = ''.join(filter(str.isalnum, predicted))
    actual_clean = ''.join(filter(str.isalnum, actual))
    
    # 如果长度不匹配，返回0
    if len(predicted_clean) != len(actual_clean):
        return 0.0
    
    # 严格匹配：只要有一个字符不对就返回0
    if predicted_clean == actual_clean:
        return 1.0
    else:
        return 0.0

def evaluate_performance(sample_size=500):
    """执行性能评估"""
    test_folder = "CCPD2020/CCPD2020/ccpd_green/test"
    
    if not os.path.exists(test_folder):
        st.error(f"测试文件夹不存在: {test_folder}")
        return None
    
    # 获取所有图片文件
    image_files = [f for f in os.listdir(test_folder) if f.lower().endswith(('.jpg', '.jpeg', '.png'))]
    
    if len(image_files) < sample_size:
        st.warning(f"测试文件夹中只有 {len(image_files)} 张图片，少于{sample_size}张")
        sample_size = len(image_files)
    
    # 随机选择图片
    selected_files = random.sample(image_files, sample_size)
    
    st.info(f"开始评估 {sample_size} 张图片的性能...")
    
    results = []
    progress_bar = st.progress(0)
    status_text = st.empty()
    
    for i, filename in enumerate(selected_files):
        status_text.text(f"处理第 {i+1}/{sample_size} 张图片: {filename}")
        
        # 解析真实车牌号
        actual_plate = parse_filename(filename)
        
        if not actual_plate:
            continue
        
        # 读取图片
        image_path = os.path.join(test_folder, filename)
        with open(image_path, 'rb') as f:
            image_bytes = f.read()
        
        # 记录开始时间
        start_time = time.time()
        
        try:
            # 调用API识别
            predicted_plate = send_image_to_doubao(image_bytes)
            
            # 记录结束时间
            end_time = time.time()
            response_time = end_time - start_time
            
            # 计算准确率
            accuracy = calculate_accuracy(predicted_plate, actual_plate)
            predicted_clean = ''.join(filter(str.isalnum, predicted_plate))
            
            results.append({
                'filename': filename,
                'actual_plate': actual_plate,
                'predicted_plate': predicted_plate,
                'predicted_clean': predicted_clean,
                'accuracy': accuracy,
                'response_time': response_time,
                'is_correct': accuracy == 1.0,
                'success': True
            })
            
        except Exception as e:
            results.append({
                'filename': filename,
                'actual_plate': actual_plate,
                'predicted_plate': None,
                'accuracy': 0.0,
                'response_time': time.time() - start_time,
                'is_correct': False,
                'success': False,
                'error': str(e)
            })
        
        # 更新进度条
        progress_bar.progress((i + 1) / sample_size)
        
        # 添加延迟避免API限制
        time.sleep(0.1)
    
    return results

def display_results(results):
    """显示评估结果"""
    if not results:
        st.error("没有评估结果")
        return
    
    df = pd.DataFrame(results)
    
    # 基本统计
    total_images = len(df)
    successful_recognitions = len(df[df['success'] == True])
    correct_recognitions = len(df[df['is_correct'] == True])
    
    avg_response_time = df['response_time'].mean()
    avg_accuracy = df['accuracy'].mean()
    
    # 显示统计信息
    col1, col2, col3, col4 = st.columns(4)
    
    with col1:
        st.metric("总图片数", total_images)
    
    with col2:
        st.metric("成功识别数", successful_recognitions, f"{successful_recognitions/total_images*100:.1f}%")
    
    with col3:
        st.metric("完全正确数", correct_recognitions, f"{correct_recognitions/total_images*100:.1f}%")
    
    with col4:
        st.metric("严格匹配率", f"{correct_recognitions/total_images*100:.1f}%")
    
    # 响应时间统计
    st.subheader("响应时间分析")
    col1, col2 = st.columns(2)
    
    with col1:
        fig, ax = plt.subplots(figsize=(8, 6))
        ax.hist(df['response_time'], bins=30, alpha=0.7, color='skyblue', edgecolor='black')
        ax.set_xlabel('响应时间 (秒)', fontsize=12)
        ax.set_ylabel('频次', fontsize=12)
        ax.set_title('响应时间分布', fontsize=14, fontweight='bold')
        plt.tight_layout()
        st.pyplot(fig)
    
    with col2:
        st.write("响应时间统计:")
        st.write(f"- 平均响应时间: {avg_response_time:.3f} 秒")
        st.write(f"- 最小响应时间: {df['response_time'].min():.3f} 秒")
        st.write(f"- 最大响应时间: {df['response_time'].max():.3f} 秒")
        st.write(f"- 响应时间标准差: {df['response_time'].std():.3f} 秒")
    
    # 准确率分析
    st.subheader("识别结果分析")
    col1, col2 = st.columns(2)
    
    with col1:
        # 创建饼图显示正确和错误的比例
        correct_count = len(df[df['accuracy'] == 1.0])
        incorrect_count = len(df[df['accuracy'] == 0.0])
        
        fig, ax = plt.subplots(figsize=(8, 6))
        labels = ['完全正确', '有错误']
        sizes = [correct_count, incorrect_count]
        colors = ['lightgreen', 'lightcoral']
        
        wedges, texts, autotexts = ax.pie(sizes, labels=labels, colors=colors, autopct='%1.1f%%', startangle=90)
        ax.set_title('识别结果分布', fontsize=14, fontweight='bold')
        
        # 设置文本字体
        for text in texts:
            text.set_fontsize(12)
        for autotext in autotexts:
            autotext.set_fontsize(11)
            autotext.set_color('white')
            autotext.set_fontweight('bold')
        
        plt.tight_layout()
        st.pyplot(fig)
    
    with col2:
        st.write("识别结果统计:")
        st.write(f"- 完全正确: {correct_count} 张 ({correct_count/total_images*100:.1f}%)")
        st.write(f"- 有错误: {incorrect_count} 张 ({incorrect_count/total_images*100:.1f}%)")
        st.write(f"- 严格匹配率: {correct_count/total_images*100:.1f}%")
    
    # 错误案例分析
    st.subheader("错误案例分析")
    error_cases = df[df['accuracy'] < 1.0].head(10)
    
    if not error_cases.empty:
        st.write("部分错误识别案例:")
        for _, row in error_cases.iterrows():
            st.write(f"**文件**: {row['filename']}")
            st.write(f"**真实车牌**: {row['actual_plate']}")
            st.write(f"**识别结果**: {row['predicted_plate']}")
            if 'predicted_clean' in row:
                st.write(f"**清理后**: {row['predicted_clean']}")
            st.write(f"**准确率**: {row['accuracy']*100:.1f}%")
            st.write("---")
    
    # 保存结果
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    results_file = f"evaluation_results_{timestamp}.json"
    
    with open(results_file, 'w', encoding='utf-8') as f:
        json.dump(results, f, ensure_ascii=False, indent=2)
    
    st.success(f"评估结果已保存到: {results_file}")

# Streamlit 界面
st.set_page_config(page_title="豆包车牌识别性能评估", page_icon="📊", layout="wide")
st.title("📊 豆包车牌识别性能评估")

st.write("本工具将评估豆包视觉大模型在CCPD2020数据集上的车牌识别性能。")

# 参数设置
col1, col2 = st.columns(2)
with col1:
    sample_size = st.number_input("测试图片数量", min_value=10, max_value=1000, value=500, step=10)
with col2:
    st.write("建议测试数量：")
    st.write("- 快速测试：50-100张")
    st.write("- 标准测试：500张")
    st.write("- 完整测试：1000张")

if st.button("开始性能评估"):
    if ARK_API_KEY == "YOUR_API_KEY_HERE" or not ARK_API_KEY:
        st.error("请先配置豆包API密钥")
    else:
        with st.spinner("正在评估性能..."):
            results = evaluate_performance(sample_size)
            if results:
                display_results(results) 