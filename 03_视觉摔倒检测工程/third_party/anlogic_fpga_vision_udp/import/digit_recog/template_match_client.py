#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#****************************************Copyright (c)***********************************#
#技术支持：www.openedv.com
#淘宝店铺：http://openedv.taobao.com
#关注微信公众平台微信号："正点原子"，免费获取FPGA & STM32资料。
#版权所有，盗版必究。
#Copyright(C) 正点原子 2018-2028
#All rights reserved
#----------------------------------------------------------------------------------------
# File name:           template_match_client.py
# Last modified Date:  2025/01/05
# Last Version:        V1.0
# Descriptions:        32×32二值化图像模板匹配上位机程序
#----------------------------------------------------------------------------------------
# Modified by:         Claude
# Modified date:       2025/01/05
# Version:             V1.0
# Descriptions:        Python client for template matching system
#                      - Load and preprocess 32×32 images
#                      - Send template and test images via UDP
#                      - Receive and display matching results
#
# Dependencies:
#   pip install pillow numpy
#
# Usage:
#   python template_match_client.py --template template.png --test test.png
#****************************************************************************************#

import socket
import struct
import numpy as np
from PIL import Image
import argparse
import time

# UDP命令码定义
CMD_UPLOAD_TEMPLATE = 0x00F0  # 上传模板图像
CMD_UPLOAD_TEST     = 0x00F1  # 上传测试图像
CMD_SET_THRESHOLD   = 0x00F2  # 设置阈值
CMD_START_MATCH     = 0x00F3  # 开始匹配
CMD_READ_RESULT     = 0x00F4  # 读取结果

# FPGA IP地址和端口配置
FPGA_IP   = "192.168.1.10"    # 修改为你的FPGA IP地址
FPGA_PORT = 8080              # 修改为你的FPGA端口

# 图像参数
IMG_WIDTH  = 32
IMG_HEIGHT = 32
IMG_BYTES  = 128              # 32×32 / 8 = 128 bytes


def load_and_binarize_image(image_path, threshold=128):
    """
    加载图像并转换为32×32二值化图像

    Args:
        image_path: 图像文件路径
        threshold: 二值化阈值 (0-255)

    Returns:
        numpy array: 32×32二值化数组 (1=白, 0=黑)
    """
    print(f"[INFO] 加载图像: {image_path}")

    # 读取图像并转换为灰度
    img = Image.open(image_path).convert('L')

    # 缩放到32×32
    img = img.resize((IMG_WIDTH, IMG_HEIGHT), Image.LANCZOS)

    # 转换为numpy数组
    img_array = np.array(img)

    # 二值化: > threshold = 1 (白), <= threshold = 0 (黑)
    binary = (img_array > threshold).astype(np.uint8)

    print(f"[INFO] 图像二值化完成: {np.sum(binary)} 个白色像素, {IMG_WIDTH*IMG_HEIGHT - np.sum(binary)} 个黑色像素")

    return binary


def pack_binary_image(binary_array):
    """
    将32×32二值化数组打包为128字节

    Args:
        binary_array: 32×32 numpy数组 (0或1)

    Returns:
        bytes: 128字节数据
    """
    data = bytearray()

    # 按行扫描，每8个像素打包为1字节 (MSB first)
    for row in binary_array:
        for col_start in range(0, IMG_WIDTH, 8):
            byte_val = 0
            for bit_idx in range(8):
                pixel = row[col_start + bit_idx]
                byte_val |= (pixel << (7 - bit_idx))
            data.append(byte_val)

    return bytes(data)


def send_udp_command(sock, cmd, data=b''):
    """
    发送UDP命令到FPGA

    Args:
        sock: socket对象
        cmd: 命令码 (16位)
        data: 数据负载 (bytes)
    """
    # 构造数据包: [命令码(2字节)] + [数据]
    packet = struct.pack('>H', cmd) + data

    print(f"[UDP] 发送命令 0x{cmd:04X}, 数据长度: {len(data)} 字节")
    sock.sendto(packet, (FPGA_IP, FPGA_PORT))


def receive_udp_result(sock, timeout=5.0):
    """
    接收UDP匹配结果

    Args:
        sock: socket对象
        timeout: 超时时间（秒）

    Returns:
        tuple: (match_result, hamming_distance)
    """
    sock.settimeout(timeout)

    try:
        data, addr = sock.recvfrom(16)

        if len(data) >= 1:
            result_byte = data[0]
            match_result = (result_byte & 0x80) >> 7  # bit 7
            hamming_dist = ((result_byte & 0x7E) >> 1) << 4  # bits 6:1 (高6位)

            print(f"[UDP] 接收结果: 匹配={'成功' if match_result else '失败'}, 汉明距离≈{hamming_dist}")

            return match_result, hamming_dist
        else:
            print("[ERROR] 接收到的数据长度不足")
            return None, None

    except socket.timeout:
        print("[ERROR] 接收结果超时")
        return None, None


def template_match_test(template_path, test_path, threshold=50, binary_threshold=128):
    """
    完整的模板匹配测试流程

    Args:
        template_path: 模板图像路径
        test_path: 测试图像路径
        threshold: 匹配阈值（不同像素数量阈值）
        binary_threshold: 图像二值化阈值
    """
    print("="*60)
    print("  32×32二值化图像模板匹配测试程序")
    print("="*60)

    # 创建UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        # 步骤1: 加载并打包模板图像
        print("\n[步骤1] 加载模板图像...")
        template_binary = load_and_binarize_image(template_path, binary_threshold)
        template_data = pack_binary_image(template_binary)

        # 步骤2: 发送模板图像
        print("\n[步骤2] 上传模板图像到FPGA...")
        send_udp_command(sock, CMD_UPLOAD_TEMPLATE, template_data)
        time.sleep(0.1)  # 等待FPGA处理

        # 步骤3: 加载并打包测试图像
        print("\n[步骤3] 加载测试图像...")
        test_binary = load_and_binarize_image(test_path, binary_threshold)
        test_data = pack_binary_image(test_binary)

        # 步骤4: 发送测试图像
        print("\n[步骤4] 上传测试图像到FPGA...")
        send_udp_command(sock, CMD_UPLOAD_TEST, test_data)
        time.sleep(0.1)

        # 步骤5: 设置匹配阈值
        print(f"\n[步骤5] 设置匹配阈值: {threshold} 像素...")
        threshold_data = struct.pack('>H', threshold)  # 大端序，2字节
        send_udp_command(sock, CMD_SET_THRESHOLD, threshold_data)
        time.sleep(0.1)

        # 步骤6: 开始匹配
        print("\n[步骤6] 开始匹配...")
        send_udp_command(sock, CMD_START_MATCH)

        # 步骤7: 接收结果
        print("\n[步骤7] 等待匹配结果...")
        match_result, hamming_dist = receive_udp_result(sock)

        # 步骤8: 显示结果
        print("\n" + "="*60)
        print("  匹配结果")
        print("="*60)
        if match_result is not None:
            print(f"  模板图像: {template_path}")
            print(f"  测试图像: {test_path}")
            print(f"  匹配结果: {'✓ 匹配成功' if match_result else '✗ 匹配失败'}")
            print(f"  汉明距离: ≈{hamming_dist} 像素 (阈值: {threshold})")
            print("="*60)
        else:
            print("  [ERROR] 未能获取匹配结果")
            print("="*60)

    except FileNotFoundError as e:
        print(f"[ERROR] 文件未找到: {e}")
    except Exception as e:
        print(f"[ERROR] 发生错误: {e}")
    finally:
        sock.close()
        print("\n[INFO] 测试完成，连接已关闭")


def visualize_binary_image(binary_array, title="Binary Image"):
    """
    可视化二值化图像（可选，需要matplotlib）

    Args:
        binary_array: 32×32二值化数组
        title: 图像标题
    """
    try:
        import matplotlib.pyplot as plt

        plt.figure(figsize=(4, 4))
        plt.imshow(binary_array, cmap='gray', vmin=0, vmax=1)
        plt.title(title)
        plt.axis('off')
        plt.tight_layout()
        plt.show()
    except ImportError:
        print("[WARN] matplotlib未安装，跳过图像可视化")


if __name__ == "__main__":
    # 命令行参数解析
    parser = argparse.ArgumentParser(description='32×32二值化图像模板匹配测试程序')
    parser.add_argument('--template', '-t', required=True, help='模板图像路径')
    parser.add_argument('--test', '-s', required=True, help='测试图像路径')
    parser.add_argument('--threshold', '-th', type=int, default=50, help='匹配阈值（不同像素数量，默认50）')
    parser.add_argument('--binary-threshold', '-bt', type=int, default=128, help='图像二值化阈值（0-255，默认128）')
    parser.add_argument('--ip', default="192.168.1.10", help='FPGA IP地址')
    parser.add_argument('--port', type=int, default=8080, help='FPGA端口')
    parser.add_argument('--visualize', '-v', action='store_true', help='可视化二值化图像')

    args = parser.parse_args()

    # 更新FPGA地址
    FPGA_IP = args.ip
    FPGA_PORT = args.port

    # 如果需要可视化
    if args.visualize:
        print("[INFO] 可视化模式开启...")
        template_binary = load_and_binarize_image(args.template, args.binary_threshold)
        test_binary = load_and_binarize_image(args.test, args.binary_threshold)
        visualize_binary_image(template_binary, "Template Image")
        visualize_binary_image(test_binary, "Test Image")

    # 执行匹配测试
    template_match_test(
        args.template,
        args.test,
        args.threshold,
        args.binary_threshold
    )
