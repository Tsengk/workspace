#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
支持通配符匹配的 ROS Bag 合并工具 (Python 2 兼容版).
适用于 ROS Melodic/Kinetic 等默认使用 Python 2 的环境.
用法: python merge_bags_py2.py -o merged.bag path/rosbag*.bag
"""

import sys
import argparse
import glob
import rosbag

def expand_paths(patterns):
    """
    将包含通配符的路径模式扩展为实际的文件路径列表.
    """
    expanded_files = []
    for pattern in patterns:
        # 使用 glob 模块查找匹配的文件
        matches = glob.glob(pattern)
        if not matches:
            print("警告: 未找到匹配模式 '{}' 的文件".format(pattern))
        else:
            # 对匹配到的文件进行排序，确保按文件名顺序处理
            matches.sort()
            expanded_files.extend(matches)
            print("模式 '{}' 匹配到 {} 个文件".format(pattern, len(matches)))
    
    return expanded_files

def merge_bags(output_bag_path, input_patterns):
    """
    合并多个通过通配符指定的 bag 文件到一个输出 bag 文件中.
    """
    # 1. 解析输入路径
    input_paths = expand_paths(input_patterns)
    
    if not input_paths:
        print("错误: 没有有效的输入文件.")
        return

    print("开始合并 {} 个 bag 文件到: {}".format(len(input_paths), output_bag_path))
    
    try:
        # 2. 以写入模式打开输出 bag
        with rosbag.Bag(output_bag_path, 'w') as outbag:
            for input_path in input_paths:
                print("正在处理: {}".format(input_path))
                try:
                    # 3. 以读取模式打开输入 bag
                    with rosbag.Bag(input_path, 'r') as inbag:
                        # 4. 遍历并写入消息，保持原始时间戳
                        for topic, msg, t in inbag.read_messages():
                            outbag.write(topic, msg, t)
                except Exception as e:
                    print("警告: 处理文件 {} 时出错: {}".format(input_path, str(e)))
                    continue
            
        print("合并完成! 输出文件: {}".format(output_bag_path))
        
    except Exception as e:
        print("错误: 合并过程中发生异常: {}".format(str(e)))
        sys.exit(1)

def main():
    parser = argparse.ArgumentParser(description='合并多个 ROS bag 文件，支持通配符 (Python 2)')
    parser.add_argument('-o', '--output', type=str, required=True, help='输出 bag 文件路径')
    parser.add_argument('inputs', nargs='+', type=str, help='输入 bag 文件路径或通配符模式 (例如: data/*.bag)')
    
    args = parser.parse_args()
    
    merge_bags(args.output, args.inputs)

if __name__ == '__main__':
    main()
