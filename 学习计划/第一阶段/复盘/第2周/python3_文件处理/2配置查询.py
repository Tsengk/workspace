#!/usr/bin/env python3

import json
import os
from collections import defaultdict

CONFIG = "/home/idriver/gsds-1.2.0-beta/basic_info/cfg/compile.json"

if not os.path.exists(CONFIG):#检查文件路径是否存在
    print(f"配置文件不存在:{CONFIG}")
    exit(1)

with open(CONFIG, "r", encoding="utf-8") as f:#文件对象 f
    data = json.load(f)#将 JSON 格式的文本文件加载为 Python 字典或列表

devices = data.get("compile_device_vec", [])#获取键，如果不存在则返回空列表
print(f"编译设备数量：{len(devices)}")

by_arch = defaultdict(list)#免去了写 if key not in dict
for dev in devices:
    arch = dev.get("compile_aarch", "unknown")
    name = dev.get("compile_name", "unknown")
    by_arch[arch].append(name)

for arch, names in sorted(by_arch.items()):#按架构名称的字母顺序排序，同时遍历键和值
    print(f"架构{arch}:{', '.join(names)}")#将属于同一个架构的所有设备名称用逗号加空格拼接成一整行

