#!/usr/bin/env python3
"""
结合 GSDS 的真实 JSON 配置文件完成 python3 文件处理
数据源: /home/idriver/gsds-1.2.0-beta/basic_info/cfg/compile.json
"""

import json
import os

CONFIG = "/home/idriver/gsds-1.2.0-beta/basic_info/cfg/compile.json"

if not os.path.exists(CONFIG):
    print(f"文件不存在: {CONFIG}")
    exit(1)

with open(CONFIG, "r", encoding="utf-8") as f:
    data = json.load(f)

devices = data.get("compile_device_vec", [])
print(f"===== GSDS 编译设备配置 =====")
print(f"共 {len(devices)} 个设备:\n")

# 按架构分组
from collections import defaultdict
by_arch = defaultdict(list)
for dev in devices:
    arch = dev.get("compile_aarch", "未知")
    name = dev.get("compile_name", "?")
    by_arch[arch].append(name)

for arch, names in sorted(by_arch.items()):
    print(f"架构 {arch}: {', '.join(names)}")
