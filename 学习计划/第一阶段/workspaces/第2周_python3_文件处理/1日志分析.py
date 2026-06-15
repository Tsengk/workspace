#!/usr/bin/env python3
"""
结合 ZMQ_TOOL 的真实日志文件完成 python3 文件处理
数据源: /home/idriver/TOOLS/zmq_tool/backend/zmq_backend.log
日志格式: 2026-04-21 14:18:41 [INFO] logger:113 - 内容
"""

import os

LOG = "/home/idriver/TOOLS/zmq_tool/backend/zmq_backend.log"

if not os.path.exists(LOG):
    print(f"文件不存在: {LOG}")
    exit(1)

with open(LOG, "r", encoding="utf-8", errors="ignore") as f:
    lines = f.readlines()

total = len(lines)
info_count = 0
warn_count = 0
error_count = 0

for line in lines:
    if "[INFO]" in line:
        info_count += 1
    elif "[WARNING]" in line:
        warn_count += 1
    elif "[ERROR]" in line:
        error_count += 1

print(f"===== ZMQ 后端日志分析 =====")
print(f"文件: {LOG}")
print(f"总行数: {total}")
print(f"INFO: {info_count}  WARNING: {warn_count}  ERROR: {error_count}\n")

# 列出所有 WARNING
print(f"=== WARNING 行 ({warn_count} 条) ===")
for line in lines:
    if "[WARNING]" in line:
        print(line.rstrip())
