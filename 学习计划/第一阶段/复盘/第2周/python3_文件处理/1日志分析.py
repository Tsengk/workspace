#!/usr/bin/env python3

import os

LOG = "/home/idriver/TOOLS/zmq_tool/backend/zmq_backend.log"

if not os.path.exists(LOG):
    print(f"日志文件不存在: {LOG}")
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

print(f"总行数: {total}")
print(f"INFO: {info_count}")
print(f"WARNING: {warn_count}")
print(f"ERROR: {error_count}")

print(f"===WARNING: {warn_count}===")
for line in lines:
    if "[WARNING]" in line:
        print(line.rsplit())#智能空白符过滤