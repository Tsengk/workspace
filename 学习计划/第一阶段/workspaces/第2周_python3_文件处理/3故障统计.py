#!/usr/bin/env python3
"""
结合 ZMQ_TOOL 的真实 CSV 文件完成 python3 文件处理
数据源: /home/idriver/TOOLS/zmq_tool/protocols/fault.csv
"""

import csv
import os

CSV = "/home/idriver/TOOLS/zmq_tool/protocols/fault.csv"

if not os.path.exists(CSV):
    print(f"文件不存在: {CSV}")
    exit(1)

with open(CSV, "r", encoding="utf-8") as f:
    reader = csv.DictReader(f)
    faults = list(reader)

print(f"===== 故障码统计 =====")
print(f"总计 {len(faults)} 条故障\n")

# 筛选可显示的故障（是否显示=1）
visible = [f for f in faults if f.get("是否显示", "").strip() == "1"]
print(f"可显示故障: {len(visible)} 条\n")

# 按行为分类
behaviors = ["循迹/突击", "自主干预", "应急避险", "跟随/突击"]
for b in behaviors:
    items = [f for f in faults if f.get(b, "0").strip() == "2"]
    print(f"{b}: {len(items)} 条")
    for item in items[:3]:  # 只打印前 3 个
        print(f"  {item['故障码']} — {item['故障描述']}")

# 输出摘要到文件
with open("故障摘要.txt", "w", encoding="utf-8") as out:
    out.write(f"故障码统计报告\n总 {len(faults)} 条，可显示 {len(visible)} 条\n")
    for b in behaviors:
        out.write(f"\n{b}:\n")
        for f in faults:
            if f.get(b, "0").strip() == "2":
                out.write(f"  {f['故障码']} — {f['故障描述']}\n")
print(f"\n已输出: 故障摘要.txt")
