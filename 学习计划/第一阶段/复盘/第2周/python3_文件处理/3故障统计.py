#!/usr/bin/env python3

import csv
import os

CSV = "/home/idriver/TOOLS/zmq_tool/protocols/fault.csv"

if not os.path.exists(CSV):
    print(f"CSV 文件不存在: {CSV}")
    exit(1)

with open(CSV, "r", encoding="utf-8") as f:
    reader = csv.DictReader(f)#将第一行的表头作为“键（Key）”，每一行的数据作为“值（Value）”，组合成一个字典
    faults = list(reader)#所有行转换成字典列表存入内存变量 faults
"""
故障码,是否显示,循迹/突击,循迹/停障/避障,自主干预,应急避险,跟随/突击,跟随/停障/避障,故障描述
1052002,1,2,2,2,2,2,2,设备连接失败
=>
faults = [
{
    "故障码": "1052002",
    "是否显示": "1",
    "循迹/突击": "2",
    "循迹/停障/避障": "2",
    "自主干预": "2",
    "应急避险": "2",
    "跟随/突击": "2",
    "跟随/停障/避障": "2",
    "故障描述": "设备连接失败"
},
{
    "故障码": "1052002",
    "是否显示": "1",
    "循迹/突击": "2",
    "循迹/停障/避障": "2",
    "自主干预": "2",
    "应急避险": "2",
    "跟随/突击": "2",
    "跟随/停障/避障": "2",
    "故障描述": "设备连接失败"
}
]
"""
print(f"故障数量: {len(faults)}")

#筛选是否可以显示的故障
visible = [f for f in faults if f.get("是否显示", "").strip() == "1"]#get("是否显示", "")找键并把值取出来“1”，.strip()去除空白
print(f"可见故障数量: {len(visible)}")

#按行为分类
behaviors = ["循迹/突击", "自主干预", "应急避险", "跟随/突击"]
for b in behaviors:
    items = [f for f in faults if f.get(b, "").strip() == "2"]
    print(f"{b}行为故障数量: {len(items)}")
    for item in items[:3]:#打印前个3故障项
        print(f"{item['故障码']}-{item['故障描述']}")

with open("故障摘要.txt", "w", encoding="utf-8:") as out:
    out.write(f"故障码统计报告\n总{len(faults)}条，可显示{len(visible)}条\n")
    for b in behaviors:
        out.write(f"\n{b}:\n")
        for f in faults:
            if f.get(b, "").strip() == "2":
                out.write(f"  {f['故障码']}-{f['故障描述']}\n")
print(f"\n故障摘要已保存到故障摘要.txt")