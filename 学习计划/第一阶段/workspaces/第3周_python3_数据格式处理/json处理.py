#!/usr/bin/env python3
"""
结合 GSDS 设备配置完成 JSON 深度解析练习

数据源（按优先级）:
  1. 本地测试文件: test_device.json (同目录)
  2. 真实文件:     /home/idriver/gsds-1.2.0-beta/basic_info/cfg/device.json

知识点:
- json.load() 读取与 json.dumps() 序列化
- 嵌套 dict/list 的遍历
- defaultdict 分组聚合
- 将结构化数据扁平化输出为表格
"""

import json
import os
from collections import defaultdict

# 优先使用本地测试文件，外部路径作为回退
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
TEST_DATA = os.path.join(SCRIPT_DIR, "test_data")
CANDIDATES = [
    os.path.join(TEST_DATA, "test_device.json"),
    "/home/idriver/gsds-1.2.0-beta/basic_info/cfg/device.json",
]

DEVICE_JSON = None
for path in CANDIDATES:
    if os.path.exists(path):
        DEVICE_JSON = path
        break

if DEVICE_JSON is None:
    print("❌ 未找到数据文件，请确认 test_device.json 或 GSDS 设备配置存在")
    exit(1)

print(f"📂 数据源: {DEVICE_JSON}")

# 1. 读取 JSON
with open(DEVICE_JSON, "r", encoding="utf-8") as f:
    data = json.load(f)

print("=" * 60)
print("GSDS 设备配置 JSON 解析")
print("=" * 60)

# 2. 探索顶层 key
print(f"\n顶层 key: {list(data.keys())}")

# 3. 解析 cpu_chip_vec —— 嵌套结构遍历
chips = data.get("cpu_chip_vec", [])
print(f"\n芯片方案数: {len(chips)}")

# 按架构统计核心数
arch_stats = {}
for chip in chips:
    name = chip.get("chip_name", "未知")
    cores = chip.get("cpu_cores", [])
    arch_stats[name] = {
        "核心数": len(cores),
        "核心列表": ", ".join(sorted(set(cores))),  # 去重排序
    }

print("\n--- CPU 芯片汇总 ---")
print(f"{'芯片':<10} {'核心数':<8} {'核心类型'}")
print("-" * 50)
for arch, info in sorted(arch_stats.items()):
    print(f"{arch:<10} {info['核心数']:<8} {info['核心列表']}")

# 4. 扁平化：把嵌套 JSON 转成二维表（便于导出 CSV）
print("\n--- 扁平化视图 (前 12 行) ---")
rows = []
for chip in chips:
    chip_name = chip.get("chip_name", "?")
    for core in chip.get("cpu_cores", []):
        rows.append({"芯片": chip_name, "核心": core})

print(f"{'芯片':<8} {'核心'}")
print("-" * 30)
for r in rows[:12]:
    print(f"{r['芯片']:<8} {r['核心']}")

# 5. 输出扁平化结果为 JSON 文件
output_path = os.path.join(SCRIPT_DIR, "device_flat.json")
with open(output_path, "w", encoding="utf-8") as out:
    json.dump(rows, out, ensure_ascii=False, indent=2)
print(f"\n已输出扁平化结果: {os.path.basename(output_path)} ({len(rows)} 条)")

# 6. 探索其他 key（如果存在）
if "compile_device_vec" in data:
    devices = data["compile_device_vec"]
    print(f"\n编译设备数: {len(devices)}")
    arch_count = defaultdict(int)
    for dev in devices:
        arch = dev.get("compile_aarch", "未知")
        arch_count[arch] += 1
    for arch, cnt in sorted(arch_count.items()):
        print(f"  架构 {arch}: {cnt} 台")

print("\n✅ JSON 解析练习完成")
