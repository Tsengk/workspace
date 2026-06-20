#!/usr/bin/env python3
"""
结合相机标定 YAML 文件完成 YAML 解析练习

数据源（按优先级）:
  1. 本地测试文件:  test_calib.yaml (同目录, 单文件)
  2. 批量标定目录:  ../第1周_python3_基础练习/标定参数提取转换/*.yaml (13 个)

知识点:
- yaml.safe_load() 读取 YAML
- 嵌套 dict 的递归遍历
- 同类多文件的批量处理
- 将解析结果输出为 JSON/CSV
"""

import os
import json
import csv
from pathlib import Path
from collections import Counter

# YAML 模块非标准库，需 pip install pyyaml
try:
    import yaml
except ImportError:
    print("⚠️  需要安装 pyyaml: pip install pyyaml")
    import subprocess
    subprocess.run(["pip", "install", "pyyaml"], check=False)
    import yaml

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# 标定文件批量目录
CALIB_DIR = os.path.join(SCRIPT_DIR, "..", "第1周_python3_基础练习", "标定参数提取转换")
# 本地单文件测试
TEST_DATA = os.path.join(SCRIPT_DIR, "test_data")
LOCAL_YAML = os.path.join(TEST_DATA, "test_calib.yaml")


# 辅助函数：处理 OpenCV yaml-cpp 格式 (%YAML:1.0 指令)
def load_calib_yaml(filepath):
    """读取标定 YAML，兼容 OpenCV 的 %YAML:1.0 指令"""
    with open(filepath, "r", encoding="utf-8") as f:
        raw = f.read()
    lines = raw.split("\n")
    cleaned_lines = [
        line for line in lines
        if not line.startswith("%YAML") and line.strip() != "---"
    ]
    return yaml.safe_load("\n".join(cleaned_lines))


# 1. 收集 YAML 文件：优先批量目录，回退到本地单文件
yaml_files = []
if os.path.isdir(CALIB_DIR):
    yaml_files = sorted(Path(CALIB_DIR).glob("*.yaml"))
if not yaml_files and os.path.exists(LOCAL_YAML):
    yaml_files = [Path(LOCAL_YAML)]

print("=" * 60)
print("相机标定参数 YAML 解析")
print("=" * 60)
print(f"发现 {len(yaml_files)} 个标定文件\n")
for f in yaml_files:
    tag = "📄" if str(f) == LOCAL_YAML else "📁"
    print(f"  {tag} {f.name}")

# 2. 逐文件解析，提取关键参数
cameras = []
fail_count = 0

for yf in yaml_files:
    try:
        data = load_calib_yaml(str(yf))
    except yaml.YAMLError as e:
        fail_count += 1
        print(f"⚠️  解析失败 {yf.name}: {e}")
        continue

    if data is None or not isinstance(data, dict):
        fail_count += 1
        continue

    cam = {
        "文件名": yf.name,
        "相机名": data.get("camera_name", "?"),
        "模型类型": data.get("model_type", "?"),
        "分辨率": f"{data.get('image_width', '?')}x{data.get('image_height', '?')}",
    }

    # 提取畸变参数
    dist = data.get("distortion_parameters", {})
    if dist:
        cam["畸变_k1"] = round(dist.get("k1", 0), 4)
        cam["畸变_k2"] = round(dist.get("k2", 0), 4)
        cam["畸变_p1"] = round(dist.get("p1", 0), 6)
        cam["畸变_p2"] = round(dist.get("p2", 0), 6)

    # 提取投影参数
    proj = data.get("projection_parameters", {})
    if proj:
        cam["投影_gamma1"] = round(proj.get("gamma1", 0), 2)
        cam["投影_gamma2"] = round(proj.get("gamma2", 0), 2)
        cam["光心_u0"] = round(proj.get("u0", 0), 2)
        cam["光心_v0"] = round(proj.get("v0", 0), 2)

    # 提取镜面参数
    mirror = data.get("mirror_parameters", {})
    if mirror:
        cam["镜面_xi"] = round(mirror.get("xi", 0), 4)

    cameras.append(cam)

if fail_count > 0:
    print(f"   ({fail_count} 个文件解析失败)")

print(f"\n成功解析 {len(cameras)} 个相机标定\n")

# 3. 打印汇总表
if cameras:
    print(f"{'相机名':<22} {'模型':<6} {'分辨率':<14} {'畸变k1':<10} {'畸变k2':<12} {'光心(u0,v0)'}")
    print("-" * 90)
    for c in cameras:
        u0 = c.get("光心_u0", "?")
        v0 = c.get("光心_v0", "?")
        print(
            f"{c['相机名']:<22} {c['模型类型']:<6} {c['分辨率']:<14} "
            f"{c.get('畸变_k1', '?'):<10} {c.get('畸变_k2', '?'):<12} "
            f"({u0}, {v0})"
        )

    # 4. 模型类型统计
    model_counter = Counter(c["模型类型"] for c in cameras)
    print(f"\n--- 模型类型分布 ---")
    for model, cnt in model_counter.most_common():
        print(f"  {model}: {cnt} 个")

# 5. 输出为 JSON
json_path = os.path.join(SCRIPT_DIR, "相机标定参数汇总.json")
with open(json_path, "w", encoding="utf-8") as out:
    json.dump(cameras, out, ensure_ascii=False, indent=2)
print(f"\n已输出 JSON: {os.path.basename(json_path)}")

# 6. 输出为 CSV (YAML→CSV 格式转换)
csv_path = os.path.join(SCRIPT_DIR, "相机标定参数汇总.csv")
if cameras:
    with open(csv_path, "w", encoding="utf-8", newline="") as out:
        writer = csv.DictWriter(out, fieldnames=cameras[0].keys())
        writer.writeheader()
        writer.writerows(cameras)
    print(f"已输出 CSV: {os.path.basename(csv_path)}")

# 7. 性能对比：找出畸变最大的相机
if cameras:
    print(f"\n--- 畸变参数分析 ---")
    max_k1 = max(cameras, key=lambda c: abs(c.get("畸变_k1", 0)))
    print(f"最大 |k1|: {max_k1['相机名']} (k1={max_k1['畸变_k1']})")

print("\n✅ YAML 解析练习完成")
