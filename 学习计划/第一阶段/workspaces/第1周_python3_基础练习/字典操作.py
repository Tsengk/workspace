#!/usr/bin/env python3
"""
练习6：字典操作 — 基于 GSDS 标定配置文件解析场景

目标：
  1. 字典 CRUD、get / setdefault / pop
  2. keys / values / items 遍历
  3. 字典推导式
  4. defaultdict / Counter
  5. 嵌套字典操作
  6. dict 合并（| 运算符, update）

场景：解析 proj_cfg 中的相机参数块，按相机名称组织为嵌套字典。
"""

from __future__ import annotations
from collections import defaultdict, Counter
from pprint import pprint

print("=" * 60)
print("标定配置字典操作")
print("=" * 60)

# ============================================================
# 一、构建嵌套字典 — 模拟 proj_cfg 解析结果
# ============================================================

calib_data: dict[str, dict] = {
    "front_120": {
        "Size": [3840, 2160],
        "K": [4238.609, 0, 1926.572, 0, 4224.351, 1088.389, 0, 0, 1],
        "D": [-0.454, 0.214, -0.000237, 0.0000544],
        "Epsilon": 1.228,
        "t": [0.0, 1.26, -3.99],
        "R_euler": [0.54, -0.08, 0.42],
    },
    "left_front_100": {
        "Size": [1920, 1080],
        "K": [5480.465, 0, 988.307, 0, 5494.363, 532.812, 0, 0, 1],
        "D": [1.441, 33.340, -0.01039, -0.01084],
        "Epsilon": 3.985,
        "t": [0.97, 1.1175, -3.025],
        "R_euler": [8.38, -11.59, 0.35],
    },
    "front_fish_eye": {
        "Size": [1920, 1280],
        "K": [971.131, 0, 959.942, 0, 972.770, 636.411, 0, 0, 1],
        "D": [0.219, -0.218, 0.0000227, 0.000348],
        "Epsilon": 1.233,
        "t": [0.04, 1.26, -3.99],
        "R_euler": [0.0, -1.59, -1.24],
    },
}

# ============================================================
# 二、字典 CRUD
# ============================================================

print("\n--- 基本操作 ---")

# get — 安全取值（不存在返回默认值）
epsilon = calib_data.get("front_120", {}).get("Epsilon", 0.0)
print(f"  front_120 epsilon: {epsilon}")

# 不存在的键
missing = calib_data.get("nonexistent", {}).get("Epsilon", "N/A")
print(f"  不存在的相机: {missing}")

# setdefault — 键不存在时设置默认值
calib_data.setdefault("back_120", {
    "Size": [3840, 2160],
    "D": [3.791, 213.378, -0.00645, 0.01626],
    "Epsilon": 7.010,
})
print(f"  setdefault back_120 -> epsilon: {calib_data['back_120']['Epsilon']}")

# pop — 删除并返回值
removed = calib_data.pop("back_120")
print(f"  pop back_120: Epsilon={removed['Epsilon']}")

# update — 合并字典
calib_data.update({
    "left_back_60": {"Epsilon": 0.383, "t": [0.795, 1.235, -0.51]},
})
print(f"  update 后字典键: {list(calib_data.keys())}")

# in 检查
print(f"  'front_120' 存在: {'front_120' in calib_data}")
print(f"  'back_120' 存在: {'back_120' in calib_data}")


# ============================================================
# 三、遍历 keys / values / items
# ============================================================

print("\n--- 遍历 ---")

# 遍历键值对
for cam_name, params in calib_data.items():
    eps = params.get("Epsilon", "?")
    size = params.get("Size", "?")
    print(f"  {cam_name:20s} Epsilon={eps:8.4f}  Size={size}")

# 只遍历键
print(f"\n  所有相机名: {list(calib_data.keys())}")

# 只遍历值 — 收集所有 epsilon
epsilons = [p["Epsilon"] for p in calib_data.values() if "Epsilon" in p]
print(f"  所有 epsilon: {[f'{e:.3f}' for e in epsilons]}")


# ============================================================
# 四、字典推导式
# ============================================================

print("\n--- 字典推导式 ---")

# 生成 {相机名: epsilon} 映射
epsilon_map = {
    name: params["Epsilon"]
    for name, params in calib_data.items()
    if "Epsilon" in params
}
print(f"  epsilon 映射: {epsilon_map}")

# 反转：{epsilon 范围: [相机列表]}
# 例如 epsilon < 1 为 "小", 1-5 为 "中", >5 为 "大"
def categorize(eps: float) -> str:
    if eps < 1:
        return "小"
    elif eps < 5:
        return "中"
    else:
        return "大"

category_map = {
    cat: [name for name, eps in epsilon_map.items() if categorize(eps) == cat]
    for cat in ["小", "中", "大"]
}
print(f"  按 epsilon 分类: {category_map}")


# ============================================================
# 五、defaultdict — 自动初始化
# ============================================================

print("\n--- defaultdict ---")

# 按视角方向分组相机
cam_by_direction = defaultdict(list)
for name in calib_data:
    if "front" in name:
        cam_by_direction["前视"].append(name)
    elif "back" in name:
        cam_by_direction["后视"].append(name)
    elif "left" in name:
        cam_by_direction["左侧"].append(name)
    elif "right" in name:
        cam_by_direction["右侧"].append(name)

for direction, cams in cam_by_direction.items():
    print(f"  {direction}: {cams}")


# ============================================================
# 六、Counter — 计数统计
# ============================================================

print("\n--- Counter ---")

# 统计各视角方向相机数量
direction_counter = Counter()
for name in calib_data:
    if "front" in name:
        direction_counter["前视"] += 1
    if "back" in name:
        direction_counter["后视"] += 1
    if "left" in name:
        direction_counter["左侧"] += 1
    if "right" in name:
        direction_counter["右侧"] += 1

print(f"  方向统计: {dict(direction_counter)}")

# 统计各角度出现次数
angle_counter = Counter()
import re
for name in calib_data:
    m = re.search(r"(\d+)", name)
    if m:
        angle_counter[f"{m.group(1)}°"] += 1
    elif "fish_eye" in name:
        angle_counter["鱼眼"] += 1

print(f"  角度统计: {dict(angle_counter)}")


# ============================================================
# 七、字典合并（Python 3.9+ | 运算符）
# ============================================================

print("\n--- 字典合并 ---")

default_params = {
    "model_type": "MEI",
    "image_width": 1920,
    "image_height": 1080,
}

camera_specific = {
    "camera_name": "front_120",
    "image_width": 3840,  # 覆盖默认值
    "image_height": 2160,
}

# 字典合并（Python 3.9+ 可用 | 运算符，3.8 用解包）
merged = {**default_params, **camera_specific}
pprint(merged)
