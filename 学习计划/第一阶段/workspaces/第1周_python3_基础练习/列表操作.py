#!/usr/bin/env python3
"""
练习5：列表操作 — 基于 GSDS 多相机参数批量管理场景

目标：
  1. 列表 CRUD：append, insert, remove, pop, extend
  2. 切片、索引、步长
  3. sort / sorted（key / reverse）
  4. map / filter / reduce
  5. 列表推导式与嵌套列表
  6. copy（浅拷贝/深拷贝）

场景：管理 13 个相机的参数列表，执行排序、筛选、分组等操作。
"""

import copy
from functools import reduce

# ============================================================
# 初始数据
# ============================================================
camera_list = [
    "front_120", "left_front_100", "right_front_100",
    "left_back_100", "right_back_100", "back_120",
    "front_60", "left_back_60", "right_back_60",
    "front_fish_eye", "left_fish_eye", "right_fish_eye", "back_fish_eye",
]

print("=" * 60)
print("相机列表批量操作 — 列表操作")
print("=" * 60)

# ============================================================
# 一、基本 CRUD
# ============================================================

print(f"\n初始列表 ({len(camera_list)} 个相机):")
print(f"  {camera_list}")

# append — 追加
camera_list.append("front_30_new")
print(f"\nappend 后: {camera_list[-3:]}")  # 最后 3 个

# insert — 插入
camera_list.insert(0, "calibration_ref")
print(f"insert 头部后: {camera_list[:4]}")

# remove — 按值删除
camera_list.remove("calibration_ref")
camera_list.remove("front_30_new")
print(f"remove 后 ({len(camera_list)} 个)")

# pop — 按索引弹出
popped = camera_list.pop()
print(f"pop 最后一个: {popped}")

# extend — 合并
extra_cams = ["left_side_30", "right_side_30"]
camera_list.extend(extra_cams)
print(f"extend 后 ({len(camera_list)} 个): {camera_list[-5:]}")


# ============================================================
# 二、切片与索引
# ============================================================

print("\n--- 切片操作 ---")

# 前 6 个（camera_names 中的主相机）
main_cams = camera_list[:6]
print(f"  前 6 个（主相机）: {main_cams}")

# 倒数 3 个
print(f"  最后 3 个: {camera_list[-3:]}")

# 步长 2 — 每隔一个取
print(f"  隔一个取: {camera_list[::2]}")

# 反转
print(f"  反转: {camera_list[::-1]}")

# 前 6 个中：索引 1 到 4（不包含 4）
print(f"  索引 1:4: {camera_list[1:4]}")


# ============================================================
# 三、排序 — sort / sorted
# ============================================================

print("\n--- 排序 ---")

# sorted 返回新列表（不修改原列表）
sorted_by_name = sorted(camera_list)
print(f"  按名称排序: {sorted_by_name}")

# 自定义排序：按角度（从名称中提取数字）
def extract_angle(name: str) -> int:
    """从相机名称中提取角度数字，没有的返回 0。"""
    import re
    match = re.search(r"(\d+)", name)
    return int(match.group(1)) if match else 0


sorted_by_angle = sorted(camera_list, key=extract_angle)
print(f"  按角度排序: {sorted_by_angle}")

# 多级排序：先类型（是否鱼眼），再角度
sorted_multi = sorted(
    camera_list,
    key=lambda n: (1 if "fish_eye" in n else 0, extract_angle(n)),
)
print(f"  先类型后角度: {sorted_multi}")


# ============================================================
# 四、map / filter / reduce
# ============================================================

print("\n--- map / filter / reduce ---")

# map: 提取每个相机名的视角方向
directions = list(map(
    lambda n: "前" if "front" in n else ("后" if "back" in n else ("左" if "left" in n else "右")),
    camera_list,
))
# 打印前几个
for cam, d in list(zip(camera_list, directions))[:5]:
    print(f"  {cam:20s} -> {d}")

# filter: 只保留鱼眼相机
fish_eye_cams = list(filter(lambda n: "fish_eye" in n, camera_list))
print(f"\n  鱼眼相机: {fish_eye_cams}")

# filter: 非鱼眼且包含数字
standard_cams = list(filter(
    lambda n: "fish_eye" not in n and any(c.isdigit() for c in n),
    camera_list,
))
print(f"  标准相机: {standard_cams}")

# reduce: 拼接所有相机名
all_names = reduce(lambda a, b: a + " | " + b, camera_list)
print(f"\n  reduce 拼接:\n  {all_names}")


# ============================================================
# 五、列表推导式
# ============================================================

print("\n--- 列表推导式 ---")

# 提取所有鱼眼相机并大写
fish_upper = [cam.upper() for cam in camera_list if "fish_eye" in cam]
print(f"  鱼眼大写: {fish_upper}")

# 嵌套推导式：生成相机名 -> 角度的映射
cam_angle_pairs = [
    (cam, extract_angle(cam))
    for cam in camera_list
    if "fish_eye" not in cam
]
print(f"  标准相机-角度对: {cam_angle_pairs}")

# 条件推导式
status = ["✓" if "fish_eye" in c else " " for c in camera_list]
for cam, s in zip(camera_list, status):
    print(f"  [{s}] {cam}")


# ============================================================
# 六、浅拷贝 vs 深拷贝
# ============================================================

print("\n--- 拷贝 ---")

# 嵌套列表：每个相机有参数列表
nested = [
    ["front_120", [-0.454, 0.214]],
    ["left_front_100", [1.441, 33.340]],
]

shallow = copy.copy(nested)
deep = copy.deepcopy(nested)

# 修改原列表嵌套元素
nested[0][1][0] = 999.999

print(f"  原列表 nested[0][1]:     {nested[0][1]}")
print(f"  浅拷贝 shallow[0][1]:     {shallow[0][1]}  <- 被影响!")
print(f"  深拷贝 deep[0][1]:        {deep[0][1]}  <- 不受影响")


# ============================================================
# 七、实用：分组统计各角度相机数量
# ============================================================

print("\n--- 分组统计 ---")
angle_counts = {}
for cam in camera_list:
    angle = extract_angle(cam)
    angle_counts[angle] = angle_counts.get(angle, 0) + 1
for angle, count in sorted(angle_counts.items()):
    print(f"  角度 {angle:4d}°: {count} 个")
