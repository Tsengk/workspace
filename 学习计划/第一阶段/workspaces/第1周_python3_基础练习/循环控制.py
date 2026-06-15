#!/usr/bin/env python3
"""
练习3：循环控制 — 基于 GSDS 多相机批量处理场景

目标：
  1. for 循环遍历列表、字典、range
  2. while 循环
  3. break / continue
  4. zip / enumerate
  5. 列表推导式（过渡到列表操作）

场景：批量处理 13 个相机的标定参数，筛选、统计、转换。
"""

# ============================================================
# 模拟数据：13 个相机的畸变参数
# ============================================================
cameras = [
    {"name": "front_120",          "k1": -0.454, "k2": 0.214,    "epsilon": 1.228},
    {"name": "left_front_100",     "k1": 1.441,  "k2": 33.340,   "epsilon": 3.985},
    {"name": "right_front_100",    "k1": -0.326, "k2": 0.128,    "epsilon": 0.573},
    {"name": "left_back_100",      "k1": -0.287, "k2": 0.106,    "epsilon": 0.947},
    {"name": "right_back_100",     "k1": 30.910, "k2": 11299.607,"epsilon": 15.855},
    {"name": "back_120",           "k1": 3.791,  "k2": 213.378,  "epsilon": 7.010},
    {"name": "front_60",           "k1": 67.068, "k2": -1302046.680, "epsilon": 55.261},
    {"name": "left_back_60",       "k1": -0.660, "k2": 0.311,    "epsilon": 0.383},
    {"name": "right_back_60",      "k1": -0.568, "k2": 0.281,    "epsilon": 0.060},
    {"name": "front_fish_eye",     "k1": 0.219,  "k2": -0.218,   "epsilon": 1.233},
    {"name": "left_fish_eye",      "k1": 0.261,  "k2": -0.250,   "epsilon": 1.285},
    {"name": "right_fish_eye",     "k1": 0.290,  "k2": -0.268,   "epsilon": 1.295},
    {"name": "back_fish_eye",      "k1": 0.286,  "k2": -0.271,   "epsilon": 1.293},
]

print("=" * 60)
print("批量相机参数处理 — 循环控制")
print("=" * 60)

# ============================================================
# 一、for 循环遍历列表
# ============================================================

print("\n--- 所有相机 epsilon 值 ---")
for cam in cameras:
    print(f"  {cam['name']:20s} epsilon = {cam['epsilon']:8.4f}")


# ============================================================
# 二、for + range
# ============================================================

print("\n--- 每 3 个相机一组（range 分页） ---")
for i in range(0, len(cameras), 3):
    group = cameras[i : i + 3]
    names = ", ".join(c["name"] for c in group)
    print(f"  第 {i // 3 + 1} 组: {names}")


# ============================================================
# 三、enumerate — 带索引遍历
# ============================================================

print("\n--- 带编号的相机列表 ---")
for idx, cam in enumerate(cameras, start=1):
    print(f"  {idx:2d}. {cam['name']}")


# ============================================================
# 四、zip — 并行遍历
# ============================================================

print("\n--- 角度相同的相机对比（30°/60°/100°/120°/鱼眼） ---")
angles = ["120°", "100°", "100°", "100°", "100°", "120°",
          "60°", "60°", "60°", "鱼眼", "鱼眼", "鱼眼", "鱼眼"]
for cam, angle in zip(cameras, angles):
    print(f"  {cam['name']:20s} 角度: {angle}")


# ============================================================
# 五、break / continue — 条件控制
# ============================================================

print("\n--- 寻找第一个 k2 超过 1000 的相机（异常检测） ---")
for cam in cameras:
    if abs(cam["k2"]) <= 1000:
        continue  # 跳过正常的
    print(f"  异常! {cam['name']}: k2 = {cam['k2']}")
    # break  # 如果只需要第一个异常，取消注释这行


# ============================================================
# 六、while — 累加统计
# ============================================================

print("\n--- while 循环：统计畸变幅度 ---")
total_distortion = 0.0
i = 0
while i < len(cameras):
    cam = cameras[i]
    # 畸变幅度 = sqrt(k1² + k2²)
    magnitude = (cam["k1"] ** 2 + cam["k2"] ** 2) ** 0.5
    total_distortion += magnitude
    i += 1

avg_distortion = total_distortion / len(cameras)
print(f"  平均畸变幅度: {avg_distortion:.2f}")


# ============================================================
# 七、列表推导式 — 快速筛选
# ============================================================

print("\n--- 列表推导式：筛选 epsilon > 5 的相机 ---")
high_epsilon = [cam["name"] for cam in cameras if cam["epsilon"] > 5]
print(f"  {high_epsilon}")

# 极端畸变检测
extreme = [cam["name"] for cam in cameras if abs(cam["k2"]) > 1000]
print(f"  极端 k2 值相机: {extreme if extreme else '无'}")


# ============================================================
# 八、嵌套循环 — 两两对比
# ============================================================

print("\n--- 嵌套循环：查找 epsilon 值差异最小的两个相机 ---")
min_diff = float("inf")
pair = ("", "")
for i in range(len(cameras)):
    for j in range(i + 1, len(cameras)):
        diff = abs(cameras[i]["epsilon"] - cameras[j]["epsilon"])
        if diff < min_diff:
            min_diff = diff
            pair = (cameras[i]["name"], cameras[j]["name"])
print(f"  最接近: {pair[0]} 与 {pair[1]}，差异 = {min_diff:.6f}")
