#!/usr/bin/env python3
"""
练习2：条件分支 — 基于 GSDS 相机参数校验场景

目标：
  1. 掌握 if / elif / else
  2. 使用比较运算符与逻辑运算符
  3. 三元表达式
  4. match-case（Python 3.10+）

场景：校验相机标定参数的合法性，根据相机类型选择处理策略。
"""

# ============================================================
# 一、if / elif / else — 参数校验
# ============================================================

print("=" * 50)
print("相机参数校验 — 条件分支")
print("=" * 50)


def validate_image_size(width: int, height: int) -> str:
    """校验图像尺寸是否在合法范围内。"""
    if width <= 0 or height <= 0:
        return "错误: 尺寸不能为负数或零"
    elif width < 640 or height < 480:
        return "警告: 分辨率过低"
    elif 640 <= width <= 1920 and 480 <= height <= 1080:
        return "通过: 标准分辨率"
    elif width > 3840 or height > 2160:
        return "警告: 分辨率超出常规范围"
    else:
        return "通过: 高分辨率"


# 测试不同尺寸
test_sizes = [(3840, 2160), (1920, 1080), (320, 240), (-1, 1080), (7680, 4320)]
for w, h in test_sizes:
    result = validate_image_size(w, h)
    print(f"  {w}x{h} -> {result}")


# ============================================================
# 二、逻辑运算符 — 多条件组合校验
# ============================================================

print("\n--- 畸变参数有效性检查 ---")


def is_distortion_valid(k1: float, k2: float, p1: float, p2: float) -> bool:
    """检查畸变参数是否合理（k1 不应极端过大或过小）。"""
    k1_valid = -100.0 <= k1 <= 100.0
    k2_valid = -10000.0 <= k2 <= 10000.0
    p1_valid = -1.0 <= p1 <= 1.0
    p2_valid = -1.0 <= p2 <= 1.0
    return k1_valid and k2_valid and p1_valid and p2_valid


# 几组测试数据
test_params = [
    ("front_120", -0.454, 0.214, -0.000237, 0.0000544),  # 正常
    ("front_60", 67.06, -1302046.68, -0.442, 0.316),      # k2 极端
]
for name, k1, k2, p1, p2 in test_params:
    ok = is_distortion_valid(k1, k2, p1, p2)
    print(f"  {name}: k1={k1} k2={k2} -> {'有效' if ok else '无效/需人工复核'}")


# ============================================================
# 三、三元表达式 — 快速判断
# ============================================================

print("\n--- 相机类型快速判定 ---")

camera_names = ["front_120", "left_fish_eye", "back_120", "front_60", "left_front_100"]
for name in camera_names:
    # 三元表达式：鱼眼 vs 非鱼眼
    cam_type = "鱼眼" if "fish_eye" in name else "标准"
    # 视角方向提取
    if "front" in name:
        direction = "前"
    elif "back" in name:
        direction = "后"
    elif "left" in name:
        direction = "左"
    else:
        direction = "右"
    print(f"  {name:20s} -> {cam_type}相机，{direction}视角")


# ============================================================
# 四、match-case（Python 3.10+）— 按相机类型分发处理
# ============================================================

print("\n--- 相机处理策略分发 ---")


def get_processing_strategy(camera_name: str) -> str:
    """根据相机类型返回处理策略（Python 3.8 兼容，3.10+ 可用 match-case）。"""
    if "fish_eye" in camera_name:
        return "使用鱼眼去畸变算法 + BEV 投影"
    elif "120" in camera_name:
        return "使用 120° 广角 MEI 模型"
    elif "100" in camera_name:
        return "使用 100° 标准 MEI 模型"
    elif "60" in camera_name:
        return "使用 60° 窄角 MEI 模型"
    else:
        return "未知相机类型，跳过"


for name in camera_names:
    strategy = get_processing_strategy(name)
    print(f"  {name:20s} -> {strategy}")
