#!/usr/bin/env python3
"""
练习1：变量与类型 — 基于 GSDS 相机标定参数场景

目标：
  1. 掌握 Python 基本类型：int, float, str, bool, list, tuple, dict
  2. 理解类型转换与 type() 检查
  3. 使用 f-string 格式化输出

场景：解析一条相机标定参数记录，计算并打印汇总信息。
"""

# ============================================================
# 一、基本类型声明
# ============================================================

# 相机名称（字符串）
camera_name: str = "front_120"

# 图像尺寸（整数）
image_width: int = 3840
image_height: int = 2160

# 畸变参数 k1（浮点数）
k1: float = -0.4540382069750924

# 是否启用畸变校正（布尔）
distortion_enabled: bool = True

# 投影参数内参矩阵前两个元素（元组 — 不可变）
gamma: tuple = (4238.609331970091, 4224.3512966406915)

# 相机支持的视角标签（列表 — 可变）
supported_views: list = ["front_120", "front_60", "front_fish_eye"]

# 相机元信息（字典）
camera_meta: dict = {
    "model_type": "MEI",
    "serial": "CAM-20240101-001",
    "location": "front",
}

# ============================================================
# 二、类型检查与打印
# ============================================================

print("=" * 50)
print("相机标定参数 — 变量与类型")
print("=" * 50)

# 使用 type() 检查类型
print(f"camera_name      类型: {type(camera_name).__name__:>8}  值: {camera_name}")
print(f"image_width      类型: {type(image_width).__name__:>8}  值: {image_width}")
print(f"image_height     类型: {type(image_height).__name__:>8}  值: {image_height}")
print(f"k1               类型: {type(k1).__name__:>8}  值: {k1}")
print(f"distortion_enabled 类型: {type(distortion_enabled).__name__:>8}  值: {distortion_enabled}")
print(f"gamma            类型: {type(gamma).__name__:>8}  值: {gamma}")
print(f"supported_views  类型: {type(supported_views).__name__:>8}  值: {supported_views}")
print(f"camera_meta      类型: {type(camera_meta).__name__:>8}  值: {camera_meta}")

# ============================================================
# 三、类型转换练习
# ============================================================

print("\n--- 类型转换 ---")

# float -> int（截断小数）
gamma1_int = int(gamma[0])
print(f"gamma[0] (float) -> int: {gamma[0]} -> {gamma1_int}")

# int -> str 拼接
resolution_str = str(image_width) + "x" + str(image_height)
print(f"分辨率字符串: {resolution_str}")

# str -> float
k1_from_str = float("0.21425673994294858")
print(f"k2 从字符串解析: {k1_from_str}")

# bool -> int
print(f"distortion_enabled -> int: {int(distortion_enabled)}")

# ============================================================
# 四、格式化输出汇总
# ============================================================

print("\n--- 汇总 ---")
print(f"相机 [{camera_name}] 分辨率={resolution_str} "
      f"畸变校正={'开启' if distortion_enabled else '关闭'} "
      f"k1={k1:.6f}")
