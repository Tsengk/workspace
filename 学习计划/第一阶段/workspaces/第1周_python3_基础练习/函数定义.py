#!/usr/bin/env python3
"""
练习4：函数定义 — 基于 GSDS 标定参数转换场景

目标：
  1. 函数定义与调用、参数默认值
  2. 关键字参数、*args、**kwargs
  3. 返回值（单值 / 多值 / None）
  4. lambda 匿名函数
  5. 类型注解
  6. 作用域 LEGB

场景：将标定参数从 proj_cfg 格式提取并转为 YAML 所需格式。
"""

from __future__ import annotations
from typing import Optional

print("=" * 60)
print("标定参数转换函数 — 函数定义")
print("=" * 60)

# ============================================================
# 一、基本函数 — 单值返回
# ============================================================


def compute_focal_length(gamma1: float, gamma2: float) -> float:
    """根据投影参数计算等效焦距（取 gamma1/gamma2 均值）。"""
    return (gamma1 + gamma2) / 2.0


# 调用
f = compute_focal_length(4238.609, 4224.351)
print(f"\n等效焦距: {f:.2f}")


# ============================================================
# 二、多值返回 — 元组解包
# ============================================================


def extract_intrinsics(K: list) -> tuple:
    """从内参矩阵 K 中提取 fx, fy, cx, cy。

    K 格式: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    """
    fx, _, cx, _, fy, cy, *_ = K
    return fx, fy, cx, cy


K_front_120 = [4238.609, 0, 1926.572, 0, 4224.351, 1088.389, 0, 0, 1]
fx, fy, cx, cy = extract_intrinsics(K_front_120)
print(f"内参: fx={fx:.2f} fy={fy:.2f} cx={cx:.2f} cy={cy:.2f}")


# ============================================================
# 三、默认参数 — 可选阈值
# ============================================================


def check_distortion(
    k1: float,
    k2: float,
    p1: float,
    p2: float,
    k1_threshold: float = 50.0,
    k2_threshold: float = 1000.0,
) -> list[str]:
    """检查畸变参数是否超出阈值，返回警告列表。"""
    warnings: list[str] = []
    if abs(k1) > k1_threshold:
        warnings.append(f"k1={k1} 超出阈值 {k1_threshold}")
    if abs(k2) > k2_threshold:
        warnings.append(f"k2={k2} 超出阈值 {k2_threshold}")
    if abs(p1) > 0.5:
        warnings.append(f"p1={p1} 偏大")
    if abs(p2) > 0.5:
        warnings.append(f"p2={p2} 偏大")
    return warnings


# 使用默认阈值
warnings = check_distortion(67.068, -1302046.68, -0.442, 0.316)
print(f"\nfront_60 畸变警告（默认阈值）: {warnings}")

# 自定义阈值
warnings = check_distortion(67.068, -1302046.68, -0.442, 0.316,
                            k1_threshold=100.0, k2_threshold=2000000.0)
print(f"front_60 畸变警告（宽松阈值）: {warnings}")


# ============================================================
# 四、*args — 批量计算
# ============================================================


def average_epsilon(*epsilons: float) -> float:
    """计算任意数量相机的 epsilon 均值。"""
    if not epsilons:
        return 0.0
    return sum(epsilons) / len(epsilons)


avg = average_epsilon(1.228, 3.985, 0.573, 0.947, 15.855, 7.010)
print(f"\n6 相机平均 epsilon: {avg:.4f}")


# ============================================================
# 五、**kwargs — 灵活配置
# ============================================================


def build_camera_config(name: str, **kwargs) -> dict:
    """构建相机配置字典，kwargs 为任意额外参数。"""
    config = {
        "camera_name": name,
        "model_type": kwargs.get("model_type", "MEI"),
        "image_width": kwargs.get("image_width", 1920),
        "image_height": kwargs.get("image_height", 1080),
    }
    # 畸变参数（可选）
    if "k1" in kwargs:
        config["distortion_parameters"] = {
            "k1": kwargs["k1"],
            "k2": kwargs.get("k2", 0.0),
            "p1": kwargs.get("p1", 0.0),
            "p2": kwargs.get("p2", 0.0),
        }
    return config


cfg = build_camera_config(
    "test_cam",
    model_type="MEI",
    image_width=3840,
    image_height=2160,
    k1=-0.454,
    k2=0.214,
    p1=-0.000237,
    p2=0.000054,
)
print(f"\n相机配置:\n{cfg}")


# ============================================================
# 六、lambda — 简单变换
# ============================================================

# lambda 用于排序、映射等场景
cam_params = [
    ("front_120",       -0.454),
    ("left_front_100",   1.441),
    ("front_60",        67.068),
    ("back_fish_eye",    0.286),
]

# 按 |k1| 绝对值降序排序
cam_params.sort(key=lambda x: abs(x[1]), reverse=True)
print("\n按 |k1| 降序:")
for name, k1 in cam_params:
    print(f"  {name:20s} k1={k1:8.4f}")

# 映射：提取所有名称
names = list(map(lambda x: x[0], cam_params))
print(f"排序后名称列表: {names}")


# ============================================================
# 七、作用域 LEGB 示例
# ============================================================

DEFAULT_MODEL = "MEI"  # Global


def create_camera(name: str):
    DEFAULT_MODEL_local = "SCARAMUZZA"  # Enclosing (nonlocal)

    def inner_get_model():
        return DEFAULT_MODEL_local

    model = inner_get_model()  # Local — 调用 enclosing 中的变量
    print(f"\n[{name}] 使用模型: {model} (enclosing), 默认模型: {DEFAULT_MODEL} (global)")


create_camera("scope_test")
