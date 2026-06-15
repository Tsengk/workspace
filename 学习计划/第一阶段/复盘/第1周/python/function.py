#!/usr/bin/env python3

from __future__ import annotations
from typing import Optional


print("=" * 50)
print("函数定义与调用")
print("=" * 50)

def compute_focal_length(gamma1: float, gamma2: float) -> float:
    return (gamma1 + gamma2) / 2.0

f = compute_focal_length(4238.609, 4224.351)
print(f"计算得到的焦距: {f:.3f}")

def extract_intrinsics(K: list) -> tuple:
    """K:[fx, 0, cx, 0, fy, cy, 0, 0, 1]"""
    fx, _, cx, _, fy, cy, *_ = K
    return fx, fy, cx, cy

K = [2000.0, 0.0, 1920.0, 0.0, 2000.0, 1080.0, 0.0, 0.0, 1.0]
fx, fy, cx, cy = extract_intrinsics(K)
print(f"fx: {fx:.2f}, fy: {fy:.2f}, cx: {cx:.2f}, cy: {cy:.2f}")


def check_distortion(
        k1: float,
        k2: float,
        p1: float,
        p2: float,
        k1_threshold: float = 50.0,
        k2_threshold: float = 1000.0,
) -> list[str]:
    warnings: list[str] = []
    if abs(k1) > k1_threshold:
        a=warnings.append(f"k1={k1} 超出阈值 {k1_threshold}")
    if abs(k2) > k2_threshold:
        warnings.append(f"k2={k2} 超出阈值 {k2_threshold}")
    if abs(p1) > 0.5:
        warnings.append(f"p1={p1} 超出合理范围 [-0.5, 0.5]")
    if abs(p2) > 0.5:
        warnings.append(f"p2={p2} 超出合理范围 [-0.5, 0.5]")
    return warnings


warnings = check_distortion(67.068, -1302046, -0.442, 0.316)
print(f"\nfront_60 畸变警告(默认阈值) : {warnings}")

warnings_strict = check_distortion(67.068, -1302046, -0.442, 0.316, k1_threshold=100.0, k2_threshold=2000000.0)
print(f"front_60 畸变警告(宽松阈值) : {warnings_strict}")

def average_epsilon(*epsilons: float) -> float:
    if not epsilons:
        return 0.0
    return sum(epsilons) / len(epsilons)

avg = average_epsilon(1.228, 3.985, 0.567, 2.345, 0.789, 1.234)
print(f"\n6相机平均epsilon: {avg:.4f}")

def build_camera_config(name: str, **kwargs) -> dict:
    config = {
        "camera_name": name,
        "model_type": kwargs.get("model_type", "MEI"),
        "image_width": kwargs.get("image_width",1920),
        "image_hight": kwargs.get("image_height",1080),
    }
    if "k1" in kwargs:
        config["distortion_parameters"] = {
            "k1": kwargs["k1"],
            "k2": kwargs.get("k2", 0.0),
            "p1": kwargs.get("p1", 0.0),
            "p2": kwargs.get("p2", 0.0),
        }
    return config

cfg = build_camera_config(
    "front_120",
    model_type="MEI",
    image_width=3840,
    image_height=2160,
    k1=-0.4540382069750924,
    k2=0.214,
    p1=-0.000237,
    p2=0.00054,
)
print(f"\n构建的相机配置: {cfg}")

cam_params = [
    ("front_120",-0.454),
    ("left_front_100",1.441),
    ("front_60",67.068),
    ("back_fish_eye",0.222),
]
cam_params.sort(key=lambda x: abs(x[1]), reverse=True)
print("\n按k1绝对值排序的相机参数:")
for name, k1 in cam_params:
    print(f"{name}: k1={k1:.3f}")

names = list(map(lambda x: x[0], cam_params))
print(f"\n相机名称列表: {names}")

DEFAULT_MODEL = "MEI"

def create_camera(name: str):
    DEFAULT_MODEL_local = "SCARAMUZZA"

    def inner_get_model():
        return DEFAULT_MODEL_local
    
    model = inner_get_model()

    print(f"\n创建相机{name}，使用模型: {model}")

create_camera("front_120")