#!/usr/bin/env python3

camera_name: str = "front_120"
image_width: int = 3840
image_height: int = 2160
K1: float = -0.4540382069750924
distortion_enabled: bool = True
gamma: tuple = (4238.609331970091, 4224.3512966406915)
supported_views: list = ["front_120", "front_60", "front_fish_eye"]
camera_meta: dict = {
    "model_type": "MEI",
    "serial": "CAM-20240101-001",
    "location": "front",
}

print("=" * 50)
print("相机标定参数 -- 变量与类型")
print("=" * 50)

print(f"camera_name      类型: {type(camera_name).__name__:>8}  值: {camera_name}")
print(f"image_width      类型: {type(image_width).__name__:>8}  值: {image_width}")
print(f"image_height     类型: {type(image_height).__name__:>8}  值: {image_height}")
print(f"K1               类型: {type(K1).__name__:>8}  值: {K1}")
print(f"distortion_enabled 类型: {type(distortion_enabled).__name__:>8}  值: {distortion_enabled}")
print(f"gamma            类型: {type(gamma).__name__:>8}  值: {gamma}")
print(f"supported_views  类型: {type(supported_views).__name__:>8}  值: {supported_views}")
print(f"camera_meta      类型: {type(camera_meta).__name__:>8}  值: {camera_meta}")


print("\n--- 类型转换 ---")

gamma1_int = int(gamma[0])
print(f"gamma[0] (float) -> gamma1_int (int): {gamma[0]} -> {gamma1_int}")

resolution_str = str(image_width) + "x" + str(image_height)
print(f"分辨率:{resolution_str}")

k1_from_str = float("0.21425673994294858")
print(f"k1从字符串中解析:{k1_from_str}")

print(f"distortion_enabled (bool) -> int: {int(distortion_enabled)}")

print("\n--- 汇总 ---")
print(f"相机[{camera_name}] 分辨率: {resolution_str}"
      f"畸变矫正: {'启用' if distortion_enabled else '禁用'}"
      f"k1={K1:.6f}")