#!/usr/bin/env python3

print("=" * 50)
print("条件分支")
print("=" * 50)

def validate_image_size(width: int, height: int) -> str:
    if width <= 0 or height <= 0:
        return "无效的图像尺寸"
    elif width < 640 or height < 480:
        return "图像尺寸过小"
    elif 640 <= width <= 1920 and 480 <= height <= 1080:
        return "图像尺寸适中"
    elif width > 1920 or height > 1080:
        return "图像尺寸过大"
    else:
        return "超高级图像尺寸"
    
test_sizes = [(3840, 2160), (1920, 1080), (800, 600), (320, 240), (-100, 200)]
for w, h in test_sizes:
    result = validate_image_size(w, h)
    print(f"图像尺寸: {w}x{h} -> {result}")


print("\n---畸变参数检查---")

def is_distortion_valid(k1: float, k2: float, p1: float, p2: float) -> bool:
    k1_valid = -100.0 <= k1 <= 100.0
    k2_valid = -1000.0 <= k2 <= 1000.0
    p1_valid = -1.0 <= p1 <= 1.0
    p2_valid = -1.0 <= p2 <= 1.0
    return k1_valid and k2_valid and p1_valid and p2_valid


test_params = [
    ("front_120", -0.4540382069750924, 0.214, -0.000237, 0.00054),
    ("front_60", 67.068, -1302046, -0.442, 0.316),
]
for name, k1, k2, p1, p2 in test_params:
    valid = is_distortion_valid(k1, k2, p1, p2)
    print(f"{name} 畸变参数有效: {valid}")

print("\n---相机类型检查---")

camera_name = ["front_120", "left_front_100", "right_front_100", "front_60", "front_fish_eye", "rear_120"]
for cam in camera_name:
    if "front" in cam:
        print(f"{cam} 是前置摄像头")
    elif "rear" in cam:
        print(f"{cam} 是后置摄像头")
    elif "left" in cam:
        print(f"{cam} 是左侧摄像头")
    elif "right" in cam:
        print(f"{cam} 是右侧摄像头")
    else:
        print(f"{cam} 的类型未知")


print("\n---相机处理策略---")

def get_processiong_strategy(camera_name: str) -> str:
    if "front" in camera_name and "fish_eye" in camera_name:
        return "使用鱼眼矫正算法"
    elif "front" in camera_name:
        return "使用前置摄像头处理流程"
    elif "rear" in camera_name:
        return "使用后置摄像头处理流程"
    elif "left" in camera_name or "right" in camera_name:
        return "使用侧面摄像头处理流程"
    else:
        return "未知摄像头类型，使用默认处理流程"
    
for cam in camera_name:
    strategy = get_processiong_strategy(cam)
    print(f"{cam} -> 处理策略: {strategy}")