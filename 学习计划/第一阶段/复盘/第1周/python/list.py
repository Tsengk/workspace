#!/usr/bin/env python3

import copy
from functools import reduce

camera_list = [
    "front_120", "left_front_100", "right_front_100", "front_60", "front_fish_eye", "rear_120"
]

print("=" * 50)
print("列表操作")
print("=" * 50)

print(f"原始相机列表: {camera_list}")

camera_list.append("rear_60")
print(f"添加rear_60后: {camera_list}")

camera_list.insert(0, "camera_top")

camera_list.remove("left_front_100")
print(f"删除left_front_100后: {camera_list}")

popped = camera_list.pop(2)
print(f"弹出索引2的元素: {popped}, 当前列表: {camera_list}")

extra_cams = ["left_side_100", "right_side_100"]
camera_list.extend(extra_cams)
print(f"扩展列表后: {camera_list}")

print("\n--- 列表切片 ---")
main_cams = camera_list[:6]
print(f"主摄像头列表: {main_cams}")

print(f"倒数第二个摄像头: {camera_list[-2]}")

print(f"每隔一个取一个摄像头: {camera_list[::2]}")

print(f"反转列表: {camera_list[::-1]}")

print(f"索引范围内的摄像头: {camera_list[1:5]}")



print("\n--- 排序 ---")
sorted_by_name = sorted(camera_list)
print(f"按名称排序: {sorted_by_name}")

sorted_by_length = sorted(camera_list, key=len)
print(f"按名称长度排序: {sorted_by_length}")

print ("\n--map/filter/reduce ---")

direction = list(map(
    lambda n: "前" if "front" in n else ("后" if "rear" in n else ("左" if "left" in n else "右")), 
    camera_list
))

for cam, d in list(zip(camera_list, direction))[:5]:
    print(f"{cam} -> 方向: {d}")

fish_eye_cams = list(filter(lambda n: "fish_eye" in n, camera_list))
print(f"\n鱼眼摄像头: {fish_eye_cams}")

standard_cams = list(filter(lambda n: "fish_eye" not in n, camera_list))
print(f"非鱼眼摄像头: {standard_cams}")

all_cams = reduce(lambda a, b: a + " | " + b, camera_list)
print(f"\n所有摄像头: {all_cams}")


fish_upper = [cam.upper() for cam in camera_list if "fish_eye" in cam]
print(f"\n鱼眼摄像头大写: {fish_upper}")

print("\n---拷贝---")
nested = [
    ["front_120", [-0.454, 0.214]],
    ["left_front_100", [1.441, 0.123]],
]
shallow = copy.copy(nested)
deep = copy.deepcopy(nested)

nested[0][1][0] = 999.999
print(f"原始嵌套列表: {nested[0][1]}")
print(f"浅拷贝列表: {shallow[0][1]}")
print(f"深拷贝列表: {deep[0][1]}")

