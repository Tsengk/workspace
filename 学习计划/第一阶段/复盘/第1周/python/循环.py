#!/usr/bin/env python3

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

print("=" * 50)
print("批量相机参数处理 — 循环控制")
print("=" * 50)

for cam in cameras:
    print(f"{cam['name']} -> k1: {cam['k1']:.6f}, k2: {cam['k2']:.6f}, epsilon: {cam['epsilon']:.4f}")