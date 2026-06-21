#!/usr/bin/env python3

import json
import csv
import yaml
import os
from pathlib import Path
from collections import defaultdict

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
TEST_DATA = os.path.join(SCRIPT_DIR, "test_data")
OUTPUT_DATA = os.path.join(SCRIPT_DIR, "output_data")

def find_file(local_name):
    local = os.path.join(TEST_DATA, local_name)
    if os.path.exists(local):
        return local
    return None

def load_calib_yaml(filepath):
    with open(filepath, "r", encoding="utf-8") as f:
        raw = f.read()
    lines = raw.split("\n")
    cleaned = "\n".join(line for line in lines if not line.startswith("%YAML") and line.strip() != "---")
    return yaml.safe_load(cleaned)
    
# JSON->CSV
print("=" * 60)
print("JSON->CSV")
print("=" * 60)

device_path = find_file("test_device.json")

if device_path:
    print(f"数据源：{device_path}")
    with open(device_path, "r", encoding="utf-8") as f:
        device_data = json.load(f)

    flat_rows = []
    for chip in device_data.get("cpu_chip_vec",[]):#字典取值
        chip_name = chip.get("chip_name","?")
        for i, core in enumerate(chip.get("cpu_cores", []), 1):
            flat_rows.append({
                "芯片": chip_name,
                "核心数": i,
                "核心类型": core,
            })
    
    output = os.path.join(OUTPUT_DATA, "device_chip_cores.csv")
    with open(output, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["芯片", "核心数", "核心类型"])
        writer.writeheader()
        writer.writerows(flat_rows)
    print(f"JSON->CSV:{os.path.basename(output)} ({len(flat_rows)}行)")
else:
    print(f"未找到设备配置JSON")

# CSV->JSON
print("=" * 60)
print("CSV->JSON")
print("=" * 60)

fault_path = find_file("test_fault.csv")

if fault_path:
    print(f"数据源：{fault_path}")
    with open(fault_path, "r", encoding="utf-8") as f:
        # 多个字典列表
        reader = csv.DictReader(f)
        faults = list(reader)
    # 如果访问不存在的键自动生成空列表[]
    by_category = defaultdict(list)
    # 单独拿一行字典赋给f
    for f in faults:
        code = f["故障码"]#字典取值
        cat = code[0]
        by_category[cat].append({
            "code": code,
            "desc": f["故障描述"],
            "visible": f.get("是否显示","0").strip() == "1",
        })
        # {
        #     "1": [
        #         {
        #             "code": "1052001",
        #             "desc": "配置参数读取失败",
        #             "visible": True,
        #         },
        #         {
        #             "code": "1052001",
        #             "desc": "配置参数读取失败",
        #             "visible": True,
        #         }
        #     ],
        #     "2": [
        #         {
        #             "code": "2052001",
        #             "desc": "配置参数读取失败",
        #             "visible": True,
        #         },
        #         {
        #             "code": "2052001",
        #             "desc": "配置参数读取失败",
        #             "visible": True,
        #         }
        #     ]
        # }
    fault_json = {
        "数据源": fault_path,
        "总条数": len(faults),
        "按大类分组": {
            cat: {"条数": len(items), "故障列表": items} for cat, items in sorted(by_category.items())
            # cat: {"条数": len(by_category[cat]), "故障列表": by_category[cat]} for cat in sorted(by_category)
            # 使用.items()
            # by_category = {
            #     "A": ["记录1", "记录2"],  # 键是 "A"，值是列表
            #     "B": ["记录3"]           # 键是 "B"，值是列表
            # }
            # ->变成一个包含元组(Tuple)的列表形式
            # [
            #     ("A", ["记录1", "记录2"]),  # 第一对
            #     ("B", ["记录3"])            # 第二对
            # ]
        },
    }

    output = os.path.join(OUTPUT_DATA, "fault_by_category.json")
    with open(output, "w", encoding="utf-8") as f:
        json.dump(fault_json, f, ensure_ascii=False, indent=2)#ensure_ascii=False保留中文原貌，indent=2遇到新的层级(字典或列表等)换行并缩进两个字符
    print(f"CSV->JSON:{os.path.basename(output)} ({len(faults)}条)")
else:
    print(f"未找到故障码CSV")

# YAML->JSON->CSV
print("=" * 60)
print("YAML->JSON->CSV")
print("=" * 60)

LOCAL_CALIB = os.path.join(TEST_DATA, "test_calib.yaml")


yaml_file = Path(LOCAL_CALIB)
all_cams = []
data = load_calib_yaml(str(yaml_file))

cam = {
    "相机名": data.get("camera_name", "?"),
    "模型": data.get("model_type", "?"),
    "宽": data.get("image_width", 0),
    "高": data.get("image_height", 0),
}
proj = data.get("projection_parameters", {})
if proj:
    cam["gamma1"] = proj.get("gamma1", 0)
    cam["gamma2"] = proj.get("gamma2", 0)
    cam["u0"] = proj.get("u0", 0)
    cam["v0"] = proj.get("v0", 0)
dist = data.get("distortion_parameters", {})
if dist:
    cam["k1"] = dist.get("k1", 0)
    cam["k2"] = dist.get("k2", 0)
all_cams.append(cam)
if all_cams:
    out_json = os.path.join(OUTPUT_DATA, "calibration_all.json")
    with open(out_json, "w", encoding="utf-8", newline="") as f:
        json.dump(all_cams, f, ensure_ascii=False, indent=2)
    print(f"YAML->JSON:{os.path.basename(out_json)}")

    out_csv = os.path.join(OUTPUT_DATA, "calibration_all.csv")
    with open(out_csv, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=all_cams[0].keys())
        writer.writeheader()
        writer.writerows(all_cams)
    print(f"JSON->CSV:{os.path.basename(out_csv)}")