#!/usr/bin/env python3
"""
格式互转综合练习：JSON ↔ CSV ↔ YAML

场景: 结合 GSDS 设备配置、ZMQ_TOOL 故障码、相机标定等数据，
      完成 JSON→CSV、CSV→JSON、YAML→JSON 的互转流水线。

数据源（按优先级）:
  1. 本地测试文件: test_device.json / test_fault.csv / test_calib.yaml
  2. 真实系统文件: GSDS / ZMQ_TOOL / 标定目录

知识点:
- json / csv / yaml 三个标准库的组合使用
- 数据扁平化（嵌套结构→二维表）
- 数据结构化（二维表→嵌套 JSON）
- 中转格式：用 dict/list 作为中间表示做任意格式互转
"""

import json
import csv
import os
from pathlib import Path
from collections import defaultdict

# 尝试导入 yaml
try:
    import yaml
except ImportError:
    import subprocess
    subprocess.run(["pip", "install", "pyyaml"], check=False)
    import yaml

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
TEST_DATA = os.path.join(SCRIPT_DIR, "test_data")


def find_file(local_name, external_path):
    """优先本地测试文件，回退到外部系统路径"""
    local = os.path.join(TEST_DATA, local_name)
    if os.path.exists(local):
        return local
    if os.path.exists(external_path):
        return external_path
    return None


# 兼容 OpenCV 的 %YAML:1.0 指令
def load_calib_yaml(filepath):
    with open(filepath, "r", encoding="utf-8") as f:
        raw = f.read()
    lines = raw.split("\n")
    cleaned = "\n".join(
        line for line in lines
        if not line.startswith("%YAML") and line.strip() != "---"
    )
    return yaml.safe_load(cleaned)


# ============================================================
# 第 1 组: JSON → CSV (嵌套结构扁平化)
# 数据源: 设备配置 JSON → 芯片-核心明细表
# ============================================================

print("=" * 60)
print("第 1 组: JSON → CSV (嵌套结构 → 二维表)")
print("=" * 60)

device_path = find_file("test_device.json",
                        "/home/idriver/gsds-1.2.0-beta/basic_info/cfg/device.json")
if device_path:
    print(f"📂 数据源: {device_path}")
    with open(device_path, "r", encoding="utf-8") as f:
        device_data = json.load(f)

    flat_rows = []
    for chip in device_data.get("cpu_chip_vec", []):
        chip_name = chip.get("chip_name", "?")
        for i, core in enumerate(chip.get("cpu_cores", []), 1):
            flat_rows.append({
                "芯片方案": chip_name,
                "核心序号": i,
                "核心类型": core,
            })

    output = os.path.join(SCRIPT_DIR, "device_chip_cores.csv")
    with open(output, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["芯片方案", "核心序号", "核心类型"])
        writer.writeheader()
        writer.writerows(flat_rows)
    print(f"JSON → CSV: {os.path.basename(output)} ({len(flat_rows)} 行)")
else:
    print("⚠️  未找到设备配置 JSON")

# ============================================================
# 第 2 组: CSV → JSON (二维表 → 结构化分组)
# 数据源: 故障码 CSV → 按大类分组的 JSON
# ============================================================

print(f"\n{'=' * 60}")
print("第 2 组: CSV → JSON (二维表 → 结构化分组)")
print("=" * 60)

fault_path = find_file("test_fault.csv",
                       "/home/idriver/TOOLS/zmq_tool/protocols/fault.csv")
if fault_path:
    print(f"📂 数据源: {fault_path}")
    with open(fault_path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        faults = list(reader)

    by_category = defaultdict(list)
    for f in faults:
        code = f["故障码"]
        cat = code[0]
        by_category[cat].append({
            "code": code,
            "desc": f["故障描述"],
            "visible": f.get("是否显示", "0").strip() == "1",
        })

    fault_json = {
        "数据源": fault_path,
        "总条数": len(faults),
        "按大类分组": {
            cat: {"条数": len(items), "故障列表": items}
            for cat, items in sorted(by_category.items())
        },
    }

    output = os.path.join(SCRIPT_DIR, "fault_by_category.json")
    with open(output, "w", encoding="utf-8") as f:
        json.dump(fault_json, f, ensure_ascii=False, indent=2)
    print(f"CSV → JSON: {os.path.basename(output)} ({len(faults)} 条)")
else:
    print("⚠️  未找到故障码 CSV")

# ============================================================
# 第 3 组: YAML → JSON → CSV (批量标定文件 → 统一格式)
# 数据源: 相机标定 YAML
# ============================================================

print(f"\n{'=' * 60}")
print("第 3 组: YAML → JSON → CSV (批量文件统一化)")
print("=" * 60)

CALIB_DIR = os.path.join(SCRIPT_DIR, "..", "第1周_python3_基础练习", "标定参数提取转换")
LOCAL_CALIB = os.path.join(TEST_DATA, "test_calib.yaml")

# 收集 YAML：优先批量目录，回退本地单文件
yaml_files = []
if os.path.isdir(CALIB_DIR):
    yaml_files = sorted(Path(CALIB_DIR).glob("*.yaml"))
if not yaml_files and os.path.exists(LOCAL_CALIB):
    yaml_files = [Path(LOCAL_CALIB)]

all_cams = []
for yf in yaml_files:
    try:
        data = load_calib_yaml(str(yf))
    except yaml.YAMLError:
        continue

    if data is None or not isinstance(data, dict):
        continue

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

print(f"📂 解析 {len(all_cams)} 台相机标定")

if all_cams:
    out_json = os.path.join(SCRIPT_DIR, "calibration_all.json")
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump(all_cams, f, ensure_ascii=False, indent=2)
    print(f"YAML → JSON: {os.path.basename(out_json)} ({len(all_cams)} 台)")

    out_csv = os.path.join(SCRIPT_DIR, "calibration_all.csv")
    with open(out_csv, "w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=all_cams[0].keys())
        writer.writeheader()
        writer.writerows(all_cams)
    print(f"JSON → CSV: {os.path.basename(out_csv)} ({len(all_cams)} 行)")

# ============================================================
# 第 4 组: 自建数据演示 内存中 JSON ↔ CSV ↔ dict 互转
# ============================================================

print(f"\n{'=' * 60}")
print("第 4 组: 内存中转 (任意格式互转的通用模式)")
print("=" * 60)

# 原则: 任何格式互转，都先转成 Python dict/list 作为中间表示(IR)
#      源格式 → dict/list(IR) → 目标格式

# 模拟一段工具软件返回的 JSON 数据（传感器状态）
sensor_json_str = '''
{
  "timestamp": "2026-06-20T10:30:00",
  "sensors": [
    {"id": "CAM_FRONT_120", "type": "camera", "status": "ok",     "fps": 30},
    {"id": "CAM_LEFT_100",  "type": "camera", "status": "ok",     "fps": 25},
    {"id": "LIDAR_MAIN",    "type": "lidar",  "status": "warning","fps": 10},
    {"id": "IMU_PRIMARY",   "type": "imu",    "status": "ok",     "fps": 100},
    {"id": "GPS_RTK",       "type": "gnss",   "status": "error",  "fps": 0}
  ]
}
'''

# 源格式(str) → IR(dict)
ir = json.loads(sensor_json_str)

# IR(dict) → CSV 字符串
print("JSON → IR → CSV:")
sensors = ir.get("sensors", [])
if sensors:
    fieldnames = list(sensors[0].keys())
    csv_lines = [",".join(fieldnames)]
    for s in sensors:
        csv_lines.append(",".join(str(s[k]) for k in fieldnames))
    csv_text = "\n".join(csv_lines)
    print(csv_text)

    # CSV 字符串 → IR(list) → JSON 筛选
    print(f"\nCSV → IR → JSON (filter: status!=ok):")
    faulty = [s for s in sensors if s["status"] != "ok"]
    print(json.dumps(faulty, ensure_ascii=False, indent=2))

# 输出传感器示例文件
sensor_json_path = os.path.join(SCRIPT_DIR, "sensor_status.json")
with open(sensor_json_path, "w", encoding="utf-8") as f:
    json.dump(ir, f, ensure_ascii=False, indent=2)

sensor_csv_path = os.path.join(SCRIPT_DIR, "sensor_status.csv")
with open(sensor_csv_path, "w", encoding="utf-8", newline="") as f:
    writer = csv.DictWriter(f, fieldnames=sensors[0].keys())
    writer.writeheader()
    writer.writerows(sensors)
print(f"\n已输出: {os.path.basename(sensor_json_path)}, {os.path.basename(sensor_csv_path)}")

print("\n✅ ✅ ✅ 格式互转综合练习完成")
print("总结: 所有格式转换都走 dict/list 中间表示，源→IR→目标，两段式清晰可测")
