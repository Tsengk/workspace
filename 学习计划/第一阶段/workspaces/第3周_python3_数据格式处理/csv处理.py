#!/usr/bin/env python3
"""
结合 ZMQ_TOOL 故障码表完成 CSV 解析与聚合分析练习

数据源（按优先级）:
  1. 本地测试文件: test_fault.csv (同目录, 30 条)
  2. 真实文件:     /home/idriver/TOOLS/zmq_tool/protocols/fault.csv (540 条)

知识点:
- csv.DictReader 读取带表头的 CSV
- 列表推导式做条件筛选
- 按多列行为值分组统计
- 将结果输出为 JSON 报告
"""

import csv
import json
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
TEST_DATA = os.path.join(SCRIPT_DIR, "test_data")
CANDIDATES = [
    os.path.join(TEST_DATA, "test_fault.csv"),
    "/home/idriver/TOOLS/zmq_tool/protocols/fault.csv",
]

FAULT_CSV = None
for path in CANDIDATES:
    if os.path.exists(path):
        FAULT_CSV = path
        break

if FAULT_CSV is None:
    print("❌ 未找到数据文件，请确认 test_fault.csv 或 ZMQ_TOOL 故障码表存在")
    exit(1)

print(f"📂 数据源: {FAULT_CSV}")

# 1. 读取 CSV
with open(FAULT_CSV, "r", encoding="utf-8") as f:
    reader = csv.DictReader(f)
    faults = list(reader)

print("=" * 60)
print("故障码 CSV 解析与聚合分析")
print("=" * 60)
print(f"总计 {len(faults)} 条故障码")
print(f"字段: {reader.fieldnames}\n")

# 2. 统计各行为列的严重级别分布
#    取值含义: 0=不影响, 1=警告, 2=故障
BEHAVIORS = ["循迹/突击", "自主干预", "应急避险", "跟随/突击"]

print("--- 各行为模式下故障严重度分布 ---")
print(f"{'行为模式':<14} {'0(不影响)':>8} {'1(警告)':>8} {'2(故障)':>8}")
print("-" * 50)

for bh in BEHAVIORS:
    cnt_0 = sum(1 for f in faults if f.get(bh, "0").strip() == "0")
    cnt_1 = sum(1 for f in faults if f.get(bh, "0").strip() == "1")
    cnt_2 = sum(1 for f in faults if f.get(bh, "0").strip() == "2")
    print(f"{bh:<14} {cnt_0:>8} {cnt_1:>8} {cnt_2:>8}")

# 3. 提取影响面最广的故障（在多数行为列中都是 2）
print("\n--- 跨模式影响最广故障 (≥2 个行为列为故障) ---")
cross_faults = []
for f in faults:
    score = sum(1 for bh in BEHAVIORS if f.get(bh, "0").strip() == "2")
    if score >= 2:
        f["_cross_score"] = score
        cross_faults.append(f)

cross_faults.sort(key=lambda x: x["_cross_score"], reverse=True)
if cross_faults:
    for f in cross_faults[:10]:
        print(f"  [{f['_cross_score']}模式] {f['故障码']} — {f['故障描述']}")
else:
    print("  (无跨模式影响 ≥2 的故障)")

# 4. 按故障码前缀分组（第1位=大类）
print("\n--- 故障码大类统计 ---")
category = {}
for f in faults:
    code = f["故障码"]
    prefix = code[0] if code else "?"
    category.setdefault(prefix, []).append(code)

for k in sorted(category.keys()):
    print(f"  大类 {k}: {len(category[k])} 条 (示例: {category[k][:3]})")

# 5. 输出 JSON 报告
report = {
    "数据源": FAULT_CSV,
    "总故障码数": len(faults),
    "行为分布": {},
    "跨模式故障Top10": [
        {"故障码": f["故障码"], "描述": f["故障描述"], "影响模式数": f["_cross_score"]}
        for f in cross_faults[:10]
    ],
}

for bh in BEHAVIORS:
    items = [f for f in faults if f.get(bh, "0").strip() == "2"]
    report["行为分布"][bh] = {"故障数": len(items), "故障码": [i["故障码"] for i in items]}

output_path = os.path.join(SCRIPT_DIR, "故障统计报告.json")
with open(output_path, "w", encoding="utf-8") as out:
    json.dump(report, out, ensure_ascii=False, indent=2)
print(f"\n已输出 JSON 报告: {os.path.basename(output_path)}")

# 6. 内存中的格式转换：从 CSV 原始数据 → 按行为模式重排的 JSON
print(f"\n--- 按行为模式索引 (内存转换 CSV→结构化 JSON) ---")
behavior_index = {}
for bh in BEHAVIORS:
    items_2 = [f for f in faults if f.get(bh, "0").strip() == "2"]
    behavior_index[bh] = [
        {"code": f["故障码"], "desc": f["故障描述"]} for f in items_2
    ]

for bh, items in behavior_index.items():
    print(f"{bh}: {len(items)} 条")
    for it in items[:2]:
        print(f"    {it['code']} {it['desc']}")

print("\n✅ CSV 解析练习完成")
