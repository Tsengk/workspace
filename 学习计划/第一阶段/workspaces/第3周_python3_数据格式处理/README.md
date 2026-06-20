# Python3 数据格式处理练习

> 对应输出物模板：3.2
> 知识点：JSON / CSV / YAML 解析与转换

## 练习目标

基于 GSDS / ICAOS_TOOL / 测试任务软件 的数据文件，完成至少 1 组真实场景格式转换。

## 目录结构

```
第3周_python3_数据格式处理/
├── test_data/                  ← 测试数据（独立目录，可脱离外部系统运行）
│   ├── test_device.json        #   设备配置 JSON（6 芯片方案 / 48 核心 / 5 编译设备）
│   ├── test_fault.csv          #   故障码 CSV（31 条，4 个行为模式，4 个大类）
│   └── test_calib.yaml         #   相机标定 YAML（MEI 模型，畸变/投影/镜面参数）
├── json处理.py                 #   JSON 深度解析
├── csv处理.py                  #   CSV 聚合分析
├── yaml处理.py                 #   YAML 批量解析
├── 格式转换.py                 #   四组互转综合练习
├── README.md
├── 学习笔记.md
└── *.json / *.csv              #   输出产物（脚本运行后生成）
```

## 运行方式

```bash
cd 第3周_python3_数据格式处理

# 单个运行（优先使用 test_data/ 下的本地文件）
python3 json处理.py
python3 csv处理.py
python3 yaml处理.py
python3 格式转换.py
```

> 脚本按优先级查找数据：
> 1. `test_data/` 本地测试文件（可脱离外部系统独立运行）
> 2. 外部真实系统路径（GSDS / ZMQ_TOOL / 标定目录）

## 测试数据

| 文件 | 内容 | 规模 |
|------|------|------|
| `test_data/test_device.json` | 模拟 GSDS 设备配置，含 `cpu_chip_vec` 和 `compile_device_vec` | 6 芯片方案，48 核心行，5 编译设备 |
| `test_data/test_fault.csv` | 模拟 ZMQ_TOOL 故障码表，覆盖惯导/相机/雷达/规划/系统等大类 | 31 条故障码 |
| `test_data/test_calib.yaml` | 单相机标定示例（MEI 模型），含畸变 k1/k2/p1/p2、投影参数、镜面 xi | 1 台相机 |

## 脚本说明

| 脚本 | 内容 | test_data 数据源 | 外部回退数据源 |
|------|------|-----------------|---------------|
| `json处理.py` | 嵌套解析、扁平化、分组聚合、输出 | `test_device.json` | GSDS `device.json` |
| `csv处理.py` | 行为模式统计、跨模式影响分析、JSON 报告 | `test_fault.csv` | ZMQ_TOOL `fault.csv` |
| `yaml处理.py` | 批量解析、畸变分析、JSON/CSV 导出 | `test_calib.yaml` | 标定批量目录 `*.yaml` |
| `格式转换.py` | JSON→CSV / CSV→JSON / YAML→JSON→CSV / IR 内存模式 | 全部三个文件 | 全部外部路径 |

## 知识点覆盖

- [x] `json.load()` / `json.dump()` / `json.loads()` / `json.dumps()`
- [x] `csv.DictReader` / `csv.DictWriter`
- [x] `yaml.safe_load()` 读取 YAML
- [x] 嵌套 dict/list 的递归/迭代遍历
- [x] `defaultdict` / `Counter` 分组聚合
- [x] 数据扁平化（嵌套结构 → 二维表）
- [x] 数据结构化（二维表 → 嵌套 JSON）
- [x] **核心模式：任何格式互转都走 dict/list 中间表示（IR）**
- [x] 批量文件 glob 遍历 (`pathlib.Path.glob`)
- [x] 异常处理（文件不存在、解析失败）
- [x] try/except 对可选依赖的处理（pyyaml）
- [x] OpenCV `%YAML:1.0` 指令兼容

## 练习说明

- 原始数据格式：JSON（GSDS 设备配置）、CSV（ZMQ_TOOL 故障码表）、YAML（相机标定文件）
- 目标格式：JSON 报告、CSV 表格、YAML 汇总
- 处理逻辑概述：
  1. 读取源文件，用对应标准库解析为 dict/list（中间表示 IR）
  2. 在 IR 上做筛选、聚合、分组、扁平化等处理
  3. 输出为目标格式文件

## 输出文件（运行后生成）

| 文件 | 来源脚本 |
|------|----------|
| `device_flat.json` | json处理.py |
| `故障统计报告.json` | csv处理.py |
| `相机标定参数汇总.json` | yaml处理.py |
| `相机标定参数汇总.csv` | yaml处理.py |
| `device_chip_cores.csv` | 格式转换.py |
| `fault_by_category.json` | 格式转换.py |
| `calibration_all.json` | 格式转换.py |
| `calibration_all.csv` | 格式转换.py |
| `sensor_status.json` | 格式转换.py |
| `sensor_status.csv` | 格式转换.py |
