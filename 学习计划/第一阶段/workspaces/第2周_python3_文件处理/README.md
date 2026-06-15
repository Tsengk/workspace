# Python3 文件处理练习

> 任务: 结合 GSDS、ZMQ_TOOL 的真实文件/日志/配置，完成 python3 文件处理

| 文件 | 内容 | 数据源 |
|------|------|--------|
| `1日志分析.py` | 读日志，按 INFO/WARNING/ERROR 统计 | `/home/idriver/TOOLS/zmq_tool/backend/zmq_backend.log` |
| `2配置查询.py` | JSON 配置解析，按架构分组 | `/home/idriver/gsds-1.2.0-beta/basic_info/cfg/compile.json` |
| `3故障统计.py` | CSV 读取筛选，多字段分类，输出报告 | `/home/idriver/TOOLS/zmq_tool/protocols/fault.csv` |

运行: `python3 1日志分析.py`
