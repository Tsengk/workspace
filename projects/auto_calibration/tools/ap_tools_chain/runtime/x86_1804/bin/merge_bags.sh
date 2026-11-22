
#!/bin/bash

# 检查输入参数数量
if [ $# -ne 2 ]; then
    echo "错误: 请提供两个参数"
    echo "用法: $0 <输入原始数据包路径> <输出目录路径>"
    exit 1
fi

RAW_INPUT="$1"
OUTPUT_DIR="$2"

# 验证输入文件是否存在
if [ ! -f "$RAW_INPUT" ]; then
    echo "错误: 输入文件不存在: $RAW_INPUT"
    exit 1
fi

# 创建输出目录（如果不存在）
mkdir -p "$OUTPUT_DIR"

# 获取用户输入的路径和文件名信息
DIR_PATH=$(dirname "$RAW_INPUT")
BASENAME=$(basename "$RAW_INPUT")
# 去除扩展名获取纯文件名
NAME_WITHOUT_EXT="${BASENAME%.*}"

echo "正在处理文件: $BASENAME"
echo "解析中间文件所在路径: $DIR_PATH"
echo "最终输出目录: $OUTPUT_DIR"

# 1. 执行 gsdsbagparaser 进行数据解析
echo "正在执行 gsdsbagparaser 解析..."
./gsdsbagparaser "$RAW_INPUT" --plugin ./libgsdsmsgpack2rosplugin.so --config ./gsds_bd_topic_bag_config.tmp

if [ $? -ne 0 ]; then
    echo "错误: gsdsbagparaser 执行失败"
    exit 1
fi

# 2. 确定输出文件名称
# 文件名称是 原始文件名称+ros.bag
OUTPUT_FILENAME="${NAME_WITHOUT_EXT}ros.bag"
OUTPUT_BAG="${OUTPUT_DIR}/${OUTPUT_FILENAME}"

echo "目标合并文件: $OUTPUT_BAG"

# 3. 在用户输入路径(即原始文件所在目录 DIR_PATH)下寻找以 ros_文件名称*.bag 的文件
SEARCH_PATTERN="${DIR_PATH}/ros_${NAME_WITHOUT_EXT}*.bag"

# 使用 glob 展开文件列表
shopt -s nullglob
BAG_FILES=($SEARCH_PATTERN)
shopt -u nullglob

if [ ${#BAG_FILES[@]} -eq 0 ]; then
    echo "警告: 在路径 $DIR_PATH 下未找到匹配模式 '$SEARCH_PATTERN' 的 .bag 文件"
    exit 1
fi

echo "找到 ${#BAG_FILES[@]} 个待合并的 bag 文件:"
for f in "${BAG_FILES[@]}"; do
    echo "  - $f"
done

# 4. 调用 Python 脚本进行合并
PYTHON_SCRIPT="./merge_bags_wildcard.py"

if [ ! -f "$PYTHON_SCRIPT" ]; then
    echo "错误: 未找到 Python 合并脚本 $PYTHON_SCRIPT"
    exit 1
fi

echo "开始调用 Python 脚本合并数据包..."

# 构建 Python 脚本的参数
ARGS="-o \"$OUTPUT_BAG\""
for bag in "${BAG_FILES[@]}"; do
    ARGS="$ARGS \"$bag\""
done

# 执行合并
eval python2 $PYTHON_SCRIPT $ARGS

if [ $? -eq 0 ]; then
    echo "成功: 合并后的数据包已保存为 $OUTPUT_BAG"
else
    echo "错误: Python 合并脚本执行失败"
    exit 1
fi
