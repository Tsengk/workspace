# predict.py — 使用训练好的分类模型预测玫瑰病害
import sys
import os
from collections import defaultdict
from ultralytics import YOLO

MODEL_PATH = 'runs/classify/rose_disease/weights/best.pt'


def predict_one(image_path):
    """预测单张图片"""
    if not os.path.exists(image_path):
        print(f"文件不存在: {image_path}")
        return None

    model = YOLO(MODEL_PATH)
    results = model(image_path)

    for r in results:
        name = r.names[r.probs.top1]
        conf = r.probs.top1conf.item()
        print(f"图片: {image_path}")
        print(f"结果: {name}  ({conf:.2%}")

        print("Top-5:")
        for idx in r.probs.top5:
            print(f"  {r.names[idx]:20s}  {r.probs.data[idx].item():.2%}")

    return results


def predict_folder(folder_path):
    """预测文件夹中所有图片"""
    if not os.path.isdir(folder_path):
        print(f"文件夹不存在: {folder_path}")
        return

    model = YOLO(MODEL_PATH)
    exts = {'.jpg', '.jpeg', '.png', '.bmp'}
    files = [f for f in os.listdir(folder_path)
             if os.path.splitext(f)[1].lower() in exts]

    if not files:
        print("未找到图片文件")
        return

    results = model(os.path.join(folder_path, f) for f in files)

    correct = total = 0
    for r in results:
        cls = r.names[r.probs.top1]
        conf = r.probs.top1conf.item()
        true_label = os.path.basename(os.path.dirname(r.path))
        match = '✅' if true_label == cls else '❌'
        if true_label == cls:
            correct += 1
        total += 1
        print(f"{match} {r.path.split('/')[-1]:40s} → {cls:20s} ({conf:.2%})")

    if total > 0:
        print(f"\n准确率: {correct}/{total} = {correct/total:.2%}")


def validate(data_path):
    """验证模型在整个验证集上的准确率（按类别统计）"""
    model = YOLO(MODEL_PATH)

    # 收集所有验证图片
    val_root = os.path.join(data_path, 'val')
    if not os.path.isdir(val_root):
        print(f"验证集不存在: {val_root}")
        return

    exts = {'.jpg', '.jpeg', '.png', '.bmp'}
    images = []
    for cls_name in sorted(os.listdir(val_root)):
        cls_dir = os.path.join(val_root, cls_name)
        if not os.path.isdir(cls_dir):
            continue
        for f in os.listdir(cls_dir):
            if os.path.splitext(f)[1].lower() in exts:
                images.append(os.path.join(cls_dir, f))

    if not images:
        print("未找到验证图片")
        return

    # 批量预测
    results = model(images)

    # 统计
    correct = 0
    per_class = defaultdict(lambda: {'correct': 0, 'total': 0})
    confusion = defaultdict(lambda: defaultdict(int))

    for r in results:
        pred = r.names[r.probs.top1]
        true = os.path.basename(os.path.dirname(r.path))
        per_class[true]['total'] += 1
        confusion[true][pred] += 1
        if pred == true:
            correct += 1
            per_class[true]['correct'] += 1

    # 输出
    total = len(images)
    print(f"\n整体准确率: {correct}/{total} = {correct/total:.2%}")
    print()
    print(f"{'类别':20s}  {'正确':>6s}  {'总数':>6s}  {'准确率':>8s}")
    print("-" * 44)

    for cls_name in sorted(per_class.keys()):
        d = per_class[cls_name]
        acc = d['correct'] / d['total'] if d['total'] > 0 else 0
        print(f"{cls_name:20s}  {d['correct']:6d}  {d['total']:6d}  {acc:7.1%}")

    # 混淆矩阵
    print(f"\n混淆矩阵 (行=真实, 列=预测):")
    classes = sorted(confusion.keys())
    # 表头
    header = "真实\\预测" + "".join(f"{c[:6]:>8s}" for c in classes)
    print(header)
    for true_cls in classes:
        row = f"{true_cls:10s}"
        for pred_cls in classes:
            row += f"{confusion[true_cls][pred_cls]:8d}"
        print(row)


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("用法:")
        print("  python predict.py 图片路径              # 单张图预测")
        print("  python predict.py 文件夹路径            # 批量预测")
        print("  python predict.py --val                 # 验证集准确率")
        sys.exit(1)

    target = sys.argv[1]

    if target == '--val':
        validate('/home/idriver/workspace/yolo_project/datasets')
    elif os.path.isfile(target):
        predict_one(target)
    elif os.path.isdir(target):
        predict_folder(target)
    else:
        print(f"路径无效: {target}")
