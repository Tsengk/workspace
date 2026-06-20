# predict.py — 使用训练好的分类模型预测玫瑰病害
import sys
import os
from collections import defaultdict
import cv2
import numpy as np
from ultralytics import YOLO

ROOT = os.path.dirname(os.path.abspath(__file__))
DEFAULT_MODEL = os.path.join(ROOT, 'runs', 'rose_disease', 'weights', 'best.pt')
BATCH_SIZE = 200
OUT_DIR = os.path.join(ROOT, 'runs', 'predict')

_model_path = None
_do_save = False


def _get_model(path):
    global _model_path
    _model_path = path


def _annotate(image_path, label, conf, top5, save_path):
    """在图片底部扩展灰底，文字打印在灰底上，不动原图"""
    img = cv2.imread(image_path)
    h, w = img.shape[:2]

    font_scale = max(0.7, w / 900)
    line_h = int(24 * font_scale)
    padding = int(12 * font_scale)
    num_lines = 1 + len(top5)
    bar_h = num_lines * line_h + padding * 2

    # 底部扩展灰色区域
    bar = np.full((bar_h, w, 3), (45, 45, 45), dtype=np.uint8)
    img = np.vstack([img, bar])

    # 主结果（白色大字）
    y = h + padding + line_h
    cv2.putText(img, f"{label}  {conf:.1%}", (padding, y),
                cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255), 2)

    # Top-5（浅灰小字）
    small_scale = font_scale * 0.5
    for i, (cls_name, cls_conf) in enumerate(top5):
        t = f"{cls_name}  {cls_conf:.1%}"
        y = h + padding + (2 + i) * line_h
        cv2.putText(img, t, (padding + 5, y),
                    cv2.FONT_HERSHEY_SIMPLEX, small_scale, (180, 180, 180), 1)

    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    cv2.imwrite(save_path, img)
    print(f"结果图已保存: {save_path}")


def predict_one(image_path):
    if not os.path.exists(image_path):
        print(f"文件不存在: {image_path}")
        return
    model = YOLO(_model_path)
    results = model(image_path)
    for r in results:
        name = r.names[r.probs.top1]
        conf = r.probs.top1conf.item()
        top5 = list(r.probs.top5)
        print(f"图片: {image_path}")
        print(f"结果: {name}  ({conf:.2%})")
        print("Top-5:")
        for idx in top5:
            print(f"  {r.names[idx]:20s}  {r.probs.data[idx].item():.2%}")

        if _do_save:
            fname = os.path.splitext(os.path.basename(image_path))[0]
            save_path = os.path.join(OUT_DIR, f"{fname}_pred.jpg")
            top5_data = [(r.names[i], r.probs.data[i].item()) for i in top5]
            _annotate(image_path, name, conf, top5_data, save_path)


def predict_folder(folder_path):
    if not os.path.isdir(folder_path):
        print(f"文件夹不存在: {folder_path}")
        return
    model = YOLO(_model_path)
    exts = {'.jpg', '.jpeg', '.png', '.bmp'}
    files = [f for f in os.listdir(folder_path)
             if os.path.splitext(f)[1].lower() in exts]
    if not files:
        print("未找到图片文件")
        return
    correct = total = 0
    for i in range(0, len(files), BATCH_SIZE):
        batch_paths = [os.path.join(folder_path, f) for f in files[i:i + BATCH_SIZE]]
        results = model(batch_paths)
        for r, full_path in zip(results, batch_paths):
            cls = r.names[r.probs.top1]
            conf = r.probs.top1conf.item()
            true_label = os.path.basename(os.path.dirname(full_path))
            match = '✅' if true_label == cls else '❌'
            if true_label == cls:
                correct += 1
            total += 1
            print(f"{match} {os.path.basename(full_path):40s} → {cls:20s} ({conf:.2%})")

            if _do_save:
                fname = os.path.splitext(os.path.basename(full_path))[0]
                save_path = os.path.join(OUT_DIR, true_label, f"{fname}_pred.jpg")
                top5_data = [(r.names[i], r.probs.data[i].item()) for i in r.probs.top5]
                _annotate(full_path, cls, conf, top5_data, save_path)

    if total > 0:
        print(f"\n准确率: {correct}/{total} = {correct/total:.2%}")


def validate(data_path):
    model = YOLO(_model_path)
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

    correct = 0
    total = 0
    per_class = defaultdict(lambda: {'correct': 0, 'total': 0})
    confusion = defaultdict(lambda: defaultdict(int))

    for i in range(0, len(images), BATCH_SIZE):
        batch = images[i:i + BATCH_SIZE]
        results = model(batch)
        for r in results:
            pred = r.names[r.probs.top1]
            true = os.path.basename(os.path.dirname(r.path))
            per_class[true]['total'] += 1
            confusion[true][pred] += 1
            if pred == true:
                correct += 1
                per_class[true]['correct'] += 1
            total += 1
        print(f"\r验证中... {min(i + BATCH_SIZE, len(images))}/{len(images)}",
              end='', flush=True)

    print(f"\n\n整体准确率: {correct}/{total} = {correct/total:.2%}")
    print()
    print(f"{'类别':20s}  {'正确':>6s}  {'总数':>6s}  {'准确率':>8s}")
    print("-" * 44)
    for cls_name in sorted(per_class.keys()):
        d = per_class[cls_name]
        acc = d['correct'] / d['total'] if d['total'] > 0 else 0
        print(f"{cls_name:20s}  {d['correct']:6d}  {d['total']:6d}  {acc:7.1%}")

    print(f"\n混淆矩阵 (行=真实, 列=预测):")
    classes = sorted(confusion.keys())
    header = "真实\\预测" + "".join(f"{c[:6]:>8s}" for c in classes)
    print(header)
    for true_cls in classes:
        row = f"{true_cls:10s}"
        for pred_cls in classes:
            row += f"{confusion[true_cls][pred_cls]:8d}"
        print(row)


if __name__ == '__main__':
    args = sys.argv[1:]
    model_path = DEFAULT_MODEL
    target = None
    do_val = False

    i = 0
    while i < len(args):
        if args[i] == '--model' and i + 1 < len(args):
            model_path = args[i + 1]
            i += 2
        elif args[i] == '--val':
            do_val = True
            i += 1
        elif args[i] == '--save':
            _do_save = True
            i += 1
        elif not target:
            target = args[i]
            i += 1
        else:
            i += 1

    _get_model(model_path)
    if not os.path.exists(_model_path):
        print(f"模型不存在: {_model_path}")
        print("请先运行 python train.py --quick 或 python train.py")
        sys.exit(1)

    if do_val:
        validate(os.path.join(ROOT, 'datasets'))
    elif target and os.path.isfile(target):
        predict_one(target)
    elif target and os.path.isdir(target):
        predict_folder(target)
    elif target:
        print(f"文件或文件夹不存在: {target}")
    else:
        print("用法:")
        print("  python predict.py 图片路径 --save              # 预测并保存结果图")
        print("  python predict.py 文件夹路径 --save            # 批量预测并保存")
        print("  python predict.py --val                        # 验证集准确率")
        print("  python predict.py 图片 --model <模型路径>       # 指定模型")
