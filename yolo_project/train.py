# train.py — YOLOv8 图像分类训练（8类玫瑰病害）
import sys
import os
from ultralytics import YOLO

# 以脚本所在目录为工程根目录，不依赖绝对路径，跨设备通用
ROOT = os.path.dirname(os.path.abspath(__file__))
DATA_PATH = os.path.join(ROOT, 'datasets')
RUNS_PATH = os.path.join(ROOT, 'runs')


def train(quick=False):
    """训练分类模型
    quick=True:  快速验证流程 (epochs=3, imgsz=224, 约 10 分钟)
    quick=False: 正式训练     (epochs=100, imgsz=640, 约 8-12 小时)
    """
    model = YOLO('yolov8n-cls.pt')

    if quick:
        print("⚡ 快速验证模式 (epochs=3, imgsz=224)")
        model.train(
            data=DATA_PATH,
            epochs=3,
            imgsz=224,
            batch=8,
            device='cpu',
            workers=0,
            project=RUNS_PATH,
            name='rose_quick',
        )
        print("快速验证完成！模型保存在 runs/rose_quick/weights/best.pt")
    else:
        print("🔥 正式训练模式 (epochs=100, imgsz=640)")
        model.train(
            data=DATA_PATH,
            epochs=100,
            imgsz=640,
            batch=8,
            device='cpu',
            workers=0,
            project=RUNS_PATH,
            name='rose_disease',
        )
        print("训练完成！模型保存在 runs/rose_disease/weights/best.pt")


if __name__ == '__main__':
    quick = '--quick' in sys.argv
    train(quick=quick)
