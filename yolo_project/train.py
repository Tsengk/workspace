# train.py — YOLOv8 图像分类训练（8类玫瑰病害）
from ultralytics import YOLO


def train():
    """训练分类模型"""
    model = YOLO('yolov8n-cls.pt')

    model.train(
        data='/home/idriver/workspace/yolo_project/datasets',
        epochs=100,
        imgsz=640,
        batch=8,
        device='cpu',
        workers=0,
        project='runs/classify',
        name='rose_disease',
    )

    print("训练完成！最佳模型保存在 runs/classify/rose_disease/weights/best.pt")


def predict(image_path):
    """单张图片预测"""
    model = YOLO('runs/classify/rose_disease/weights/best.pt')
    results = model(image_path)

    for r in results:
        top1 = r.names[r.probs.top1]       # 预测类别名
        conf = r.probs.top1conf.item()     # 置信度
        print(f"预测结果: {top1}  ({conf:.2%})")


def predict_batch(folder_path):
    """批量预测文件夹中所有图片"""
    model = YOLO('runs/classify/rose_disease/weights/best.pt')
    results = model(folder_path)

    for r in results:
        name = r.path.split('/')[-1]
        cls = r.names[r.probs.top1]
        conf = r.probs.top1conf.item()
        print(f"{name:40s} → {cls:20s}  ({conf:.2%})")


if __name__ == '__main__':
    train()
