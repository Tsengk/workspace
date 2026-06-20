from ultralytics import YOLO

def main():
    # 1. 加载预训练模型
    # yolov8n.pt (最快), yolov8s.pt (平衡), yolov8m.pt (高精)
    # 首次运行会自动从 GitHub 下载权重


    model = YOLO('yolov8n.pt')  


    # 2. 启动训练
    results = model.train(
        # 注意：这里最好填 data.yaml 的绝对路径，或者相对路径
        data='datasets/synthetic_shapes/data.yaml', 
        epochs=10,
        imgsz=640,
        batch=16,           # 批次大小 (显存不够就改小，如 8 或 4)
        device='cpu',           # GPU ID (如果用 CPU 请填 'cpu')
        workers=4,          # 数据加载线程数 (如果报错 Shared Memory 错误，改成 0)
        project='runs/train', # 结果保存路径
        name='exp01'        # 本次实验名称
)
    
    # 3. (可选) 训练结束后简单验证
    print("训练完成！最佳模型保存在 runs/train/exp01/weights/best.pt")

if __name__ == '__main__':
    main()