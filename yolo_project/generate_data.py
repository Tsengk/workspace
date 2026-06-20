import os
import cv2
import numpy as np
import random
import yaml

# --- 配置 ---
DATASET_NAME = "synthetic_shapes"
ROOT_DIR = os.path.join(os.getcwd(), "datasets", DATASET_NAME)
IMG_SIZE = 640
NUM_TRAIN = 80  # 训练集数量
NUM_VAL = 20    # 验证集数量

def create_yolo_folders():
    """创建符合 YOLO 要求的目录结构"""
    dirs = [
        os.path.join(ROOT_DIR, "images/train"),
        os.path.join(ROOT_DIR, "images/val"),
        os.path.join(ROOT_DIR, "labels/train"),
        os.path.join(ROOT_DIR, "labels/val"),
    ]
    for d in dirs:
        os.makedirs(d, exist_ok=True)
    print(f"目录结构已创建: {ROOT_DIR}")

def generate_sample(save_dir_img, save_dir_lbl, index):
    """生成一张图片和一个对应的标签文件"""
    # 1. 创建全黑背景
    img = np.zeros((IMG_SIZE, IMG_SIZE, 3), dtype=np.uint8)
    
    # 2. 随机生成一个矩形参数
    w = random.randint(50, 200)
    h = random.randint(50, 200)
    x_min = random.randint(0, IMG_SIZE - w)
    y_min = random.randint(0, IMG_SIZE - h)
    x_max = x_min + w
    y_max = y_min + h
    
    # 3. 在图片上画出白色矩形
    cv2.rectangle(img, (x_min, y_min), (x_max, y_max), (255, 255, 255), -1)
    
    # 4. 计算 YOLO 格式坐标 (归一化 0-1)
    # 格式: class_id x_center y_center width height
    x_center = (x_min + w / 2) / IMG_SIZE
    y_center = (y_min + h / 2) / IMG_SIZE
    norm_w = w / IMG_SIZE
    norm_h = h / IMG_SIZE
    
    # 5. 保存图片
    filename = f"shape_{index:03d}"
    cv2.imwrite(os.path.join(save_dir_img, f"{filename}.jpg"), img)
    
    # 6. 保存标签 (类别 ID 设为 0)
    label_line = f"0 {x_center:.6f} {y_center:.6f} {norm_w:.6f} {norm_h:.6f}"
    with open(os.path.join(save_dir_lbl, f"{filename}.txt"), "w") as f:
        f.write(label_line)

def create_yaml():
    """生成 data.yaml 配置文件"""
    yaml_content = {
        'path': ROOT_DIR,
        'train': 'images/train',
        'val': 'images/val',
        'nc': 1,
        'names': {0: 'rectangle'}
    }
    yaml_path = os.path.join(ROOT_DIR, 'data.yaml')
    with open(yaml_path, 'w') as f:
        yaml.dump(yaml_content, f, sort_keys=False)
    print(f"配置文件已生成: {yaml_path}")
    return yaml_path

if __name__ == "__main__":
    create_yolo_folders()
    
    print("正在生成训练集...")
    for i in range(NUM_TRAIN):
        generate_sample(os.path.join(ROOT_DIR, "images/train"), 
                        os.path.join(ROOT_DIR, "labels/train"), i)
        
    print("正在生成验证集...")
    for i in range(NUM_VAL):
        generate_sample(os.path.join(ROOT_DIR, "images/val"), 
                        os.path.join(ROOT_DIR, "labels/val"), i)
                        
    yaml_file = create_yaml()
    print("\n✅ 数据集生成完毕！")
    print(f"请在 train.py 中设置: data='{yaml_file}'")