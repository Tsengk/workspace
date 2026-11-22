# BEV 环视标定使用指南

## 标定板布局

### 前后板（大板）
- 尺寸：4.4m × 1.2m
- 布局：两边黑块 + 中间棋盘格
- 棋盘格：6×4 角点，格子尺寸 0.2m

### 左右板（小板）
- 尺寸：1.2m × 1.6m
- 布局：纯棋盘格
- 棋盘格：4×6 角点，格子尺寸 0.2m

## 启动标定

```bash
roslaunch bev_surround_view bev_surround_view_cali.launch
```

## 触发标定

```bash
rosservice call /bev_surround_view/trigger_calibration
```

## 可视化话题

- `/bev_surround_view/detection_vis_front` - 前相机检测结果
- `/bev_surround_view/detection_vis_rear` - 后相机检测结果
- `/bev_surround_view/detection_vis_left` - 左相机检测结果
- `/bev_surround_view/detection_vis_right` - 右相机检测结果
- `/bev_surround_view/bev_image` - BEV 拼接图

## 查看可视化

```bash
rqt_image_view
```

选择对应话题查看检测到的棋盘格角点和 BEV 拼接结果。
