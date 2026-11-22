#pragma once
#include <QWidget>
#include <QLabel>
#include <opencv2/core.hpp>

namespace calibration_gui {

/// 简单的 2D 图像查看器（显示相机图像 / 投影结果）
class ImageViewer : public QWidget {
    Q_OBJECT
public:
    explicit ImageViewer(QWidget* parent = nullptr);

    void showImage(const cv::Mat& img);
    void clear();

    QSize sizeHint() const override { return QSize(640, 360); }
    QSize minimumSizeHint() const override { return QSize(160, 90); }

protected:
    void resizeEvent(QResizeEvent* event) override;

private:
    QLabel* label_;
    cv::Mat current_img_;
    void updateDisplay();
};

} // namespace calibration_gui
