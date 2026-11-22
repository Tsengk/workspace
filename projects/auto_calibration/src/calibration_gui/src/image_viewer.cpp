#include "calibration_gui/image_viewer.h"
#include <QVBoxLayout>
#include <QResizeEvent>
#include <opencv2/imgproc.hpp>

namespace calibration_gui {

ImageViewer::ImageViewer(QWidget* parent) : QWidget(parent)
{
    label_ = new QLabel(this);
    label_->setAlignment(Qt::AlignCenter);
    label_->setStyleSheet("QLabel { background-color: #1a1a2e; color: #888; }");
    label_->setText("无图像");

    // 关键：阻止 label 根据 pixmap 内容自行撑大布局
    label_->setScaledContents(false);
    label_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(label_);

    // 自身也用 Expanding，让父布局控制大小
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

void ImageViewer::showImage(const cv::Mat& img)
{
    if (img.empty()) return;
    current_img_ = img.clone();
    updateDisplay();
}

void ImageViewer::clear()
{
    current_img_ = cv::Mat();
    label_->setText("无图像");
    label_->setPixmap(QPixmap());
}

void ImageViewer::resizeEvent(QResizeEvent* /*event*/)
{
    updateDisplay();
}

void ImageViewer::updateDisplay()
{
    if (current_img_.empty()) return;

    // 用 widget 自身尺寸（而非 label 尺寸）计算缩放，避免反馈循环扩大
    const int w = width();
    const int h = height();
    if (w <= 0 || h <= 0) return;

    double scale = std::min(static_cast<double>(w) / current_img_.cols,
                            static_cast<double>(h) / current_img_.rows);
    // 限制不放大（scale <= 1），只缩小不放大可去掉此行
    // scale = std::min(scale, 1.0);

    cv::Mat display;
    cv::resize(current_img_, display,
               cv::Size(std::max(1, static_cast<int>(current_img_.cols * scale)),
                        std::max(1, static_cast<int>(current_img_.rows * scale))));

    cv::Mat rgb;
    if (display.channels() == 3)
        cv::cvtColor(display, rgb, cv::COLOR_BGR2RGB);
    else if (display.channels() == 1)
        cv::cvtColor(display, rgb, cv::COLOR_GRAY2RGB);
    else
        rgb = display;

    QImage qimg(rgb.data, rgb.cols, rgb.rows,
                static_cast<int>(rgb.step), QImage::Format_RGB888);
    // 直接设置固定像素大小的 pixmap，label 不会因此撑大
    label_->setPixmap(QPixmap::fromImage(qimg.copy()));
}

} // namespace calibration_gui
