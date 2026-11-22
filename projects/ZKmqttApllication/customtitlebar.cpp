#include "customtitlebar.h"
#include <QApplication>
#include <QStyle>

CustomTitleBar::CustomTitleBar(QWidget *parent)
    : QWidget(parent)
    , isPressed(false)
    , parentWindow(parent)
{
    setupUI();
    setFixedHeight(30);
    
    // 确保标题栏在最顶层
    raise();
    setAttribute(Qt::WA_StyledBackground, true);
}

void CustomTitleBar::setupUI()
{
    QHBoxLayout *layout = new QHBoxLayout(this);
    layout->setContentsMargins(5, 0, 0, 0);
    layout->setSpacing(0);

    // 窗口图标
    iconLabel = new QLabel(this);
    iconLabel->setFixedSize(20, 20);
    iconLabel->setScaledContents(true);
    
    // 窗口标题
    titleLabel = new QLabel("应用程序", this);
    titleLabel->setStyleSheet("QLabel { color: white; font-weight: bold; }");
    
    // 最小化按钮
    minimizeButton = new QPushButton(this);
    minimizeButton->setFixedSize(30, 30);
    minimizeButton->setText("−");
    minimizeButton->setStyleSheet(
        "QPushButton {"
        "   border: none;"
        "   background-color: transparent;"
        "   color: white;"
        "   font-size: 16px;"
        "   font-weight: bold;"
        "}"
        "QPushButton:hover {"
        "   background-color: rgba(255, 255, 255, 0.1);"
        "}"
    );
    
    // 最大化/还原按钮
    maximizeButton = new QPushButton(this);
    maximizeButton->setFixedSize(30, 30);
    maximizeButton->setText("□");
    maximizeButton->setStyleSheet(
        "QPushButton {"
        "   border: none;"
        "   background-color: transparent;"
        "   color: white;"
        "   font-size: 14px;"
        "   font-weight: bold;"
        "}"
        "QPushButton:hover {"
        "   background-color: rgba(255, 255, 255, 0.1);"
        "}"
    );
    
    // 关闭按钮
    closeButton = new QPushButton(this);
    closeButton->setFixedSize(30, 30);
    closeButton->setText("×");
    closeButton->setStyleSheet(
        "QPushButton {"
        "   border: none;"
        "   background-color: transparent;"
        "   color: white;"
        "   font-size: 18px;"
        "   font-weight: bold;"
        "}"
        "QPushButton:hover {"
        "   background-color: #e81123;"
        "   color: white;"
        "}"
    );
    
    layout->addWidget(iconLabel);
    layout->addSpacing(5);
    layout->addWidget(titleLabel);
    layout->addStretch();
    layout->addWidget(minimizeButton);
    layout->addWidget(maximizeButton);
    layout->addWidget(closeButton);
    
    // 设置标题栏背景色
    setStyleSheet(
        "CustomTitleBar {"
        "   background-color: #2d2d30;"
        "   border-bottom: 1px solid #3e3e42;"
        "}"
    );
    
    // 连接信号
    connect(minimizeButton, &QPushButton::clicked, this, &CustomTitleBar::onMinimizeClicked);
    connect(maximizeButton, &QPushButton::clicked, this, &CustomTitleBar::onMaximizeClicked);
    connect(closeButton, &QPushButton::clicked, this, &CustomTitleBar::onCloseClicked);
}

void CustomTitleBar::setTitle(const QString &title)
{
    titleLabel->setText(title);
}

void CustomTitleBar::setWindowIcon(const QIcon &icon)
{
    iconLabel->setPixmap(icon.pixmap(20, 20));
}

void CustomTitleBar::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton && parentWindow) {
        isPressed = true;
        lastMousePosition = event->globalPos();
        event->accept();
    } else {
        QWidget::mousePressEvent(event);
    }
}

void CustomTitleBar::mouseMoveEvent(QMouseEvent *event)
{
    if (isPressed && parentWindow && (event->buttons() & Qt::LeftButton)) {
        QPoint currentPos = event->globalPos();
        QPoint offset = currentPos - lastMousePosition;
        parentWindow->move(parentWindow->pos() + offset);
        lastMousePosition = currentPos;
        event->accept();
    } else {
        QWidget::mouseMoveEvent(event);
    }
}

void CustomTitleBar::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        isPressed = false;
        event->accept();
    } else {
        QWidget::mouseReleaseEvent(event);
    }
}

void CustomTitleBar::mouseDoubleClickEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        onMaximizeClicked();
    }
    QWidget::mouseDoubleClickEvent(event);
}

void CustomTitleBar::onMinimizeClicked()
{
    if (parentWindow) {
        parentWindow->showMinimized();
    }
}

void CustomTitleBar::onMaximizeClicked()
{
    if (parentWindow) {
        if (parentWindow->isMaximized()) {
            parentWindow->showNormal();
            maximizeButton->setText("□");
        } else {
            parentWindow->showMaximized();
            maximizeButton->setText("❐");
        }
    }
}

void CustomTitleBar::onCloseClicked()
{
    if (parentWindow) {
        parentWindow->close();
    }
}