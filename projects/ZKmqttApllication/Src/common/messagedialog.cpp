#include "messagedialog.h"
#include <QApplication>
#include <QScreen>
#include <QPainter>
#include <QMouseEvent>
#include <QDesktopWidget>
#include <QGraphicsDropShadowEffect>
#include <QPropertyAnimation>
#include <QParallelAnimationGroup>
#include <QGraphicsOpacityEffect>
#include <QEasingCurve>
#include <QKeyEvent>

// 样式常量定义 - 匹配系统深色主题
const QString MessageDialog::BACKGROUND_COLOR = "#3c4043";
const QString MessageDialog::BORDER_COLOR = "#5f6368";  
const QString MessageDialog::TEXT_COLOR = "#e8eaed";
const QString MessageDialog::TITLE_COLOR = "#ffffff"; // 标题使用更亮的白色
const QString MessageDialog::BUTTON_NORMAL_COLOR = "#48494a";
const QString MessageDialog::BUTTON_HOVER_COLOR = "#5f6368";
const QString MessageDialog::BUTTON_PRESSED_COLOR = "#3c4043";

MessageDialog::MessageDialog(QWidget *parent)
    : QDialog(parent)
    , messageType(Information)
    , autoCloseEnabled(false)
    , autoCloseInterval(3000)
    , showAnimation(nullptr)
    , animationGroup(nullptr)
    , shadowEffect(nullptr)
    , opacityEffect(nullptr)
{
    setWindowFlags(Qt::Dialog | Qt::FramelessWindowHint | Qt::WindowSystemMenuHint | Qt::WindowStaysOnTopHint);
    setAttribute(Qt::WA_TranslucentBackground);
    setModal(true);
    
    autoCloseTimer = new QTimer(this);
    autoCloseTimer->setSingleShot(true);
    connect(autoCloseTimer, &QTimer::timeout, this, &MessageDialog::onAutoCloseTimeout);
    
    setupUI();
    updateStyle();
    
    // 添加精致的阴影效果
    shadowEffect = new QGraphicsDropShadowEffect(this);
    shadowEffect->setBlurRadius(24);
    shadowEffect->setColor(QColor(0, 0, 0, 40));
    shadowEffect->setOffset(0, 8);
    setGraphicsEffect(shadowEffect);
}

MessageDialog::~MessageDialog()
{
    if (showAnimation) {
        showAnimation->stop();
        showAnimation->deleteLater();
    }
    // animationGroup 和其他 QGraphicsEffect 会由Qt的父对象自动管理
}

void MessageDialog::setupUI()
{
    setFixedSize(360, 140);
    
    mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(20, 16, 20, 16); // 优化上下边距
    mainLayout->setSpacing(12); // 减少间距
    
    // 内容区域
    contentLayout = new QHBoxLayout();
    contentLayout->setSpacing(12); // 减小图标与文字间距
    
    // 图标标签
    iconLabel = new QLabel();
    iconLabel->setFixedSize(24, 24);
    iconLabel->setScaledContents(true);
    iconLabel->setAlignment(Qt::AlignTop | Qt::AlignHCenter); // 顶部对齐
    
    // 文本区域
    QVBoxLayout *textLayout = new QVBoxLayout();
    textLayout->setSpacing(6); // 减小文字间距
    textLayout->setContentsMargins(0, 0, 0, 0);
    
    // 标题标签
    titleLabel = new QLabel();
    titleLabel->setWordWrap(true);
    titleLabel->setStyleSheet(QString("QLabel { color: %1; font-size: 13px; font-weight: 600; font-family: 'Microsoft YaHei', 'Segoe UI'; background: transparent; margin: 0px; }").arg(TITLE_COLOR));
    
    // 消息标签
    messageLabel = new QLabel();
    messageLabel->setWordWrap(true);
    messageLabel->setStyleSheet(QString("QLabel { color: %1; font-size: 12px; font-weight: 400; font-family: 'Microsoft YaHei', 'Segoe UI'; background: transparent; line-height: 1.5; margin: 0px; }").arg(TEXT_COLOR));
    
    textLayout->addWidget(titleLabel);
    textLayout->addWidget(messageLabel);
    
    contentLayout->addWidget(iconLabel, 0, Qt::AlignTop);
    contentLayout->addLayout(textLayout, 1);
    
    // 按钮区域
    buttonLayout = new QHBoxLayout();
    buttonLayout->setSpacing(8);
    buttonLayout->setContentsMargins(0, 8, 0, 0); // 按钮上方间距
    buttonLayout->addStretch();
    
    okButton = new QPushButton("确定");
    okButton->setFixedSize(72, 28);
    okButton->setCursor(Qt::PointingHandCursor);
    okButton->setFocusPolicy(Qt::StrongFocus); // 支持键盘焦点
    connect(okButton, &QPushButton::clicked, this, &MessageDialog::onButtonClicked);
    
    cancelButton = new QPushButton("取消");
    cancelButton->setFixedSize(72, 28);
    cancelButton->setCursor(Qt::PointingHandCursor);
    cancelButton->setFocusPolicy(Qt::StrongFocus);
    cancelButton->hide();
    connect(cancelButton, &QPushButton::clicked, this, &MessageDialog::reject);
    
    buttonLayout->addWidget(cancelButton);
    buttonLayout->addWidget(okButton);
    
    mainLayout->addLayout(contentLayout, 1);
    mainLayout->addLayout(buttonLayout, 0);
    
    updateIcon();
}

void MessageDialog::updateStyle()
{
    // 设置整体样式
    setStyleSheet(QString(
        "MessageDialog {"
        "   background-color: %1;"
        "   border: 1px solid %2;"
        "   border-radius: 8px;"
        "}"
    ).arg(BACKGROUND_COLOR, BORDER_COLOR));
    
    // 主按钮样式(确定按钮)
    QString primaryButtonStyle = QString(
        "QPushButton {"
        "   background-color: #4285f4;"
        "   border: 1px solid #4285f4;"
        "   border-radius: 6px;"
        "   color: #ffffff;"
        "   font-size: 11px;"
        "   font-weight: 500;"
        "   font-family: 'Microsoft YaHei', 'Segoe UI';"
        "   padding: 6px 16px;"
        "}"
        "QPushButton:hover {"
        "   background-color: #3367d6;"
        "   border-color: #3367d6;"
        "   transform: scale(1.02);"
        "}"
        "QPushButton:pressed {"
        "   background-color: #2d5bcc;"
        "   transform: scale(0.98);"
        "}"
        "QPushButton:focus {"
        "   outline: none;"
        "   box-shadow: 0 0 0 2px rgba(66, 133, 244, 0.3);"
        "}"
    );
    
    // 次要按钮样式(取消按钮)
    QString secondaryButtonStyle = QString(
        "QPushButton {"
        "   background-color: transparent;"
        "   border: 1px solid %2;"
        "   border-radius: 6px;"
        "   color: #ffffff;"
        "   font-size: 11px;"
        "   font-weight: 400;"
        "   font-family: 'Microsoft YaHei', 'Segoe UI';"
        "   padding: 6px 16px;"
        "}"
        "QPushButton:hover {"
        "   background-color: rgba(255, 255, 255, 0.08);"
        "   border-color: #8ab4f8;"
        "   transform: scale(1.02);"
        "}"
        "QPushButton:pressed {"
        "   background-color: rgba(255, 255, 255, 0.16);"
        "   transform: scale(0.98);"
        "}"
        "QPushButton:focus {"
        "   outline: none;"
        "   box-shadow: 0 0 0 2px rgba(138, 180, 248, 0.3);"
        "}"
    ).arg(BUTTON_NORMAL_COLOR, BORDER_COLOR, TEXT_COLOR, BUTTON_HOVER_COLOR);
    
    okButton->setStyleSheet(primaryButtonStyle);
    cancelButton->setStyleSheet(secondaryButtonStyle);
}

void MessageDialog::updateIcon()
{
    QPixmap pixmap(24, 24); // 缩小图标尺寸
    pixmap.fill(Qt::transparent);
    
    QPainter painter(&pixmap);
    painter.setRenderHint(QPainter::Antialiasing);
    
    switch (messageType) {
        case Information:
            // 绘制信息图标（i） - 简洁风格
            painter.setPen(QPen(QColor(66, 133, 244), 2));
            painter.setBrush(QBrush(QColor(66, 133, 244)));
            painter.drawEllipse(10, 4, 3, 3); // 点
            painter.setPen(QPen(QColor(66, 133, 244), 2));
            painter.drawLine(12, 8, 12, 18); // 竖线
            break;
            
        case Warning:
            // 绘制警告图标（!） - 简洁风格
            painter.setPen(QPen(QColor(251, 188, 4), 2));
            painter.drawLine(12, 2, 12, 16); // 竖线
            painter.setBrush(QBrush(QColor(251, 188, 4)));
            painter.drawEllipse(10, 19, 3, 3); // 点
            break;
            
        case Error:
            // 绘制错误图标（×） - 简洁风格
            painter.setPen(QPen(QColor(234, 67, 53), 2));
            painter.drawLine(6, 6, 18, 18);
            painter.drawLine(18, 6, 6, 18);
            break;
            
        case Success:
            // 绘制成功图标（✓） - 简洁风格
            painter.setPen(QPen(QColor(52, 168, 83), 2));
            painter.drawLine(6, 12, 10, 16);
            painter.drawLine(10, 16, 18, 8);
            break;
            
        case Question:
            // 绘制问号图标（?） - 简洁风格
            painter.setPen(QPen(QColor(95, 99, 104), 2));
            painter.drawArc(QRect(8, 4, 8, 6), 0, 180 * 16); // 问号上半部分
            painter.drawLine(12, 11, 12, 14); // 问号下半部分
            painter.setBrush(QBrush(QColor(95, 99, 104)));
            painter.drawEllipse(10, 17, 3, 3); // 点
            break;
    }
    
    iconLabel->setPixmap(pixmap);
}

void MessageDialog::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event)
    
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    
    // 绘制背景
    painter.setBrush(QBrush(QColor(BACKGROUND_COLOR)));
    painter.setPen(QPen(QColor(BORDER_COLOR)));
    painter.drawRoundedRect(rect(), 8, 8);
}

void MessageDialog::keyPressEvent(QKeyEvent *event)
{
    switch (event->key()) {
        case Qt::Key_Return:
        case Qt::Key_Enter:
            // Enter键确认
            onButtonClicked();
            break;
        case Qt::Key_Escape:
            // ESC键取消
            if (cancelButton->isVisible()) {
                reject();
            } else {
                onButtonClicked();
            }
            break;
        default:
            QDialog::keyPressEvent(event);
            break;
    }
}

void MessageDialog::showEvent(QShowEvent *event)
{
    QDialog::showEvent(event);
    
    // 居中显示
    if (parentWidget()) {
        QRect parentRect = parentWidget()->geometry();
        move(parentRect.center() - rect().center());
    } else {
        QScreen *screen = QApplication::primaryScreen();
        QRect screenRect = screen->availableGeometry();
        move(screenRect.center() - rect().center());
    }
    
    animateShow();
    
    if (autoCloseEnabled) {
        autoCloseTimer->start(autoCloseInterval);
    }
}

void MessageDialog::animateShow()
{
    // 简化动画，只使用缩放效果，避免GraphicsEffect切换问题
    showAnimation = new QPropertyAnimation(this, "geometry");
    showAnimation->setDuration(250);
    showAnimation->setEasingCurve(QEasingCurve::OutCubic);
    
    QRect startGeometry = geometry();
    QRect endGeometry = geometry();
    
    startGeometry.setWidth(startGeometry.width() * 0.85);
    startGeometry.setHeight(startGeometry.height() * 0.85);
    startGeometry.moveCenter(endGeometry.center());
    
    setGeometry(startGeometry);
    
    showAnimation->setStartValue(startGeometry);
    showAnimation->setEndValue(endGeometry);
    showAnimation->start();
}

void MessageDialog::onButtonClicked()
{
    if (autoCloseTimer->isActive()) {
        autoCloseTimer->stop();
    }
    accept();
}

void MessageDialog::onAutoCloseTimeout()
{
    accept();
}

// 静态方法实现
void MessageDialog::showInformation(QWidget *parent, const QString &title, const QString &message)
{
    MessageDialog dialog(parent);
    dialog.setMessageType(Information);
    dialog.setTitle(title);
    dialog.setMessage(message);
    dialog.exec();
}

void MessageDialog::showWarning(QWidget *parent, const QString &title, const QString &message)
{
    MessageDialog dialog(parent);
    dialog.setMessageType(Warning);
    dialog.setTitle(title);
    dialog.setMessage(message);
    dialog.exec();
}

void MessageDialog::showError(QWidget *parent, const QString &title, const QString &message)
{
    MessageDialog dialog(parent);
    dialog.setMessageType(Error);
    dialog.setTitle(title);
    dialog.setMessage(message);
    dialog.exec();
}

void MessageDialog::showSuccess(QWidget *parent, const QString &title, const QString &message)
{
    MessageDialog dialog(parent);
    dialog.setMessageType(Success);
    dialog.setTitle(title);
    dialog.setMessage(message);
    dialog.exec();
}

int MessageDialog::showQuestion(QWidget *parent, const QString &title, const QString &message)
{
    MessageDialog dialog(parent);
    dialog.setMessageType(Question);
    dialog.setTitle(title);
    dialog.setMessage(message);
    dialog.setButtonText("是", "否");
    dialog.cancelButton->show();
    return dialog.exec();
}

// 实例方法实现
void MessageDialog::setMessageType(MessageType type)
{
    messageType = type;
    updateIcon();
}

void MessageDialog::setTitle(const QString &title)
{
    titleLabel->setText(title);
}

void MessageDialog::setMessage(const QString &message)
{
    messageLabel->setText(message);
}

void MessageDialog::setIcon(const QIcon &icon)
{
    iconLabel->setPixmap(icon.pixmap(48, 48));
}

void MessageDialog::setAutoClose(bool enable, int msec)
{
    autoCloseEnabled = enable;
    autoCloseInterval = msec;
}

void MessageDialog::addButton(const QString &text, int result)
{
    QPushButton *button = new QPushButton(text);
    button->setFixedSize(80, 30);
    button->setCursor(Qt::PointingHandCursor);
    
    QString buttonStyle = QString(
        "QPushButton {"
        "   background-color: %1;"
        "   border: 1px solid %2;"
        "   border-radius: 4px;"
        "   color: %3;"
        "   font-size: 12px;"
        "   font-weight: bold;"
        "   padding: 5px 10px;"
        "}"
        "QPushButton:hover {"
        "   background-color: %4;"
        "}"
        "QPushButton:pressed {"
        "   background-color: %5;"
        "}"
    ).arg(BUTTON_NORMAL_COLOR, BORDER_COLOR, TEXT_COLOR, BUTTON_HOVER_COLOR, BUTTON_PRESSED_COLOR);
    
    button->setStyleSheet(buttonStyle);
    
    connect(button, &QPushButton::clicked, this, [this, result]() {
        if (autoCloseTimer->isActive()) {
            autoCloseTimer->stop();
        }
        done(result);
    });
    
    buttonLayout->insertWidget(buttonLayout->count() - 1, button);
}

void MessageDialog::setButtonText(const QString &okText, const QString &cancelText)
{
    okButton->setText(okText);
    if (!cancelText.isEmpty()) {
        cancelButton->setText(cancelText);
        cancelButton->show();
    }
}