#ifndef MESSAGEDIALOG_H
#define MESSAGEDIALOG_H

#include <QDialog>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPixmap>
#include <QIcon>
#include <QPropertyAnimation>
#include <QParallelAnimationGroup>
#include <QGraphicsDropShadowEffect>
#include <QGraphicsOpacityEffect>
#include <QTimer>

class MessageDialog : public QDialog
{
    Q_OBJECT

public:
    enum MessageType {
        Information,
        Warning,
        Error,
        Success,
        Question
    };

    explicit MessageDialog(QWidget *parent = nullptr);
    virtual ~MessageDialog();

    // 静态方法用于快速创建不同类型的消息弹窗
    static void showInformation(QWidget *parent, const QString &title, const QString &message);
    static void showWarning(QWidget *parent, const QString &title, const QString &message);
    static void showError(QWidget *parent, const QString &title, const QString &message);
    static void showSuccess(QWidget *parent, const QString &title, const QString &message);
    static int showQuestion(QWidget *parent, const QString &title, const QString &message);

    // 实例方法
    void setMessageType(MessageType type);
    void setTitle(const QString &title);
    void setMessage(const QString &message);
    void setIcon(const QIcon &icon);
    void setAutoClose(bool enable, int msec = 3000);
    
    // 添加自定义按钮
    void addButton(const QString &text, int result = QDialog::Accepted);
    void setButtonText(const QString &okText, const QString &cancelText = QString());

protected:
    void showEvent(QShowEvent *event) override;
    void paintEvent(QPaintEvent *event) override;
    void keyPressEvent(QKeyEvent *event) override;

private slots:
    void onButtonClicked();
    void onAutoCloseTimeout();

private:
    void setupUI();
    void updateStyle();
    void updateIcon();
    void animateShow();
    
    QVBoxLayout *mainLayout;
    QHBoxLayout *contentLayout;
    QHBoxLayout *buttonLayout;
    
    QLabel *iconLabel;
    QLabel *titleLabel;
    QLabel *messageLabel;
    
    QPushButton *okButton;
    QPushButton *cancelButton;
    
    QTimer *autoCloseTimer;
    
    MessageType messageType;
    bool autoCloseEnabled;
    int autoCloseInterval;
    
    QPropertyAnimation *showAnimation;
    QParallelAnimationGroup *animationGroup;
    QGraphicsDropShadowEffect *shadowEffect;
    QGraphicsOpacityEffect *opacityEffect;
    
    // 颜色和样式常量
    static const QString BACKGROUND_COLOR;
    static const QString BORDER_COLOR;
    static const QString TEXT_COLOR;
    static const QString TITLE_COLOR;
    static const QString BUTTON_NORMAL_COLOR;
    static const QString BUTTON_HOVER_COLOR;
    static const QString BUTTON_PRESSED_COLOR;
};

#endif // MESSAGEDIALOG_H
