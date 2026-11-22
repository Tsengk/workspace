#ifndef CUSTOMTITLEBAR_H
#define CUSTOMTITLEBAR_H

#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QHBoxLayout>
#include <QMouseEvent>

class CustomTitleBar : public QWidget
{
    Q_OBJECT

public:
    explicit CustomTitleBar(QWidget *parent = nullptr);
    void setTitle(const QString &title);
    void setWindowIcon(const QIcon &icon);

protected:
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void mouseDoubleClickEvent(QMouseEvent *event) override;

private slots:
    void onMinimizeClicked();
    void onMaximizeClicked();
    void onCloseClicked();

private:
    void setupUI();
    
    QLabel *iconLabel;
    QLabel *titleLabel;
    QPushButton *minimizeButton;
    QPushButton *maximizeButton;
    QPushButton *closeButton;
    
    QPoint lastMousePosition;
    bool isPressed;
    
    QWidget *parentWindow;
};

#endif // CUSTOMTITLEBAR_H