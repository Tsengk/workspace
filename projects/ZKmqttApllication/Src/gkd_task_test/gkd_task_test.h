#ifndef GKD_TASK_TEST_H
#define GKD_TASK_TEST_H

#include <QWidget>
#include <QListWidgetItem>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QResizeEvent>
#include <QMouseEvent>
#include <QMenu>
#include <QAction>
#include <memory>

class PlanRequestWidget;
class TaskDeliveryWidget;
class ElectromagneticTestWidget;
class VelocityConfigWidget;
class C2Manager;
class CustomTitleBar;
struct GKDCommonCfg;

namespace Ui {
class gkd_task_test;
}

class gkd_task_test : public QWidget
{
    Q_OBJECT

public:
    explicit gkd_task_test(QWidget *parent = 0);
    ~gkd_task_test();

    // 设置C2Manager实例
    void setC2Manager(std::shared_ptr<C2Manager> c2Manager);

    // 设置GKDCommonCfg实例
    void setGKDConfig(std::shared_ptr<GKDCommonCfg> gkdConfig);

protected:
    void resizeEvent(QResizeEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    bool eventFilter(QObject *obj, QEvent *event) override;

private slots:
    void onPlanFileLoaded(const QString &filePath);
    void onTaskInfoChanged(const QJsonObject &taskInfo);
    void onTaskDelivered(const QJsonObject &taskInfo);
    void onToggleStayOnTop();
    void onTitleBarContextMenu(const QPoint &pos);

private:
    void setupContextMenu();
    Qt::CursorShape getCursorShape(const QPoint &point);
    int getResizeDirection(const QPoint &point);

    Ui::gkd_task_test *ui;
    CustomTitleBar *titleBar;
    PlanRequestWidget *planRequestWidget;
    TaskDeliveryWidget *taskDeliveryWidget;
    ElectromagneticTestWidget *electromagneticTestWidget;
    VelocityConfigWidget *velocityConfigWidget;

    // 共享配置
    std::shared_ptr<GKDCommonCfg> gkdConfig;

    // 右键菜单相关
    QMenu *contextMenu;
    QAction *stayOnTopAction;
    bool isStayOnTop;

    // 窗口拉伸相关
    bool isResizing;
    int resizeDirection;
    QPoint lastMousePos;
    QRect originalGeometry;
    static const int RESIZE_BORDER = 8; // 拉伸边界宽度

    // 拉伸方向常量
    enum ResizeDirection {
        None = 0,
        Left = 1,
        Right = 2,
        Top = 4,
        Bottom = 8,
        TopLeft = Top | Left,
        TopRight = Top | Right,
        BottomLeft = Bottom | Left,
        BottomRight = Bottom | Right
    };
};

#endif // GKD_TASK_TEST_H
