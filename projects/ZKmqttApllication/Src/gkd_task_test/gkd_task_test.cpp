#include "gkd_task_test.h"
#include "ui_gkd_task_test.h"
#include "PlanRequestWidget.h"
#include "TaskDeliveryWidget.h"
#include "VelocityConfigWidget.h"
#include "../gkd/c2manager.h"
#include "../common/gkd_config.h"
#include "../../customtitlebar.h"
#include <QDebug>
#include <QMenu>
#include <QAction>

gkd_task_test::gkd_task_test(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::gkd_task_test),
    titleBar(nullptr),
    planRequestWidget(nullptr),
    taskDeliveryWidget(nullptr),
    electromagneticTestWidget(nullptr),
    velocityConfigWidget(nullptr),
    gkdConfig(nullptr),
    contextMenu(nullptr),
    stayOnTopAction(nullptr),
    isStayOnTop(false),
    isResizing(false),
    resizeDirection(None)
{
    // 设置无边框
    setWindowFlags(Qt::FramelessWindowHint);
    ui->setupUi(this);

    // 设置最小窗口大小
    setMinimumSize(400, 300);

    // 安装事件过滤器以处理鼠标事件
    installEventFilter(this);

    // 创建自定义标题栏
    titleBar = new CustomTitleBar(this);
    titleBar->setTitle("任务干预界面");

    // 将标题栏放置在窗口顶部，但为拉伸留出空间
    titleBar->setGeometry(0, RESIZE_BORDER, this->width(), 30 - RESIZE_BORDER);
    titleBar->show();

    // 调整主窗口内容的位置，为标题栏留出空间
    // gkd_task_test 使用的是 verticalLayout，而不是 centralwidget
    if (ui->verticalLayout) {
        ui->verticalLayout->setContentsMargins(0, 35, 0, 0);
    }

    // 确保标题栏在最上层
    titleBar->raise();

    // 创建右键菜单
    setupContextMenu();
    
    planRequestWidget = ui->planRequestWidget;
    taskDeliveryWidget = ui->taskDeliveryWidget;
    electromagneticTestWidget = ui->electromagneticTestWidget;
    velocityConfigWidget = ui->velocityConfigWidget;

    connect(planRequestWidget, &PlanRequestWidget::planFileLoaded, this, &gkd_task_test::onPlanFileLoaded);
    connect(planRequestWidget, &PlanRequestWidget::taskInfoChanged, this, &gkd_task_test::onTaskInfoChanged);
    connect(taskDeliveryWidget, &TaskDeliveryWidget::taskDelivered, this, &gkd_task_test::onTaskDelivered);
}

gkd_task_test::~gkd_task_test()
{
    delete ui;
    if (contextMenu) {
        delete contextMenu;
    }
}

void gkd_task_test::setC2Manager(std::shared_ptr<C2Manager> c2Manager)
{
    if (planRequestWidget) {
        planRequestWidget->setC2Manager(c2Manager);
    }
    if (taskDeliveryWidget) {
        taskDeliveryWidget->setC2Manager(c2Manager);
    }
    if (electromagneticTestWidget) {
        // 通过事件总线通信，不再需要直接引用传递
        qDebug() << "ElectromagneticTestWidget 已准备就绪，通过事件总线通信";
    }
}

void gkd_task_test::setGKDConfig(std::shared_ptr<GKDCommonCfg> gkdConfig)
{
    this->gkdConfig = gkdConfig;
    if (velocityConfigWidget) {
        velocityConfigWidget->setGKDConfig(gkdConfig);
        qDebug() << "VelocityConfigWidget GKDConfig配置已设置";
    }
}


void gkd_task_test::onPlanFileLoaded(const QString &filePath)
{
    qDebug() << "计划文件已加载:" << filePath;
}

void gkd_task_test::onTaskInfoChanged(const QJsonObject &taskInfo)
{
    qDebug() << "任务信息已更新:" << taskInfo;
}

void gkd_task_test::onTaskDelivered(const QJsonObject &taskInfo)
{
    qDebug() << "任务已下发:" << taskInfo;
}

void gkd_task_test::resizeEvent(QResizeEvent *event)
{
    QWidget::resizeEvent(event);

    // 调整标题栏大小以适应窗口宽度
    if (titleBar) {
        titleBar->setGeometry(0, RESIZE_BORDER, event->size().width(), 30 - RESIZE_BORDER);
        titleBar->raise(); // 确保标题栏始终在最上层
    }
}

void gkd_task_test::setupContextMenu()
{
    contextMenu = new QMenu(this);

    stayOnTopAction = new QAction("置顶", this);
    stayOnTopAction->setCheckable(true);
    stayOnTopAction->setChecked(isStayOnTop);
    connect(stayOnTopAction, &QAction::triggered, this, &gkd_task_test::onToggleStayOnTop);

    contextMenu->addAction(stayOnTopAction);

    // 为标题栏设置上下文菜单
    if (titleBar) {
        titleBar->setContextMenuPolicy(Qt::CustomContextMenu);
        connect(titleBar, &QWidget::customContextMenuRequested,
                this, &gkd_task_test::onTitleBarContextMenu);
    }
}

void gkd_task_test::onToggleStayOnTop()
{
    isStayOnTop = !isStayOnTop;
    stayOnTopAction->setChecked(isStayOnTop);
    stayOnTopAction->setText(isStayOnTop ? "取消置顶" : "置顶");

    Qt::WindowFlags flags = windowFlags();
    if (isStayOnTop) {
        flags |= Qt::WindowStaysOnTopHint;
    } else {
        flags &= ~Qt::WindowStaysOnTopHint;
    }

    setWindowFlags(flags);
    show(); // 重新显示窗口以应用新的标志

    qDebug() << "窗口置顶状态:" << (isStayOnTop ? "已置顶" : "已取消置顶");
}

void gkd_task_test::onTitleBarContextMenu(const QPoint &pos)
{
    if (contextMenu) {
        contextMenu->exec(titleBar->mapToGlobal(pos));
    }
}

void gkd_task_test::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        lastMousePos = event->globalPos();
        originalGeometry = geometry();
        resizeDirection = getResizeDirection(event->pos());

        if (resizeDirection != None) {
            isResizing = true;
            event->accept();
            return;
        }
    }
    QWidget::mousePressEvent(event);
}

void gkd_task_test::mouseMoveEvent(QMouseEvent *event)
{
    if (!isResizing) {
        // 更新鼠标光标
        Qt::CursorShape cursor = getCursorShape(event->pos());
        setCursor(cursor);
    } else {
        // 执行窗口拉伸
        QPoint delta = event->globalPos() - lastMousePos;
        QRect newGeometry = originalGeometry;

        if (resizeDirection & Left) {
            newGeometry.setLeft(originalGeometry.left() + delta.x());
        }
        if (resizeDirection & Right) {
            newGeometry.setRight(originalGeometry.right() + delta.x());
        }
        if (resizeDirection & Top) {
            newGeometry.setTop(originalGeometry.top() + delta.y());
        }
        if (resizeDirection & Bottom) {
            newGeometry.setBottom(originalGeometry.bottom() + delta.y());
        }

        // 确保不小于最小尺寸
        if (newGeometry.width() >= minimumSize().width() &&
            newGeometry.height() >= minimumSize().height()) {
            setGeometry(newGeometry);
        }

        event->accept();
        return;
    }

    QWidget::mouseMoveEvent(event);
}

void gkd_task_test::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::LeftButton) {
        isResizing = false;
        resizeDirection = None;
        setCursor(Qt::ArrowCursor);
    }
    QWidget::mouseReleaseEvent(event);
}

bool gkd_task_test::eventFilter(QObject *obj, QEvent *event)
{
    if (obj == this && event->type() == QEvent::HoverMove) {
        QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
        Qt::CursorShape cursor = getCursorShape(mouseEvent->pos());
        setCursor(cursor);
    }
    return QWidget::eventFilter(obj, event);
}

Qt::CursorShape gkd_task_test::getCursorShape(const QPoint &point)
{
    int direction = getResizeDirection(point);

    switch (direction) {
        case Left:
        case Right:
            return Qt::SizeHorCursor;
        case Top:
        case Bottom:
            return Qt::SizeVerCursor;
        case TopLeft:
        case BottomRight:
            return Qt::SizeFDiagCursor;
        case TopRight:
        case BottomLeft:
            return Qt::SizeBDiagCursor;
        default:
            return Qt::ArrowCursor;
    }
}

int gkd_task_test::getResizeDirection(const QPoint &point)
{
    int direction = None;

    // 标题栏区域判断
    bool inTitleBar = (point.y() > RESIZE_BORDER && point.y() <= 30);

    if (point.x() <= RESIZE_BORDER)
        direction |= Left;
    else if (point.x() >= width() - RESIZE_BORDER)
        direction |= Right;

    // 顶部拉伸：允许在窗口最顶端的细边区域进行拉伸
    if (point.y() <= RESIZE_BORDER)
        direction |= Top;
    else if (point.y() >= height() - RESIZE_BORDER)
        direction |= Bottom;

    // 如果在标题栏内部区域，清除所有拉伸方向
    if (inTitleBar && point.x() > RESIZE_BORDER && point.x() < width() - RESIZE_BORDER) {
        direction = None;
    }

    return direction;
}
