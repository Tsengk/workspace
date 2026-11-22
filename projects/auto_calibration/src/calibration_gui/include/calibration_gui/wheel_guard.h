#pragma once
#include <QAbstractScrollArea>
#include <QAbstractSpinBox>
#include <QApplication>
#include <QComboBox>
#include <QCoreApplication>
#include <QEvent>
#include <QObject>
#include <QWidget>

namespace calibration_gui {

/// 全局滚轮禁用过滤器。
///
/// 安装在 QApplication 上，彻底禁用 QComboBox 和 QAbstractSpinBox
///（及其子类 QSpinBox / QDoubleSpinBox）的鼠标滚轮响应。
///
/// 滚轮事件一律转发到最近的 QAbstractScrollArea 祖先的 viewport，
/// 实现"滚轮只滚动面板，不改变控件值"。
///
/// 使用方式（main_window.cpp）：
///   installGlobalWheelGuard();
class WheelGuard : public QObject {
    Q_OBJECT
public:
    explicit WheelGuard(QObject* parent = nullptr) : QObject(parent) {}

protected:
    bool eventFilter(QObject* obj, QEvent* event) override
    {
        if (event->type() != QEvent::Wheel)
            return QObject::eventFilter(obj, event);

        auto* target = qobject_cast<QWidget*>(obj);
        if (!target)
            return QObject::eventFilter(obj, event);

        // 从事件目标向上查找 QComboBox 或 QAbstractSpinBox 祖先
        QWidget* control = find_control_ancestor(target);
        if (!control)
            return QObject::eventFilter(obj, event);  // 非目标控件 → 放行

        // 找到 → 转发滚轮事件到 QScrollArea 面板
        for (QWidget* p = control->parentWidget(); p; p = p->parentWidget()) {
            auto* sa = qobject_cast<QAbstractScrollArea*>(p);
            if (sa) {
                QCoreApplication::sendEvent(sa->viewport(), event);
                return true;  // 已转发，阻止原始投递
            }
        }
        return true;  // 无 ScrollArea → 直接丢弃
    }

private:
    static QWidget* find_control_ancestor(QWidget* w)
    {
        for (; w; w = w->parentWidget()) {
            if (qobject_cast<QComboBox*>(w) || qobject_cast<QAbstractSpinBox*>(w))
                return w;
        }
        return nullptr;
    }
};

/// 在 QApplication 上安装全局滚轮禁用。
inline void installGlobalWheelGuard()
{
    qApp->installEventFilter(new WheelGuard(qApp));
}

} // namespace calibration_gui
