#ifndef UTILS_COMMON_HPP
#define UTILS_COMMON_HPP

#include <QSystemTrayIcon>
#include <QString>
#include <QMutex>
#include <QDebug>

/**
 * @brief 托盘消息通知管理器
 * 
 * 提供统一的托盘消息提示接口，允许其他模块通过此接口使用托盘消息提示功能
 * 采用单例模式，确保全局只有一个实例
 */
class TrayNotificationManager
{
public:
    /**
     * @brief 获取单例实例
     * @return 管理器实例引用
     */
    static TrayNotificationManager& getInstance()
    {
        static TrayNotificationManager instance;
        return instance;
    }

    /**
     * @brief 设置托盘图标实例
     * @param trayIcon 托盘图标指针，由MainWindow传入
     * 
     * 注意：此方法需要在MainWindow初始化托盘后调用
     */
    void setTrayIcon(QSystemTrayIcon* trayIcon)
    {
        QMutexLocker locker(&mutex_);
        trayIcon_ = trayIcon;
        qDebug() << "TrayNotificationManager: 托盘图标已设置";
    }

    /**
     * @brief 显示信息类型的托盘消息
     * @param title 消息标题
     * @param message 消息内容
     * @param timeout 显示时长（毫秒），默认3000ms
     */
    void showInfo(const QString& title, const QString& message, int timeout = 3000)
    {
        showMessage(title, message, QSystemTrayIcon::Information, timeout);
    }

    /**
     * @brief 显示警告类型的托盘消息
     * @param title 消息标题  
     * @param message 消息内容
     * @param timeout 显示时长（毫秒），默认3000ms
     */
    void showWarning(const QString& title, const QString& message, int timeout = 3000)
    {
        showMessage(title, message, QSystemTrayIcon::Warning, timeout);
    }

    /**
     * @brief 显示错误类型的托盘消息
     * @param title 消息标题
     * @param message 消息内容
     * @param timeout 显示时长（毫秒），默认3000ms
     */
    void showError(const QString& title, const QString& message, int timeout = 3000)
    {
        showMessage(title, message, QSystemTrayIcon::Critical, timeout);
    }

    /**
     * @brief 显示自定义类型的托盘消息
     * @param title 消息标题
     * @param message 消息内容
     * @param icon 消息图标类型
     * @param timeout 显示时长（毫秒），默认3000ms
     */
    void showMessage(const QString& title, const QString& message, 
                    QSystemTrayIcon::MessageIcon icon = QSystemTrayIcon::Information, 
                    int timeout = 3000)
    {
        QMutexLocker locker(&mutex_);
        
        // 检查托盘图标是否可用
        if (!trayIcon_) {
            qDebug() << "TrayNotificationManager: 托盘图标未设置，无法显示消息:" << title << "-" << message;
            return;
        }

        if (!trayIcon_->isVisible()) {
            qDebug() << "TrayNotificationManager: 托盘图标不可见，无法显示消息:" << title << "-" << message;
            return;
        }

        // 显示托盘消息
        trayIcon_->showMessage(title, message, icon, timeout);
        qDebug() << "TrayNotificationManager: 显示托盘消息 -" << title << ":" << message;
    }

    /**
     * @brief 检查托盘图标是否可用
     * @return true表示可用，false表示不可用
     */
    bool isTrayAvailable() const
    {
        QMutexLocker locker(&mutex_);
        return (trayIcon_ != nullptr) && trayIcon_->isVisible();
    }

private:
    // 私有构造函数，防止外部实例化
    TrayNotificationManager() : trayIcon_(nullptr) {}
    
    // 禁止拷贝构造和赋值
    TrayNotificationManager(const TrayNotificationManager&) = delete;
    TrayNotificationManager& operator=(const TrayNotificationManager&) = delete;

private:
    QSystemTrayIcon* trayIcon_;     // 托盘图标指针
    mutable QMutex mutex_;          // 线程安全互斥锁
};

// 便利宏定义
#define TRAY_NOTIFICATION (TrayNotificationManager::getInstance())

#endif // UTILS_COMMON_HPP