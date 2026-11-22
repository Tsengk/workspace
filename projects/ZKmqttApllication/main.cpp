#include "mainwindow.h"

#include <QApplication>
#include <QSystemTrayIcon>
#include <QMessageBox>
#include <QDebug>
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
   QTextCodec::setCodecForLocale(QTextCodec::codecForName("UTF-8"));
    //样式，加入后启动速度回变慢，影响不大
    QString filePath = QString("%1/3drparty/QDarkStyleSheet/qdarkstyle/dark/darkstyle.qss").arg(QCoreApplication::applicationDirPath());
    //w.setStyleSheet("QMainWindow { background-color: blue; }");
  //  w.showFullScreen();

    QFile f(filePath);
    if (!f.exists())
    {
        printf("Unable to set stylesheet, file not found\n");
    }
    else
    {
        f.open(QFile::ReadOnly | QFile::Text);
        QTextStream ts(&f);
        a.setStyleSheet(ts.readAll());
    }
  MainWindow w;
    
    // 检查系统托盘是否可用
    if (!QSystemTrayIcon::isSystemTrayAvailable()) {
        QMessageBox::critical(0, QObject::tr("Systray"),
                             QObject::tr("I couldn't detect any system tray "
                                        "on this system."));
        return 1;
    }
    
    // 应用关闭策略设置为不退出应用程序
    QApplication::setQuitOnLastWindowClosed(false);
    
    // 不直接显示主窗口，而是让系统托盘控制显示
    return a.exec();
}
