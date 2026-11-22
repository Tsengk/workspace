#include <QApplication>
#include "calibration_gui/main_window.h"

int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    app.setApplicationName("Calibration Tool");
    app.setOrganizationName("iDriver");

    calibration_gui::MainWindow win(argc, argv);
    win.setWindowTitle("自动驾驶传感器标定工具");
    win.showMaximized();  // 默认最大化启动

    return app.exec();
}
