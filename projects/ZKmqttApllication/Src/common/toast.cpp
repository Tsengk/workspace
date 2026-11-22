#include "toast.h"
#include <QApplication>
#include <QDebug>
#include <QDesktopWidget>
#include <QPainter>
#include <QPropertyAnimation>
#include <QScreen>
#include <QTime>
#include <QTimer>
#include <QWindow>

Toast::Toast(QObject* parent) : QObject(parent) {
    widget_ = new QWidget;
    widget_->setFixedHeight(60);
    label_ = new QLabel;
    label_->setFixedHeight(30);
    label_->move(0, 0);

    QFont ft;
    ft.setPointSize(10);
    label_->setFont(ft);
    label_->setAlignment(Qt::AlignCenter);
    label_->setStyleSheet("color:white");
    widget_->setStyleSheet(
        "border: none;background-color:black;border-radius:10px");
    QHBoxLayout* la = new QHBoxLayout;
    la->addWidget(label_);
    la->setContentsMargins(0, 0, 0, 0);

    widget_->setLayout(la);
    widget_->hide();
    widget_->setWindowFlags(Qt::FramelessWindowHint | Qt::Tool |
                            Qt::WindowStaysOnTopHint);
    widget_->setAttribute(Qt::WA_TranslucentBackground, true);
    timer_ = new QTimer();
    timer_->setInterval(3000);
    connect(timer_, SIGNAL(timeout()), this, SLOT(OnTimerStayOut()));
}

void Toast::Text(const QString& color, const QString& bgcolor,
                 const int& mestime, const QString& text) {
    qDebug() << "Toast: " << text;
    QApplication::beep();
    QString label_text = QTime::currentTime().toString() + " " + text;
    QFontMetrics fm(label_->font());
    int width = fm.boundingRect(label_text).width() + 30;
    widget_->setFixedWidth(width);
    label_->setFixedWidth(width);
    label_->setText(label_text);
    QString style = QString("color:").append(StringToRGBA(color));
    label_->setStyleSheet(style);

    widget_->setStyleSheet(QString("border: none;border-radius:10px;")
                               .append("background-color:")
                               .append(StringToRGBA(bgcolor)));

    QWindow* window = nullptr;
    for (auto& w : qApp->topLevelWindows()) {
        if (w->objectName() != "MainWindowWindow") {
            continue;
        }
        window = w;
        break;
    }

    if (window) {
        widget_->move(window->x() + (window->width() - widget_->width()) / 2,
                      window->y() - 15);
        connect(window, &QWindow::activeChanged, this, [this, window]() {
            if (window->isActive()) {
                return;
            }
            disconnect(window);
            widget_->hide();
        });
        connect(window, &QWindow::visibleChanged, this,
                [this, window](bool arg) {
                    if (arg) {
                        return;
                    }
                    disconnect(window);
                    widget_->hide();
                });
    } else {
        widget_->move(0, 0);
    }
    widget_->show();
    timer_->setInterval(mestime);
    timer_->stop();
    timer_->start();
}

void Toast::Error(const QString& text, const int& mestime) {
    Text("FFFFFF", "FF0000", mestime, text);
}

void Toast::Success(const QString& text, const int& mestime) {
    Text("000000", "00FF00", mestime, text);
}

void Toast::Warning(const QString& text, const int& mestime) {
    Text("FF0000", "FFFF00", mestime, text);
}

void Toast::Tips(const QString& text, const int& mestime) {
    Text("FFFFFF", "0080FF", mestime, text);
}


QString Toast::StringToRGBA(const QString& color) {
    int r = color.mid(0, 2).toInt(nullptr, 16);
    int g = color.mid(2, 2).toInt(nullptr, 16);
    int b = color.mid(4, 2).toInt(nullptr, 16);
    int a = color.length() >= 8 ? color.mid(6, 2).toInt(nullptr, 16)
                                : QString("FF").toInt(nullptr, 16);
    QString style = QString("rgba(")
                        .append(QString::number(r))
                        .append(",")
                        .append(QString::number(g))
                        .append(",")
                        .append(QString::number(b))
                        .append(",")
                        .append(QString::number(a))
                        .append(");");
    return style;
}

void Toast::OnTimerStayOut() {
    timer_->stop();
    widget_->hide();
}
