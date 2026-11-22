#ifndef TOAST_H
#define TOAST_H

#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>

class Toast : public QObject {
    Q_OBJECT

public:
    explicit Toast(QObject* parent = nullptr);

    static Toast& Instance() {
        static Toast instance;
        return instance;
    }
    void Text(const QString& color = "FFFFFF", const QString& bgcolor = "000000",
              const int& mestime = 3000, const QString& textconst = "");
    //错误
    void Error(const QString& text, const int& mestime = 3000);
    //成功
    void Success(const QString& text, const int& mestime = 3000);
    //警告
    void Warning(const QString& text, const int& mestime = 3000);
    //提示
    void Tips(const QString& text, const int& mestime = 3000);
signals:

private slots:
    void OnTimerStayOut();

private:
    QWidget* widget_;
    QLabel* label_;
    QTimer* timer_;
    qreal transparent_;
    QString StringToRGBA(const QString& color);
};

#define TOAST_MSG (Toast::Instance())

#endif // TOAST_H
