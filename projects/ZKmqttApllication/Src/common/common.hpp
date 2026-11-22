#ifndef COMMON_HPP
#define COMMON_HPP
#include <string>
#include <vector>
#include <QSettings>
#include <QDebug>
#include <QCoreApplication>
#include <QFile>
#include <QTextStream>
#include <QJsonObject>
#include <vector>
#include <functional>
#include <iostream>
#include <utility>
#include<QTextCodec>
#include<QString>
#include<string>
#include<sys/timeb.h>
#include <rest_rpc.hpp>
#include <filesystem>
#include <sstream>
#include <iomanip>
#define SINGLETON(Class) \
    public: \
    static Class  &Instance() {\
            static Class Class;\
            return Class;\
        }\
        Class (Class &&) = delete;\
        Class (Class  const&) = delete;\
    private: \
        Class()

namespace common {
struct LatLon {
    double lat;
    double lon;
};

//获取当前可执行程序的路径
static std::string getcurrentDir()
{
    std::experimental::filesystem::path exec_path =
        std::experimental::filesystem::absolute(
            std::experimental::filesystem::v1::current_path());

    return exec_path.string();

}

static std::string utf8ToGbk(const std::string& data)
{
    if (data.empty()) {
        return "";
    }

    try {
        static QTextCodec* gbkCodec = QTextCodec::codecForName("GBK");
        if (!gbkCodec) {
            qDebug() << "Failed to get GBK codec";
            return data;
        }

        // 检查输入是否为有效的UTF-8
        QString qstr = QString::fromUtf8(data.c_str());
        if (qstr.isEmpty() && !data.empty()) {
            qDebug() << "Invalid UTF-8 input";
            return data;
        }

        // 转换为GBK
        QByteArray gbk_bytes = gbkCodec->fromUnicode(qstr);
        if (gbk_bytes.isEmpty()) {
            qDebug() << "Failed to convert to GBK";
            return data;
        }

        return std::string(gbk_bytes.constData(), gbk_bytes.size());
    } catch (const std::exception& e) {
        qDebug() << "utf8ToGbk error:" << e.what();
        return data;
    } catch (...) {
        qDebug() << "utf8ToGbk unknown error";
        return data;
    }
}
} // namespace common

static std::string currentTime(){
    // 获取当前时间点
    auto now = std::chrono::system_clock::now();

    // 转换为time_t类型
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);

    // 转换为本地时间
    std::tm* local_time = std::localtime(&now_time);

    // 获取毫秒部分
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  now.time_since_epoch()) % 1000;

    // 格式化输出
    std::stringstream ss;
    ss  << std::put_time(local_time, "%Y-%m-%d %H:%M:%S")
       << "." << std::setfill('0') << std::setw(3) << ms.count();
    return ss.str();
}
static int64_t currentTimeMillisec(){
    struct timeb tb;
    ftime(&tb);
    return  static_cast<int64_t>( tb.time * 1000 + tb.millitm);
}

static std::string doubleToString(double value, int precision = 6, bool fixed = true) {
    std::stringstream ss;
    if (fixed) {
        ss << std::fixed;
    }
    ss << std::setprecision(precision) << value;
    return ss.str();
}

static std::string utf8ToGbk(const std::string &data)
{
    if (data.empty()) {
        return "";
    }

    try {
        static QTextCodec* gbkCodec = QTextCodec::codecForName("GBK");
        if (!gbkCodec) {
            qDebug() << "Failed to get GBK codec";
            return data;
        }

        // 检查输入是否为有效的UTF-8
        QString qstr = QString::fromUtf8(data.c_str());
        if (qstr.isEmpty() && !data.empty()) {
            qDebug() << "Invalid UTF-8 input, returning original data";
            return data;
        }

        // 转换为GBK
        QByteArray gbk_bytes = gbkCodec->fromUnicode(qstr);
        if (gbk_bytes.isEmpty()) {
            qDebug() << "Failed to convert to GBK, returning original data";
            return data;
        }

        // 验证转换结果不包含无效字符
        std::string result(gbk_bytes.constData(), gbk_bytes.size());
        if (result.empty() && !data.empty()) {
            qDebug() << "Empty result after conversion, returning original data";
            return data;
        }

        return result;
    } catch (const std::exception& e) {
        qDebug() << "utf8ToGbk error:" << e.what() << ", returning original data";
        return data;
    } catch (...) {
        qDebug() << "utf8ToGbk unknown error, returning original data";
        return data;
    }
}

#endif // COMMON_HPP
