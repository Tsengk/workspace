#ifndef ConnBase_H
#define ConnBase_H
#include <functional>
#include <QHash>
#include <QMap>
#include <QJsonObject>
#include "mqtt/mqconfig.h"
#include "common/common_config.h"
enum class ConnStatus { INIT, CONNECTING, RECONNECTING, CONNECTED };
// enum class UdpStatus { INIT, CONNECTING, RECONNECTING, CONNECTED };
// enum class MqttStatus { INIT, CONNECTING, RECONNECTING, CONNECTED };
enum class ConnType {MQTT, UDP};

enum Result { Failed, Success, Timeout };
typedef std::function<void(QJsonObject& root)> Callback;
typedef std::function<void(QString& msg)> MsgCallback;
typedef std::function<void(bool)> ReplyCallback;
typedef std::function<void(bool, QJsonObject& data)> ResponseCallback;
typedef std::function<void()> ReplySuccess;

class ConnBase {

public:
    ConnBase() {};
    ~ConnBase() {};
    virtual void Start(const string& clientid,const string& ip,int16_t port,int16_t keepalive) = 0;
    virtual void Stop() = 0;

     virtual bool PrimitivePublish(const QString& topic, QJsonObject& root) = 0;
     virtual bool PrimitivePublish(const QString& topic, const QString& payload) = 0;

    // virtual bool Publish(const QString& topic, QJsonObject& root) = 0;
    // virtual bool Publish(const QString& topic, const QString& payload) = 0;

    // virtual bool Request(const QString& topic, QJsonObject& root, int timetout = 4000) = 0 ;
    // virtual bool Request(const QString& topic, QJsonObject& root, ReplySuccess success, int timetout = 4000) = 0 ;
    // virtual bool Request(const QString& topic, QJsonObject& root, ReplyCallback callback, int timetout = 4000) = 0;
    // virtual bool Request(const QString& topic, const QJsonObject& root, ResponseCallback callback, int timetout = 4000) = 0;
    // virtual void Subscribe(const char* topic, Callback callback) = 0 ;
    // virtual void Subscribe(const char* topic, MsgCallback callback) = 0;

    virtual void Subscribe(const QString&, Callback callback) = 0 ;
    //virtual void Subscribe(const QString&, MsgCallback callback) = 0;

    //取消订阅
    virtual void UnSubscribe(const char* topic) = 0;
    virtual void OnConnected(std::function<void(bool)> cb) = 0;

    virtual void OnConnectStatus(std::function<void(ConnStatus)> cb) = 0;
    virtual void setConnected(bool newConnected) = 0;
    virtual bool connected() const = 0;
    virtual const QString& host() const = 0;
    virtual bool Logined() = 0;

public:
        // bool m_connected = false;
        // QString m_host;
        // bool logined_ = false;

};

#endif // ConnBase_H
