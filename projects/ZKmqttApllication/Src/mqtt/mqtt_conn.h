#ifndef MqttConn_H
#define MqttConn_H

#include <QTimer>
#include <QObject>
#include "conn_base.h"
#include "common/common.hpp"
#include "mqtt/mqconfig.h"
#include "common/common_config.h"
#include <thread>
#include <QThread>
#include<mutex>
#include <algorithm> // 包含 std::find
#include <thread>
struct mosquitto;  // 修改为struct，与mosquitto库保持一致
class MqttConn : public QObject, public ConnBase {
    Q_OBJECT

public:
    MqttConn();
    ~MqttConn();

    void Start(const string& clientid,const string& ip,int16_t port,int16_t keepalive);
    // void LLMStart(const QString& ip, int port);
    void Stop();
    void startReconnectTimer();
    bool NormalPublish(const QString& topic, const std::string& payload) ;
    void Subscribe(const QString& topic, Callback callback);
    bool PrimitivePublish(const QString& topic, QJsonObject& root);
    bool PrimitivePublish(const QString& topic, const QString& payload);

    // bool Publish(const QString& topic, QJsonObject& root);
    //bool Publish(const QString& topic, const QString& payload);

    // bool Publish_(const QString& topic, QJsonObject& root);
    // bool Publish_(const QString& topic, const QString& payload);
    // bool LLMPublish(const QString& topic, QJsonObject& root);
    // bool NormalPublish(const QString& topic, QJsonObject& root);
    // bool NormalPublish(const QString& topic, const QJsonObject& root);
    // bool NormalPublish(const QString& topic, const QString& payload);
    // bool NormalPublish(const QString& topic, const std::string& payload);


    // bool Request(const QString& topic, QJsonObject& root, int timetout = 4000);
    // bool Request(const QString& topic, QJsonObject& root, ReplySuccess success,
    //              int timetout = 4000);
    // bool NormalRequest(const QString& topic, QJsonObject& root, ReplySuccess success,
    //              int timetout = 4000);
    // bool Request(const QString& topic, QJsonObject& root,
    //              ReplyCallback callback, int timetout = 4000);
    // bool NormalRequest(const QString& topic, QJsonObject& root,
    //                          ReplyCallback callback, int timetout);

    // bool Request(const QString& topic, const QJsonObject& root,
    //              ResponseCallback callback, int timetout = 4000);
    // bool NormalRequest(const QString& topic, const QJsonObject& root,
    //              ResponseCallback callback, int timetout = 4000);
    // void Subscribe(const char* topic, Callback callback);
     //void Subscribe(const char* topic, MsgCallback callback);

    // void Subscribe(const QString& topic, Callback callback);
     void Subscribe(const QString& topic, MsgCallback callback);
    //取消订阅
    void UnSubscribe(const char* topic);
    void OnConnected(std::function<void(bool)> cb);

    void OnConnectStatus(std::function<void(ConnStatus)> cb);

    void setConnected(bool newConnected);

    bool connected() const;

    const QString& host() const;

    bool Login();

    bool Logined();

private slots:
    void OnMessage(QString topic, QString payload);
    void OnConnect( MqttConn* client,int rc);

private:
    static void on_connect(struct mosquitto* mosq, void* obj, int rc);
    static void on_disconnect(struct mosquitto* mosq, void* obj, int rc);
    static void on_message(struct mosquitto*, void* obj,
                           const struct mosquitto_message* msg);
    static void on_message_null(struct mosquitto*, void* obj,
                                const struct mosquitto_message* msg) {}
    void SetupRcLogin();
    void RcRelease();

signals:
    void sgnConnected(bool ok);
    void sgnConnectStatus(ConnStatus status);
    void connectedChanged();

private:
    struct mosquitto* m_client;
    QHash<QString, QList<Callback>> m_callback;
    QHash<QString, QList<MsgCallback>> m_msg_callback;
    bool autoReconnect_ = true;
    QString m_host;
    bool m_connected = false;

    QHash<int, ReplyCallback> replay_callback_;
    QHash<int, ResponseCallback> response_callback_;

    int sn_ = 1;
    QTimer* detect_timer_;

    bool rc_result_ = false;
    QString topic_header_;
    bool login_return_ = false;
    int login_random_;
    bool logined_;
        // 心跳定时器线程
    std::thread *heart_beat_timer_thread;

    // 心跳定时器
    QTimer* heartbeat_timer_;
    QThread heart_beat_timer_thread_;


};
#endif // MqttConn_H
