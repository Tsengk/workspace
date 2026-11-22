#include <mosquitto.h>

#include <random>
#include <QCoreApplication>
#include <QDateTime>
#include <QDebug>
#include <QJsonDocument>
#include <QJsonObject>
#include <QThread>
#include <QTimer>
#include <quuid.h>
#include "mqtt_conn.h"

// #include "config.h"
#include "iostream"


MqttConn::MqttConn() : QObject(nullptr) {
    emit sgnConnectStatus(ConnStatus::INIT);
}

MqttConn::~MqttConn() {
    if (!rc_result_) {
        return;
    }

    mosquitto_message_callback_set(m_client, on_message_null);



}


bool MqttConn::NormalPublish(const QString& topic, const std::string& payload)  {
    QString rc_topic;

    rc_topic = topic;
    //    QJsonDocument doc(root);
    //    QString payload = doc.toJson(QJsonDocument::Compact);
    //    auto data = payload.toUtf8();
    int pub_ret = mosquitto_publish(m_client, nullptr, rc_topic.toLatin1().constData(),
                                    static_cast<int>(payload.length()), payload.c_str(), 2, false);
    //    int rc = mosquitto_publish(mosq_, nullptr, message.topic.c_str(),
    //                             message.payload.length(), message.payload.c_str(),
    //                             message.qos, message.retain);
    if (pub_ret != MOSQ_ERR_SUCCESS) {
        qCritical() << "ERROR: Publish to " << topic << " failed: " << pub_ret;
    }

    return pub_ret == MOSQ_ERR_SUCCESS;
}
void MqttConn::Subscribe(const QString& topic, Callback callback){
    qDebug() << "Subscribe: " << topic;
    m_callback[topic].append(callback);
    if (m_connected) {
        mosquitto_subscribe(m_client, nullptr, topic.toUtf8().constData(), 2);
    } else {
        QString str_topic = topic;
        connect(this, &MqttConn::sgnConnected, this, [this, str_topic] {
            mosquitto_subscribe(m_client, nullptr, str_topic.toUtf8().constData(),2);
        });
    }
}
void MqttConn::Start(const string& clientid,const string& ip,int16_t port,int16_t keepalive) {
    qDebug() << "MqttConn::Start()";
  //  QString id = QString("MRCT1_").append(QUuid::createUuid().toString());
    QString id = QString::fromStdString(clientid);
    qDebug() << id;
    mosquitto_lib_init();
    m_client = mosquitto_new(id.toLatin1().data(), true, this);
    if (m_client == nullptr) {
        qCritical() << "mosquitto_new";
        return;
    }

    // if(CONFIG.remoteConnect_ || ip == CONFIG.llm_ip){
    //  mosquitto_username_pw_set(m_client, mqcfg.username.c_str(), mqcfg.passwrd.c_str());
    //  }

    mosquitto_connect_callback_set(m_client, on_connect);
    mosquitto_disconnect_callback_set(m_client, on_disconnect);
    mosquitto_message_callback_set(m_client, on_message);

    if (mosquitto_loop_start(m_client) != MOSQ_ERR_SUCCESS) {
        qCritical() << "ERROR: mosquitto_loop_start";
    }

    connect(this, &MqttConn::sgnConnectStatus, this,
            [this](ConnStatus status) {
                switch (status) {
                case ConnStatus::INIT:
                    qDebug() << "sgnConnectStatus ConnStatus::INIT";
                    break;
                case ConnStatus::CONNECTING:
                    qDebug() << "sgnConnectStatus ConnStatus::CONNECTING";
                    break;
                case ConnStatus::RECONNECTING:
                    qDebug() << "sgnConnectStatus ConnStatus::RECONNECTING";
                    break;
                case ConnStatus::CONNECTED:
                    qDebug() << "sgnConnectStatus ConnStatus::CONNECTED";
                    break;
                default:
                    break;
                }
            });
    m_connected = false;
    emit sgnConnectStatus(ConnStatus::CONNECTING);



    int rc = mosquitto_connect(m_client, ip.c_str(), port, keepalive);
    if (rc != MOSQ_ERR_SUCCESS) {

        if (autoReconnect_) {
            // startReconnectTimer();
        }


        qDebug() << "ERROR: mosquitto_connect, rc=" << rc << QString::fromStdString(ip) << ":"<<port;
    }


//    auto component_callback = [this](const QJsonObject& root) {
//        int sn = root["sn"].toInt();
//        qDebug() << "/component/result: " << sn;
//        auto itr = replay_callback_.find(sn);
//        if (itr == replay_callback_.end()) {
//            qDebug() << "/component/result: " << sn << " not found callback.";
//            return;
//        }
//        auto callback = *itr;
//        replay_callback_.remove(sn);
//        bool result = root["result"].toBool();
//        callback(result);
//    };

//    QString component_topic = "/component/result";
//    // if(CONFIG.remoteConnect_){
//    //      component_topic = "/" + CONFIG.vin + "/component/result";
//    // }
//    Subscribe(component_topic, component_callback);



    SetupRcLogin();
}



void MqttConn::Stop() {
    // 断开连接和清理资源...
    mosquitto_disconnect(m_client);
    mosquitto_destroy(m_client);
    mosquitto_lib_cleanup();
    mosquitto_loop_stop(m_client, true);
}

void MqttConn::startReconnectTimer()
{
    // if (reconnectThreadRunning_) return;

    // reconnectThreadRunning_ = true;
    // reconnectThread_ = std::make_unique<std::thread>([this]() {
    //     while (reconnectThreadRunning_ && !connected_) {
    //         reconnect();
    //         std::this_thread::sleep_for(reconnectInterval_);
    //     }
    // });
    // reconnectThread_->detach();
}

 bool MqttConn::PrimitivePublish(const QString& topic, QJsonObject& root)  {
    // root["sn"] = sn_++;
     QJsonDocument doc(root);
     auto payload = doc.toJson(QJsonDocument::Compact);

     return PrimitivePublish(topic, payload);
 }

 bool MqttConn::PrimitivePublish(const QString& topic,
                                   const QString& payload) {
     if (!payload.contains("tick")) {
         qDebug() << "Publish: " << topic << payload.left(100);
     }

     auto data = payload.toUtf8();
     int pub_ret =
         mosquitto_publish(m_client, nullptr, topic.toLatin1().constData(),
                           data.length(), data.data(), 2, false);
     if (pub_ret != MOSQ_ERR_SUCCESS) {
         qCritical() << "ERROR: Publish to " << topic << " failed: " << pub_ret;
     }

     return pub_ret == MOSQ_ERR_SUCCESS;
 }

// bool MqttConn::Publish(const QString& topic, QJsonObject& root) {
//     root["sn"] = sn_++;
//     QJsonDocument doc(root);
//     auto payload = doc.toJson(QJsonDocument::Compact);

//     return Publish(topic, payload);
// }

// bool MqttConn::Publish(const QString& topic, const QString& payload) {
//     QString rc_topic;
// //    = topic_header_ + "/" + CONFIG.vin + topic;

//     // if (CONFIG.remoteConnect_ != true) {
//     //     rc_topic = topic_header_ + "/" + CONFIG.vin + topic;

//     //     if (!payload.contains("heartbeat")) {
//     //         qDebug() << "Publish: " << topic << payload.left(100);
//     //     }

//     //     if (!rc_result_) {
//     //         qDebug() << "Publish error: " << topic;
//     //         return false;
//     //     }
//     // } else {
//     //     rc_topic = "root/" + CONFIG.vin + topic;
//     // }


//     auto data = payload.toUtf8();
//     int pub_ret =
//         mosquitto_publish(m_client, nullptr, rc_topic.toLatin1().constData(),
//                           data.length(), data.data(), 2, false);
//     if (pub_ret != MOSQ_ERR_SUCCESS) {
//         qCritical() << "ERROR: Publish to " << topic << " failed: " << pub_ret;
//     }

//     return pub_ret == MOSQ_ERR_SUCCESS;
// }

// bool MqttConn::NormalPublish(const QString& topic, const QString& payload) {
//     auto data = payload.toUtf8();
//     int pub_ret =
//         mosquitto_publish(m_client, nullptr, topic.toLatin1().constData(),
//                           data.length(), data.data(), 2, false);
//     if (pub_ret != MOSQ_ERR_SUCCESS) {
//         qCritical() << "ERROR: Publish to " << topic << " failed: " << pub_ret;
//     }

//     return pub_ret == MOSQ_ERR_SUCCESS;
// }

// bool MqttConn::Publish_(const QString& topic, QJsonObject& root) {
//     root["sn"] = sn_++;
//     QJsonDocument doc(root);
//     auto payload = doc.toJson(QJsonDocument::Compact);

//     return Publish_(topic, payload);
// }

// bool MqttConn::Publish_(const QString& topic, const QString& payload) {
//     QString rc_topic;

//     rc_topic = "root/" + topic;
//     auto data = payload.toUtf8();
//     int pub_ret =
//         mosquitto_publish(m_client, nullptr, rc_topic.toLatin1().constData(),
//                           data.length(), data.data(), 2, false);
//     if (pub_ret != MOSQ_ERR_SUCCESS) {
//         qCritical() << "ERROR: Publish to " << topic << " failed: " << pub_ret;
//     }

//     return pub_ret == MOSQ_ERR_SUCCESS;
// }

// bool MqttConn::Request(const QString& topic, QJsonObject& root,
//                          int timetout) {
//     return Request(
//         topic, root,
//         []() {
//         },
//         timetout);
// }

// bool MqttConn::Request(const QString& topic, QJsonObject& root,
//                          ReplySuccess success, int timetout) {
//     return Request(
//         topic, root,
//         [success](bool) {
//             success();
//             return;
//         },
//         timetout);
// }

// bool MqttConn::NormalRequest(const QString& topic, QJsonObject& root,
//                          ReplySuccess success, int timetout) {
//     return NormalRequest(
//         topic, root,
//         [success](bool) {
//             success();
//             return;
//         },
//         timetout);
// }

// bool MqttConn::Request(const QString& topic, QJsonObject& root,
//                          ReplyCallback callback, int timetout) {
//     if (!m_connected) {
//         std::cout<<"指令发送失败！(未连接)"<<std::endl;

//         return false;
//     }

//     int this_sn = sn_++;
//     if (topic.startsWith("/adb_")) {
//         root["uniqueseq"] = this_sn;
//     } else {
//         root["sn"] = this_sn;
//     }
//     QJsonDocument doc(root);
//     auto payload = doc.toJson(QJsonDocument::Compact);

//     bool res = Publish(topic, payload);

//     if (!res) {
//         std::cout<<"指令发送失败！"<<std::endl;

//         return false;
//     }
//     qDebug() << "Request Replay sn " << this_sn;
//     replay_callback_[this_sn] = callback;
//     QTimer::singleShot(timetout, this, [this, this_sn]() {
//         auto itr = replay_callback_.find(this_sn);
//         if (itr == replay_callback_.end()) {
//             return;
//         }
//         replay_callback_.remove(this_sn);
//         std::cout<<"指令响应超时"<<std::endl;
//     });
//     return true;
// }

// bool MqttConn::NormalRequest(const QString& topic, QJsonObject& root,
//                          ReplyCallback callback, int timetout) {
//     if (!m_connected) {
//         std::cout<<"指令发送失败！(未连接)"<<std::endl;
//         return false;
//     }

//     int this_sn = sn_++;
//     if (topic.startsWith("/adb_")) {
//         root["uniqueseq"] = this_sn;
//     } else {
//         root["sn"] = this_sn;
//     }
//     QJsonDocument doc(root);
//     auto payload = doc.toJson(QJsonDocument::Compact);

//     bool res = NormalPublish(topic, payload);

//     if (!res) {
//         std::cout<<"指令发送失败！"<<std::endl;
//         return false;
//     }
//     qDebug() << "Request Replay sn " << this_sn;
//     replay_callback_[this_sn] = callback;
//     QTimer::singleShot(timetout, this, [this, this_sn]() {
//         auto itr = replay_callback_.find(this_sn);
//         if (itr == replay_callback_.end()) {
//             return;
//         }
//         replay_callback_.remove(this_sn);
//         std::cout<<"指令响应超时"<<std::endl;
//     });
//     return true;
// }

// bool MqttConn::Request(const QString& topic, const QJsonObject& root, ResponseCallback callback, int timetout) {
//     if (!m_connected) {
//         std::cout<<"指令发送失败！(未连接)"<<std::endl;
//         return false;
//     }

//     int this_sn = sn_++;
//     QJsonObject sn_root = root;
//     sn_root["sn"] = this_sn;

//     QJsonDocument doc(sn_root);
//     auto payload = doc.toJson(QJsonDocument::Compact);

//     bool res = Publish(topic, payload);

//     if (!res) {
//         std::cout<<"指令发送失败！"<<std::endl;

//         return false;
//     }
//     qDebug() << "Request response sn " << this_sn;
//     response_callback_[this_sn] = callback;
//     QTimer::singleShot(timetout, this, [this, this_sn]() {
//         auto itr = response_callback_.find(this_sn);
//         if (itr == response_callback_.end()) {
//             return;
//         }
//         response_callback_.remove(this_sn);
//           std::cout<<"指令响应超时"<<std::endl;

//     });
//     return true;
// }

// bool MqttConn::NormalRequest(const QString& topic, const QJsonObject& root, ResponseCallback callback, int timetout) {
//     if (!m_connected) {

//          std::cout<<"指令发送失败！(未连接)"<<std::endl;
//         return false;
//     }

//     int this_sn = sn_++;
//     QJsonObject sn_root = root;
//     sn_root["sn"] = this_sn;

//     QJsonDocument doc(sn_root);
//     auto payload = doc.toJson(QJsonDocument::Compact);

//     bool res = Publish(topic, payload);

//     if (!res) {
//         std::cout<<"指令发送失败！"<<std::endl;
//         return false;
//     }
//     qDebug() << "Request response sn " << this_sn;
//     response_callback_[this_sn] = callback;
//     QTimer::singleShot(timetout, this, [this, this_sn]() {
//         auto itr = response_callback_.find(this_sn);
//         if (itr == response_callback_.end()) {
//             return;
//         }
//         response_callback_.remove(this_sn);
//               std::cout<<"指令响应超时"<<std::endl;
//     });
//     return true;
// }

// void MqttConn::Subscribe(const QString& topic, Callback callback){
//     qDebug() << "Subscribe: " << topic;
//     m_callback[topic].append(callback);
//     if (m_connected) {
//         mosquitto_subscribe(m_client, nullptr, topic.toUtf8().constData(), 2);
//     } else {
//         QString str_topic = topic;
//         connect(this, &MqttConn::sgnConnected, this, [this, str_topic] {
//             mosquitto_subscribe(m_client, nullptr, str_topic.toUtf8().constData(),2);
//         });
//     }
// }

// void MqttConn::Subscribe(const char* topic, Callback callback) {
//     qDebug() << "Subscribe: " << topic;
//     m_callback[topic].append(callback);
//     if (m_connected) {
//         mosquitto_subscribe(m_client, nullptr, topic, 2);
//     } else {
//         QString str_topic = topic;
//         connect(this, &MqttConn::sgnConnected, this, [this, str_topic] {
//             mosquitto_subscribe(m_client, nullptr, str_topic.toUtf8().data(),
//                                 2);
//         });
//     }
// }

void MqttConn::Subscribe(const QString& topic, MsgCallback callback){
    qDebug() << "Subscribe: " << topic;
    m_msg_callback[topic].append(callback);

    if (m_connected) {
        mosquitto_subscribe(m_client, nullptr, topic.toUtf8().constData(), 2);
    } else {
        connect(this, &MqttConn::sgnConnected, this, [this, topic] {
            mosquitto_subscribe(m_client, nullptr, topic.toUtf8().constData(), 2);
        });
    }
}

// void MqttConn::Subscribe(const char* topic, MsgCallback callback) {
//     qDebug() << "Subscribe: " << topic;
//     m_msg_callback[topic].append(callback);

//     if (m_connected) {
//         mosquitto_subscribe(m_client, nullptr, topic, 2);
//     } else {
//         connect(this, &MqttConn::sgnConnected, this, [this, topic] {
//             mosquitto_subscribe(m_client, nullptr, topic, 2);
//         });
//     }
// }

//取消订阅
void MqttConn::UnSubscribe(const char* topic)
{
    if(m_connected)
    {
        mosquitto_unsubscribe(m_client,nullptr,topic);
    }
}

void MqttConn::OnConnected(std::function<void(bool)> cb) {
    connect(this, &MqttConn::sgnConnected, cb);
}

void MqttConn::OnConnectStatus(std::function<void(ConnStatus)> cb) {
    connect(this, &MqttConn::sgnConnectStatus, cb);
}

void MqttConn::OnMessage(QString topic, QString payload) {
    auto msg_itr = m_msg_callback.find(topic);
    if (msg_itr != m_msg_callback.end()) {
        for (auto cb : msg_itr.value()) {
            cb(payload);
        }
    }

    auto itr = m_callback.find(topic);
    if (itr == m_callback.end()) {
        return;
    }
    QJsonParseError error;
    QJsonDocument doc = QJsonDocument::fromJson(payload.toUtf8(), &error);
    if (error.error != QJsonParseError::NoError) {
        return;
    }
    QJsonObject root = doc.object();
    for (auto cb : itr.value()) {
        cb(root);
    }
}

void MqttConn::OnConnect(MqttConn *client, int rc)
{
    client->m_connected = true;
    emit client->sgnConnectStatus(ConnStatus::CONNECTED);
    emit client->sgnConnected(rc == 0);
}

void MqttConn::on_connect(mosquitto* mosq, void* obj, int rc) {
    Q_UNUSED(mosq)
    qDebug() << "MqttConn: on_connect, rc=" << rc;
    MqttConn* client = static_cast<MqttConn*>(obj);
    if (rc != 0) {
        return;
    }


    client->m_connected = true;
    emit client->sgnConnectStatus(ConnStatus::CONNECTED);
    emit client->sgnConnected(rc == 0);

    //  QMetaObject::invokeMethod(client,"OnConnect",Qt::DirectConnection);
}

void MqttConn::on_disconnect(mosquitto* mosq, void* obj, int rc) {
    Q_UNUSED(mosq)
    qDebug() << "MqttConn: on_disconnect, rc=" << rc;
    MqttConn* client = static_cast<MqttConn*>(obj);
    if (rc == 0) {
        return;
    }
    // QMetaObject::invokeMethod(client, [mosq, client, rc]() {
    //     if (client->m_connected) {
    //         qDebug() << "网络已断开，请注意状态栏连接状态！";

    //         std::cout<<"网络已断开，请注意状态栏连接状态！" <<std::endl;


    //         client->m_connected = false;
    //         emit client->sgnConnectStatus(ConnStatus::RECONNECTING);
    //     }
    // });
    return;
}

void MqttConn::on_message(mosquitto*, void* obj,
                          const mosquitto_message* msg) {
    MqttConn* client = static_cast<MqttConn*>(obj);
    QMetaObject::invokeMethod(client, "OnMessage", Qt::QueuedConnection,
                              Q_ARG(QString, msg->topic),
                              Q_ARG(QString, (char*)msg->payload));
}

void MqttConn::SetupRcLogin() {


    //     heartbeat_timer_ = new QTimer();
    //     qDebug() << "std::thread id =" << QThread::currentThreadId();
    //     heartbeat_timer_->callOnTimeout([this]() {
    //         if (!rc_result_) {
    //             return;
    //         }
    //         QString topic = "/" + CONFIG.vin + "/RC/request/heartbeat";
    //         QJsonObject root{{"topic_header", topic_header_},
    //                          {"timestamp", QDateTime::currentMSecsSinceEpoch()}};
    //         qDebug() << "test1";
    //         PrimitivePublish(topic, root);
    //     });
    //     heartbeat_timer_->start(1000);
    //     heartbeat_timer_->moveToThread(&heart_beat_timer_thread_);
    //     connect(this, &MqttConn::sgnConnected, this, [this] {
    //         Login();
    //     });
    //     heart_beat_timer_thread_.start();

    //     QString response_topic = "/" + CONFIG.vin + "/RC/response/login";

    //     Subscribe(response_topic.toUtf8().constData(),
    //               [this, response_topic](QJsonObject& root) {
    //                   QString topic_header = root["topic_header"].toString();
    // //                  if (login_random_ != root["random"].toInt()) {
    // //                      login_return_ = false;
    // //                      rc_result_ = false;
    // //                      topic_header_ = "";
    // //                      EB.Publish(EV_LOGIN_FAILED, true);
    // //                      EB.Publish(EV_TOAST_USER_ERROR, QString("登录已失效"));
    // //                      return;
    // //                  }
    //                   login_return_ = true;
    //                   if (!root["result"].toBool()) {
    //                       rc_result_ = false;
    //                       std::cout<<"权限请求失败"<<std::endl;

    //                       return;
    //                   }
    //                   std::cout<<"登录成功"<<std::endl;


    //                   rc_result_ = true;
    //                   topic_header_ = topic_header;
    //                   logined_ = true;
    //               });

    //     QString response_heartbeat = "/" + CONFIG.vin + "/RC/response/heartbeat";
    //     Subscribe(response_heartbeat.toUtf8().constData(),
    //               [this, response_topic](QJsonObject& root) {
    // //                  if (!login_return_) {
    // //                      return;
    // //                  }
    // //                  QString header_of_header = root["topic_header"].toString();
    // //                  if (topic_header_.contains(header_of_header) && !header_of_header.isEmpty()) {
    // //                      return;
    // //                  }
    // //                  login_return_ = false;
    // //                  rc_result_ = false;
    // //                  topic_header_ = "";
    // //                  EB.Publish(EV_LOGIN_FAILED, true);
    // //                  EB.Publish(EV_TOAST_USER_ERROR, QString("登录已失效"));
    // //                  logined_ = false;
    //               });

    //     EB.Subscribe(EV_LOGIN, [this](bool) {
    //         Login();
    //     });
}

const QString& MqttConn::host() const {
    return m_host;
}

bool MqttConn::Login() {

    // if (CONFIG.remoteConnect_ == true) {
    //     return false;
    // }
    static bool login_pending = false;

    if (login_pending) {
        qDebug("login pending");
        return false;
    }

    login_pending = true;
    login_return_ = false;
    rc_result_ = false;
    // if (CONFIG.vin.isEmpty()) {

    //     EB.Publish(EV_TOAST_USER_ERROR, QString("vin为空"));
    //     return false;
    // }

    // srand(uint32_t(QDateTime::currentMSecsSinceEpoch()));
    // login_random_ = rand();
    // QString topic = "/" + CONFIG.vin + "/RC/request/login";
    // QJsonObject root{{"terminal_type", "mrct"},
    //                  {"version", "MRCT"},
    //                  {"root_key", "88888888"},
    //                  {"jurisdiction", 7},
    //                  {"random", login_random_},
    //                  {"timestamp", QDateTime::currentMSecsSinceEpoch()}};
    // PrimitivePublish(topic, root);
    // QTimer::singleShot(3000, this, [this]() {
    //     login_pending = false;
    //     if (login_return_) {
    //         return;
    //     }
    //     std::cout<<"登录响应超时"<<std::endl;

    // });
    return true;
}

bool MqttConn::Logined()
{
    return logined_;
}

void MqttConn::setConnected(bool newConnected) {
    if (m_connected == newConnected)
        return;
    m_connected = newConnected;
    emit connectedChanged();
}

bool MqttConn::connected() const {
    return m_connected;
}
