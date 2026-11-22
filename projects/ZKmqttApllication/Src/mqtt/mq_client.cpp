#include "mq_client.h"
#include <iostream>
#include <cstring>
#include "utils/json.hpp"
#include <thread>
#include <QUuid>


using json = nlohmann::json;

MqClient::MqClient()  {
    //    mosquitto_lib_init();
    //    mosq_ = mosquitto_new(QUuid::createUuid().toString().toStdString().c_str(), true, this);

    //    if (mosq_) {
    //        mosquitto_connect_callback_set(mosq_, onConnect);
    //        mosquitto_disconnect_callback_set(mosq_, onDisconnect);
    //        mosquitto_message_callback_set(mosq_, onMessage);
    //        mosquitto_log_callback_set(mosq_, onLog);
    //        mosquitto_loop_start(mosq_);
    //    }


    //    instance.connect("39.105.94.149", 1883, "special_car", "idriverplus");
    //    MqttSubList();
    //    instance.connect()
    conn_ptr_ = std::make_shared<MqttConn>();


}



MqClient::~MqClient() {
    // stopReconnectTimer();
    // if (mosq_) {
    //     disconnect();
    //     mosquitto_destroy(mosq_);
    // }
    // mosquitto_lib_cleanup();
}

// bool MqClient::connect(const std::string& host, int port,
//                       const std::string& username,
//                       const std::string& password) {
//     if (!mosq_) return false;

//     // 保存连接信息用于重连
//     host_ = host;
//     port_ = port;
//     username_ = username;
//     password_ = password;

//     // 设置认证信息
//     if (!username.empty()) {
//         mosquitto_username_pw_set(mosq_, username.c_str(), password.c_str());
//     }

//     int rc = mosquitto_connect(mosq_, host.c_str(), port, 60);
//     if (rc != MOSQ_ERR_SUCCESS) {
//         std::cerr << "连接MQTT服务器失败: " << mosquitto_strerror(rc) << std::endl;
//         if (autoReconnect_) {
//             startReconnectTimer();
//         }
//         return false;
//     }

//     return true;
// }

// void MqClient::disconnect() {
//     stopReconnectTimer();
//     if (mosq_ && connected_) {
//         mosquitto_disconnect(mosq_);
//     }
//     connected_ = false;
// }

// bool MqClient::publish(const std::string& topic, const json& data, int qos) {
//     return primitivePublish(MqttMessage(topic, data.dump(), qos));
// }

// bool MqClient::publish(const std::string& topic, const std::string& payload, int qos) {
//     return primitivePublish(MqttMessage(topic, payload, qos));
// }

// bool MqClient::subscribe(const std::string& topic, TopicCallback callback, int qos) {
//     if (!mosq_) return false;

//     std::lock_guard<std::mutex> lock(mutex_);
//     topicCallbacks_[topic].push_back(callback);
//     return mosquitto_subscribe(mosq_, nullptr, topic.c_str(), qos) == MOSQ_ERR_SUCCESS;
// }

// bool MqClient::subscribe(const std::string& topic, MessageCallback callback, int qos) {
//     if (!mosq_) return false;

//     std::lock_guard<std::mutex> lock(mutex_);
//     messageCallbacks_[topic].push_back(callback);
//     return mosquitto_subscribe(mosq_, nullptr, topic.c_str(), qos) == MOSQ_ERR_SUCCESS;
// }

// bool MqClient::primitivePublish(const MqttMessage& message) {
// //    if (!mosq_ || !connected_) return false;

//     // 添加连接状态检查
//     if (!mosq_ || !connected_) {
//         std::cerr << "MQTT client not connected" << std::endl;
//         return false;
//     }

//     // 添加参数有效性检查
//     if (message.topic.empty() || !message.payload.c_str()) {
//         std::cerr << "Invalid message parameters" << std::endl;
//         return false;
//     }

//     int rc = mosquitto_publish(mosq_, nullptr, message.topic.c_str(),
//                              message.payload.length(), message.payload.c_str(),
//                              message.qos, message.retain);
//     // 详细的错误处理
//     if (rc != MOSQ_ERR_SUCCESS) {
//         std::cerr << "Failed to publish message: " << mosquitto_strerror(rc) << std::endl;
//         return false;
//     }
//     return rc == MOSQ_ERR_SUCCESS;
// }

// bool MqClient::request(const std::string& topic, const json& root,
//                       ReplyCallback callback, int timeout) {
//     int sn = requestSn_++;
//     json request = root;
//     request["sn"] = sn;

//     if (callback) {
//         replyCallbacks_[sn] = callback;
//     }

//     return primitivePublish(MqttMessage(topic, request.dump()));
// }

void MqClient::Start(const string& clientid,const string& ip,int16_t port,int16_t keepalive) {

    conn_ptr_->Start(clientid,ip, port,keepalive);

    // mosq_ = mosquitto_new(QUuid::createUuid().toString().toStdString().c_str(), true, this);
    // if (mosq_ == nullptr) {
    //     std::cout  << "mosquitto_new" << std::endl;
    //     return;
    // }


    // mosquitto_username_pw_set(mosq_, "special_car", "idriverplus");


    // //        mosquitto_connect_callback_set(mosq_, onConnect);
    // //        mosquitto_disconnect_callback_set(mosq_, onDisconnect);
    // //        mosquitto_message_callback_set(mosq_, onMessage);
    // //        mosquitto_log_callback_set(mosq_, onLog);
    // //        mosquitto_loop_start(mosq_);

    // mosquitto_connect_callback_set(mosq_, onConnect);
    // mosquitto_disconnect_callback_set(mosq_, onDisconnect);
    // mosquitto_message_callback_set(mosq_, onMessage);

    // if (mosquitto_loop_start(mosq_) != MOSQ_ERR_SUCCESS) {
    //     std::cout << "ERROR: mosquitto_loop_start" << std::endl;
    // }

    // int rc = mosquitto_connect_async(mosq_, _mqcfg.c_str(), port, 5);
    // if (rc != MOSQ_ERR_SUCCESS) {
    //     std::cout << "ERROR: mosquitto_connect, rc=" << rc << _mqcfg << port << std::endl;
    // }
}


void MqClient::Stop(){
    conn_ptr_->Stop();
}

// void MqClient::Start(const std::string& ip, int port) {
//     conn_ptr_->LLMStart(QString::fromStdString(_mqcfg), 1883);

// //    qDebug() << "MqttConn::Start()";
// //    QString id = QString("MRCT%1").arg(QDateTime::currentMSecsSinceEpoch());
// //    qDebug() << id;
//     mosq_ = mosquitto_new(QUuid::createUuid().toString().toStdString().c_str(), true, this);
//     if (mosq_ == nullptr) {
//         std::cout  << "mosquitto_new" << std::endl;
//         return;
//     }


//     mosquitto_username_pw_set(mosq_, "special_car", "idriverplus");


// //        mosquitto_connect_callback_set(mosq_, onConnect);
// //        mosquitto_disconnect_callback_set(mosq_, onDisconnect);
// //        mosquitto_message_callback_set(mosq_, onMessage);
// //        mosquitto_log_callback_set(mosq_, onLog);
// //        mosquitto_loop_start(mosq_);

//     mosquitto_connect_callback_set(mosq_, onConnect);
//     mosquitto_disconnect_callback_set(mosq_, onDisconnect);
//     mosquitto_message_callback_set(mosq_, onMessage);

//     if (mosquitto_loop_start(mosq_) != MOSQ_ERR_SUCCESS) {
//          std::cout << "ERROR: mosquitto_loop_start" << std::endl;
//     }

//     int rc = mosquitto_connect_async(mosq_, _mqcfg.c_str(), port, 5);
//     if (rc != MOSQ_ERR_SUCCESS) {
//         std::cout << "ERROR: mosquitto_connect, rc=" << rc << _mqcfg << port << std::endl;
//     }
// }

// void MqClient::handleMessage(const std::string& topic, const std::string& payload) {
//     // 处理普通回调
//     auto it = topicCallbacks_.find(topic);
//     if (it != topicCallbacks_.end()) {
//         for (const auto& callback : it->second) {
//             callback(payload);
//         }
//     }

//     // 处理带主题的回调
//     auto msgIt = messageCallbacks_.find(topic);
//     if (msgIt != messageCallbacks_.end()) {
//         for (const auto& callback : msgIt->second) {
//             callback(topic, payload);
//         }
//     }

//     // 处理请求-响应回调
//     try {
//         json root = json::parse(payload);
//         if (root.contains("sn")) {
//             int sn = root["sn"].get<int>();
//             auto replyIt = replyCallbacks_.find(sn);
//             if (replyIt != replyCallbacks_.end()) {
//                 replyIt->second(root);
//                 replyCallbacks_.erase(replyIt);
//             }
//         }
//     } catch (const json::parse_error& e) {
//         std::cerr << "JSON解析错误: " << e.what() << std::endl;
//     }
// }

// // 静态回调函数实现
// void MqClient::onConnect(struct mosquitto* mosq, void* obj, int rc) {
//     MqClient* client = static_cast<MqClient*>(obj);
//     if (rc == 0) {
//         client->connected_ = true;
//         std::cout << "已成功连接到MQTT服务器" << std::endl;
//     } else {
//         std::cerr << "连接失败，返回码: " << rc << std::endl;
//     }
// }

// void MqClient::onDisconnect(struct mosquitto* mosq, void* obj, int rc) {
//     MqClient* client = static_cast<MqClient*>(obj);
//     client->connected_ = false;
//     std::cout << "已断开与MQTT服务器的连接" << std::endl;
// }

// void MqClient::onMessage(struct mosquitto* mosq, void* obj,
//                         const struct mosquitto_message* message) {
//     MqClient* client = static_cast<MqClient*>(obj);
//     std::string topic(message->topic);
//     std::string payload(static_cast<char*>(message->payload), message->payloadlen);

//     // 处理消息回调
//     client->handleMessage(topic, payload);
// }

// void MqClient::startReconnectTimer() {
//     if (reconnectThreadRunning_) return;

//     reconnectThreadRunning_ = true;
//     reconnectThread_ = std::make_unique<std::thread>([this]() {
//         while (reconnectThreadRunning_ && !connected_) {
//             reconnect();
//             std::this_thread::sleep_for(reconnectInterval_);
//         }
//     });
//     reconnectThread_->detach();
// }

// void MqClient::stopReconnectTimer() {
//     reconnectThreadRunning_ = false;
//     if (reconnectThread_ && reconnectThread_->joinable()) {
//         reconnectThread_->join();
//     }
// }

// void MqClient::reconnect() {
//     if (!mosq_ || connected_) return;

//     std::cout << "尝试重新连接到MQTT服务器..." << std::endl;

//     if (!username_.empty()) {
//         mosquitto_username_pw_set(mosq_, username_.c_str(), password_.c_str());
//     }

//     int rc = mosquitto_connect(mosq_, host_.c_str(), port_, 60);
//     if (rc != MOSQ_ERR_SUCCESS) {
//         std::cerr << "重连失败: " << mosquitto_strerror(rc) << std::endl;
//     }
// }

// void MqClient::onLog(struct mosquitto* mosq, void* obj, int level, const char* str) {
//     // 可以根据需要处理日志
//     // 这里简单地打印日志信息
//     switch(level) {
//         case MOSQ_LOG_ERR:
//             std::cerr << "MQTT Error: " << str << std::endl;
//             break;
//         case MOSQ_LOG_WARNING:
//             std::cout << "MQTT Warning: " << str << std::endl;
//             break;
//         default:
//             // 其他日志级别可以选择性忽略
//             break;
//     }
// }
