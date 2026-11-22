#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

#include <mosquitto.h>
#include <string>
#include <functional>
#include <memory>
#include <map>
#include <list>
#include <mutex>
#include <queue>
#include <atomic>
#include <chrono>
#include "utils/json.hpp"
#include <iostream>
#include <thread>
#include "mqtt/mqtt_conn.h"
#include "mqtt/mqconfig.h"
#include "common/common_config.h"


// 使用更清晰的类型别名
using json = nlohmann::json;
// using TopicCallback = std::function<void(const std::string&)>;
// using MessageCallback = std::function<void(const std::string&, const std::string&)>;
// using ReplyCallback = std::function<void(const json&)>;
// using ConnectCallback = std::function<void(bool)>;

// MQTT消息结构体
struct MqttMessage {
    std::string topic;
    std::string payload;
    int qos;
    bool retain;

    MqttMessage(const std::string& t, const std::string& p, int q = 0, bool r = false)
        : topic(t), payload(p), qos(q), retain(r) {}
};

class MqClient {
public:
    // // 获取单例实例
    // static MqClient& getInstance() {
    //     return *instance;
    // }
    // 构造函数改为私有
    MqClient();
    ~MqClient();
    // 删除拷贝构造和赋值操作符
    MqClient(const MqClient&) = delete;
    MqClient& operator=(const MqClient&) = delete;

    // 连接相关
    //  bool connect(const std::string& host, int port = 1883,
    //          const std::string& username = "",
    //        const std::string& password = "");
    // void disconnect();
    // bool isConnected() const { return connected_; }
    // void setConnectCallback(ConnectCallback cb) { connectCallback_ = cb; }

    // 发布消息
    // bool publish(const std::string& topic, const json& data, int qos = 0);
    // bool publish(const std::string& topic, const std::string& payload, int qos = 0);

    // 订阅相关
    // bool subscribe(const std::string& topic, TopicCallback callback, int qos = 0);
    // bool subscribe(const std::string& topic, MessageCallback callback, int qos = 0);
    // bool unsubscribe(const std::string& topic);

    // 请求-响应模式
    // bool request(const std::string& topic, const json& data,
    //             ReplyCallback callback = nullptr, int timeout = 4000);

    // 重连相关
    // void enableAutoReconnect(bool enable = true);
    // void setReconnectInterval(std::chrono::seconds interval);
    // void Start(const std::string& ip, int port);
    void Start(const string& clientid,const string& ip,int16_t port,int16_t keepalive);
    void Stop();
    MqttConn* GetMqttConn() {
        return conn_ptr_.get();
    }

private:


    // MQTT回调处理
    // static void onConnect(struct mosquitto* mosq, void* obj, int rc);
    // static void onDisconnect(struct mosquitto* mosq, void* obj, int rc);
    // static void onMessage(struct mosquitto* mosq, void* obj,
    //                      const struct mosquitto_message* message);
    // static void onLog(struct mosquitto* mosq, void* obj,
    //                  int level, const char* str);

    // void handleMessage(const std::string& topic, const std::string& payload);
    //  bool primitivePublish(const MqttMessage& message);
    // void startReconnectTimer();
    // void stopReconnectTimer();
    //void reconnect();
    // void processMessageQueue();

    // MQTT客户端相关
    //  struct mosquitto* mosq_;
    std::atomic<bool> connected_{false};
    std::mutex mutex_;
    std::string host_;
    int port_;
    std::string username_;
    std::string password_;

    // 重连相关
    std::atomic<bool> autoReconnect_{true};
    std::chrono::seconds reconnectInterval_{5};
    std::unique_ptr<std::thread> reconnectThread_;
    std::atomic<bool> reconnectThreadRunning_{false};

    // 消息队列
    std::queue<MqttMessage> messageQueue_;
    std::mutex queueMutex_;
    size_t maxQueueSize_{100};

    // 回调存储
    // std::map<std::string, std::list<TopicCallback>> topicCallbacks_;
    // std::map<std::string, std::list<MessageCallback>> messageCallbacks_;
    // std::map<int, ReplyCallback> replyCallbacks_;
    // ConnectCallback connectCallback_;



    int requestSn_{0};
    std::shared_ptr<MqttConn> conn_ptr_;
};
#endif // MQTT_CLIENT_H

