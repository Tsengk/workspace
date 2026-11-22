
#pragma once
#include <functional>
#include <mutex>
#include <string>
#include <unordered_map>
#include <memory>

class EventBus {
private:
    typedef std::unordered_multimap<std::string, std::shared_ptr<void>> map;
    EventBus() = default;



public:
    ~EventBus() = default;
    EventBus(const EventBus&) = delete;
    EventBus& operator=(const EventBus&) = delete;


    static EventBus& Instance() {
        static EventBus instance;
        return instance;
    }


    template <typename Handler, typename... Args>
    void Subscribe(const char* topic, Handler handler) {
        std::function<void(Args...)> func = handler;
        auto ptr = std::make_shared<std::function<void(Args...)>>(func);
        std::unique_lock<std::mutex> locker(sub_mtx_);
        subscribers_.emplace(topic, std::static_pointer_cast<void>(ptr));
    }


    template <typename... Args>
    void Publish(const char* topic, Args... args) {
        std::unique_lock<std::mutex> locker(pub_mtx_);
        const auto range = subscribers_.equal_range(topic);
        for (auto itr = range.first; itr != range.second; ++itr) {
            auto func_ptr = std::static_pointer_cast<std::function<void(Args...)>>(itr->second);
            (*func_ptr)(args...);
        }
    }


protected:
    map subscribers_;
    std::mutex pub_mtx_;
    std::mutex sub_mtx_;
};


#define EB (EventBus::Instance())


