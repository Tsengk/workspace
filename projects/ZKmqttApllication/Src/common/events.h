#pragma once

#include <string>

namespace Events {
    // 事件主题常量
    static const char* ADD_ELECTROMAGNETIC_TASK = "add_electromagnetic_task";
    static const char* EXECUTE_ELECTROMAGNETIC_TASK = "execute_electromagnetic_task";
    static const char* TEST_REMOVE_ZK_ITEM = "TEST_REMOVE_ZK_ITEM";
    // 电磁侦干任务事件数据结构
    struct ElectromagneticTaskEvent {
        std::string taskId;
        std::string unitId;
        std::string targetId;
        std::string frequency;
        
        ElectromagneticTaskEvent() = default;
        ElectromagneticTaskEvent(const std::string& tid, const std::string& uid, 
                               const std::string& tgtId, const std::string& freq)
            : taskId(tid), unitId(uid), targetId(tgtId), frequency(freq) {}
    };
}