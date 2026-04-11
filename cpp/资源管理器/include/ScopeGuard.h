#ifndef SCOPE_GUARD_H
#define SCOPE_GUARD_H

#include <functional>
#include <utility>

class ScopeGuard {
private:
    std::function<void()> cleanup_func;//存储清理逻辑
    bool dismissed;//标记是否取消清除

public:
    explicit ScopeGuard(std::function<void()> on_exit) : cleanup_func(std::move(on_exit)), dismissed(false) {}
    ~ScopeGuard() {
        if(!dismissed && cleanup_func) {
            cleanup_func();
        }
    }
    ScopeGuard(const ScopeGuard&) = delete;
    ScopeGuard& operator=(const ScopeGuard&) = delete;
    ScopeGuard(ScopeGuard&& other) = delete;
    ScopeGuard& operator=(ScopeGuard&&) = delete;

    //手动取消清理
    void dismiss() {
        dismissed = true;
    }
};

// 宏定义：用于生成隐式的 ScopeGuard 对象，模拟 Go 语言的 defer 关键字
// 利用 __LINE__ 宏生成唯一的变量名，防止在同一作用域内发生变量名冲突
#define SCOPEGUARD_LINENAME_CAT(name, line) name##line
#define SCOPEGUARD_MACRO_EXPAND(name, line) SCOPEGUARD_LINENAME_CAT(name, line)
#define DEFER(cleanup_code) ScopeGuard SCOPEGUARD_MACRO_EXPAND(guard_, __LINE__)([&](){ cleanup_code; })
#endif