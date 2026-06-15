#!/usr/bin/env node
/**
 * 结合 ZMQ_TOOL 后端 main.py 的启动模式：先初始化日志 → 再初始化组件 → 最后启动服务
 * 用回调函数模拟这种"下一步依赖上一步"的启动流程
 */

"use strict";

// 模拟异步初始化（类似 main.py 中 import 并初始化各组件）
function initModule(name, delay, callback) {
    console.log(`  [开始] ${name} 初始化...`);
    setTimeout(() => {
        if (name === "MapService") {
            // 模拟失败（类似 main.py 中 MapServiceWsBridge 可能连接拒绝）
            callback(new Error(`${name} 连接失败: Connection refused`));
        } else {
            callback(null, `${name} 初始化完成`);
        }
    }, delay);
}

// ZMQ Tool 后端实际启动顺序: logger → SocketManager → VideoManager → MapService
console.log("===== ZMQ Tool 后端启动（回调方式）=====\n");

initModule("Logger", 200, (err, data) => {
    if (err) return console.error("启动终止:", err.message);
    console.log("  ✅", data);

    initModule("SocketManager", 300, (err, data) => {
        if (err) return console.error("启动终止:", err.message);
        console.log("  ✅", data);

        initModule("VideoManager", 250, (err, data) => {
            if (err) return console.error("启动终止:", err.message);
            console.log("  ✅", data);

            initModule("MapService", 200, (err, data) => {
                if (err) {
                    console.error("  ❌", err.message);
                    console.log("  ⚠️ MapService 失败，服务降级启动");
                } else {
                    console.log("  ✅", data);
                }
                console.log("\n  🏁 启动流程结束");
            });
        });
    });
});
