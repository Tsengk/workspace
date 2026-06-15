#!/usr/bin/env node
/**
 * 结合 ZMQ_TOOL 后端启动流程，将回调改写为 Promise
 * 对照 backend/main.py: initLogger → initWebSocket → initVideo → startServer
 */

"use strict";

function initModule(name, delay, shouldFail = false) {
    return new Promise((resolve, reject) => {
        console.log(`  [开始] ${name}...`);
        setTimeout(() => {
            if (shouldFail) reject(new Error(`${name} 失败`));
            else            resolve(`${name} 完成`);
        }, delay);
    });
}

console.log("===== ZMQ Tool 后端启动（Promise）=====\n");

initModule("Logger", 200)
    .then(result => {
        console.log("  ✅", result);
        return initModule("SocketManager", 300);
    })
    .then(result => {
        console.log("  ✅", result);
        return initModule("VideoManager", 250);
    })
    .then(result => {
        console.log("  ✅", result);
        return initModule("MapService", 200, true);  // 模拟失败
    })
    .then(result => {
        console.log("  ✅", result);
    })
    .catch(err => {
        console.error("  ❌", err.message);
        console.log("  ⚠️ 服务降级启动");
    })
    .finally(() => {
        console.log("  🏁 启动流程结束");
    });
