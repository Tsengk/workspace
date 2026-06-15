#!/usr/bin/env node
/**
 * 结合 ZMQ_TOOL 后端启动流程，用 async/await 改写
 * 对比 backend/main.py 中各组件的初始化顺序
 */

"use strict";

function initModule(name, delay, shouldFail = false) {
    return new Promise((resolve, reject) => {
        setTimeout(() => {
            if (shouldFail) reject(new Error(`${name} 失败`));
            else            resolve(`${name} 完成`);
        }, delay);
    });
}

async function startup() {
    console.log("===== ZMQ Tool 后端启动（async/await）=====\n");
    const start = Date.now();

    try {
        // 第一步：基础组件（串行有依赖）
        let r = await initModule("Logger", 200);
        console.log("  ✅", r);

        // 第二步：Socket 和 Video 可以并行
        let [sock, video] = await Promise.all([
            initModule("SocketManager", 300),
            initModule("VideoManager", 250),
        ]);
        console.log("  ✅", sock);
        console.log("  ✅", video);

        // 第三步：MapService（可能失败）
        r = await initModule("MapService", 200, true);
        console.log("  ✅", r);

    } catch (err) {
        console.error("  ❌", err.message);
        console.log("  ⚠️ 服务降级启动");
    } finally {
        console.log(`  ⏱️ 启动耗时: ${Date.now() - start}ms`);
        console.log("  🏁 启动流程结束");
    }
}

startup();
