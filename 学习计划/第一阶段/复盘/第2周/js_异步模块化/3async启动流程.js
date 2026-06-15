#!/usr/bin/env node
"use strict";

function initModule(name, delay, shouldFail = false) {
    return new Promise((resolve, reject) => {
        setTimeout(() => {
            if (shouldFail) reject(new Error(`${name} 失败`));
            else resolve(`${name} 成功`);
        }, delay)
    });
}
//声明异步函数
//函数被调用时会自动返回promise
//Promise.all([...]) 接收一个由多个 Promise 组成的数组，并返回一个全新的、总的 Promise
async function startup() {
    console.log("=======ZMQ TOOL 后端启动=======");
    const start = Date.now();

    try {
        let r = await initModule("Logger", 200);//主线程释放,挂起等待异步操作完成，200毫秒后继续执行函数剩下的代码
        console.log("启动成功", r);
        let [sock, video] = await Promise.all([
            initModule("SocketManager", 300),
            initModule("VideoManager", 300),
        ]);
        console.log("启动成功", sock, video);
        r = await initModule("MapService", 200, true);
        console.log("启动成功", r);
    } catch (err) {
        console.error("启动终止:", err.message);
        console.log("MapServer启动失败,服务降级启动");
    } finally {
        const end = Date.now();
        console.log(`总耗时: ${end - start} ms`);
    }
}

startup();