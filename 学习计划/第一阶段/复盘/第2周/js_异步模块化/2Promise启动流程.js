#!/usr/bin/env node
"use strict";
//new Promise()创建一个将要发生异步事件的包装占位对象,状态为 pending（待定）:可以理解为一张成功或失败的凭证
//Promise对象表示一个异步操作的最终完成或失败
//resolve和reject是Promise内部的两个函数，用于改变Promise的状态
//resolve：用于将 Promise 的状态从 pending 变更为 fulfilled（已兑现/已成功）
//reject：用于将 Promise 的状态从 pending 变更为 rejected（已拒绝/已失败）

function initModule(name, delay, shouldFail = false) {
    return new Promise((resolve, reject) => {
        console.log(`开始初始化模块: ${name}`);
        setTimeout(() => {
            if (shouldFail) reject(new Error(`${name} 失败`));
            else resolve(`${name} 成功`);
        },delay)
    });
}

console.log("=======ZMQ TOOL 后端启动=======");

initModule("Logger",200)//返回promise对象,[[PromiseState]]：存储状态（pending、fulfilled 或 rejected）;[[PromiseResult]]：存储数据值
    .then(result => {//resolve("Logger 成功")=>[[PromiseState]] 变成了 fulfilled;[[PromiseResult]] 变成了 "Logger 成功"
        console.log("启动成功", result);
        return initModule("SocketManager", 200);//返回新的Promise对象，状态为pending，下一个then会等待接收这个Promise的resolve值
    })
    .then(result => {
        console.log("启动成功", result);
        return initModule("VideoManager", 200);
    })
    .then(result => {
        console.log("启动成功", result);
        return initModule("MapService", 200, true);//模拟失败
    })
    .then(result => {
        console.log("启动成功", result);
    })
    .catch(err => {//reject(new Error(`${name} 失败`))=>[[PromiseState]] 变成了 rejected;[[PromiseResult]] 变成了 Error对象
        console.error("启动终止:", err.message);
        console.log("MapServer启动失败,服务降级启动");
    })
    .finally(() => {//无论成功还是失败都会执行
        console.log("  🏁 启动流程结束");
    });