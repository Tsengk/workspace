#!/usr/bin/env node
"use strict";

//模拟异步模块初始化
function initModule(name, delay, callback) {
    console.log(`初始化模块: ${name}`);
    //setTimeout(函数, 延迟时间),先延迟再执行函数
    setTimeout(() => {
        if (name === "MapService") {
            callback(new Error(`${name} 连接超时：Connection timeout`))//new Error()生成错误对象，存放在err.message属性里
        } else {
            callback(null, `${name}初始化完成`);
        }
    },delay);
}

console.log("=======ZMQ TOOL 后端启动=======");

initModule("Logger", 1000,(err, data) => {
    if (err) return console.error("启动终止:", err.message);
    console.log("启动成功", data);

    initModule("SocketManager", 1000,(err, data) => {
        if (err) return console.error("启动终止:", err.message);
        console.log("启动成功", data);
        initModule("VideoManager", 1000,(err,data) => {
            if (err) return console.error("启动终止:", err.message);
            console.log("启动成功", data);

            initModule("MapService", 1000, (err,data) => {
                if (err) {
                    console.error("启动终止:", err.message);
                    console.log("MapServer启动失败,服务降级启动");
                } else {
                    console.log("启动成功", data);
                }
                console.log("\n启动流程结束");
            })
        })
    })
})