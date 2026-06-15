#!/usr/bin/env node
/**
 * 结合 ZMQ_TOOL 前端 useWebSocket.ts 的真实代码场景分析异步逻辑
 * 源码: /home/idriver/TOOLS/zmq_tool/frontend/src/composables/useWebSocket.ts
 */

"use strict";

// ==== 以下是从 useWebSocket.ts 精简的核心重连逻辑 ====

// 原版代码:
// let _reconnectAttempts = 0;
// const BASE_RECONNECT_DELAY = 1000;  // 1 秒
//
// _ws.onclose = () => {
//   if (_manualDisconnect) return;     // 手动断开不重连
//   const delay = Math.min(BASE_RECONNECT_DELAY * Math.pow(2, _reconnectAttempts), 30000);
//   _reconnectAttempts++;
//   setTimeout(connect, delay);
// };

// ==== 回答以下问题 ====

console.log("===== WebSocket 重连逻辑分析 =====");
console.log("源码位置: useWebSocket.ts\n");

console.log("问题 1: new WebSocket(url) 是同步还是异步？");
console.log("  → 同步返回 WebSocket 对象");
console.log("  → TCP 握手和 HTTP Upgrade 是异步的");
console.log("  → 所以连接成功后 onopen 才触发\n");

console.log("问题 2: _manualDisconnect 标记有什么作用？");
console.log("  → 区分「用户点断开」和「网络断开」");
console.log("  → 用户主动断开时不触发自动重连");
console.log("  → disconnect() 先设标记=true，再调 close()，onclose 检查标记后跳过重连\n");

console.log("问题 3: 指数退避公式 min(1000 × 2^N, 30000) 的含义？");
for (let n = 0; n <= 5; n++) {
    const delay = Math.min(1000 * Math.pow(2, n), 30000);
    console.log(`  N=${n} → ${delay}ms`);
}
console.log("  → 第 5 次后卡在 30s 上限，防止无限增长\n");

console.log("问题 4: 为什么用 setTimeout 而不是直接调 connect()？");
console.log("  → 给服务器恢复留时间");
console.log("  → 避免重连风暴");
console.log("  → 指数退避减少服务器压力\n");

console.log("问题 5: onmessage 的 handlers.forEach 有什么隐患？");
console.log("  → 一个 handler 抛异常会导致后续 handler 跳过");
console.log("  → 改进: handlers.forEach(h => { try { h(msg) } catch {} })");
