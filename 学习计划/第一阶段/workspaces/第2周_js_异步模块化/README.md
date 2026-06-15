# JS 异步与模块化练习

> 任务: 结合 ZMQ_TOOL 实际代码场景，完成 JavaScript 异步与模块化练习

| 文件 | 内容 | 对应真实代码 |
|------|------|-------------|
| `1回调读取配置.js` | 回调嵌套模拟后端启动 | backend/main.py 启动流程 |
| `2Promise启动流程.js` | .then/.catch/.finally 链式调用 | 同上 |
| `3async启动流程.js` | async/await + Promise.all 并行优化 | 同上 |
| `4WebSocket重连分析.js` | 指数退避/手动断开/异常隔离 | frontend/src/composables/useWebSocket.ts |

运行: `node 1回调读取配置.js`
