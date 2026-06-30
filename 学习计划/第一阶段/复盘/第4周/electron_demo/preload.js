// contextBridge：Electron 专为此场景提供的安全工具，用来安全地向前端公开 API，而不会暴露底层的 Node.js 内部结构
// ipcRenderer：渲染进程用来向主进程发送异步消息的模块
const { contextBridge, ipcRenderer } = require('electron');
// 前端通过window.electronAPI.file.open()触发文件打开
// 前端通过window.electronAPI.file.read(filePath)触发文件读取
contextBridge.exposeInMainWorld('electronAPI', {
    file: {
        open: () => ipcRenderer.invoke('file:open'),
        read: (filePath) => ipcRenderer.invoke('file:read', filePath),
    },
});