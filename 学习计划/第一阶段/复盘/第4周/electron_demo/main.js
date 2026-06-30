// 引入 Electron 管理生命的 app、创建窗口的 BrowserWindow、原生对话框 dialog 和处理 IPC 的 ipcMain
const { app, BrowserWindow, dialog, ipcMain} = require('electron')
const path = require('path');
const fs = require('fs');

let mainWindow = null;

function createWindow() {
    mainWindow = new BrowserWindow({
        width: 1400,
        height: 900,
        minWidth: 900,
        minHeight: 600,
        webPreferences: {
            preload: path.join(__dirname, 'preload.js'),
            nodeIntegration: false,// 禁止渲染进程访问 Node
            contextIsolation: true,// 启用上下文隔离
        },
        title: 'ICAOS_TOOL',
    });
    mainWindow.loadFile('index.html');
}
// IPC处理,IPC (进程间通信)
// 所有的核心文件操作都通过 ipcMain.handle 放在了主进程中处理

// 打开文件对话框
// Electron 的 Invoke-Handle（双向 IPC） 模式,
// 'file:read'：注册了名为 file:open 的 IPC 频道,当渲染进程通过 ipcRenderer.invoke('file:open') 触发它时，这个函数会开始执行，并且能够将最终的返回值直接传递回前端
// 使用了 async 关键字，因为弹窗等待用户操作是一个典型的异步阻塞事件，必须使用 await

// dialog.showOpenDialog原生对话框
// mainWindow：作为第一个参数传入，意思是把这个对话框绑定到主窗口
// title：对话框的标题
// filters: 文件过滤器
// properties: ['openFile']：规定了对话框的行为,openFile 表示只能选择文件，不能选择文件夹
ipcMain.handle('file:open', async () => {
    const result = await dialog.showOpenDialog(mainWindow,{
        title: '选择日志文件',
        filters: [
            { name: '日志/文本文件', extensions: ['log', 'txt', 'csv', 'json', 'yaml']},
            { name: '所有文件', extensions: ['*']},
        ],
        properties: ['openFile'],
    });
    // 用户取消则返回 null
    if (result.canceled || result.filePaths.length === 0) {
        return null;
    }
    // 选中则返回文件路径字符串
    // properties 只有 openFile（单选），所以选中的文件路径永远是数组的第一项
    return result.filePaths[0];
});

//读取文件
// 基于 ipcMain.handle 双向通信机制，接收前端传过来的文件路径，处理后将文件的详细信息打包返回
// 'file:read'：注册了名为 file:read 的 IPC 频道，前端通过 ipcRenderer.invoke('file:read', path) 调用
// _event：作为第一个参数，代表 IPC 事件对象（可以用来获取发送请求的 webContents 等）
// filePath：前端传过来的绝对文件路径
ipcMain.handle('file:read', async (_event, filePath) => {
    try {
        const content = fs.readFileSync(filePath, 'utf-8');//指定了 'utf-8' 编码去读取文件
        const stat = fs.statSync(filePath);//读取文件的状态/元数据（不读取内容，只读属性）。它能拿到文件大小、创建时间、修改时间、文件权限等系统级信息
        return {
            success: true,// 状态标识：成功
            filePath,// 完整的绝对路径
            fileName: path.basename(filePath),// 文件名
            content,// 文件的文本内容
            size: stat.size,// 文件大小（单位：字节 Byte）
            modifiedAt: stat.mtime.toISOString(),// 最后的修改时间（转换为标准的国际标准化组织时间字符串，如 2026-06-30T15:30:00.000Z）
        };
    } catch (err) {
        return {
            success: false,// 状态标识：失败
            error: err.message,// 具体的错误信息
        };
    }
});

//应用生命周期
// 初始化与窗口创建
app.whenReady().then(() => {
    createWindow();

    app.on('activate', () => {
        if (BrowserWindow.getAllWindows().length === 0) {
            createWindow();
        }
    });
});
// 退出机制
app.on('window-all-closed', () => {
    if (process.platform !== 'darwin') {
        app.quit();
    }
});