const { app, BrowserWindow, dialog, ipcMain } = require('electron');
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
      nodeIntegration: false,
      contextIsolation: true,
    },
    title: 'ICAOS_TOOL — 日志查看器',
  });

  mainWindow.loadFile('index.html');
}

// ── IPC 处理 ──────────────────────────────────────────────

// 打开文件对话框
ipcMain.handle('file:open', async () => {
  const result = await dialog.showOpenDialog(mainWindow, {
    title: '选择日志文件',
    filters: [
      { name: '日志/文本文件', extensions: ['log', 'txt', 'csv', 'json', 'yaml'] },
      { name: '所有文件', extensions: ['*'] },
    ],
    properties: ['openFile'],
  });

  if (result.canceled || result.filePaths.length === 0) {
    return null;
  }
  return result.filePaths[0];
});

// 读取文件内容
ipcMain.handle('file:read', async (_event, filePath) => {
  try {
    const content = fs.readFileSync(filePath, 'utf-8');
    const stat = fs.statSync(filePath);
    return {
      success: true,
      filePath,
      fileName: path.basename(filePath),
      content,
      size: stat.size,
      modifiedAt: stat.mtime.toISOString(),
    };
  } catch (err) {
    return {
      success: false,
      error: err.message,
    };
  }
});

// ── 应用生命周期 ──────────────────────────────────────────

app.whenReady().then(() => {
  createWindow();

  app.on('activate', () => {
    if (BrowserWindow.getAllWindows().length === 0) {
      createWindow();
    }
  });
});

app.on('window-all-closed', () => {
  if (process.platform !== 'darwin') {
    app.quit();
  }
});
