# 日志查看器 — Electron 最小 Demo

> 对应输出物模板：4.1
> 知识点：主进程、渲染进程、IPC 通信、文件系统访问

## 练习目标

基于 ICAOS_TOOL 桌面工具场景，完成可启动的 Electron 最小 Demo——一个暗色主题日志查看器。

---

## 从 0 到 1 搭建步骤

### 第〇步：环境确认

```bash
node -v   # >= 16.20.2
npm -v
```

### 第一步：初始化项目

```bash
mkdir electron-log-viewer
cd electron-log-viewer
npm init -y
```

编辑 `package.json`，将 `"main": "index.js"` 改为 `"main": "main.js"`，添加启动脚本：

```json
{
  "name": "electron-log-viewer",
  "version": "1.0.0",
  "main": "main.js",
  "scripts": {
    "start": "electron ."
  }
}
```

安装 Electron：

```bash
npm install --save-dev electron@^27.1.3
```

### 第二步：主进程 — `main.js`

主进程是应用的入口，负责创建窗口和注册 IPC。

```javascript
const { app, BrowserWindow, dialog, ipcMain } = require('electron');
const path = require('path');
const fs = require('fs');

let mainWindow = null;

function createWindow() {
  mainWindow = new BrowserWindow({
    width: 1400,
    height: 900,
    webPreferences: {
      preload: path.join(__dirname, 'preload.js'),  // 安全桥
      nodeIntegration: false,   // 禁止渲染进程访问 Node
      contextIsolation: true,   // 启用上下文隔离
    },
  });
  mainWindow.loadFile('index.html');
}

// 打开文件对话框
ipcMain.handle('file:open', async () => {
  const result = await dialog.showOpenDialog(mainWindow, {
    filters: [{ name: '日志文件', extensions: ['log', 'txt'] }],
    properties: ['openFile'],
  });
  return result.canceled ? null : result.filePaths[0];
});

// 读取文件
ipcMain.handle('file:read', async (_event, filePath) => {
  const content = fs.readFileSync(filePath, 'utf-8');
  return { fileName: path.basename(filePath), content };
});

app.whenReady().then(createWindow);
app.on('window-all-closed', () => { if (process.platform !== 'darwin') app.quit(); });
```

**要点：**
- `ipcMain.handle('file:open', ...)` — 主进程注册 IPC 处理器，响应渲染进程的 `invoke` 调用
- `dialog.showOpenDialog` — 系统原生文件对话框
- `nodeIntegration: false` + `contextIsolation: true` — 安全标配

### 第三步：预加载脚本 — `preload.js`

安全的中间层，用 `contextBridge` 把主进程能力暴露给渲染进程。

```javascript
const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('electronAPI', {
  file: {
    open: () => ipcRenderer.invoke('file:open'),
    read: (filePath) => ipcRenderer.invoke('file:read', filePath),
  },
});
```

**要点：**
- 渲染进程不能直接 `require('fs')`，只能调用 `window.electronAPI.file.open()`
- `invoke` 是异步的，返回 Promise

### 第四步：渲染界面 — `index.html`

纯 HTML，不依赖任何构建工具。

```html
<!DOCTYPE html>
<html lang="zh-CN">
<head><meta charset="UTF-8"/><title>日志查看器</title>
  <link rel="stylesheet" href="styles.css"/></head>
<body>
  <div class="main-layout">
    <!-- 顶部栏 -->
    <header class="app-header">
      <div class="app-brand">
        <span class="brand-icon">◈</span>
        <span class="brand-name">ICAOS_TOOL</span>
      </div>
    </header>

    <div class="main-body">
      <nav class="sidebar">
        <button class="nav-btn active">
          <span class="nav-icon">▤</span>
          <span class="nav-label">日志</span>
        </button>
      </nav>

      <main class="content-area">
        <div class="log-panel">
          <div class="log-shell">
            <!-- 标题栏 -->
            <div class="log-header">
              <div class="log-title"><span class="log-status-dot stopped"></span><h2>日志查看器</h2></div>
              <button id="btnOpen" class="log-btn log-btn-open" onclick="openFile()">📂 打开文件</button>
            </div>

            <!-- 搜索栏 -->
            <div class="log-controls">
              <span id="fileName">未打开文件</span>
              <span id="lineCount">0 行</span>
              <input id="searchInput" type="text" placeholder="关键字过滤..." oninput="applyFilter()" disabled/>
              <span id="matchCount"></span>
            </div>

            <!-- 表格 -->
            <div class="log-table-wrap">
              <div id="logEmpty" class="log-empty">点击「📂 打开文件」选择日志文件</div>
              <table id="logTable" style="display:none">
                <thead><tr><th>#</th><th>时间戳</th><th>级别</th><th>内容</th></tr></thead>
                <tbody id="logBody"></tbody>
              </table>
            </div>
          </div>
        </div>
      </main>
    </div>
  </div>
  <script src="renderer.js"></script>
</body>
</html>
```

**布局结构：**
```
main-layout (100vw × 100vh, flex column)
├── app-header   (46px, 固定)
└── main-body    (flex: 1, 填满剩余)
    ├── sidebar  (64px, 固定宽)
    └── content-area (flex: 1)
        └── log-panel → log-shell (带圆角+渐变的面板)
            ├── log-header   (标题)
            ├── log-controls (搜索+信息)
            └── log-table-wrap (表格区, overflow-y: auto)
```

### 第五步：样式 — `styles.css`

完整复刻 ICAOS_TOOL 暗色主题。核心设计值：

| 要素 | 值 |
|------|-----|
| 页面底色 | `#0d0e12` |
| 面板底色 | `#0d1117` |
| 顶栏底色 | `#0a0b0f` |
| 品牌蓝 | `#4a90d9` |
| 边框色 | `#21262d` |
| 面板圆角 | `18px` |

完整 CSS 见本目录 `styles.css`（300+ 行），覆盖：
- 根布局、顶栏、侧栏、内容区
- log-shell 面板（18px 圆角 + 渐变 + 阴影）
- 表格（sticky 表头、暗色滚动条、级别着色）
- 搜索栏（focus 高亮、过滤隐藏 `.filtered-out { display: none }`）

### 第六步：渲染逻辑 — `renderer.js`

三块核心逻辑：

**6.1 打开文件（IPC 调用）**

```javascript
async function openFile() {
  const filePath = await window.electronAPI.file.open();  // → 主进程 dialog
  if (!filePath) return;
  const result = await window.electronAPI.file.read(filePath);  // → 主进程 fs
  renderLines(result.content);
}
```

**6.2 日志解析**

用正则匹配四种常见日志格式：

```javascript
function parseLine(line, index) {
  // [2024-01-15 10:30:45] [ERROR] text
  let m = line.match(/^\[(\d{4}-\d{2}-\d{2}\s+\d{2}:\d{2}:\d{2})\]\[(\w+)\]\s*(.*)/);
  if (m) return { lineNum: index+1, timestamp: m[1], level: m[2], content: m[3] };

  // 2024-01-15T10:30:45Z INFO text
  m = line.match(/^(\d{4}-\d{2}-\d{2}T[\d:.]+Z?)\s+(\w+)\s+(.*)/);
  if (m) return { lineNum: index+1, timestamp: m[1], level: m[2], content: m[3] };

  // [ERROR] text
  m = line.match(/^\[(\w+)\]\s*(.*)/);
  if (m && isLevel(m[1])) return { lineNum: index+1, timestamp: '', level: m[1], content: m[2] };

  // ERROR: text
  m = line.match(/^(\w+):\s+(.*)/);
  if (m && isLevel(m[1])) return { lineNum: index+1, timestamp: '', level: m[1], content: m[2] };

  // 纯文本行
  return { lineNum: index+1, timestamp: '', level: '', content: line };
}
```

**6.3 实时过滤**

```javascript
function applyFilter() {
  const keyword = searchInput.value.trim().toLowerCase();
  const rows = document.querySelectorAll('.log-row');
  rows.forEach((row, i) => {
    const line = allLines[i];
    const match = line.content.toLowerCase().includes(keyword);
    row.classList.toggle('filtered-out', !match);
  });
}
```

### 第七步：启动

```bash
npm install   # 首次需要下载 Electron 二进制
npm start     # 启动应用
```

窗口出现后，点击 **📂 打开文件**，选择任意 `.log` 或 `.txt` 文件即可查看。

---

## 界面风格

完全对齐 ICAOS_TOOL 设计系统：
- **暗色主题**：`#0d0e12` 底色 + `#0d1117` 面板
- **品牌色**：`#4a90d9` 蓝色强调
- **面板**：18px 圆角 + 渐变背景 + 阴影
- **表格**：深色条纹、sticky 表头、等宽字体列

## 项目结构

```
├── main.js          # 主进程：窗口创建、文件对话框、IPC 处理
├── preload.js       # contextBridge：安全暴露 file:open / file:read
├── index.html       # 渲染界面
├── renderer.js      # 渲染逻辑：日志解析、渲染、搜索过滤
├── styles.css       # ICAOS_TOOL 暗色主题样式
└── package.json     # electron ^27.1.3
```

## 功能说明

| 界面功能 | 交互方式 | 对应工具软件功能 |
|----------|----------|----------------|
| 打开文件 | 点击「📂 打开文件」按钮 → 系统对话框选择 .log/.txt 文件 | ICAOS_TOOL 日志浏览 |
| 日志展示 | 表格展示：行号、时间戳、级别、内容四列 | FaultPanel 故障表格 |
| 级别着色 | ERROR 红色/WARN 橙色/INFO 绿色/DEBUG 灰色 | FaultPanel 故障等级 |
| 关键字过滤 | 搜索框输入关键字，实时筛选匹配行 | ICAOS_TOOL 搜索过滤 |
| 行数统计 | 工具栏显示总行数 / 过滤匹配数 | FaultPanel 统计栏 |
| 文件信息 | 显示文件名、文件大小 | ICAOS_TOOL 文件信息 |

## 关键知识回顾

| 概念 | 文件 | 核心 API |
|------|------|----------|
| 主进程 | `main.js` | `BrowserWindow`, `ipcMain.handle`, `dialog`, `fs` |
| 预加载 | `preload.js` | `contextBridge.exposeInMainWorld` |
| 渲染进程 | `renderer.js` | `window.electronAPI.*`（通过 preload 注入） |
| IPC 通信 | main ↔ renderer | `ipcRenderer.invoke` → `ipcMain.handle` |
| 安全隔离 | — | `contextIsolation: true` + `nodeIntegration: false` |

## 运行截图

> 粘贴截图路径
