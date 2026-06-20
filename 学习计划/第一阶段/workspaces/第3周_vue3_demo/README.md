# Vue3 页面 Demo — 测试任务看板

> 对应输出物模板：3.1
> 知识点：Vue3 组件、模板语法、响应式、Composition API

## 练习目标

基于 MRCT / ICAOS_TOOL / 测试任务软件 的页面场景，完成 1 个完整页面 Demo。

## 启动方式

```bash
cd 第3周_vue3_demo
npm install
npm run dev
```

浏览器打开 `http://localhost:3000`

## 项目说明

**页面主题**：测试任务看板（Test Task Dashboard）

**场景对应**：MRCT / 测试任务软件中的测试任务管理功能

**涉及组件**：

| 组件 | 文件 | 功能 |
|------|------|------|
| App | `src/App.vue` | 主页面布局，顶栏 + 左列表 + 右详情 |
| TaskList | `src/components/TaskList.vue` | 任务卡片列表，支持类型/目标/关键词筛选 |
| TaskDetail | `src/components/TaskDetail.vue` | 任务详情面板，支持状态变更操作 |
| TaskForm | `src/components/TaskForm.vue` | 新建任务表单，含表单校验 |

**数据管理**：

| 文件 | 功能 |
|------|------|
| `src/data/tasks.js` | 模拟数据 + reactive 简易状态管理 store |

**5 个模拟任务**覆盖：回归测试、性能测试、冒烟测试、接口测试，对应 ZMQ_TOOL、GSDS、ICAOS_TOOL、MRCT、测试任务软件。

## 知识点覆盖

- [x] Vue3 Composition API (`<script setup>`)
- [x] 响应式数据：`ref` / `reactive` / `computed`
- [x] 组件通信：`props` (父→子) / `emit` (子→父)
- [x] 条件渲染：`v-if` / `v-show`
- [x] 列表渲染：`v-for`
- [x] 双向绑定：`v-model`
- [x] 事件处理：`@click` / `@submit.prevent`
- [x] 表单校验（手动校验 + 错误提示）
- [x] 简易状态管理（reactive 单例 store 模式）
- [x] Scoped CSS（组件样式隔离）
- [x] 过渡与交互状态（hover / disabled）

## 功能清单

1. **任务列表**：以卡片形式展示所有任务，显示状态标签和元信息
2. **筛选功能**：按任务类型、目标工具软件下拉筛选，支持关键词搜索
3. **统计条**：实时显示 总数/完成/运行中/失败/待执行 统计
4. **任务详情**：点击卡片在右侧面板展示详情和操作日志
5. **状态变更**：在详情面板可启动/标记完成/标记失败任务
6. **新建任务**：展开表单填写任务信息，提交后加入列表

---

## 0→1 部署流程

从空白环境到生产可用的完整链路。

### 第 1 步：环境准备

```bash
# 1.1 安装 Node.js（推荐 LTS，≥18.x）
#     方式 A：官网下载 https://nodejs.org
#     方式 B：nvm 管理多版本
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.0/install.sh | bash
source ~/.bashrc
nvm install 20
nvm use 20

# 1.2 验证
node -v    # v20.x.x
npm -v     # 10.x.x
```

### 第 2 步：创建项目（Vite 脚手架）

```bash
# 2.1 交互式创建（推荐新手）
npm create vite@latest my-project -- --template vue
cd my-project

# 2.2 或手动搭建（本 Demo 方式）
mkdir my-project && cd my-project
npm init -y
npm install vue
npm install -D vite @vitejs/plugin-vue
```

### 第 3 步：编写核心文件

按顺序创建以下文件即可跑通最小 Vue3 应用：

```
my-project/
├── index.html              # 入口 HTML，<div id="app"> + <script type="module" src="/src/main.js">
├── vite.config.js          # 引入 @vitejs/plugin-vue，配端口
├── package.json            # scripts: { "dev": "vite", "build": "vite build", "preview": "vite preview" }
└── src/
    ├── main.js             # createApp(App).mount('#app')
    ├── App.vue             # <script setup> + <template> + <style>
    ├── components/          # 子组件 .vue 文件
    └── data/                # 数据 / 状态管理模块
```

**关键约定**：
- 每个 `.vue` 文件 = `<script setup>` + `<template>` + `<style scoped>` 三块
- 组件通信只走 `props` 向下、`emit` 向上，不跨级
- 状态共享用 `reactive()` 单例，不引入 Vuex/Pinia 除非项目需要

### 第 4 步：本地开发

```bash
npm install          # 装依赖（仅首次 / package.json 变更后）
npm run dev          # 启动开发服务器 → http://localhost:3000
```

Vite 特性：
- 原生 ESM，冷启动毫秒级（不打包）
- HMR 热更新，改代码浏览器即时刷新
- 支持直接在 `.vue` 中写 TS / CSS 预处理器

### 第 5 步：构建产物

```bash
npm run build        # 输出到 dist/
```

产物结构：
```
dist/
├── index.html              # 入口（压缩后 ~0.4 KB）
├── assets/
│   ├── index-xxxxx.css     # CSS（压缩 + hash 命名）
│   └── index-xxxxx.js      # JS bundle（tree-shaking 后）
```

构建产物是纯静态文件，不依赖 Node.js 运行时。

### 第 6 步：生产部署

**方案 A — Nginx 静态托管（最常用）**

```bash
# 6A.1 把 dist/ 拷贝到服务器
scp -r dist/* user@server:/var/www/my-app/

# 6A.2 Nginx 配置
server {
    listen 80;
    server_name your-domain.com;
    root /var/www/my-app;
    index index.html;

    # SPA 路由回退：所有路径都返回 index.html
    location / {
        try_files $uri $uri/ /index.html;
    }

    # 静态资源强缓存（文件名带 hash，可永久缓存）
    location /assets/ {
        expires 1y;
        add_header Cache-Control "public, immutable";
    }
}
```

**方案 B — Node.js / Express 托管**

```bash
npm install express
```

```js
// server.js
const express = require('express')
const path = require('path')
const app = express()

app.use(express.static(path.join(__dirname, 'dist')))
app.get('*', (req, res) => {
  res.sendFile(path.join(__dirname, 'dist', 'index.html'))
})
app.listen(8080, () => console.log('http://localhost:8080'))
```

**方案 C — Docker 部署**

```dockerfile
# Dockerfile
FROM nginx:alpine
COPY dist/ /usr/share/nginx/html/
COPY nginx.conf /etc/nginx/conf.d/default.conf
EXPOSE 80
```

```bash
docker build -t my-app .
docker run -d -p 80:80 my-app
```

### 第 7 步：验证与检查清单

| 检查项 | 命令 / 方法 | 预期 |
|--------|------------|------|
| 开发环境可启动 | `npm run dev` | 浏览器打开无报错 |
| 构建无错误 | `npm run build` | dist/ 目录生成 |
| 构建产物可预览 | `npm run preview` | 功能与开发环境一致 |
| 页面加载正常 | 浏览器访问 | 顶栏 + 列表 + 详情三栏布局 |
| 交互功能正常 | 点击卡片 / 筛选 / 新建 | 详情切换、筛选生效、表单提交 |
| 控制台无报错 | F12 → Console | 无红色错误 |
| SPA 路由回退 | 访问 `/nonexistent` | 返回 index.html 而非 404 |

### 常见问题

| 现象 | 原因 | 解决 |
|------|------|------|
| `npm run dev` 报 EACCES | 端口被占用 | 改 `vite.config.js` 中 `server.port` |
| 构建后页面空白 | 路径问题 | Nginx 配 `try_files`；或 `vite.config.js` 设 `base: './'` |
| 组件样式不生效 | 缺 `<style scoped>` | 加上 `scoped` 属性避免样式泄露 |
| `v-model` 不更新 | 绑定了非响应式数据 | 用 `ref()` 或 `reactive()` 包裹 |
| 生产刷新 404 | SPA 路由未回退 | Nginx `try_files $uri /index.html` |
