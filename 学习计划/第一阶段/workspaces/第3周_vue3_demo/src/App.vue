<!--
  测试任务看板 — 主页面
  场景：结合 MRCT / 测试任务软件，管理自动化测试任务
  布局：左侧任务列表 + 右侧详情面板

  知识点覆盖：
  - Composition API (<script setup>)
  - 响应式数据 (ref / reactive / computed)
  - 组件通信 (props / emit)
  - 条件渲染 (v-if / v-show)
  - 列表渲染 (v-for)
  - 双向绑定 (v-model)
  - 事件处理 (@click / @submit)
  - 表单校验
  - 简易状态管理（reactive store）
-->
<script setup>
import { ref } from 'vue'
import TaskList from './components/TaskList.vue'
import TaskDetail from './components/TaskDetail.vue'
import TaskForm from './components/TaskForm.vue'

const selectedTask = ref(null)

function selectTask(task) {
  selectedTask.value = task
}

function closeDetail() {
  selectedTask.value = null
}

function onTaskCreated() {
  // 新建任务后自动选中
}
</script>

<template>
  <div class="app-shell">
    <!-- 顶栏 -->
    <header class="topbar">
      <h1>🧪 测试任务看板</h1>
      <span class="subtitle">结合 MRCT / 测试任务软件场景 · Vue3 Demo</span>
    </header>

    <!-- 主体 -->
    <div class="main-area">
      <div class="left-panel">
        <TaskForm @task-created="onTaskCreated" />
        <TaskList @select-task="selectTask" />
      </div>

      <TaskDetail :task="selectedTask" @close="closeDetail" />
    </div>
  </div>
</template>

<style>
/* === 全局重置 === */
*,
*::before,
*::after {
  box-sizing: border-box;
  margin: 0;
  padding: 0;
}
body {
  font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto,
    'Helvetica Neue', Arial, sans-serif;
  background: #f5f5f5;
  color: #333;
}

/* === 页面布局 === */
.app-shell {
  display: flex;
  flex-direction: column;
  height: 100vh;
}

.topbar {
  padding: 14px 24px;
  background: linear-gradient(135deg, #6366f1 0%, #8b5cf6 100%);
  color: #fff;
  display: flex;
  align-items: baseline;
  gap: 16px;
  flex-shrink: 0;
}
.topbar h1 {
  font-size: 20px;
  font-weight: 700;
}
.subtitle {
  font-size: 13px;
  opacity: 0.8;
}

.main-area {
  display: flex;
  flex: 1;
  overflow: hidden;
}

.left-panel {
  flex: 1;
  padding: 16px 20px;
  overflow-y: auto;
}
</style>
