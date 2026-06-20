<script setup>
import { ref } from 'vue'//把一个普通的数据，变成一个“响应式”的数据
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

<!-- 定义组件要渲染的 HTML 结构 -->
<template>
  <div class="app-shell">
    <header class="topbar">
      <h1>任务看板</h1>
      <span class="subtitle">测试任务软件场景</span>
    </header>

    <div class="main-area">
      <div class="left-panel">
        <!-- 子组件（表单/列表）发生了某些用户操作。
        子组件通过 emit('事件名') 向上“发射”一个信号。
        父组件在标签上用 @事件名="处理函数" 接住这个信号，并更新全局的数据 -->
        <TaskForm @task-created="onTaskCreated" />
        <TaskList @select-task="selectTask" />
      </div>
      <!-- 父组件把被选中的任务数据(selectedTask)通过一个名为 task 的管道，传递给子组件 <TaskDetail>
      当用户在详情面板上点击了“关闭”按钮或右上角的 X 时，子组件 <TaskDetail> 会向外发射一个名为 close 的信号
      父组件听到 close 信号后，就会立刻执行自己的 closeDetail 函数 -->
      <TaskDetail :task="selectedTask" @close="closeDetail" />
    </div>
  </div>
</template>

<style>
  /* 确保页面上的任何一个角落、任何一个元素，都能应用上花括号里的规则 */
  *,
  *::before,
  *::after {
    box-sizing: border-box;
    margin: 0;
    padding: 0;
  }
  body {
    font-family: -apple-system, BlinkMacSystemFront, 'Segoe UI', Roboto, 'Helvetica Neue', Arial, sans-serif;
    background: #f5f5f5;
    color: #333;
  }

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
    font-size: 13px;
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