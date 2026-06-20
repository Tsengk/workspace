<!--
  任务列表组件
  功能：展示筛选后的任务卡片、支持类型/目标/关键词筛选、点击查看详情
  知识点：v-for、v-if、v-model、computed、事件发射
-->
<script setup>
import { ref, computed } from 'vue'
import { store, TASK_TYPES, TASK_TARGETS } from '../data/tasks.js'

const emit = defineEmits(['select-task'])

// 搜索文本（双向绑定到 store）
const searchText = ref('')

// 状态标签样式映射
const statusMap = {
  pending: { label: '待执行', class: 'status-pending' },
  running: { label: '运行中', class: 'status-running' },
  completed: { label: '已完成', class: 'status-completed' },
  failed: { label: '失败', class: 'status-failed' },
}

// 更新 store 筛选条件
function onFilterChange() {
  store.filterType = filterType.value
  store.filterTarget = filterTarget.value
  store.searchText = searchText.value
}

const filterType = ref('全部')
const filterTarget = ref('全部')

// computed: 根据 UI 本地状态触发重新筛选
const displayTasks = computed(() => {
  // 同步到 store
  store.filterType = filterType.value
  store.filterTarget = filterTarget.value
  store.searchText = searchText.value
  return store.filteredTasks
})
</script>

<template>
  <div class="task-list">
    <!-- 筛选栏 -->
    <div class="filter-bar">
      <label>
        类型：
        <select v-model="filterType">
          <option v-for="t in TASK_TYPES" :key="t" :value="t">{{ t }}</option>
        </select>
      </label>
      <label>
        目标工具：
        <select v-model="filterTarget">
          <option v-for="t in TASK_TARGETS" :key="t" :value="t">{{ t }}</option>
        </select>
      </label>
      <label>
        搜索：
        <input
          v-model="searchText"
          type="text"
          placeholder="任务名 / 日志关键词..."
        />
      </label>
    </div>

    <!-- 统计条 -->
    <div class="stats-bar">
      <span>共 {{ store.stats.total }} 个任务</span>
      <span class="stat-dot stat-ok">{{ store.stats.completed }} 完成</span>
      <span class="stat-dot stat-run">{{ store.stats.running }} 运行中</span>
      <span class="stat-dot stat-fail">{{ store.stats.failed }} 失败</span>
      <span class="stat-dot stat-pend">{{ store.stats.pending }} 待执行</span>
    </div>

    <!-- 任务卡片列表 -->
    <div v-if="displayTasks.length === 0" class="empty">
      没有匹配的任务
    </div>
    <ul class="cards">
      <li
        v-for="task in displayTasks"
        :key="task.id"
        class="card"
        @click="emit('select-task', task)"
      >
        <div class="card-header">
          <span class="card-id">#{{ task.id }}</span>
          <span :class="['badge', statusMap[task.status].class]">
            {{ statusMap[task.status].label }}
          </span>
        </div>
        <div class="card-title">{{ task.name }}</div>
        <div class="card-meta">
          <span>🎯 {{ task.target }}</span>
          <span>📋 {{ task.type }}</span>
          <span>🕐 {{ task.createTime }}</span>
        </div>
      </li>
    </ul>
  </div>
</template>

<style scoped>
.task-list {
  flex: 1;
  min-width: 0;
}

.filter-bar {
  display: flex;
  gap: 16px;
  margin-bottom: 12px;
  flex-wrap: wrap;
}
.filter-bar label {
  display: flex;
  align-items: center;
  gap: 4px;
  font-size: 13px;
  color: #555;
}
.filter-bar select,
.filter-bar input {
  padding: 4px 8px;
  border: 1px solid #d0d5dd;
  border-radius: 6px;
  font-size: 13px;
  background: #fff;
}

.stats-bar {
  display: flex;
  gap: 16px;
  margin-bottom: 16px;
  font-size: 13px;
  color: #666;
}
.stat-dot::before {
  content: '';
  display: inline-block;
  width: 8px;
  height: 8px;
  border-radius: 50%;
  margin-right: 4px;
}
.stat-ok::before {
  background: #22c55e;
}
.stat-run::before {
  background: #3b82f6;
}
.stat-fail::before {
  background: #ef4444;
}
.stat-pend::before {
  background: #a1a1aa;
}

.empty {
  padding: 40px;
  text-align: center;
  color: #999;
}

.cards {
  list-style: none;
  padding: 0;
  display: flex;
  flex-direction: column;
  gap: 8px;
}
.card {
  background: #fff;
  border: 1px solid #e5e7eb;
  border-radius: 8px;
  padding: 12px 16px;
  cursor: pointer;
  transition: box-shadow 0.15s, border-color 0.15s;
}
.card:hover {
  border-color: #6366f1;
  box-shadow: 0 2px 8px rgba(99, 102, 241, 0.12);
}
.card-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 6px;
}
.card-id {
  font-size: 12px;
  color: #999;
}
.card-title {
  font-size: 15px;
  font-weight: 600;
  margin-bottom: 6px;
}
.card-meta {
  display: flex;
  gap: 12px;
  font-size: 12px;
  color: #888;
}

.badge {
  font-size: 11px;
  padding: 2px 8px;
  border-radius: 12px;
  font-weight: 500;
}
.status-pending {
  background: #f4f4f5;
  color: #71717a;
}
.status-running {
  background: #dbeafe;
  color: #2563eb;
}
.status-completed {
  background: #dcfce7;
  color: #16a34a;
}
.status-failed {
  background: #fef2f2;
  color: #dc2626;
}
</style>
