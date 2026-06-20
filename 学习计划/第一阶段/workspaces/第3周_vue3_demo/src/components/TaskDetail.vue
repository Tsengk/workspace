<!--
  任务详情面板
  功能：展示单个任务详细信息、执行状态变更操作（运行/完成/失败）
  知识点：props、emit、v-if、watch、transition
-->
<script setup>
import { ref, watch } from 'vue'
import { store } from '../data/tasks.js'

const props = defineProps({
  task: { type: Object, default: null },
})

const emit = defineEmits(['close'])

const statusMap = {
  pending: '待执行',
  running: '运行中',
  completed: '已完成',
  failed: '失败',
}

function setStatus(status) {
  if (!props.task) return
  const logs = {
    running: '任务启动中...',
    completed: `✓ 测试通过 (${new Date().toLocaleTimeString('zh-CN')})`,
    failed: `✗ 检测到异常 (${new Date().toLocaleTimeString('zh-CN')})`,
  }
  store.updateStatus(props.task.id, status, logs[status])
}
</script>

<template>
  <div v-if="task" class="detail-panel">
    <div class="panel-header">
      <h3>任务详情 #{{ task.id }}</h3>
      <button class="btn-close" @click="emit('close')">✕</button>
    </div>

    <div class="detail-body">
      <div class="field">
        <span class="label">任务名称</span>
        <span class="value">{{ task.name }}</span>
      </div>
      <div class="field">
        <span class="label">类型 / 目标</span>
        <span class="value">{{ task.type }} · {{ task.target }}</span>
      </div>
      <div class="field">
        <span class="label">执行人</span>
        <span class="value">{{ task.executor }}</span>
      </div>
      <div class="field">
        <span class="label">创建时间</span>
        <span class="value">{{ task.createTime }}</span>
      </div>
      <div class="field">
        <span class="label">状态</span>
        <span :class="['badge', 'status-' + task.status]">
          {{ statusMap[task.status] }}
        </span>
      </div>
      <div class="field">
        <span class="label">日志</span>
        <pre class="log-box">{{ task.log }}</pre>
      </div>

      <!-- 操作按钮 -->
      <div class="actions">
        <button
          :disabled="task.status === 'running'"
          @click="setStatus('running')"
        >
          ▶ 启动
        </button>
        <button
          :disabled="task.status === 'completed'"
          class="btn-ok"
          @click="setStatus('completed')"
        >
          ✓ 标记完成
        </button>
        <button
          :disabled="task.status === 'failed'"
          class="btn-fail"
          @click="setStatus('failed')"
        >
          ✗ 标记失败
        </button>
      </div>
    </div>
  </div>

  <!-- 空状态 -->
  <div v-else class="detail-empty">
    <p>👈 点击左侧任务查看详情</p>
  </div>
</template>

<style scoped>
.detail-panel {
  width: 340px;
  border-left: 1px solid #e5e7eb;
  background: #fafafa;
  display: flex;
  flex-direction: column;
  overflow-y: auto;
}

.panel-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 16px;
  border-bottom: 1px solid #e5e7eb;
}
.panel-header h3 {
  margin: 0;
  font-size: 16px;
}
.btn-close {
  border: none;
  background: none;
  font-size: 18px;
  cursor: pointer;
  color: #999;
}

.detail-body {
  padding: 16px;
  display: flex;
  flex-direction: column;
  gap: 14px;
}

.field {
  display: flex;
  flex-direction: column;
  gap: 2px;
}
.label {
  font-size: 11px;
  text-transform: uppercase;
  color: #999;
  letter-spacing: 0.5px;
}
.value {
  font-size: 14px;
  color: #333;
}

.log-box {
  margin: 0;
  padding: 8px 10px;
  background: #1e1e2e;
  color: #cdd6f4;
  border-radius: 6px;
  font-size: 12px;
  white-space: pre-wrap;
  max-height: 120px;
  overflow-y: auto;
}

.actions {
  display: flex;
  flex-wrap: wrap;
  gap: 8px;
  margin-top: 4px;
}
.actions button {
  padding: 6px 14px;
  border: 1px solid #d0d5dd;
  border-radius: 6px;
  background: #fff;
  cursor: pointer;
  font-size: 13px;
  transition: background 0.15s;
}
.actions button:hover:not(:disabled) {
  background: #f0f0ff;
}
.actions button:disabled {
  opacity: 0.4;
  cursor: not-allowed;
}
.btn-ok:hover:not(:disabled) {
  border-color: #22c55e;
  color: #16a34a;
}
.btn-fail:hover:not(:disabled) {
  border-color: #ef4444;
  color: #dc2626;
}

.badge {
  font-size: 12px;
  padding: 2px 10px;
  border-radius: 12px;
  font-weight: 500;
  display: inline-block;
  width: fit-content;
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

.detail-empty {
  width: 340px;
  border-left: 1px solid #e5e7eb;
  display: flex;
  align-items: center;
  justify-content: center;
  color: #999;
  font-size: 14px;
}
</style>
