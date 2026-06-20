<script setup>
import { reactive, ref } from 'vue'
import { store, TASK_TYPES, TASK_TARGETS } from '../data/tasks.js'

const emit = defineEmits(['task-created'])
const types = TASK_TYPES.filter((t) => t !== '全部')
const targets = TASK_TARGETS.filter((t) => t !== '全部')

const form = reactive({
    name: '',
    type: '回归测试',
    target: 'ZMQ_TOOL',
    executor: 'nongshizeng',
})

const errors = reactive({})
const showForm = ref(false)

function validate() {
    errors.name = form.name.trim() ? '' : '任务名称不能为空'
    return !errors.name
}

function submit() {
    if (!validate()) return

    store.addTask({
        name: form.name.trim(),
        type: form.type,
        target: form.target,
        executor: form.executor || 'nongshizeng',
        status: 'pending'
    })

    // 重置表单
    form.name = ''
    form.type = '回归测试'
    form.target = 'ZMQ_TOOL'
    form.executor = 'nongshizeng'
    Object.keys(errors).forEach((k) => (errors[k] = ''))

    // 发送信号
    emit('task-created')
}
</script>

<template>
  <div class="task-form">
    <button class="btn-toggle" @click="showForm = !showForm">
      {{ showForm ? '收起表单 ' : '+ 新建任务' }}
    </button>

    <form v-if="showForm" @submit.prevent="submit" class="form-body">
      <div class="form-row">
        <label>
          任务名称 <span class="required">*</span>
          <input
            v-model="form.name"
            type="text"
            placeholder="例如：ZMQ 通信协议回归测试"
          />
          <span v-if="errors.name" class="err">{{ errors.name }}</span>
        </label>
      </div>

      <div class="form-row inline">
        <label>
          类型
          <select v-model="form.type">
            <option v-for="t in types" :key="t" :value="t">{{ t }}</option>
          </select>
        </label>
        <label>
          目标工具
          <select v-model="form.target">
            <option v-for="t in targets" :key="t" :value="t">{{ t }}</option>
          </select>
        </label>
      </div>

      <div class="form-row">
        <label>
          执行人
          <input v-model="form.executor" type="text" />
        </label>
      </div>

      <button type="submit" class="btn-submit">创建任务</button>
    </form>
  </div>
</template>

<style scoped>
.task-form {
  margin-bottom: 16px;
}

.btn-toggle {
  padding: 8px 16px;
  border: 1px dashed #a5b4fc;
  border-radius: 8px;
  background: #eef2ff;
  color: #4338ca;
  cursor: pointer;
  font-size: 14px;
  width: 100%;
  transition: background 0.15s;
}
.btn-toggle:hover {
  background: #e0e7ff;
}

.form-body {
  margin-top: 12px;
  padding: 16px;
  background: #fff;
  border: 1px solid #e5e7eb;
  border-radius: 8px;
  display: flex;
  flex-direction: column;
  gap: 12px;
}

.form-row label {
  display: flex;
  flex-direction: column;
  gap: 4px;
  font-size: 13px;
  color: #555;
}
.form-row.inline {
  display: flex;
  gap: 16px;
}
.form-row.inline label {
  flex: 1;
}
.required {
  color: #ef4444;
}

.form-row input,
.form-row select {
  padding: 6px 10px;
  border: 1px solid #d0d5dd;
  border-radius: 6px;
  font-size: 13px;
}

.err {
  color: #ef4444;
  font-size: 12px;
}

.btn-submit {
  padding: 8px 20px;
  border: none;
  border-radius: 6px;
  background: #6366f1;
  color: #fff;
  cursor: pointer;
  font-size: 14px;
  align-self: flex-start;
  transition: background 0.15s;
}
.btn-submit:hover {
  background: #4f46e5;
}
</style>
