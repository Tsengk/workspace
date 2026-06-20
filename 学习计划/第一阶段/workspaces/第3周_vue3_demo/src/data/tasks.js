/**
 * 模拟测试任务数据 & 简易响应式状态管理
 *
 * 场景：结合 MRCT / 测试任务软件，管理一组自动化测试任务
 * 任务类型包括：回归测试、性能测试、接口测试、冒烟测试
 */

import { reactive } from 'vue'

// 初始模拟数据
const INITIAL_TASKS = [
  {
    id: 1,
    name: 'ZMQ 通信协议回归测试',
    type: '回归测试',
    target: 'ZMQ_TOOL',
    status: 'completed', // pending | running | completed | failed
    executor: 'nongshizeng',
    createTime: '2026-06-18 09:30',
    duration: '12m',
    log: '48/48 用例通过 ✓',
  },
  {
    id: 2,
    name: 'GSDS 地图服务性能测试',
    type: '性能测试',
    target: 'GSDS',
    status: 'running',
    executor: 'nongshizeng',
    createTime: '2026-06-19 14:00',
    duration: '运行中...',
    log: '当前 QPS: 156, 延迟 P99: 23ms',
  },
  {
    id: 3,
    name: 'ICAOS_TOOL 前端组件冒烟',
    type: '冒烟测试',
    target: 'ICAOS_TOOL',
    status: 'failed',
    executor: 'nongshizeng',
    createTime: '2026-06-19 16:20',
    duration: '3m',
    log: 'TabPanel 组件渲染异常: 3 个用例失败',
  },
  {
    id: 4,
    name: 'MRCT 视频推流接口测试',
    type: '接口测试',
    target: 'MRCT',
    status: 'pending',
    executor: 'nongshizeng',
    createTime: '2026-06-20 08:00',
    duration: '—',
    log: '—',
  },
  {
    id: 5,
    name: '测试任务软件 相机标定校验',
    type: '回归测试',
    target: '测试任务软件',
    status: 'completed',
    executor: 'nongshizeng',
    createTime: '2026-06-17 10:00',
    duration: '8m',
    log: '16/16 标定参数校验通过 ✓',
  },
]

const TASK_TYPES = ['全部', '回归测试', '性能测试', '接口测试', '冒烟测试']
const TASK_TARGETS = ['全部', 'ZMQ_TOOL', 'GSDS', 'ICAOS_TOOL', 'MRCT', '测试任务软件']

// 简易 store
export const store = reactive({
  tasks: [...INITIAL_TASKS],
  nextId: INITIAL_TASKS.length + 1,
  filterType: '全部',
  filterTarget: '全部',
  searchText: '',

  // getter: 筛选后的任务列表
  get filteredTasks() {
    return this.tasks.filter((t) => {
      const matchType = this.filterType === '全部' || t.type === this.filterType
      const matchTarget =
        this.filterTarget === '全部' || t.target === this.filterTarget
      const matchSearch =
        !this.searchText ||
        t.name.includes(this.searchText) ||
        t.log.includes(this.searchText)
      return matchType && matchTarget && matchSearch
    })
  },

  // 统计
  get stats() {
    const total = this.tasks.length
    const completed = this.tasks.filter((t) => t.status === 'completed').length
    const failed = this.tasks.filter((t) => t.status === 'failed').length
    const running = this.tasks.filter((t) => t.status === 'running').length
    const pending = this.tasks.filter((t) => t.status === 'pending').length
    return { total, completed, failed, running, pending }
  },

  // 添加任务
  addTask(task) {
    this.tasks.unshift({
      id: this.nextId++,
      ...task,
      createTime: new Date().toLocaleString('zh-CN'),
      duration: '—',
      log: '—',
    })
  },

  // 更新任务状态
  updateStatus(id, status, log) {
    const task = this.tasks.find((t) => t.id === id)
    if (task) {
      task.status = status
      if (log) task.log = log
    }
  },
})

export { TASK_TYPES, TASK_TARGETS }
