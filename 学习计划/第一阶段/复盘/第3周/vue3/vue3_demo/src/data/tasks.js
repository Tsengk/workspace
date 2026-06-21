// 把一个多层嵌套的普通 JavaScript 对象，一键变成“深层响应式对象”。 
// 只要这个对象里的任何一个属性发生改变，Vue 都会立刻捕捉到并自动更新
import { reactive } from 'vue'

// 模拟初始数据
const INITIAL_TASKS = [
  {
    id: 1,
    name: 'ZMQ',
    type: '回归测试',
    target: 'ZMQ_TOOL',
    status: 'completed', 
    executor: 'nongshizeng',
    createTime: '2026-06-18 09:30',
    duration: '12m',
    log: '48/48 用例通过',
  },
  {
    id: 2,
    name: 'GSDS',
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
    name: 'ICAOS_TOOL',
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
    name: 'MRCT',
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
    name: '测试任务软件',
    type: '回归测试',
    target: '测试任务软件',
    status: 'completed',
    executor: 'nongshizeng',
    createTime: '2026-06-17 10:00',
    duration: '8m',
    log: '16/16 标定参数校验通过',
  },
]

const TASK_TYPES = ['全部', '回归测试', '性能测试', '接口测试', '冒烟测试']
const TASK_TARGETS = ['全部', 'ZMQ_TOOL', 'GSDS', 'ICAOS_TOOL', 'MRCT', '测试任务软件']

// store
// reactive()包围的对象变成“响应式对象”,里面的数据发生变化对应页面用到的数据会自动更新
export const store = reactive({
    tasks: [...INITIAL_TASKS],
    nextId: INITIAL_TASKS.length + 1,
    filterType: '全部',
    filterTarget: '全部',
    searchText: '',

    // get:访问器，像读取普通数据一样去读取一个函数的计算结果；使用时用store.stats，而不是 store.stats()
    // this指向store对象
    // t传入的参数为this.tasks[0,1,2,.....]
    // 获取筛选后的任务列表
    get filteredTasks() {
        return this.tasks.filter((t) => {
            const matchType = this.filterType === '全部' || t.type === this.filterType
            const matchTarget = this.filterTarget === '全部' || t.target === this.filterTarget
            const matchSearch = !this.searchText || t.name.includes(this.searchText) || t.log.includes(this.searchText)
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
            log: '—'
        })
    },

    // 更新状态
    updateStatus(id, status, log) {
        const task = this.tasks.find((t) => t.id === id)
        if (task) {
            task.status = status
            if (log) task.log = log
        }
    },
})

export { TASK_TYPES, TASK_TARGETS}