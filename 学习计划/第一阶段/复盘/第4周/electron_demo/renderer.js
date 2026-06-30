// 状态,全局变量
let allLines = [];
let currentFilter = '';

// DOM（Document Object Model，文档对象模型）引用
// 文档节点：整个网页的入口（document）
// 元素节点：所有的 HTML 标签（如 <div>, <tr>, <p>）
// 文本节点：标签里面的文字
// 属性节点：标签上的属性（如 class, src, id）
const btnOpen = document.getElementById('btnOpen');
const btnClear = document.getElementById('btnClear');
const searchInput = document.getElementById('searchInput');
const logEmpty = document.getElementById('logEmpty');
const logTable = document.getElementById('logTable');
const logBody = document.getElementById('logBody');
const fileNameEl = document.getElementById('fileName');
const lineCountEl = document.getElementById('lineCount');
const fileSizeEl = document.getElementById('fileSize');
const matchCountEl = document.getElementById('matchCount');
const errorBanner = document.getElementById('errorBanner');
const errorMsg = document.getElementById('errorMsg');
const logStatusDot = document.getElementById('logStatusDot');

// 文件操作
async function openFile() {
    try {
        const filePath = await window.electronAPI.file.open();
        if (!filePath) return;//用户取消

        logStatusDot.className = 'log-status-dot running';
        showError(null);
        searchInput.value = '';
        currentFilter = '';

        const result = await window.electronAPI.file.read(filePath);

        if(!result.success) {
            showError(`读取失败：${result.error}`);
            logStatusDot.className = 'log-status-dot stopped';
            return;
        }

        fileNameEl.textContent = result.fileName;
        fileSizeEl.textContent = formatSize(result.size);
        renderLines(result.content);

        btnClear.disabled = false;//启动清楚按钮
        searchInput.disabled = false;//启动搜索框
        searchInput.focus();
    } catch (err) {
        showError(`操作失败：${err.message}`);
        logStatusDot.className = 'log-status-dot stopped';
    }
}

// 渲染
function renderLines(rawContent) {
    const rawLines = rawContent.split('\n');//换行符为分隔符，rawLines数组的每个元素就是一行数据
    allLines = rawLines.map((line,i) => {parseLine(line,i)});

    lineCountEl.textContent = `${allLines.length}行`;
    renderTable(allLines);
    updateMatchCount(allLines.length, allLines.length)

    logEmpty.style.display = 'none';
    logTable.style.display = 'table';
}

function parseLine(line, index) {
    const lineNum = index + 1;
    let timestamp = '';
    let level = '';
    let content = line;

    return {lineNum, timestamp, level, content};
}

function renderTable(lines) {
    // 使用DocumentFragment批量插入
    // 在内存中创建一个轻量级的、隐形的“虚拟 DOM 容器”（即文档片段）
    const fragment = document.createDocumentFragment();
    for (const line of lines) {
        fragment.appendChild(createRow(line));//新生成的行没有立刻挂载到真实的网页上，而是先塞进了这个内存中的 fragment 里
    }
    logBody.innerHTML = '';//清除之前的数据
    logBody.appendChild(fragment);//把装满了新日志行的 fragment 一次性追加到真实的网页 logBody 中
}
// 创建表格行
function createRow(line) {
    const tr = document.createElement('tr');//在内存中创建一个全新的 HTML <tr>元素
    tr.className = `log-row level-${line.level.toLowerCase()}`;

    tr.innerHTML = [
        `<td class="col-line">${line.level.toLowerCase()}</td>`,
        `<td class="col-time">${escapeHtml(line.timestamp)}</td>`,
        `<td class="col-level level-${line.level.toLowerCase()}">${escapeHtml(line.level)}</td>`,
        `<td class="col-content" title="${escapeHtml(line.content)}">${escapeHtml(line.content)}</td>`,
    ].join('')//每一行连接不换行
    return tr;
}
// 过滤
// forEach()的用法：
// 数组.forEach(function(当前元素, 当前索引, 原数组) {
//     // 你的业务逻辑代码
// });
function applyFilter() {
    currentFilter = searchInput.value.trim().toLowerCase();

    if(!currentFilter) {
        const rows = logBody.querySelectorAll('.log-row');
        rows.forEach(r => r.classList.remove('filtered-out'));//.log-row.filtered-out { display: none; }同时拥有这这两个类名时,元素在页面上彻底隐藏
        updateMatchCount(allLines.length, allLines.length);
        return;
    }

    let visibleCount = 0;
    const rows = logBody.querySelectorAll('.log-row');
    rows.forEach((row,i) => {
        const line = allLines[i];
        // 逐行查找是否存在关键字
        const match =
            line.content.toLowerCase().includes(currentFilter) ||
            line.timestamp.toLowerCase().includes(currentFilter) ||
            line.level.toLowerCase().includes(currentFilter);

        if (match) {
            row.classList.remove('filtered-out');
            visibleCount++;
        } else {
            row.classList.add('filter-out');
        }
    });

    updateMatchCount(visibleCount, allLines.length);
}

function updateMatchCount(visible, total) {
    if (currentFilter) {
        matchCountEl.textContent = `${visible}/${total}`;
    } else {
        matchCountEl.textContent = '';
    }
}

function clearLog() {
    allLines = [];
    currentFilter = '';
    logBody.innerHTML = '';
    searchInput.value = '';
    fileNameEl.textContent = '未打开文件';
    lineCountEl.textContent = '0行';
    fileSizeEl.textContent = '';
    matchCountEl.textContent = '';
    logEmpty.style.display = 'flex';//使用弹性盒模型flex显示该内容
    logTable.style.display = 'none';//隐藏该表格内容
    btnClear.disabled = true;//禁用按钮
    searchInput.disabled = true;//禁用按钮
    showError(null);
}

function showError(msg) {
    if (msg) {
        errorMsg.textContent = '!' + msg;
        errorBanner.style.display = 'flex';
    } else {
        errorBanner.style.display = 'none';
    }
}
//按钮触发
function dismissError() {
    errorBanner.style.display = 'none';
}
// 将字符串中的危险字符（如 <, >, &, ", '）转化为安全的 HTML 实体（字符实体）
function escapeHtml(str) {
    const div = document.createElement('div');
    div.textContent = str;
    return div.innerHTML;
}
//单位转换
function formatSize(bytes) {
    if (bytes < 1024) return bytes + 'B';
    if (bytes < 1024 * 1024) return bytes + 'KB';
    return (bytes/(1024 * 2024)).toFixed(1) + 'MB';
}