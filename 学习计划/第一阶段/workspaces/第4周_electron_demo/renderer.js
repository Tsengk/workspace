// ── 状态 ────────────────────────────────────────────────
let allLines = [];
let currentFilter = '';

// ── DOM 引用 ─────────────────────────────────────────────
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

// ── 文件操作 ─────────────────────────────────────────────

async function openFile() {
  try {
    const filePath = await window.electronAPI.file.open();
    if (!filePath) return; // 用户取消

    logStatusDot.className = 'log-status-dot running';
    showError(null);
    searchInput.value = '';
    currentFilter = '';

    const result = await window.electronAPI.file.read(filePath);

    if (!result.success) {
      showError(`读取失败：${result.error}`);
      logStatusDot.className = 'log-status-dot stopped';
      return;
    }

    fileNameEl.textContent = result.fileName;
    fileSizeEl.textContent = formatSize(result.size);
    renderLines(result.content);
    logStatusDot.className = 'log-status-dot stopped';

    btnClear.disabled = false;
    searchInput.disabled = false;
    searchInput.focus();
  } catch (err) {
    showError(`操作失败：${err.message}`);
    logStatusDot.className = 'log-status-dot stopped';
  }
}

// ── 日志解析 ─────────────────────────────────────────────

/**
 * 智能解析每行日志，尝试提取：时间戳、级别、内容
 *
 * 支持的常见日志格式：
 *   [2024-01-15 10:30:45] [ERROR] something happened
 *   2024-01-15T10:30:45.123Z INFO module - message
 *   [WARN] connection timeout
 *   DEBUG: processing frame 123
 *   纯文本行（无时间戳/级别）
 */
function parseLine(line, index) {
  const lineNum = index + 1;
  let timestamp = '';
  let level = '';
  let content = line;

  // 模式1: [2024-01-15 10:30:45] [LEVEL] text
  let m = line.match(/^\[(\d{4}-\d{2}-\d{2}\s+\d{2}:\d{2}:\d{2}(?:\.\d+)?)\]\s*\[(\w+)\]\s*(.*)/);
  if (m) {
    [, timestamp, level, content] = m;
    return { lineNum, timestamp, level: level.toUpperCase(), content };
  }

  // 模式2: 2024-01-15T10:30:45.123Z LEVEL module - text
  m = line.match(/^(\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}(?:\.\d+)?Z?)\s+(\w+)\s+(.*)/);
  if (m) {
    [, timestamp, level, content] = m;
    return { lineNum, timestamp: timestamp.replace('T', ' ').replace('Z', ''), level: level.toUpperCase(), content };
  }

  // 模式3: [LEVEL] text
  m = line.match(/^\[(\w+)\]\s*(.*)/);
  if (m && ['ERROR', 'WARN', 'WARNING', 'INFO', 'DEBUG', 'TRACE', 'FATAL'].includes(m[1].toUpperCase())) {
    [, level, content] = m;
    return { lineNum, timestamp: '', level: level.toUpperCase(), content };
  }

  // 模式4: LEVEL: text
  m = line.match(/^(\w{4,7}):\s+(.*)/);
  if (m && ['ERROR', 'WARN', 'WARNING', 'INFO', 'DEBUG', 'TRACE', 'FATAL'].includes(m[1].toUpperCase())) {
    [, level, content] = m;
    return { lineNum, timestamp: '', level: level.toUpperCase(), content };
  }

  // 模式5: 纯文本（无级别）
  return { lineNum, timestamp: '', level: '', content };
}

// ── 渲染 ─────────────────────────────────────────────────

function renderLines(rawContent) {
  const rawLines = rawContent.split('\n');
  allLines = rawLines.map((line, i) => parseLine(line, i));

  lineCountEl.textContent = `${allLines.length} 行`;
  renderTable(allLines);
  updateMatchCount(allLines.length, allLines.length);

  logEmpty.style.display = 'none';
  logTable.style.display = 'table';
}

function renderTable(lines) {
  // 使用 DocumentFragment 批量插入，优化性能
  const fragment = document.createDocumentFragment();
  for (const line of lines) {
    fragment.appendChild(createRow(line));
  }
  logBody.innerHTML = '';
  logBody.appendChild(fragment);
}

function createRow(line) {
  const tr = document.createElement('tr');
  tr.className = `log-row level-${line.level.toLowerCase()}`;

  tr.innerHTML = [
    `<td class="col-line">${line.lineNum}</td>`,
    `<td class="col-time">${escapeHtml(line.timestamp)}</td>`,
    `<td class="col-level level-${line.level.toLowerCase()}">${escapeHtml(line.level)}</td>`,
    `<td class="col-content" title="${escapeHtml(line.content)}">${escapeHtml(line.content)}</td>`,
  ].join('');

  return tr;
}

// ── 过滤 ─────────────────────────────────────────────────

function applyFilter() {
  currentFilter = searchInput.value.trim().toLowerCase();

  if (!currentFilter) {
    // 无过滤：显示全部
    const rows = logBody.querySelectorAll('.log-row');
    rows.forEach(r => r.classList.remove('filtered-out'));
    updateMatchCount(allLines.length, allLines.length);
    return;
  }

  let visibleCount = 0;
  const rows = logBody.querySelectorAll('.log-row');
  rows.forEach((row, i) => {
    const line = allLines[i];
    const match =
      line.content.toLowerCase().includes(currentFilter) ||
      line.timestamp.toLowerCase().includes(currentFilter) ||
      line.level.toLowerCase().includes(currentFilter);

    if (match) {
      row.classList.remove('filtered-out');
      visibleCount++;
    } else {
      row.classList.add('filtered-out');
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
  lineCountEl.textContent = '0 行';
  fileSizeEl.textContent = '';
  matchCountEl.textContent = '';
  logEmpty.style.display = 'flex';
  logTable.style.display = 'none';
  btnClear.disabled = true;
  searchInput.disabled = true;
  showError(null);
}

// ── 错误横幅 ─────────────────────────────────────────────

function showError(msg) {
  if (msg) {
    errorMsg.textContent = '⚠️ ' + msg;
    errorBanner.style.display = 'flex';
  } else {
    errorBanner.style.display = 'none';
  }
}

function dismissError() {
  errorBanner.style.display = 'none';
}

// ── 辅助 ─────────────────────────────────────────────────

function escapeHtml(str) {
  const div = document.createElement('div');
  div.textContent = str;
  return div.innerHTML;
}

function formatSize(bytes) {
  if (bytes < 1024) return bytes + ' B';
  if (bytes < 1024 * 1024) return (bytes / 1024).toFixed(1) + ' KB';
  return (bytes / (1024 * 1024)).toFixed(1) + ' MB';
}
