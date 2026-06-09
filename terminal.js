(function() {
const COMMAND_WS_PORT = 8766;
const MAX_TERMINAL_CHARS = 120000;
const MAX_HISTORY = 30;

let ws = null;
let connected = false;
let running = false;
let commandCatalog = {};
let reconnectTimer = null;

const outputEl = document.getElementById('terminal-output');
const inputEl = document.getElementById('command-input');
const runBtn = document.getElementById('run-btn');
const clearBtn = document.getElementById('clear-btn');
const statusEl = document.getElementById('command-status');
const serverStatusEl = document.getElementById('server-status');
const backendPill = document.getElementById('backend-pill');
const runPill = document.getElementById('run-pill');
const commandListEl = document.getElementById('command-list');
const historyListEl = document.getElementById('history-list');

document.addEventListener('DOMContentLoaded', () => {
  updateClock();
  setInterval(updateClock, 1000);
  bindEvents();
  connectCommandServer();
  appendOutput('[SYS] DroneGuard Command Center ready.\n');
  appendOutput('[SYS] Start server.py, then enter a whitelisted command ID such as 1.\n\n');
});

function bindEvents() {
  runBtn.addEventListener('click', runCommand);
  clearBtn.addEventListener('click', () => {
    outputEl.textContent = '';
    appendOutput('[SYS] Terminal cleared.\n');
  });
  inputEl.addEventListener('keydown', event => {
    if (event.key === 'Enter') runCommand();
  });
}

function updateClock() {
  const clock = document.getElementById('hdr-clock');
  if (clock) clock.textContent = new Date().toTimeString().slice(0, 8) + ' UTC+5:30';
}

function connectCommandServer() {
  if (ws) {
    try { ws.close(); } catch (error) { }
    ws = null;
  }

  const wsHost = window.location.hostname && window.location.protocol !== 'file:'
    ? window.location.hostname
    : 'localhost';

  setConnected(false, 'CONNECTING');

  try {
    ws = new WebSocket(`ws://${wsHost}:${COMMAND_WS_PORT}`);
  } catch (error) {
    appendOutput(`[ERR] WebSocket init failed: ${error.message}\n`);
    scheduleReconnect();
    return;
  }

  ws.onopen = () => {
    connected = true;
    clearTimeout(reconnectTimer);
    setConnected(true, 'ONLINE');
    appendOutput('[SYS] Connected to server.py command backend.\n');
    send({ type: 'list' });
  };

  ws.onmessage = event => {
    let message;
    try {
      message = JSON.parse(event.data);
    } catch (error) {
      appendOutput(`${event.data}\n`);
      return;
    }
    handleServerMessage(message);
  };

  ws.onclose = () => {
    connected = false;
    running = false;
    setConnected(false, 'OFFLINE');
    setRunState(false, 'IDLE');
    appendOutput('[ERR] Command server disconnected. Retrying in 3s...\n');
    scheduleReconnect();
  };

  ws.onerror = () => {
    setConnected(false, 'ERROR');
  };
}

function scheduleReconnect() {
  clearTimeout(reconnectTimer);
  reconnectTimer = setTimeout(connectCommandServer, 3000);
}

function send(payload) {
  if (!ws || ws.readyState !== WebSocket.OPEN) return false;
  ws.send(JSON.stringify(payload));
  return true;
}

function runCommand() {
  const command = inputEl.value.trim();
  if (!command || running || !connected) return;

  appendOutput(`\n> ${command}\n`);
  addHistory(command, 'SENT');
  setRunState(true, `RUNNING ${command}`);
  send({ type: 'run', command });
}

function handleServerMessage(message) {
  if (message.type === 'catalog') {
    commandCatalog = message.commands || {};
    renderCommandCatalog();
    return;
  }

  if (message.type === 'status') {
    appendOutput(`[SYS] ${message.message}\n`);
    if (message.state === 'running') setRunState(true, message.message);
    if (message.state === 'idle') setRunState(false, message.message || 'IDLE');
    return;
  }

  if (message.type === 'output') {
    appendOutput(message.data || '');
    return;
  }

  if (message.type === 'error') {
    appendOutput(`[ERR] ${message.message}\n`);
    setRunState(false, 'ERROR');
    addHistory(inputEl.value.trim(), 'ERROR');
    return;
  }

  if (message.type === 'complete') {
    const label = `EXIT ${message.returncode}`;
    appendOutput(`\n[SYS] Command complete: ${label}\n`);
    setRunState(false, label);
    addHistory(inputEl.value.trim(), label);
  }
}

function appendOutput(text) {
  outputEl.textContent += text;
  if (outputEl.textContent.length > MAX_TERMINAL_CHARS) {
    outputEl.textContent = outputEl.textContent.slice(-MAX_TERMINAL_CHARS);
  }
  outputEl.scrollTop = outputEl.scrollHeight;
}

function renderCommandCatalog() {
  const entries = Object.entries(commandCatalog);
  if (!entries.length) {
    commandListEl.innerHTML = '<div class="command-item"><strong>--</strong><span>No commands advertised.</span></div>';
    return;
  }

  commandListEl.innerHTML = entries
    .map(([id, script]) => `<div class="command-item"><strong>${escapeHtml(id)}</strong><span>${escapeHtml(script)}</span></div>`)
    .join('');
}

function addHistory(command, result) {
  if (!command) return;
  const item = document.createElement('li');
  item.textContent = `${new Date().toTimeString().slice(0, 8)}  ID ${command}  ${result}`;
  historyListEl.prepend(item);

  while (historyListEl.children.length > MAX_HISTORY) {
    historyListEl.removeChild(historyListEl.lastChild);
  }
}

function setConnected(isConnected, text) {
  connected = isConnected;
  serverStatusEl.textContent = text;
  serverStatusEl.className = 'ws-indicator ' + (isConnected ? 'live' : text === 'CONNECTING' ? 'sim' : 'err');
  setPill(backendPill, isConnected ? 'COMMAND SERVER ONLINE' : `COMMAND SERVER ${text}`, isConnected ? 'active' : 'warn');
  runBtn.disabled = !isConnected || running;
  statusEl.textContent = isConnected ? 'READY' : text;
  statusEl.className = 'status-value ' + (isConnected ? 'ok' : 'err');
}

function setRunState(isRunning, text) {
  running = isRunning;
  runBtn.disabled = !connected || running;
  inputEl.disabled = running;
  statusEl.textContent = text;
  statusEl.className = 'status-value ' + (isRunning ? 'warn' : text.startsWith('EXIT 0') || text === 'READY' || text === 'IDLE' ? 'ok' : '');
  setPill(runPill, text, isRunning ? 'warn' : 'active');
}

function setPill(el, text, cls) {
  el.className = 'pill ' + (cls || '');
  const label = el.querySelector('.pill-text');
  if (label) label.textContent = text;
}

function escapeHtml(value) {
  return String(value).replace(/[&<>"']/g, char => ({
    '&': '&amp;',
    '<': '&lt;',
    '>': '&gt;',
    '"': '&quot;',
    "'": '&#39;',
  }[char]));
}
})();
