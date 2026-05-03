/* ══════════════════════════════════════════════════════════════
   DroneGuard v4 — MAVLink Telemetry Dashboard
   Pure JavaScript — WebSocket client, IMU/Attitude rendering,
   GPS map, vehicle control, RC channels, simulation fallback
   ══════════════════════════════════════════════════════════════ */

// ── STATE ─────────────────────────────────────────────────────
let ws = null, wsConnected = false, droneConnected = false, droneArmed = false;
let simMode = false, flightTime = 0, flightInterval = null, wsReconnectTimer = null;
let streamHz = 20;

// Telemetry values (raw from server)
let roll = 0, pitch = 0, yaw = 0;
let gx = 0, gy = 0, gz = 0;
let ax = 0, ay = 0, az = 9.8;
let lat = null, lon = null, alt = 0, ralt = 0;
let spd = 0, aspd = 0, vspd_live = 0;
let batt = 0, battI = 0, bpct = -1, ekfOk = false;
let gpsFix = 0, sats = 0, hdop = 99;

// RC channels from transmitter (1-8)
const rcChannels = { 1: 1500, 2: 1500, 3: 1500, 4: 1500, 5: 1500, 6: 1500, 7: 1500, 8: 1500 };
let rcRssi = 0;

// IMU graph buffers
const imuBufs = { gx: [], gy: [], gz: [], ax: [], ay: [], az: [] };
const IMU_LEN = 200;
Object.keys(imuBufs).forEach(k => { imuBufs[k] = Array(IMU_LEN).fill(0); });

// Smoothed display values (lerped every rAF frame)
let d_roll = 0, d_pitch = 0, d_yaw = 0;
let d_spd = 0, d_vspd = 0, d_alt = 0, d_ralt = 0;
let s_gx = 0, s_gy = 0, s_gz = 0, s_ax = 0, s_ay = 0, s_az = 9.8;

// Smooth display buffers for 60fps graphs
const smoothBufs = { gx: [], gy: [], gz: [], ax: [], ay: [], az: [] };
const SMOOTH_LEN = 300;
Object.keys(smoothBufs).forEach(k => { smoothBufs[k] = Array(SMOOTH_LEN).fill(0); });

const LERP = 0.18;


// ══════════════════════════════════════════════════════════════
// INIT
// ══════════════════════════════════════════════════════════════

document.addEventListener('DOMContentLoaded', () => {
  initRcBars();
  initGpsMap();
  setInterval(updateClock, 1000);
  updateClock();
  connectBackend();
  setInterval(() => { if (simMode) runSimTick(); }, 100);
  startRenderLoop();
});


// ── RC BARS INIT ──────────────────────────────────────────────
function initRcBars() {
  const container = document.getElementById('rc-bars');
  let html = '';
  for (let i = 1; i <= 8; i++) {
    html += `<div class="reading-row">
      <div class="reading-label">CH${i}</div>
      <div class="reading-bar-wrap">
        <div class="reading-bar" id="rc${i}" style="width:50%;background:var(--accent)"></div>
      </div>
      <div class="reading-val" id="rc-v${i}" style="color:var(--accent)">1500</div>
    </div>`;
  }
  container.innerHTML = html;
}


// ── CLOCK ─────────────────────────────────────────────────────
function updateClock() {
  document.getElementById('hdr-clock').textContent =
    new Date().toTimeString().slice(0, 8) + ' UTC+5:30';
}


// ══════════════════════════════════════════════════════════════
// WEBSOCKET CONNECTION
// ══════════════════════════════════════════════════════════════

function mavLog(msg, cls = '') {
  const logEl = document.getElementById('mavlog');
  const p = document.createElement('p');
  p.className = cls;
  p.innerHTML = `<span class="log-ts">[${new Date().toTimeString().slice(0, 8)}]</span> ${msg}`;
  logEl.appendChild(p);
  if (logEl.children.length > 150) logEl.removeChild(logEl.firstChild);
  logEl.scrollTop = logEl.scrollHeight;
}

function wsSend(data) {
  if (!ws || ws.readyState !== WebSocket.OPEN) return;
  ws.send(typeof data === 'string' ? data : JSON.stringify(data));
}

function setWsIndicator(text, cls) {
  const el = document.getElementById('ws-indicator');
  el.textContent = text;
  el.className = 'ws-indicator ' + (cls || '');
}

function setPill(id, text, cls) {
  const el = document.getElementById(id);
  el.className = 'pill ' + (cls || '');
  const txt = el.querySelector('.pill-text');
  if (txt) txt.textContent = text;
}

function connectBackend() {
  if (ws) { try { ws.close(); } catch (e) { } ws = null; }
  setWsIndicator('CONNECTING...', 'sim');
  setPill('pill-ws', 'CONNECTING...', 'warn');

  const wsHost = window.location.hostname && window.location.protocol !== 'file:'
    ? window.location.hostname
    : 'localhost';
  try { ws = new WebSocket(`ws://${wsHost}:8765`); }
  catch (e) { mavLog('WS init failed: ' + e.message, 'err'); enterSimMode(); return; }

  ws.onopen = () => {
    wsConnected = true;
    simMode = false;
    clearTimeout(wsReconnectTimer);
    setWsIndicator('BACKEND LIVE', 'live');
    setPill('pill-ws', 'BACKEND CONNECTED', 'active');
    mavLog('WebSocket connected to server.py.', 'ok');
    mavLog('Waiting for live MAVLink telemetry from backend.', 'ok');
    document.getElementById('conn-btn').disabled = false;
    const b = document.getElementById('gps-sim-banner');
    if (b) b.style.display = 'none';
  };

  ws.onmessage = (evt) => {
    if (!droneConnected) return;
    let d;
    try { d = JSON.parse(evt.data); } catch (e) { return; }

    // Attitude
    if (typeof d.raw_roll === 'number') roll = d.raw_roll;
    if (typeof d.raw_pitch === 'number') pitch = d.raw_pitch;
    if (typeof d.raw_yaw === 'number') yaw = d.raw_yaw;

    // Accelerometer
    if (typeof d.raw_ax === 'number') ax = d.raw_ax;
    if (typeof d.raw_ay === 'number') ay = d.raw_ay;
    if (typeof d.raw_az === 'number') az = d.raw_az;

    // GPS
    if (d.lat !== null && d.lat !== undefined) lat = d.lat;
    if (d.lon !== null && d.lon !== undefined) lon = d.lon;
    if (typeof d.alt_msl === 'number') alt = d.alt_msl;
    if (typeof d.alt_rel === 'number') ralt = d.alt_rel;
    if (typeof d.groundspeed === 'number') spd = d.groundspeed;
    if (typeof d.airspeed === 'number') aspd = d.airspeed;
    if (typeof d.vspeed === 'number') vspd_live = d.vspeed;
    if (typeof d.heading_deg === 'number') yaw = d.heading_deg;
    if (typeof d.gps_fix === 'number') gpsFix = d.gps_fix;
    if (typeof d.satellites === 'number') sats = d.satellites;
    if (typeof d.hdop === 'number') hdop = d.hdop;
    if (typeof d.connection === 'string') setConnectionMeta(d.connection, d.baud, d.stream_hz);

    // Battery
    if (typeof d.batt_voltage === 'number') batt = d.batt_voltage;
    if (typeof d.batt_current === 'number') battI = d.batt_current;
    if (typeof d.batt_pct === 'number') bpct = d.batt_pct;
    if (typeof d.ekf_ok === 'boolean') ekfOk = d.ekf_ok;

    // Armed status
    if (typeof d.armed === 'boolean') {
      droneArmed = d.armed;
      document.getElementById('arm-status').textContent = droneArmed ? 'ARMED' : 'DISARMED';
      document.getElementById('arm-status').className = 'arm-indicator ' + (droneArmed ? 'armed' : 'disarmed');

      const pillArmed = document.getElementById('pill-armed');
      if (droneArmed) {
        pillArmed.style.display = 'flex';
        pillArmed.className = 'pill danger';
        pillArmed.querySelector('.pill-text').textContent = 'ARMED';
      } else {
        pillArmed.style.display = 'none';
      }
    }

    // Flight mode
    if (typeof d.flight_mode === 'string') syncFlightMode(d.flight_mode);

    // RC Channels
    for (let i = 1; i <= 8; i++) {
      const key = 'rc' + i;
      if (typeof d[key] === 'number' && d[key] > 0) rcChannels[i] = d[key];
    }
    if (typeof d.rc_rssi === 'number') rcRssi = d.rc_rssi;

    // Gyro rates
    if (typeof d.raw_gx === 'number') gx = d.raw_gx;
    else gx = roll * 0.01745;
    if (typeof d.raw_gy === 'number') gy = d.raw_gy;
    else gy = pitch * 0.01745;
    if (typeof d.raw_gz === 'number') gz = d.raw_gz;
    else gz = yaw * 0.01745 * 0.01;

    // Push IMU into buffers
    const _imu = { gx, gy, gz, ax, ay, az };
    Object.keys(_imu).forEach(k => {
      imuBufs[k].push(_imu[k]);
      if (imuBufs[k].length > IMU_LEN) imuBufs[k].shift();
    });

    renderTelemetryUI();
    updateGpsMap();
  };

  ws.onclose = () => {
    wsConnected = false;
    if (!simMode) {
      setWsIndicator('BACKEND OFFLINE', '');
      setPill('pill-ws', 'BACKEND OFFLINE', '');
      mavLog('Backend disconnected — retrying in 3s...', 'err');
      wsReconnectTimer = setTimeout(connectBackend, 3000);
    }
  };

  ws.onerror = () => {
    if (!simMode) {
      mavLog('Cannot reach server.py — switching to SIM mode.', 'err');
      enterSimMode();
    }
  };
}

function enterSimMode() {
  simMode = true;
  wsConnected = false;
  setWsIndicator('SIM MODE', 'sim');
  setPill('pill-ws', 'SIM MODE', 'warn');
  mavLog('Simulation mode active. Start server.py for live drone data.', 'err');
  document.getElementById('conn-btn').disabled = false;
  const b = document.getElementById('gps-sim-banner');
  if (b) b.style.display = 'flex';
}


// ══════════════════════════════════════════════════════════════
// VEHICLE CONTROL
// ══════════════════════════════════════════════════════════════

function toggleConnect() {
  const btn = document.getElementById('conn-btn');
  const st = document.getElementById('conn-status');

  if (!droneConnected) {
    if (wsConnected || simMode) {
      droneConnected = true;
      st.textContent = wsConnected ? '● CONNECTED' : '● CONNECTED (SIM)';
      st.className = 'conn-status connected';
      btn.textContent = 'DISCONNECT';
      ['arm-btn', 'disarm-btn', 'rtl-btn'].forEach(id => document.getElementById(id).disabled = false);
      setPill('pill-drone', wsConnected ? 'DRONE CONNECTED' : 'DRONE (SIM)', 'active');
      mavLog(wsConnected ? `MAVLink stream active @ ${streamHz} Hz.` : '[SIM] Simulated MAVLink heartbeat.', 'ok');
      flightTime = 0;
      if (flightInterval) clearInterval(flightInterval);
      flightInterval = setInterval(() => flightTime++, 1000);
    } else {
      mavLog('Waiting for backend... (start server.py)', 'err');
    }
  } else {
    droneConnected = false;
    droneArmed = false;
    st.textContent = '● DISCONNECTED';
    st.className = 'conn-status disconnected';
    btn.textContent = 'CONNECT';
    ['arm-btn', 'disarm-btn', 'rtl-btn'].forEach(id => document.getElementById(id).disabled = true);
    document.getElementById('arm-status').textContent = 'DISARMED';
    document.getElementById('arm-status').className = 'arm-indicator disarmed';
    document.getElementById('pill-armed').style.display = 'none';
    setPill('pill-drone', 'DRONE DISCONNECTED', '');
    if (flightInterval) { clearInterval(flightInterval); flightInterval = null; }
    mavLog('Disconnected.', 'err');
  }
}

function setConnectionMeta(connection, baud, telemetryHz) {
  const vals = document.querySelectorAll('.conn-meta-val');
  if (vals[0] && connection) vals[0].textContent = connection;
  if (vals[1] && baud) vals[1].textContent = baud;
  if (vals[2]) vals[2].textContent = `${window.location.hostname || 'localhost'}:8765`;
  if (telemetryHz) {
    streamHz = telemetryHz;
    const hzText = `${streamHz} Hz`;
    const hzBadge = document.getElementById('hz-badge');
    const tableHz = document.getElementById('t-hz');
    if (hzBadge && wsConnected) hzBadge.textContent = hzText;
    if (tableHz && wsConnected) tableHz.textContent = hzText;
  }
}

function armDrone() {
  if (!droneConnected) return;
  if (wsConnected) { wsSend('ARM'); mavLog('Sent: "ARM" → arducopter_arm()', 'ok'); }
  else mavLog('[SIM] ARM command.', 'ok');
  droneArmed = true;
  document.getElementById('arm-status').textContent = 'ARMED';
  document.getElementById('arm-status').className = 'arm-indicator armed';
  const p = document.getElementById('pill-armed');
  p.style.display = 'flex';
  p.className = 'pill danger';
  p.querySelector('.pill-text').textContent = 'ARMED';
}

function disarmDrone() {
  if (!droneConnected) return;
  if (wsConnected) { wsSend('DISARM'); mavLog('Sent: "DISARM" → arducopter_disarm()', 'ok'); }
  else mavLog('[SIM] DISARM command.', 'ok');
  droneArmed = false;
  document.getElementById('arm-status').textContent = 'DISARMED';
  document.getElementById('arm-status').className = 'arm-indicator disarmed';
  document.getElementById('pill-armed').style.display = 'none';
}

function sendRTL() {
  if (!droneConnected) return;
  if (wsConnected) { wsSend({ cmd: 'RTL' }); mavLog('Sent: RTL command → backend', 'ok'); }
  else mavLog('[SIM] RTL command.', 'ok');
  syncFlightMode('RTL');
}

function setMode(el) {
  const modeName = el.textContent.trim();
  if (wsConnected && droneConnected) {
    wsSend({ cmd: 'SET_MODE', mode: modeName });
    mavLog('Sent: SET_MODE → ' + modeName + ' → backend', 'ok');
  } else {
    mavLog((simMode ? '[SIM] ' : '') + 'Flight mode selected: ' + modeName, 'ok');
  }
  syncFlightMode(modeName);
}

function syncFlightMode(name) {
  const displayMap = { 'ALT_HOLD': 'ALT HOLD', 'POSHOLD': 'POSHOLD', 'SMART_RTL': 'SMART_RTL' };
  const display = displayMap[name] || name;
  document.querySelectorAll('.mode-chip').forEach(b => b.classList.toggle('active', b.textContent.trim() === display));
  const badge = document.getElementById('active-mode-badge');
  if (badge) badge.textContent = display;
  const qs = document.getElementById('qs-mode');
  if (qs) qs.textContent = display;
}


// ══════════════════════════════════════════════════════════════
// TELEMETRY UI UPDATES
// ══════════════════════════════════════════════════════════════

function renderTelemetryUI() {
  const _set = (id, v) => { const e = document.getElementById(id); if (e) e.textContent = v; };

  // Telemetry table
  _set('t-lat', lat != null ? lat.toFixed(7) + '°' : '--');
  _set('t-lon', lon != null ? lon.toFixed(7) + '°' : '--');
  _set('t-alt', Number.isFinite(alt) ? alt.toFixed(1) + ' m' : '--');
  _set('t-ralt', Number.isFinite(ralt) ? ralt.toFixed(1) + ' m' : '--');
  _set('t-spd', Number.isFinite(spd) ? spd.toFixed(2) + ' m/s' : '--');
  _set('t-aspd', Number.isFinite(aspd) ? aspd.toFixed(2) + ' m/s' : '--');
  _set('t-hdg', Math.round(((yaw % 360) + 360) % 360) + '°');
  _set('t-vspd', typeof vspd_live === 'number' ? vspd_live.toFixed(2) + ' m/s' : '--');
  _set('t-batt', batt > 0 ? batt.toFixed(2) + ' V' : '--');
  _set('t-bpct', bpct >= 0 ? bpct + '%' : '--');
  _set('t-curr', Number.isFinite(battI) ? battI.toFixed(2) + ' A' : '--');
  _set('t-sats', sats || '--');
  _set('t-hdop', hdop < 99 ? hdop.toFixed(2) : '--');
  _set('t-ekf', ekfOk ? '✓ OK' : '✗ FAIL');
  const ekfEl = document.getElementById('t-ekf');
  if (ekfEl) ekfEl.style.color = ekfOk ? 'var(--accent2)' : 'var(--danger)';

  const ft = flightTime;
  _set('t-time', [Math.floor(ft / 3600), Math.floor((ft % 3600) / 60), ft % 60].map(v => String(v).padStart(2, '0')).join(':'));
  _set('t-hz', wsConnected ? `${streamHz} Hz` : '--');

  // GPS panel
  const fixNames = ['NO FIX', 'NO FIX', '2D FIX', '3D FIX', 'DGPS', 'RTK FLOAT', 'RTK FIXED'];
  const fixColors = ['var(--danger)', 'var(--danger)', 'var(--amber)', 'var(--accent2)', 'var(--accent2)', 'var(--accent)', 'var(--accent)'];
  const fixEl = document.getElementById('gps-fix-badge');
  if (fixEl) {
    fixEl.textContent = fixNames[Math.min(gpsFix, 6)] || '--';
    fixEl.style.color = fixColors[Math.min(gpsFix, 6)];
  }

  _set('gps-lat', lat != null ? lat.toFixed(6) : '--');
  _set('gps-lon', lon != null ? lon.toFixed(6) : '--');
  _set('gps-alt', Number.isFinite(alt) ? alt.toFixed(1) : '--');
  _set('gps-sats', sats ? sats : '--');
  _set('gps-hdop', hdop < 99 ? hdop.toFixed(2) : '--');
  _set('gps-spd', Number.isFinite(spd) ? spd.toFixed(1) + ' m/s' : '--');
  _set('gps-vspd', vspd_live.toFixed(2) + ' m/s');
  _set('gps-ralt', Number.isFinite(ralt) ? ralt.toFixed(1) : '--');

  // Quick stats
  _set('qs-alt', Number.isFinite(alt) ? alt.toFixed(1) + ' m' : '0.0 m');
  _set('qs-spd', Number.isFinite(spd) ? spd.toFixed(1) + ' m/s' : '0.0 m/s');
  _set('qs-batt', batt ? batt.toFixed(1) + ' V' : '-- V');
  _set('qs-sats', sats ? sats : '--');
  _set('qs-time', [Math.floor(ft / 3600), Math.floor((ft % 3600) / 60), ft % 60].map(v => String(v).padStart(2, '0')).join(':'));

  // Hz badge
  _set('hz-badge', wsConnected ? `${streamHz} Hz` : '-- Hz');

  updateRcBars();
}

function updateRcBars() {
  for (let i = 1; i <= 8; i++) {
    const val = rcChannels[i] || 1500;
    const pct = Math.max(0, Math.min(100, ((val - 1000) / 1000) * 100));
    const bar = document.getElementById('rc' + i);
    const lbl = document.getElementById('rc-v' + i);
    if (bar) {
      bar.style.width = pct.toFixed(1) + '%';
      if (val < 1050 || val > 1950) bar.style.background = 'var(--danger)';
      else if (val < 1100 || val > 1900) bar.style.background = 'var(--amber)';
      else bar.style.background = 'var(--accent)';
    }
    if (lbl) lbl.textContent = val;
  }
  const rssiEl = document.getElementById('rc-rssi-val');
  if (rssiEl) rssiEl.textContent = rcRssi ? rcRssi : '--';
}


// ══════════════════════════════════════════════════════════════
// GPS MAP (Leaflet)
// ══════════════════════════════════════════════════════════════

let leafletMap = null, droneMarker = null, flightPath = null, simFence = null, pathCoords = [];
const TRIDENT_LAT = 20.3403, TRIDENT_LON = 85.8083;
let simLocationMarker = null;
let mapInitialized = false;
let lastMapMode = 'sim';

function hasValidGps() {
  return Number.isFinite(lat) && Number.isFinite(lon)
    && lat >= -90 && lat <= 90
    && lon >= -180 && lon <= 180
    && (Math.abs(lat) > 0.001 || Math.abs(lon) > 0.001);
}

function initGpsMap() {
  if (mapInitialized) return;
  mapInitialized = true;
  if (typeof L === 'undefined') {
    mavLog('Leaflet map library did not load. Check internet access for the CDN.', 'err');
    return;
  }

  leafletMap = L.map('gps-map', { zoomControl: true, attributionControl: false }).setView([TRIDENT_LAT, TRIDENT_LON], 17);
  L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', { maxZoom: 19 }).addTo(leafletMap);

  // Drone marker
  const droneIcon = L.divIcon({
    html: '<div class="drone-dot"></div>',
    iconSize: [22, 22], iconAnchor: [11, 11], className: ''
  });
  droneMarker = L.marker([TRIDENT_LAT, TRIDENT_LON], { icon: droneIcon }).addTo(leafletMap);
  droneMarker.bindPopup(
    `<b style="color:#ff3355">🔴 DRONE — SIM</b><br>
     Trident Academy of Technology<br>
     Bhubaneswar, Odisha<br>
     <span style="color:#888">Lat: ${TRIDENT_LAT.toFixed(6)} · Lon: ${TRIDENT_LON.toFixed(6)}</span>`
  );

  // Location pin
  const locIcon = L.divIcon({
    html: `<div style="
      background:#00d4ff;border:2px solid #fff;border-radius:4px 4px 0 4px;
      width:14px;height:14px;transform:rotate(45deg);
      box-shadow:0 0 10px #00d4ff88;
    "></div>`,
    iconSize: [14, 14], iconAnchor: [7, 14], className: ''
  });
  simLocationMarker = L.marker([TRIDENT_LAT, TRIDENT_LON], { icon: locIcon, zIndexOffset: -10 }).addTo(leafletMap);
  simLocationMarker.bindTooltip(
    '📍 Trident Academy of Technology, BBSR',
    { permanent: true, direction: 'top', offset: [0, -18], className: 'trident-tooltip' }
  );

  // Geofence circle
  simFence = L.circle([TRIDENT_LAT, TRIDENT_LON], {
    radius: 25, color: '#ff3355', fillColor: '#ff335520', fillOpacity: 0.25,
    weight: 1.5, dashArray: '4 4'
  }).addTo(leafletMap);

  flightPath = L.polyline([], { color: '#00d4ff', weight: 2, opacity: 0.7 }).addTo(leafletMap);
}

function updateGpsMap() {
  if (!mapInitialized) {
    // Initialize on first data
    initGpsMap();
  }
  if (!leafletMap) return;
  if (!hasValidGps()) {
    droneMarker.setPopupContent(
      `<b style="color:#ff3355">DRONE - NO VALID GPS</b><br>
       Fix: ${gpsFix || 0} &nbsp;|&nbsp; Sats: ${sats || 0}<br>
       Waiting for MAVLink GPS_RAW_INT or GLOBAL_POSITION_INT.`
    );
    return;
  }
  const pos = [lat, lon];
  droneMarker.setLatLng(pos);

  const mode = simMode ? 'sim' : 'live';
  if (mode !== lastMapMode) {
    pathCoords = [];
    if (flightPath) flightPath.setLatLngs(pathCoords);
    if (simLocationMarker) {
      if (simMode && !leafletMap.hasLayer(simLocationMarker)) simLocationMarker.addTo(leafletMap);
      if (!simMode && leafletMap.hasLayer(simLocationMarker)) leafletMap.removeLayer(simLocationMarker);
    }
    if (simFence) {
      if (simMode && !leafletMap.hasLayer(simFence)) simFence.addTo(leafletMap);
      if (!simMode && leafletMap.hasLayer(simFence)) leafletMap.removeLayer(simFence);
    }
    lastMapMode = mode;
  }

  const fixNames = ['NO FIX', 'NO FIX', '2D FIX', '3D FIX', 'DGPS', 'RTK FLOAT', 'RTK FIXED'];
  const fixLabel = simMode ? '3D FIX (SIM)' : (fixNames[Math.min(gpsFix || 0, 6)] || `${gpsFix}`);
  const satsLabel = simMode ? `${sats} (SIM)` : `${sats}`;
  droneMarker.setPopupContent(
    `<b style="color:#ff3355">🔴 DRONE ${simMode ? '— SIMULATION' : ''}</b><br>
     ${simMode ? 'Trident Academy of Technology, BBSR<br>' : ''}
     <span style="color:#888">Lat: ${lat.toFixed(6)}</span><br>
     <span style="color:#888">Lon: ${lon.toFixed(6)}</span><br>
     Alt: ${alt.toFixed(1)} m &nbsp;|&nbsp; Fix: ${fixLabel} &nbsp;|&nbsp; Sats: ${satsLabel}`
  );

  pathCoords.push(pos);
  if (pathCoords.length > 500) pathCoords.shift();
  flightPath.setLatLngs(pathCoords);
  if (pathCoords.length === 1 || pathCoords.length % 30 === 0) {
    leafletMap.setView(pos, Math.max(leafletMap.getZoom(), 17));
  }
}


// ══════════════════════════════════════════════════════════════
// SIMULATION MODE
// ══════════════════════════════════════════════════════════════

function runSimTick() {
  if (!simMode || !droneConnected) return;

  roll = Math.max(-45, Math.min(45, roll + (Math.random() - 0.5) * 4));
  pitch = Math.max(-45, Math.min(45, pitch + (Math.random() - 0.5) * 3));
  yaw = (yaw + (Math.random() - 0.5) * 2 + 360) % 360;

  ax = (Math.random() - 0.5) * 0.8 + Math.sin(roll * Math.PI / 180) * 9.8 * 0.1;
  ay = (Math.random() - 0.5) * 0.8 + Math.sin(pitch * Math.PI / 180) * 9.8 * 0.1;
  az = 9.8 + (Math.random() - 0.5) * 0.3;
  gx = roll * 0.01745 + (Math.random() - 0.5) * 0.05;
  gy = pitch * 0.01745 + (Math.random() - 0.5) * 0.05;
  gz = yaw * 0.01745 * 0.01 + (Math.random() - 0.5) * 0.02;

  if (Math.random() < 0.04) ax += (Math.random() - 0.5) * 4;
  if (Math.random() < 0.04) gx += (Math.random() - 0.5) * 1.5;

  // GPS sim — anchored to Trident Academy
  const SIM_LAT = 20.3403, SIM_LON = 85.8083;
  if (lat == null) { lat = SIM_LAT; lon = SIM_LON; }
  const driftR = 0.00018;
  lat = SIM_LAT + Math.sin(Date.now() / 8000) * driftR * 0.7 + (Math.random() - 0.5) * 0.00004;
  lon = SIM_LON + Math.cos(Date.now() / 9500) * driftR * 0.9 + (Math.random() - 0.5) * 0.00004;
  alt = Math.max(0, alt + (Math.random() - 0.3) * 0.5);
  ralt = alt;
  spd = Math.random() * 2;
  vspd_live = (Math.random() - 0.5) * 0.3;
  gpsFix = 3;
  sats = 10 + Math.floor(Math.random() * 3);
  hdop = 0.9 + Math.random() * 0.5;
  if (!batt) batt = 16.8;
  batt = Math.max(10, batt - 0.001);
  bpct = Math.round((batt / 16.8) * 100);
  battI = 1.2 + Math.random() * 0.4;
  ekfOk = true;
  aspd = spd + (Math.random() - 0.5) * 0.3;

  // RC sim
  rcChannels[1] = 1500 + Math.round((Math.random() - 0.5) * 80);
  rcChannels[2] = 1500 + Math.round((Math.random() - 0.5) * 60);
  rcChannels[3] = 1200 + Math.round(Math.random() * 400);
  rcChannels[4] = 1500 + Math.round((Math.random() - 0.5) * 50);
  for (let i = 5; i <= 8; i++) rcChannels[i] = 1500 + Math.round((Math.random() - 0.5) * 20);
  rcRssi = Math.round(180 + Math.random() * 75);

  Object.entries({ gx, gy, gz, ax, ay, az }).forEach(([k, v]) => {
    imuBufs[k].push(v);
    if (imuBufs[k].length > IMU_LEN) imuBufs[k].shift();
  });

  updateGpsMap();
  updateRcBars();
  renderTelemetryUI();
}


// ══════════════════════════════════════════════════════════════
// MATH HELPERS
// ══════════════════════════════════════════════════════════════

function lerp(a, b, t) { return a + (b - a) * t; }

function lerpAngle(a, b, t) {
  const ar = a * Math.PI / 180, br = b * Math.PI / 180;
  const ax2 = Math.cos(ar), ay2 = Math.sin(ar);
  const bx = Math.cos(br), by = Math.sin(br);
  const ix = lerp(ax2, bx, t), iy = lerp(ay2, by, t);
  return Math.atan2(iy, ix) * 180 / Math.PI;
}


// ══════════════════════════════════════════════════════════════
// IMU GRAPH RENDERING
// ══════════════════════════════════════════════════════════════

const _imuCache = {};

function drawImuGraph(canvasId, buf, minV, maxV, lineColor) {
  const canvas = document.getElementById(canvasId);
  if (!canvas) return;
  const W = canvas.offsetWidth || 600;
  const H = 70;
  if (canvas.width !== W) canvas.width = W;
  if (canvas.height !== H) canvas.height = H;

  const ctx = canvas.getContext('2d');
  let cache = _imuCache[canvasId];

  if (!cache || cache.W !== W || cache.H !== H) {
    cache = {
      W, H,
      bg: (() => { const g = ctx.createLinearGradient(0, 0, 0, H); g.addColorStop(0, 'rgba(0,0,0,0.5)'); g.addColorStop(1, 'rgba(0,0,0,0.25)'); return g; })(),
      lineGrad: (() => { const g = ctx.createLinearGradient(0, 0, W, 0); g.addColorStop(0, 'rgba(0,212,255,0.0)'); g.addColorStop(0.25, lineColor + '99'); g.addColorStop(1, lineColor); return g; })(),
      fillGrad: (() => { const g = ctx.createLinearGradient(0, 0, 0, H); g.addColorStop(0, lineColor + '28'); g.addColorStop(1, 'rgba(0,0,0,0)'); return g; })(),
    };
    _imuCache[canvasId] = cache;
  }

  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = cache.bg;
  ctx.fillRect(0, 0, W, H);

  // Grid
  ctx.strokeStyle = 'rgba(255,255,255,0.04)';
  ctx.lineWidth = 1;
  [0.25, 0.5, 0.75].forEach(f => {
    ctx.beginPath(); ctx.moveTo(0, H * f); ctx.lineTo(W, H * f); ctx.stroke();
  });

  // Zero line
  const range = maxV - minV;
  const toY = v => H - ((v - minV) / range) * H;
  const zeroY = toY(0);
  ctx.strokeStyle = 'rgba(255,255,255,0.15)';
  ctx.setLineDash([3, 5]);
  ctx.beginPath(); ctx.moveTo(0, zeroY); ctx.lineTo(W, zeroY); ctx.stroke();
  ctx.setLineDash([]);

  if (buf.length < 2) return;

  const pts = buf.length > W ? buf.slice(-W) : buf;
  const n = pts.length;
  const step = W / Math.max(n - 1, 1);

  // Smooth curve
  ctx.beginPath();
  for (let i = 0; i < n; i++) {
    const x = i * step;
    const y = toY(pts[i]);
    if (i === 0) { ctx.moveTo(x, y); continue; }
    const x0 = (i - 1) * step, y0 = toY(pts[i - 1]);
    const cpx = (x0 + x) * 0.5;
    ctx.bezierCurveTo(cpx, y0, cpx, y, x, y);
  }

  ctx.strokeStyle = cache.lineGrad;
  ctx.lineWidth = 1.8;
  ctx.lineJoin = 'round';
  ctx.stroke();

  // Fill below
  ctx.lineTo((n - 1) * step, H);
  ctx.lineTo(0, H);
  ctx.closePath();
  ctx.fillStyle = cache.fillGrad;
  ctx.fill();

  // Current value dot
  const lastY = toY(pts[n - 1]);
  ctx.fillStyle = lineColor;
  ctx.beginPath(); ctx.arc(W - 2, lastY, 3, 0, Math.PI * 2); ctx.fill();
  ctx.fillStyle = lineColor + '44';
  ctx.beginPath(); ctx.arc(W - 2, lastY, 6, 0, Math.PI * 2); ctx.fill();
}


// ══════════════════════════════════════════════════════════════
// ATTITUDE INDICATOR
// ══════════════════════════════════════════════════════════════

function drawAttitude(rollDeg, pitchDeg, yawDeg) {
  const canvas = document.getElementById('attitude-canvas');
  if (!canvas) return;
  const W = 280, H = 280, cx = 140, cy = 140, r = 130;
  if (canvas.width !== W || canvas.height !== H) { canvas.width = W; canvas.height = H; }
  const ctx = canvas.getContext('2d');
  ctx.clearRect(0, 0, W, H);

  const rollRad = rollDeg * Math.PI / 180;
  const pitchPx = pitchDeg * 2.4;

  ctx.save();
  ctx.translate(cx, cy);
  ctx.rotate(rollRad);

  // Clip to circle
  ctx.beginPath(); ctx.arc(0, 0, r, 0, Math.PI * 2); ctx.clip();

  // Sky
  const skyGrad = ctx.createLinearGradient(0, -r + pitchPx, 0, pitchPx);
  skyGrad.addColorStop(0, '#0a1f3a');
  skyGrad.addColorStop(1, '#1a4a6e');
  ctx.fillStyle = skyGrad;
  ctx.fillRect(-r, -r, r * 2, r * 2 + pitchPx);

  // Ground
  const gndGrad = ctx.createLinearGradient(0, pitchPx, 0, r);
  gndGrad.addColorStop(0, '#5a3a1a');
  gndGrad.addColorStop(1, '#2a1a08');
  ctx.fillStyle = gndGrad;
  ctx.fillRect(-r, pitchPx, r * 2, r * 2);

  // Horizon line
  ctx.strokeStyle = 'rgba(255,255,255,0.9)';
  ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(-r, pitchPx); ctx.lineTo(r, pitchPx); ctx.stroke();

  // Pitch ladder
  ctx.strokeStyle = 'rgba(255,255,255,0.55)';
  ctx.fillStyle = 'rgba(255,255,255,0.7)';
  ctx.font = 'bold 10px Share Tech Mono';
  ctx.textAlign = 'center';
  for (let deg = -40; deg <= 40; deg += 10) {
    if (deg === 0) continue;
    const y = pitchPx - deg * 2.4;
    const lw = deg % 20 === 0 ? 48 : 28;
    ctx.lineWidth = deg % 20 === 0 ? 1.5 : 1;
    ctx.beginPath(); ctx.moveTo(-lw, y); ctx.lineTo(lw, y); ctx.stroke();
    if (deg % 20 === 0) {
      ctx.fillText(Math.abs(deg), lw + 14, y + 4);
      ctx.fillText(Math.abs(deg), -lw - 14, y + 4);
    }
  }

  ctx.restore();

  // Outer ring
  const ringGrad = ctx.createLinearGradient(cx - r, cy - r, cx + r, cy + r);
  ringGrad.addColorStop(0, '#00d4ff');
  ringGrad.addColorStop(1, '#0088aa');
  ctx.strokeStyle = ringGrad;
  ctx.lineWidth = 2;
  ctx.beginPath(); ctx.arc(cx, cy, r, 0, Math.PI * 2); ctx.stroke();

  // Roll arc markings
  ctx.strokeStyle = 'rgba(0,212,255,0.6)';
  ctx.lineWidth = 1.5;
  [0, 10, 20, 30, 45, 60].forEach(a => {
    [-a, a].forEach(angle => {
      if (angle === 0 && a !== 0) return;
      const rad = (angle - 90) * Math.PI / 180;
      const inner = r + 4, outer = r + (a % 30 === 0 ? 14 : 8);
      ctx.beginPath();
      ctx.moveTo(cx + inner * Math.cos(rad), cy + inner * Math.sin(rad));
      ctx.lineTo(cx + outer * Math.cos(rad), cy + outer * Math.sin(rad));
      ctx.stroke();
    });
  });

  // Roll pointer triangle
  ctx.save(); ctx.translate(cx, cy); ctx.rotate(rollRad);
  ctx.fillStyle = '#00d4ff'; ctx.beginPath();
  ctx.moveTo(0, -(r + 18)); ctx.lineTo(-6, -(r + 6)); ctx.lineTo(6, -(r + 6));
  ctx.closePath(); ctx.fill();
  ctx.restore();

  // Aircraft symbol
  ctx.strokeStyle = '#ffffff'; ctx.lineWidth = 2.5; ctx.lineCap = 'round';
  ctx.beginPath(); ctx.moveTo(cx - 42, cy); ctx.lineTo(cx - 18, cy); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(cx + 18, cy); ctx.lineTo(cx + 42, cy); ctx.stroke();
  ctx.fillStyle = '#fff'; ctx.beginPath(); ctx.arc(cx, cy, 4, 0, Math.PI * 2); ctx.fill();
  ctx.beginPath(); ctx.moveTo(cx, cy - 10); ctx.lineTo(cx, cy + 10); ctx.stroke();

  // Heading text
  const hdg = Math.round(((yawDeg % 360) + 360) % 360);
  ctx.font = 'bold 11px Orbitron'; ctx.fillStyle = '#00d4ff'; ctx.textAlign = 'center';
  ctx.fillText('HDG ' + hdg + '°', cx, cy + r - 6);
}


// ══════════════════════════════════════════════════════════════
// COMPASS STRIP
// ══════════════════════════════════════════════════════════════

function drawCompass(yawDeg) {
  const canvas = document.getElementById('compass-canvas');
  if (!canvas) return;
  const W = canvas.offsetWidth || 280, H = 56;
  if (canvas.width !== W) canvas.width = W;
  if (canvas.height !== H) canvas.height = H;
  const ctx = canvas.getContext('2d');
  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = 'rgba(0,0,0,0.5)';
  ctx.fillRect(0, 0, W, H);

  const hdg = ((yawDeg % 360) + 360) % 360;
  const pxPerDeg = W / 90;
  const dirs = { 0: 'N', 45: 'NE', 90: 'E', 135: 'SE', 180: 'S', 225: 'SW', 270: 'W', 315: 'NW', 360: 'N' };

  ctx.font = '10px Share Tech Mono'; ctx.textAlign = 'center';
  for (let d = -50; d <= 50; d++) {
    const ang = ((hdg + d) % 360 + 360) % 360;
    const x = W / 2 + d * pxPerDeg;
    if (x < 0 || x > W) continue;
    const isMajor = ang % 45 === 0, isMinor = ang % 10 === 0;
    if (!isMinor) continue;
    const tickH = isMajor ? 16 : ang % 5 === 0 ? 10 : 6;
    ctx.strokeStyle = isMajor ? 'rgba(0,212,255,0.9)' : 'rgba(0,212,255,0.35)';
    ctx.lineWidth = isMajor ? 1.5 : 1;
    ctx.beginPath(); ctx.moveTo(x, H - tickH); ctx.lineTo(x, H); ctx.stroke();
    if (isMajor) {
      ctx.fillStyle = '#00d4ff';
      ctx.fillText(dirs[ang] || ang, x, H - tickH - 4);
    } else if (ang % 30 === 0) {
      ctx.fillStyle = 'rgba(0,212,255,0.5)';
      ctx.fillText(ang, x, H - tickH - 4);
    }
  }

  // Centre pointer
  ctx.fillStyle = '#00ff88';
  ctx.beginPath(); ctx.moveTo(W / 2, 0); ctx.lineTo(W / 2 - 6, 12); ctx.lineTo(W / 2 + 6, 12); ctx.closePath(); ctx.fill();

  // Heading label
  ctx.font = 'bold 13px Orbitron'; ctx.fillStyle = '#00ff88'; ctx.textAlign = 'center';
  ctx.fillText(Math.round(hdg) + '°', W / 2, H - 20);
}


// ══════════════════════════════════════════════════════════════
// RENDER LOOP (requestAnimationFrame — 60fps smooth)
// ══════════════════════════════════════════════════════════════

let rafRunning = false;
let rafPrevTime = 0;

function startRenderLoop() {
  if (rafRunning) return;
  rafRunning = true;

  function frame(ts) {
    const dt = Math.min((ts - (rafPrevTime || ts)) / 1000, 0.1);
    rafPrevTime = ts;

    const f = 1 - Math.pow(1 - LERP, dt * 60);
    const fi = 1 - Math.pow(0.3, dt * 60);

    // Attitude smooth
    d_roll = lerpAngle(d_roll, roll, f);
    d_pitch = lerp(d_pitch, pitch, f);
    d_yaw = lerpAngle(d_yaw, yaw, f);
    d_spd = lerp(d_spd, spd, f);
    d_vspd = lerp(d_vspd, vspd_live || 0, f);
    d_alt = lerp(d_alt, alt, f);
    d_ralt = lerp(d_ralt, ralt, f);

    // IMU smooth
    s_gx = lerp(s_gx, gx, fi);
    s_gy = lerp(s_gy, gy, fi);
    s_gz = lerp(s_gz, gz, fi);
    s_ax = lerp(s_ax, ax, fi);
    s_ay = lerp(s_ay, ay, fi);
    s_az = lerp(s_az, az, fi);

    // Feed smooth buffers
    smoothBufs.gx.push(s_gx); if (smoothBufs.gx.length > SMOOTH_LEN) smoothBufs.gx.shift();
    smoothBufs.gy.push(s_gy); if (smoothBufs.gy.length > SMOOTH_LEN) smoothBufs.gy.shift();
    smoothBufs.gz.push(s_gz); if (smoothBufs.gz.length > SMOOTH_LEN) smoothBufs.gz.shift();
    smoothBufs.ax.push(s_ax); if (smoothBufs.ax.length > SMOOTH_LEN) smoothBufs.ax.shift();
    smoothBufs.ay.push(s_ay); if (smoothBufs.ay.length > SMOOTH_LEN) smoothBufs.ay.shift();
    smoothBufs.az.push(s_az); if (smoothBufs.az.length > SMOOTH_LEN) smoothBufs.az.shift();

    // Draw
    drawAttitude(d_roll, d_pitch, d_yaw);
    drawCompass(d_yaw);

    drawImuGraph('gx-canvas', smoothBufs.gx, -2, 2, '#00d4ff');
    drawImuGraph('gy-canvas', smoothBufs.gy, -2, 2, '#00ff88');
    drawImuGraph('gz-canvas', smoothBufs.gz, -2, 2, '#aa88ff');
    drawImuGraph('ax-canvas', smoothBufs.ax, -15, 15, '#ff6688');
    drawImuGraph('ay-canvas', smoothBufs.ay, -15, 15, '#ffdd00');
    drawImuGraph('az-canvas', smoothBufs.az, 5, 15, '#ff8844');

    // Numeric labels
    const fmt = (v, dp) => isNaN(v) ? '--' : v.toFixed(dp);
    document.getElementById('p-roll').textContent = fmt(d_roll, 1) + '°';
    document.getElementById('p-pitch').textContent = fmt(d_pitch, 1) + '°';
    document.getElementById('p-yaw').textContent = Math.round(((d_yaw % 360) + 360) % 360) + '°';
    document.getElementById('gx-val').textContent = fmt(s_gx, 3) + ' rad/s';
    document.getElementById('gy-val').textContent = fmt(s_gy, 3) + ' rad/s';
    document.getElementById('gz-val').textContent = fmt(s_gz, 3) + ' rad/s';
    document.getElementById('ax-val').textContent = fmt(s_ax, 3) + ' m/s²';
    document.getElementById('ay-val').textContent = fmt(s_ay, 3) + ' m/s²';
    document.getElementById('az-val').textContent = fmt(s_az, 3) + ' m/s²';

    const sEl = document.getElementById('spd-box');
    const vEl = document.getElementById('vspd-box');
    if (sEl) sEl.textContent = fmt(d_spd, 1) + ' m/s';
    if (vEl) vEl.textContent = fmt(d_vspd, 2) + ' m/s';

    requestAnimationFrame(frame);
  }
  requestAnimationFrame(frame);
}
