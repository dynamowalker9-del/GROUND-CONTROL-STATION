# DroneGuard Connection Analysis

## 🔗 Frontend-to-Backend Communication

### Connection Flow
```
HTML/JavaScript (Browser)
    ↓
WebSocket (:8765 for telemetry, :8766 for commands)
    ↓
Python server.py
    ↓
MAVLink (PyMavLink)
    ↓
Drone (via Serial/USB)
```

---

## 📡 How Many Times Code Connects to Drones?

### **1. Main Server (server.py)**
- **Drone Connections: 1**
  - Line 42: `master = mavutil.mavlink_connection(DRONE_CONNECTION, ...)`
  - Line 48: `master.wait_heartbeat(timeout=30)`
  - Uses `autoreconnect=True` for automatic reconnection on disconnect

### **2. Scripts (When Executed)**
Each script connects to **2 drones independently**:

| Script | Leader Connection | Follower Connection | Total |
|--------|-------------------|-------------------|-------|
| **scripts1.py** | PIX6 (115200 baud) | Mini Pix (57600 baud) | 2 |
| **scripts2.py** | PIX6 (115200 baud) | Mini Pix (57600 baud) | 2 |
| **scripts3.py** | PIX6 (115200 baud) | Mini Pix (57600 baud) | 2 |

**Total Drone Connections When All Scripts Run: 6 additional connections**

### **3. Frontend (HTML/JS - app.js)**
- **No direct drone connection** ✓
- Communicates with `server.py` only via WebSocket
- The "CONNECT" button is **UI-only** (doesn't create actual connections)
- Falls back to simulation mode if `server.py` is unavailable

---

## 🔌 Connection Details

### Backend Connection (Automatic)
```javascript
// app.js - Line 115
function connectBackend() {
  ws = new WebSocket(`ws://${wsHost}:8765`);
  // Retries every 3 seconds if failed
  wsReconnectTimer = setTimeout(connectBackend, 3000);
}
```
- **Port:** 8765 (Telemetry)
- **Retry:** Every 3 seconds if disconnected
- **Fallback:** SIM MODE if Python server unreachable

### Command Server Connection (Automatic)
```javascript
// terminal.js - Line 51
connectCommandServer() {
  ws = new WebSocket(`ws://${wsHost}:8766`);
  // Retries every 3 seconds if failed
}
```
- **Port:** 8766 (Commands)
- **Retry:** Every 3 seconds if disconnected

### Drone Connection (Manual via UI)
```javascript
// app.js - Line 252
function toggleConnect() {
  if (!droneConnected) {
    droneConnected = true;  // Just sets flag
    // Does NOT create actual MAVLink connection
  }
}
```
- **User clicks:** "CONNECT" button in dashboard
- **Action:** Enables ARM/DISARM/RTL buttons
- **Actual Connection:** Already done by `server.py` at startup

---

## ✅ What Works Well

| Component | Status | Details |
|-----------|--------|---------|
| HTML → Python | ✅ **Good** | WebSocket communication works |
| Python → Drone | ✅ **Good** | Connects once, auto-reconnects |
| Browser Fallback | ✅ **Good** | SIM MODE if server unavailable |
| Command Execution | ✅ **Good** | Scripts can run via port 8766 |
| Telemetry Streaming | ✅ **Good** | Real-time data via port 8765 |

---

## ⚠️ Issues Found

### **1. Hardcoded Serial Ports in Scripts**
All scripts hardcode specific USB device IDs:
```python
leader = mavutil.mavlink_connection(
    '/dev/serial/by-id/usb-ArduPilot_RadiolinkPIX6_1C003A001951313530353239-if00',
    baud=115200
)
```
**Fix Needed:** These will **NOT work** on your Windows system (Linux paths).

### **2. Resource Intensive Script Execution**
- Each script creates 2 independent drone connections
- Running multiple scripts simultaneously = 6+ concurrent MAVLink connections
- Could cause serial port conflicts or resource exhaustion

### **3. No Error Handling in server.py Startup**
```python
master.wait_heartbeat(timeout=30)  # If fails, server crashes
```
**Fix:** Add try-catch to handle connection failures gracefully.

### **4. Mismatch: server.py vs script serial ports**
- **server.py:** Connects to `COM21` (Windows)
- **Scripts:** Hardcoded `/dev/serial/by-id/...` (Linux paths)

**These scripts won't work on Windows without modification!**

---

## 📊 Connection Summary Table

```
┌─────────────────┬──────────────┬─────────┬────────────────┐
│ Component       │ Type         │ Count   │ Auto-Reconnect │
├─────────────────┼──────────────┼─────────┼────────────────┤
│ server.py       │ MAVLink      │ 1       │ Yes (built-in) │
│ scripts1.py     │ MAVLink      │ 2       │ No             │
│ scripts2.py     │ MAVLink      │ 2       │ No             │
│ scripts3.py     │ MAVLink      │ 2       │ No             │
│ Frontend (WS)   │ WebSocket    │ 2       │ Yes (8765,8766)│
└─────────────────┴──────────────┴─────────┴────────────────┘
```

---

## 🔧 Recommended Fixes

### Fix 1: Update Script Serial Ports for Windows
Replace hardcoded Linux paths with Windows COM ports:
```python
# Before (Linux)
leader = mavutil.mavlink_connection(
    '/dev/serial/by-id/usb-ArduPilot_...',
    baud=115200
)

# After (Windows)
leader = mavutil.mavlink_connection('COM20', baud=115200)
follower = mavutil.mavlink_connection('COM21', baud=57600)
```

### Fix 2: Add Error Handling to server.py
```python
try:
    master.wait_heartbeat(timeout=30)
    print(f"Heartbeat received — SysID={master.target_system}")
except Exception as e:
    print(f"ERROR: Failed to connect to drone: {e}")
    sys.exit(1)
```

### Fix 3: Make Scripts Configurable
Use environment variables instead of hardcoded paths:
```python
import os
LEADER_PORT = os.getenv("LEADER_PORT", "COM20")
FOLLOWER_PORT = os.getenv("FOLLOWER_PORT", "COM21")
```

---

## Summary

| Aspect | Status |
|--------|--------|
| **HTML ↔ Python Backend** | ✅ Working correctly |
| **Python ↔ Drone** | ⚠️ Works but needs Windows port fixes |
| **Auto-reconnection** | ✅ Frontend retries every 3s |
| **Manual Drone Connect** | ✓ UI button works (already connected via server) |
| **Script Integration** | ❌ Serial ports won't work on Windows |

