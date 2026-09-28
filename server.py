

import asyncio
import json
import math
import glob
import os
import sys
import threading
import time
from pathlib import Path

import websockets
from pymavlink import mavutil


def discover_drone_connection() -> list:
    """Auto-detect a list of serial devices for the drone connection (Windows & Linux), ordered by preference."""
    if sys.platform.startswith('win'):
        # Try these ports in order (based on typical ArduPilot setups)
        possible_ports = ["COM21", "COM22", "COM20", "COM12", "COM13", "COM5", "COM7", "COM6", "COM9", "COM27","COM8"]

        print("[auto-detect] Checking for ArduPilot on common COM ports...")
        print("[auto-detect] Tip: Set DRONE_CONNECTION env var to force a specific port")

        # Add any higher COM ports we should try
        possible_ports.extend([f"COM{i}" for i in range(1, 30) if f"COM{i}" not in possible_ports])

        print(f"[auto-detect] Will try these ports in order: {possible_ports}")
        return possible_ports

    # Linux: Check for /dev/serial/by-id
    serial_by_id = Path("/dev/serial/by-id")
    ports = []
    if serial_by_id.exists():
        for entry in sorted(serial_by_id.iterdir()):
            if entry.is_symlink() or entry.is_char_device():
                ports.append(str(entry))
        if ports:
            print(f"[auto-detect] Found ports by-id: {ports}")
            return ports

    for pattern in ("/dev/ttyACM*", "/dev/ttyUSB*", "/dev/ttyAMA*", "/dev/ttyS*"):
        matches = sorted(glob.glob(pattern))
        if matches:
            print(f"[auto-detect] Found ports by pattern {pattern}: {matches}")
            return matches

    print(f"[auto-detect] No ports found, defaulting to [/dev/ttyUSB0]")
    return ["/dev/ttyUSB0"]


DRONE_CONNECTION = os.getenv("DRONE_CONNECTION") or discover_drone_connection()
print(f"[DEBUG] DRONE_CONNECTION set to: {DRONE_CONNECTION}")
DRONE_BAUD = int(os.getenv("DRONE_BAUD", "115200"))
WS_HOST = os.getenv("WS_HOST", "0.0.0.0")
# Use port 0 for auto-assignment, or read from environment
WS_PORT = int(os.getenv("WS_PORT", "0"))  # 0 = auto-assign available port
STREAM_HZ = int(os.getenv("STREAM_HZ", "20"))
RC_STREAM_HZ = int(os.getenv("RC_STREAM_HZ", "50"))
COMMAND_WS_HOST = os.getenv("COMMAND_WS_HOST", "0.0.0.0")
COMMAND_WS_PORT = int(os.getenv("COMMAND_WS_PORT", "0"))  # 0 = auto-assign available port
COMMAND_TIMEOUT = int(os.getenv("COMMAND_TIMEOUT", "300"))
PROJECT_ROOT = Path(__file__).resolve().parent

# Store actual ports after server starts
ACTUAL_WS_PORT = None
ACTUAL_COMMAND_WS_PORT = None

COMMAND_MAP = {
    "1": "scripts/scripts1.py",
    "2": "scripts/scripts2.py",
    "3": "scripts/scripts3.py",
}


COPTER_MODES = {
    0: "STABILIZE",  1: "ACRO",        2: "ALT_HOLD",     3: "AUTO",
    4: "GUIDED",     5: "LOITER",      6: "RTL",           7: "CIRCLE",
    9: "LAND",      11: "DRIFT",      13: "SPORT",        14: "FLIP",
   15: "AUTOTUNE",  16: "POSHOLD",    17: "BRAKE",        18: "THROW",
   19: "AVOID_ADSB",20: "GUIDED_NOGPS",21: "SMART_RTL",
}
MODE_NAME_TO_ID = {v: k for k, v in COPTER_MODES.items()}

# Initialize MAVLink connection with error handling
master = None
if isinstance(DRONE_CONNECTION, list):
    # Try each port in the list until one works
    for port in DRONE_CONNECTION:
        try:
            print(f"Trying to connect to {port} @ {DRONE_BAUD} baud ...")
            master = mavutil.mavlink_connection(
                port, baud=DRONE_BAUD,
                autoreconnect=True,
                source_system=255,
                source_component=0,
            )
            master.wait_heartbeat(timeout=10)
            print(f"Heartbeat received on {port} - SysID={master.target_system}  CompID={master.target_component}")
            break  # Success, exit loop
        except Exception as e:
            print(f"[INFO] Failed to connect to {port}: {e}")
            master = None
            continue  # Try next port
else:
    # Single port (backward compatibility)
    try:
        print(f"Connecting to {DRONE_CONNECTION} @ {DRONE_BAUD} baud ...")
        master = mavutil.mavlink_connection(
            DRONE_CONNECTION, baud=DRONE_BAUD,
            autoreconnect=True,
            source_system=255,
            source_component=0,
        )
        master.wait_heartbeat(timeout=30)
        print(f"Heartbeat received - SysID={master.target_system}  CompID={master.target_component}")
    except Exception as e:
        print(f"[ERROR] Failed to connect to drone at {DRONE_CONNECTION}: {e}")
        print("[INFO] Server will still start and accept WebSocket connections.")
        print("[INFO] Telemetry will not be available until drone connects.")
        master = None

if master is None:
    print("[WARNING] No MAVLink connection established. Running in telemetry-only mode.")


def request_streams():
    """Ask ArduPilot to stream all telemetry at the configured rate."""
    if master is None or master.target_system == 0:
        return   # not connected yet

    # Request all data streams
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        STREAM_HZ,
        1,    # start
    )

    # Explicitly request RC channels (some autopilots need this)
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
        RC_STREAM_HZ,
        1,    # start
    )

    try:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS,
            int(1_000_000 / max(1, RC_STREAM_HZ)),
            0, 0, 0, 0, 0,
        )
    except (AttributeError, TypeError):
        pass

    print(f"Data streams requested @ {STREAM_HZ} Hz; RC_CHANNELS @ {RC_STREAM_HZ} Hz")

request_streams()


telem = {
    # Attitude
    "raw_roll":  0.0,  "raw_pitch": 0.0,  "raw_yaw": 0.0,
    # Gyro (real angular rates from ATTITUDE message)
    "raw_gx":    0.0,  "raw_gy":    0.0,  "raw_gz":  0.0,
    # Accelerometer
    "raw_ax":    0.0,  "raw_ay":    0.0,  "raw_az":  9.8,
    # GPS position - None until a valid fix arrives
    "lat":       None, "lon":       None,
    "alt_msl":   0.0,  "alt_rel":   0.0,
    "gps_fix":   0,    "satellites": 0,   "hdop": 99.0,
    # Velocity
    "groundspeed": 0.0, "airspeed":   0.0,
    "heading_deg": 0.0, "vspeed":     0.0,
    # Battery
    "batt_voltage": 0.0, "batt_current": 0.0, "batt_pct": -1,
    # RC Channels (raw transmitter PWM) - None until the first real packet
    "rc1": None, "rc2": None, "rc3": None, "rc4": None,
    "rc5": None, "rc6": None, "rc7": None, "rc8": None,
    "rc_rssi": 0,
    # RC status
    "rc_count_updates": 0, "rc_last_update_ms": 0,
    # Status
    "flight_mode": "UNKNOWN", "armed": False, "ekf_ok": False,
    # Parameter setup status
    "param_loaded": 0, "param_count": 0, "param_last_update_ms": 0,
    # Timestamp
    "ts": 0,
    "connection": DRONE_CONNECTION,
    "baud": DRONE_BAUD,
    "stream_hz": STREAM_HZ,
    "rc_stream_hz": RC_STREAM_HZ,
}
lock = threading.Lock()

parameter_store = {}
parameter_lock = threading.Lock()
parameter_generation = 0
parameter_expected_count = 0
parameter_last_update_ms = 0


def valid_coord(latitude, longitude):
    """Reject unset or impossible MAVLink coordinates before sending them to the map."""
    return (
        latitude is not None
        and longitude is not None
        and -90 <= latitude <= 90
        and -180 <= longitude <= 180
        and (abs(latitude) > 0.001 or abs(longitude) > 0.001)
    )


def normalize_param_id(param_id):
    """Convert MAVLink PARAM_VALUE param_id into a clean Python string."""
    if isinstance(param_id, bytes):
        raw = param_id.decode("ascii", errors="ignore")
    else:
        raw = str(param_id)
    return raw.split("\x00", 1)[0].strip()


def request_parameters():
    """Request the full parameter list from the flight controller."""
    global parameter_generation, parameter_expected_count, parameter_last_update_ms
    if master is None:
        return False, "drone not connected"
    parameter_last_update_ms = int(time.time() * 1000)
    with parameter_lock:
        parameter_store.clear()
        parameter_expected_count = 0
        parameter_generation += 1
    with lock:
        telem["param_loaded"] = 0
        telem["param_count"] = 0
        telem["param_last_update_ms"] = parameter_last_update_ms
    master.mav.param_request_list_send(master.target_system, master.target_component)
    return True, "parameter refresh requested"


def snapshot_parameters():
    with parameter_lock:
        parameters = [
            {
                "name": name,
                "value": item["value"],
                "type": item["type"],
                "index": item["index"],
                "count": item["count"],
            }
            for name, item in sorted(parameter_store.items())
        ]
        return {
            "type": "PARAMETERS",
            "source": "drone",
            "parameters": parameters,
            "loaded_count": len(parameters),
            "expected_count": parameter_expected_count,
            "last_update_ms": parameter_last_update_ms,
            "generation": parameter_generation,
        }


def set_parameter(name, value):
    """Write one MAVLink parameter. ArduPilot acknowledges later with PARAM_VALUE."""
    if master is None:
        return False, "drone not connected"

    param_name = normalize_param_id(name).upper()
    if not param_name:
        return False, "missing parameter name"
    if len(param_name) > 16:
        return False, "parameter name is longer than MAVLink's 16 byte limit"

    with parameter_lock:
        known_param = parameter_store.get(param_name)
        param_type = known_param["type"] if known_param else mavutil.mavlink.MAV_PARAM_TYPE_REAL32

    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        param_name.encode("ascii", errors="ignore"),
        float(value),
        int(param_type),
    )
    return True, f"parameter write sent: {param_name}={value}"


async def send_parameter_list(ws):
    await ws.send(json.dumps(snapshot_parameters()))


def reader():
    global parameter_generation, parameter_expected_count, parameter_last_update_ms
    last_stream_req = time.time()

    while True:
        # Wait if drone not connected yet
        if master is None:
            time.sleep(1.0)
            continue

        try:
            msg = master.recv_match(blocking=True, timeout=1.0)
        except Exception as e:
            print(f"[reader] recv error: {e}")
            time.sleep(0.5)
            continue


        if msg is None:
            if time.time() - last_stream_req > 5.0:
                request_streams()
                last_stream_req = time.time()
            continue

        mtype = msg.get_type()
        if mtype in ("BAD_DATA", "UNKNOWN"):
            continue

        last_stream_req = time.time()

        with lock:
            telem["ts"] = int(time.time() * 1000)

            if mtype == "ATTITUDE":
                r = math.degrees(msg.roll)
                p = math.degrees(msg.pitch)
                y = math.degrees(msg.yaw)
                telem["raw_roll"]  = round(r, 4)
                telem["raw_pitch"] = round(p, 4)
                # Normalise yaw to 0-360
                telem["raw_yaw"]   = round(y % 360 if y >= 0 else y % 360 + 360, 4)
                # Real gyro rates (rad/s) - use these for the gyro graphs
                telem["raw_gx"] = round(msg.rollspeed,  5)
                telem["raw_gy"] = round(msg.pitchspeed, 5)
                telem["raw_gz"] = round(msg.yawspeed,   5)


            elif mtype in ("RAW_IMU", "SCALED_IMU2"):
                # RAW_IMU/SCALED_IMU acceleration is in milli-g; display expects m/s^2.
                telem["raw_ax"] = round(msg.xacc * 9.80665 / 1000.0, 4)
                telem["raw_ay"] = round(msg.yacc * 9.80665 / 1000.0, 4)
                telem["raw_az"] = round(msg.zacc * 9.80665 / 1000.0, 4)


            elif mtype == "HIGHRES_IMU":
                telem["raw_ax"] = round(msg.xacc,  4)
                telem["raw_ay"] = round(msg.yacc,  4)
                telem["raw_az"] = round(msg.zacc,  4)
                telem["raw_gx"] = round(msg.xgyro, 5)
                telem["raw_gy"] = round(msg.ygyro, 5)
                telem["raw_gz"] = round(msg.zgyro, 5)


            elif mtype == "GLOBAL_POSITION_INT":
                la = msg.lat / 1e7
                lo = msg.lon / 1e7

                if valid_coord(la, lo):
                    telem["lat"] = round(la, 7)
                    telem["lon"] = round(lo, 7)
                telem["alt_msl"] = round(msg.alt / 1000.0, 2)
                telem["alt_rel"] = round(msg.relative_alt / 1000.0, 2)

                if msg.hdg != 65535:
                    telem["heading_deg"] = round(msg.hdg / 100.0, 1)

                vx = msg.vx / 100.0
                vy = msg.vy / 100.0
                telem["groundspeed"] = round(math.sqrt(vx*vx + vy*vy), 3)

                telem["vspeed"] = round(-msg.vz / 100.0, 3)


            elif mtype == "GPS_RAW_INT":
                telem["gps_fix"]    = msg.fix_type
                telem["satellites"] = msg.satellites_visible

                telem["hdop"] = round(msg.eph / 100.0, 2) if msg.eph != 65535 else 99.0

                if msg.fix_type >= 2:
                    la = msg.lat / 1e7
                    lo = msg.lon / 1e7
                    if valid_coord(la, lo):
                        telem["lat"] = round(la, 7)
                        telem["lon"] = round(lo, 7)


            elif mtype == "VFR_HUD":
                telem["airspeed"]    = round(msg.airspeed, 2)
                telem["groundspeed"] = round(msg.groundspeed, 2)
                telem["heading_deg"] = float(msg.heading)   # already 0-360
                telem["vspeed"]      = round(msg.climb, 2)
                telem["alt_msl"]     = round(msg.alt, 2)

            elif mtype == "SYS_STATUS":

                if msg.voltage_battery != 65535:
                    telem["batt_voltage"] = round(msg.voltage_battery / 1000.0, 3)
                if msg.current_battery >= 0:
                    telem["batt_current"] = round(msg.current_battery / 100.0, 2)
                if msg.battery_remaining >= 0:
                    telem["batt_pct"] = msg.battery_remaining

            elif mtype == "BATTERY_STATUS":
                if msg.id == 0:
                    if msg.battery_remaining >= 0:
                        telem["batt_pct"] = msg.battery_remaining

                    valid_v = [v for v in msg.voltages if v != 65535]
                    if valid_v:
                        telem["batt_voltage"] = round(sum(valid_v) / 1000.0, 3)

            elif mtype == "PARAM_VALUE":
                param_name = normalize_param_id(msg.param_id).upper()
                if param_name:
                    parameter_expected_count = int(getattr(msg, "param_count", 0) or 0)
                    parameter_last_update_ms = int(time.time() * 1000)
                    with parameter_lock:
                        parameter_store[param_name] = {
                            "value": round(float(msg.param_value), 6),
                            "type": int(getattr(msg, "param_type", 0) or 0),
                            "index": int(getattr(msg, "param_index", -1) or -1),
                            "count": parameter_expected_count,
                        }
                        parameter_generation += 1
                    telem["param_loaded"] = len(parameter_store)
                    telem["param_count"] = parameter_expected_count
                    telem["param_last_update_ms"] = parameter_last_update_ms


            elif mtype == "HEARTBEAT":
                if (msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID and
                        msg.type != mavutil.mavlink.MAV_TYPE_GCS):
                    armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    telem["armed"]       = armed
                    telem["flight_mode"] = COPTER_MODES.get(
                        msg.custom_mode, f"MODE_{msg.custom_mode}"
                    )


            elif mtype == "EKF_STATUS_REPORT":

                f = msg.flags
                ATTITUDE_OK  = (1 << 0)
                VEL_HORIZ_OK = (1 << 1)
                POS_HORIZ_OK = (1 << 3)
                telem["ekf_ok"] = bool(f & ATTITUDE_OK and f & VEL_HORIZ_OK and f & POS_HORIZ_OK)

            # ── RC Channels from transmitter ─────────────────────
            elif mtype == "RC_CHANNELS":
                # Primary RC input source from receiver
                telem["rc1"] = msg.chan1_raw
                telem["rc2"] = msg.chan2_raw
                telem["rc3"] = msg.chan3_raw
                telem["rc4"] = msg.chan4_raw
                telem["rc5"] = msg.chan5_raw
                telem["rc6"] = msg.chan6_raw
                telem["rc7"] = msg.chan7_raw
                telem["rc8"] = msg.chan8_raw
                telem["rc_rssi"] = msg.rssi
                telem["rc_count_updates"] += 1
                telem["rc_last_update_ms"] = int(time.time() * 1000)

            elif mtype == "RC_CHANNELS_RAW":
                # Fallback for older firmwares that send RC_CHANNELS_RAW instead
                telem["rc1"] = msg.chan1_raw
                telem["rc2"] = msg.chan2_raw
                telem["rc3"] = msg.chan3_raw
                telem["rc4"] = msg.chan4_raw
                telem["rc5"] = msg.chan5_raw
                telem["rc6"] = msg.chan6_raw
                telem["rc7"] = msg.chan7_raw
                telem["rc8"] = msg.chan8_raw
                telem["rc_rssi"] = msg.rssi
                telem["rc_count_updates"] += 1
                telem["rc_last_update_ms"] = int(time.time() * 1000)

            elif mtype == "SERVO_OUTPUT_RAW":
                # Also capture servo outputs as fallback RC display
                # Only update if we haven't received real RC data yet this session
                if telem["rc_count_updates"] == 0:
                    telem["rc1"] = msg.servo1_raw
                    telem["rc2"] = msg.servo2_raw
                    telem["rc3"] = msg.servo3_raw
                    telem["rc4"] = msg.servo4_raw
                    telem["rc5"] = getattr(msg, 'servo5_raw', 1000)
                    telem["rc6"] = getattr(msg, 'servo6_raw', 1000)
                    telem["rc7"] = getattr(msg, 'servo7_raw', 1000)
                    telem["rc8"] = getattr(msg, 'servo8_raw', 1000)


threading.Thread(target=reader, daemon=True, name="mav-reader").start()



async def handle_command(ws, raw: str):

    try:
        cmd = json.loads(raw)
    except (json.JSONDecodeError, ValueError):
        s = raw.strip()
        if s == "ARM":
            _arm(True)
        elif s == "DISARM":
            _arm(False)
        return

    action = cmd.get("cmd", "").upper()

    if action == "ARM":
        _arm(True)

    elif action == "DISARM":
        _arm(False)

    elif action in ("SET_MODE", "SETMODE"):
        raw_name = cmd.get("mode", "").strip().upper()

        label_map = {
            "ALT HOLD":      "ALT_HOLD",
            "ALT_HOLD":      "ALT_HOLD",
            "POSHOLD":       "POSHOLD",
            "SMART_RTL":     "SMART_RTL",
            "GUIDED NO GPS": "GUIDED_NOGPS",
        }
        mode_name = label_map.get(raw_name, raw_name.replace(" ", "_"))
        mode_id   = MODE_NAME_TO_ID.get(mode_name)
        if mode_id is None:
            print(f"[cmd] Unknown mode: {mode_name}")
            return
        if master is None:
            print(f"[cmd] ERROR: Cannot SET_MODE - drone not connected")
            return
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )
        print(f"[cmd] SET_MODE -> {mode_name} (id={mode_id})")

    elif action == "RTL":
        if master is None:
            print(f"[cmd] ERROR: Cannot RTL - drone not connected")
            return
        mode_id = MODE_NAME_TO_ID.get("RTL", 6)
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )
        print("[cmd] RTL")

    elif action == "TAKEOFF":
        if master is None:
            print(f"[cmd] ERROR: Cannot TAKEOFF - drone not connected")
            return
        alt = float(cmd.get("alt", 5.0))
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,       # confirmation
            0, 0, 0, 0, 0, 0, alt,
        )
        print(f"[cmd] TAKEOFF to {alt} m")

    elif action == "RESTREAM":
        request_streams()
        print("[cmd] RESTREAM")

    elif action == "GET_PARAMETERS":
        await send_parameter_list(ws)

    elif action == "REFRESH_PARAMETERS":
        ok, message = request_parameters()
        await ws.send(command_json(
            "PARAM_STATUS" if ok else "PARAM_ERROR",
            state="requested" if ok else "error",
            message=message,
        ))
        await send_parameter_list(ws)

    elif action in ("SET_PARAMETER", "SET_PARAM"):
        name = cmd.get("name", "")
        try:
            value = float(cmd.get("value"))
        except (TypeError, ValueError):
            await ws.send(command_json("PARAM_ERROR", message="invalid numeric parameter value"))
            return

        ok, message = set_parameter(name, value)
        await ws.send(command_json(
            "PARAM_ACK" if ok else "PARAM_ERROR",
            name=normalize_param_id(name).upper(),
            value=value,
            message=message,
        ))
        if ok:
            print(f"[cmd] {message}")


def _arm(arm: bool):
    """Send arm/disarm via MAV_CMD_COMPONENT_ARM_DISARM (works on all pymavlink versions)."""
    if master is None:
        print(f"[cmd] ERROR: Cannot ARM/DISARM - drone not connected")
        return
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,              # confirmation
        1 if arm else 0,  # param1: 1=arm, 0=disarm
        0, 0, 0, 0, 0, 0,
    )
    print(f"[cmd] {'ARM' if arm else 'DISARM'}")



async def _send_loop(ws):
    """Push telemetry snapshots to the browser at the configured stream rate."""
    send_interval = 1.0 / max(1, STREAM_HZ, RC_STREAM_HZ)
    last_parameter_generation = -1
    last_parameter_push = 0.0
    while True:
        await asyncio.sleep(send_interval)
        with lock:
            payload = dict(telem)
        try:
            await ws.send(json.dumps(payload))
            now = time.time()
            if parameter_generation != last_parameter_generation and now - last_parameter_push >= 1.0:
                await send_parameter_list(ws)
                last_parameter_generation = parameter_generation
                last_parameter_push = now
        except websockets.exceptions.ConnectionClosed:
            break
        except Exception as e:
            # Transient errors - keep trying instead of silently dying
            print(f"[send] warning: {e}")
            await asyncio.sleep(0.1)


async def _gcs_heartbeat():
    """Send a GCS heartbeat every second so ArduPilot knows a GCS is active."""
    while True:
        await asyncio.sleep(1.0)
        if master is None:
            continue
        try:
            master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0,
            )
        except Exception:
            pass


def command_json(message_type, **payload):
    return json.dumps({"type": message_type, **payload})


def resolve_command_script(command_id):
    script = COMMAND_MAP.get(command_id)
    if script is None:
        raise ValueError(f"Command '{command_id}' is not whitelisted.")

    script_path = (PROJECT_ROOT / script).resolve()
    try:
        script_path.relative_to(PROJECT_ROOT)
    except ValueError as exc:
        raise ValueError("Whitelisted script resolves outside project root.") from exc

    if not script_path.is_file():
        raise FileNotFoundError(f"Mapped script does not exist: {script}")

    return script, script_path


async def stream_command_output(ws, stream):
    while True:
        chunk = await stream.readline()
        if not chunk:
            break
        await ws.send(command_json("output", data=chunk.decode("utf-8", errors="replace")))


async def run_whitelisted_command(ws, command_id):
    script_label, script_path = resolve_command_script(command_id)
    await ws.send(command_json(
        "status",
        state="running",
        message=f"Running command {command_id}: {script_label}",
    ))

    process = await asyncio.create_subprocess_exec(
        sys.executable,
        "-u",
        str(script_path),
        cwd=str(PROJECT_ROOT),
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.STDOUT,
    )

    try:
        await asyncio.wait_for(stream_command_output(ws, process.stdout), timeout=COMMAND_TIMEOUT)
        returncode = await asyncio.wait_for(process.wait(), timeout=5)
    except asyncio.TimeoutError:
        process.kill()
        await process.wait()
        await ws.send(command_json("error", message=f"Command timed out after {COMMAND_TIMEOUT}s."))
        return

    await ws.send(command_json(
        "complete",
        command=command_id,
        script=script_label,
        returncode=returncode,
    ))


async def handle_terminal_message(ws, raw):
    try:
        message = json.loads(raw)
    except json.JSONDecodeError:
        await ws.send(command_json("error", message="Invalid JSON message."))
        return

    message_type = message.get("type")
    if message_type == "list":
        await ws.send(command_json("catalog", commands=COMMAND_MAP))
        return

    if message_type != "run":
        await ws.send(command_json("error", message="Unsupported message type."))
        return

    command_id = str(message.get("command", "")).strip()
    if not command_id:
        await ws.send(command_json("error", message="Missing command ID."))
        return

    try:
        await run_whitelisted_command(ws, command_id)
    except (ValueError, FileNotFoundError) as exc:
        await ws.send(command_json("error", message=str(exc)))
    except Exception as exc:
        await ws.send(command_json("error", message=f"Command executor failed: {exc}"))
    finally:
        await ws.send(command_json("status", state="idle", message="IDLE"))


async def terminal_ws_handler(ws):
    addr = getattr(ws, "remote_address", "?")
    print(f"[terminal-ws] + connected: {addr}")
    await ws.send(command_json("catalog", commands=COMMAND_MAP))
    try:
        async for message in ws:
            await handle_terminal_message(ws, message)
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        print(f"[terminal-ws] - disconnected: {addr}")



async def ws_handler(ws):
    addr = getattr(ws, "remote_address", "?")
    print(f"[ws] + connected: {addr}")
    sender = asyncio.create_task(_send_loop(ws))
    try:
        async for message in ws:
            await handle_command(ws, message)
    except websockets.exceptions.ConnectionClosed:
        pass
    except Exception as e:
        print(f"[ws] handler error: {e}")
    finally:
        sender.cancel()
        print(f"[ws] - disconnected: {addr}")


async def main():

    asyncio.create_task(_gcs_heartbeat())

    # Start WebSocket servers and capture actual ports
    global ACTUAL_WS_PORT, ACTUAL_COMMAND_WS_PORT

    ws_server = await websockets.serve(ws_handler, WS_HOST, WS_PORT)
    cmd_server = await websockets.serve(terminal_ws_handler, COMMAND_WS_HOST, COMMAND_WS_PORT)

    # Extract actual port numbers (in case port 0 was used for auto-assignment)
    ACTUAL_WS_PORT = ws_server.sockets[0].getsockname()[1]
    ACTUAL_COMMAND_WS_PORT = cmd_server.sockets[0].getsockname()[1]

    # Write config file for client auto-discovery
    config = {
        "ws_host": WS_HOST if WS_HOST != "0.0.0.0" else "localhost",
        "ws_port": ACTUAL_WS_PORT,
        "command_ws_host": COMMAND_WS_HOST if COMMAND_WS_HOST != "0.0.0.0" else "localhost",
        "command_ws_port": ACTUAL_COMMAND_WS_PORT,
    }
    config_path = PROJECT_ROOT / "server_config.json"
    with open(config_path, 'w') as f:
        json.dump(config, f, indent=2)
    print(f"[config] Saved to {config_path}")

    print("=" * 50)
    print("  DroneGuard MAVLink Backend v4")
    print(f"  Telemetry WS: ws://{WS_HOST}:{ACTUAL_WS_PORT}")
    print(f"  Command WS:   ws://{COMMAND_WS_HOST}:{ACTUAL_COMMAND_WS_PORT}")
    print(f"  Drone:     {DRONE_CONNECTION} @ {DRONE_BAUD} baud")
    print(f"  Stream:    {STREAM_HZ} Hz")
    print(f"  RC stream: {RC_STREAM_HZ} Hz raw PWM")
    print("  Command whitelist:")
    for command_id, script in COMMAND_MAP.items():
        print(f"    {command_id}: {script}")
    print("=" * 50)

    async with (ws_server, cmd_server):
        await asyncio.Future()   # run forever


if __name__ == "__main__":
    asyncio.run(main())
