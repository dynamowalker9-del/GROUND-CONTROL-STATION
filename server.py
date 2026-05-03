
 
import asyncio
import json
import math
import os
import threading
import time

import websockets
from pymavlink import mavutil

DRONE_CONNECTION = os.getenv("DRONE_CONNECTION", "COM18")
DRONE_BAUD = int(os.getenv("DRONE_BAUD", "9600"))
WS_HOST = os.getenv("WS_HOST", "0.0.0.0")
WS_PORT = int(os.getenv("WS_PORT", "8765"))
STREAM_HZ = int(os.getenv("STREAM_HZ", "20"))


COPTER_MODES = {
    0: "STABILIZE",  1: "ACRO",        2: "ALT_HOLD",     3: "AUTO",
    4: "GUIDED",     5: "LOITER",      6: "RTL",           7: "CIRCLE",
    9: "LAND",      11: "DRIFT",      13: "SPORT",        14: "FLIP",
   15: "AUTOTUNE",  16: "POSHOLD",    17: "BRAKE",        18: "THROW",
   19: "AVOID_ADSB",20: "GUIDED_NOGPS",21: "SMART_RTL",
}
MODE_NAME_TO_ID = {v: k for k, v in COPTER_MODES.items()}

print(f"Connecting to {DRONE_CONNECTION} @ {DRONE_BAUD} baud ...")
master = mavutil.mavlink_connection(
    DRONE_CONNECTION, baud=DRONE_BAUD,
    autoreconnect=True,
    source_system=255,    
    source_component=0,
)
master.wait_heartbeat(timeout=30)
print(f"Heartbeat received — SysID={master.target_system}  CompID={master.target_component}")


def request_streams():
    """Ask ArduPilot to stream all telemetry at the configured rate."""
    if master.target_system == 0:
        return   # not connected yet
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        STREAM_HZ,
        1,    # start
    )
    print(f"Data streams requested @ {STREAM_HZ} Hz")

request_streams()


telem = {
    # Attitude
    "raw_roll":  0.0,  "raw_pitch": 0.0,  "raw_yaw": 0.0,
    # Gyro (real angular rates from ATTITUDE message)
    "raw_gx":    0.0,  "raw_gy":    0.0,  "raw_gz":  0.0,
    # Accelerometer
    "raw_ax":    0.0,  "raw_ay":    0.0,  "raw_az":  9.8,
    # GPS position — None until a valid fix arrives
    "lat":       None, "lon":       None,
    "alt_msl":   0.0,  "alt_rel":   0.0,
    "gps_fix":   0,    "satellites": 0,   "hdop": 99.0,
    # Velocity
    "groundspeed": 0.0, "airspeed":   0.0,
    "heading_deg": 0.0, "vspeed":     0.0,
    # Battery
    "batt_voltage": 0.0, "batt_current": 0.0, "batt_pct": -1,
    # RC Channels (transmitter inputs) — 1500 = centre
    "rc1": 0, "rc2": 0, "rc3": 0, "rc4": 0,
    "rc5": 0, "rc6": 0, "rc7": 0, "rc8": 0,
    "rc_rssi": 0,
    # Status
    "flight_mode": "UNKNOWN", "armed": False, "ekf_ok": False,
    # Timestamp
    "ts": 0,
    "connection": DRONE_CONNECTION,
    "baud": DRONE_BAUD,
    "stream_hz": STREAM_HZ,
}
lock = threading.Lock()


def valid_coord(latitude, longitude):
    """Reject unset or impossible MAVLink coordinates before sending them to the map."""
    return (
        latitude is not None
        and longitude is not None
        and -90 <= latitude <= 90
        and -180 <= longitude <= 180
        and (abs(latitude) > 0.001 or abs(longitude) > 0.001)
    )



def reader():
    last_stream_req = time.time()

    while True:
        
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
                # Real gyro rates (rad/s) — use these for the gyro graphs
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
                telem["rc1"] = msg.chan1_raw
                telem["rc2"] = msg.chan2_raw
                telem["rc3"] = msg.chan3_raw
                telem["rc4"] = msg.chan4_raw
                telem["rc5"] = msg.chan5_raw
                telem["rc6"] = msg.chan6_raw
                telem["rc7"] = msg.chan7_raw
                telem["rc8"] = msg.chan8_raw
                telem["rc_rssi"] = msg.rssi

            elif mtype == "RC_CHANNELS_RAW":
                # Fallback for older firmwares that send RC_CHANNELS_RAW
                telem["rc1"] = msg.chan1_raw
                telem["rc2"] = msg.chan2_raw
                telem["rc3"] = msg.chan3_raw
                telem["rc4"] = msg.chan4_raw
                telem["rc5"] = msg.chan5_raw
                telem["rc6"] = msg.chan6_raw
                telem["rc7"] = msg.chan7_raw
                telem["rc8"] = msg.chan8_raw
                telem["rc_rssi"] = msg.rssi

            elif mtype == "SERVO_OUTPUT_RAW":
                # Also capture servo outputs as fallback RC display
                # Only use if no RC_CHANNELS data has arrived yet
                if telem["rc1"] == 0:
                    telem["rc1"] = msg.servo1_raw
                    telem["rc2"] = msg.servo2_raw
                    telem["rc3"] = msg.servo3_raw
                    telem["rc4"] = msg.servo4_raw
                    telem["rc5"] = getattr(msg, 'servo5_raw', 0)
                    telem["rc6"] = getattr(msg, 'servo6_raw', 0)
                    telem["rc7"] = getattr(msg, 'servo7_raw', 0)
                    telem["rc8"] = getattr(msg, 'servo8_raw', 0)


threading.Thread(target=reader, daemon=True, name="mav-reader").start()



async def handle_command(raw: str):
    
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
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )
        print(f"[cmd] SET_MODE → {mode_name} (id={mode_id})")

    elif action == "RTL":
        mode_id = MODE_NAME_TO_ID.get("RTL", 6)
        master.mav.set_mode_send(
            master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )
        print("[cmd] RTL")

    elif action == "TAKEOFF":
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


def _arm(arm: bool):
    """Send arm/disarm via MAV_CMD_COMPONENT_ARM_DISARM (works on all pymavlink versions)."""
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
    while True:
        await asyncio.sleep(0.05)
        with lock:
            payload = dict(telem)
        try:
            await ws.send(json.dumps(payload))
        except websockets.exceptions.ConnectionClosed:
            break
        except Exception as e:
            # Transient errors — keep trying instead of silently dying
            print(f"[send] warning: {e}")
            await asyncio.sleep(0.1)


async def _gcs_heartbeat():
    """Send a GCS heartbeat every second so ArduPilot knows a GCS is active."""
    while True:
        await asyncio.sleep(1.0)
        try:
            master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0,
            )
        except Exception:
            pass



async def ws_handler(ws):
    addr = getattr(ws, "remote_address", "?")
    print(f"[ws] + connected: {addr}")
    sender = asyncio.create_task(_send_loop(ws))
    try:
        async for message in ws:
            await handle_command(message)
    except websockets.exceptions.ConnectionClosed:
        pass
    except Exception as e:
        print(f"[ws] handler error: {e}")
    finally:
        sender.cancel()
        print(f"[ws] - disconnected: {addr}")


async def main():
    
    asyncio.create_task(_gcs_heartbeat())

    print("=" * 50)
    print("  DroneGuard MAVLink Backend v4")
    print(f"  WebSocket: ws://{WS_HOST}:{WS_PORT}")
    print(f"  Drone:     {DRONE_CONNECTION} @ {DRONE_BAUD} baud")
    print(f"  Stream:    {STREAM_HZ} Hz")
    print("=" * 50)

    async with websockets.serve(ws_handler, WS_HOST, WS_PORT):
        await asyncio.Future()   # run forever


if __name__ == "__main__":
    asyncio.run(main())
