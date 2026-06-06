from pymavlink import mavutil
import math
import time

# =====================================================
# CONNECT
# =====================================================

leader = mavutil.mavlink_connection(
    '/dev/serial/by-id/usb-ArduPilot_RadiolinkPIX6_1C003A001951313530353239-if00',
    baud=115200
)

follower = mavutil.mavlink_connection(
    '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
    baud=57600
)

print("Waiting for leader heartbeat...")
leader.wait_heartbeat()

print("Waiting for follower heartbeat...")
follower.wait_heartbeat()

print("Connected to both drones")

# =====================================================
# SETTINGS
# =====================================================

OFFSET_METERS = 5.0

# =====================================================
# HELPER
# =====================================================

def get_offset_position(lat, lon, heading_deg, offset_m):

    heading_rad = math.radians(heading_deg)

    # Behind leader
    north_offset = -offset_m * math.cos(heading_rad)
    east_offset  = -offset_m * math.sin(heading_rad)

    earth_radius = 6378137.0

    dlat = north_offset / earth_radius
    dlon = east_offset / (
        earth_radius *
        math.cos(math.radians(lat))
    )

    new_lat = lat + math.degrees(dlat)
    new_lon = lon + math.degrees(dlon)

    return new_lat, new_lon

# =====================================================
# FOLLOW LOOP
# =====================================================

while True:

    msg = leader.recv_match(
        type='GLOBAL_POSITION_INT',
        blocking=True
    )

    if msg is None:
        continue

    leader_lat = msg.lat / 1e7
    leader_lon = msg.lon / 1e7
    leader_alt = msg.relative_alt / 1000.0
    leader_hdg = msg.hdg / 100.0

    target_lat, target_lon = get_offset_position(
        leader_lat,
        leader_lon,
        leader_hdg,
        OFFSET_METERS
    )

    follower.mav.set_position_target_global_int_send(
        0,
        follower.target_system,
        follower.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,

        0b110111111000,

        int(target_lat * 1e7),
        int(target_lon * 1e7),
        leader_alt,

        0, 0, 0,
        0, 0, 0,
        0, 0
    )

    print(
        f"Leader Hdg={leader_hdg:.1f} "
        f"Target=({target_lat:.7f}, {target_lon:.7f})"
    )

    time.sleep(1)