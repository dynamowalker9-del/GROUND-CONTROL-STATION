from pymavlink import mavutil
import time

# Leader PIX6
leader = mavutil.mavlink_connection(
    '/dev/serial/by-id/usb-ArduPilot_RadiolinkPIX6_1C003A001951313530353239-if00',
    baud=115200
)

# Follower Mini Pix
follower = mavutil.mavlink_connection(
    '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
    baud=57600
)

print("Waiting for leader heartbeat...")
leader.wait_heartbeat()

print("Waiting for follower heartbeat...")
follower.wait_heartbeat()

print("Connected to both drones")

# Get RTL mode IDs
leader_rtl = leader.mode_mapping()['RTL']
follower_rtl = follower.mode_mapping()['RTL']

print("Sending RTL to Leader...")

leader.mav.set_mode_send(
    leader.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    leader_rtl
)

time.sleep(1)

print("Sending RTL to Follower...")

follower.mav.set_mode_send(
    follower.target_system,
    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
    follower_rtl
)

print("RTL command sent to both drones")