from pymavlink import mavutil
import time

# Leader PIX6 (READ ONLY)
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

print("Both drones connected")

triggered = False


def set_guided(vehicle):
    mode = vehicle.mode_mapping()['GUIDED']

    vehicle.mav.set_mode_send(
        vehicle.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode
    )

    print("Follower -> GUIDED")
    time.sleep(2)


def arm(vehicle):

    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        21196,  # force arm
        0, 0, 0, 0, 0
    )

    print("Follower -> ARM")
    time.sleep(3)


def takeoff(vehicle, altitude=10):

    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        altitude
    )

    print(f"Follower -> TAKEOFF {altitude}m")


while True:

    msg = leader.recv_match(
        type='HEARTBEAT',
        blocking=True
    )

    leader_armed = bool(
        msg.base_mode &
        mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    )

    if leader_armed and not triggered:

        print("Leader armed detected")

        set_guided(follower)

        arm(follower)

        takeoff(follower, 10)

        triggered = True

    elif not leader_armed:

        triggered = False

    time.sleep(0.1)