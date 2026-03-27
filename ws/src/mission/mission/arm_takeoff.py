#!/usr/bin/env python3
import time
from pymavlink import mavutil


def set_param(master, param_id, value, param_type):
    """
    Sends a PARAM_SET message to PX4.
    param_id should be a bytes literal.
    param_type should be one of the mavlink.MAV_PARAM_TYPE_* constants.
    """
    print(
        f"Setting parameter {param_id.decode('utf-8')} to {value} (type {param_type})"
    )
    master.mav.param_set_send(
        master.target_system, master.target_component, param_id, value, param_type
    )


def force_arm(master):
    """
    Force arm the vehicle using MAV_CMD_COMPONENT_ARM_DISARM with the force flag.
    """
    print("Forcing arm...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # confirmation
        1,  # param1: 1 = arm
        21196,  # param2: force flag to bypass pre-arm checks
        0,
        0,
        0,
        0,
        0,
    )


def send_takeoff(master, altitude):
    """
    Send the takeoff command (NAV_TAKEOFF) to PX4.
    For local takeoff, param7 is the target altitude in meters.
    """
    print(f"Sending takeoff command for altitude {altitude} m...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,  # confirmation
        0,
        0,
        0,
        0,
        0,
        0,
        altitude,  # target altitude in meters
    )


def main():
    # Use the UDP connection string that worked for you.
    connection_string = "udp:127.0.0.1:14550"
    print("Connecting to MAVLink at:", connection_string)
    master = mavutil.mavlink_connection(connection_string, source_system=1)

    print("Waiting for heartbeat...")
    if not master.wait_heartbeat(timeout=30):
        print("No heartbeat received. Exiting.")
        return
    print("Heartbeat received from system:", master.target_system)

    # --- Override key parameters ---
    # Set SYS_HAS_GPS to 1 so PX4 thinks a GPS is present.
    set_param(master, b"SYS_HAS_GPS", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    time.sleep(1)
    # Disable EKF yaw drift check by setting EKF2_REQ_HDRIFT to 0.
    set_param(master, b"EKF2_REQ_HDRIFT", 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    time.sleep(1)
    # Allow arming without GPS fix.
    set_param(master, b"COM_ARM_WO_GPS", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)

    # --- Force Arm ---
    armed = False
    taken_off = False
    while armed != 128 or not taken_off:
        force_arm(master)
        armed = master.motors_armed()
        print("Motors armed?", armed)
        time.sleep(3)

        # --- Send Takeoff Command ---
        desired_altitude = 10  # target altitude in meters
        send_takeoff(master, desired_altitude)
        time.sleep(3)


if __name__ == "__main__":
    main()
