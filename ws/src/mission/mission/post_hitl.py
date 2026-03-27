#!/usr/bin/env python3
import time
import sys, termios, tty, select, threading
from pymavlink import mavutil
import signal

# Global GPS values; they will be updated by the key-capture thread.
lat = 473979694  # Latitude (WGS84) [degE7]
lon = 85461644  # Longitude (WGS84) [degE7]

# Flag to control thread termination
running = True
old_settings = None

def restore_terminal():
    """Restore terminal settings"""
    global old_settings
    if old_settings:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print("\nTerminal settings restored")

def signal_handler(sig, frame):
    """Handle signals like SIGINT (Ctrl+C)"""
    global running
    print("\nSignal received, exiting gracefully...")
    running = False
    restore_terminal()
    sys.exit(0)

def key_capture():
    """Continuously capture key presses at a high frequency and update lat/lon."""
    global lat, lon, old_settings, running
    
    # Save terminal settings
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())
    
    try:
        # Run loop until running flag is False
        while running:
            # Use a very short timeout for select for high-frequency capture
            rlist, _, _ = select.select([sys.stdin], [], [], 0.001)
            if rlist:
                key = sys.stdin.read(1)
                if key == "a":
                    lat += 10
                    print("Increased latitude:", lat)
                elif key == "d":
                    lat -= 10
                    print("Decreased latitude:", lat)
                elif key == "w":
                    lon += 10
                    print("Increased longitude:", lon)
                elif key == "s":
                    lon -= 10
                    print("Decreased longitude:", lon)
                elif key == "\x03":  # Ctrl+C
                    running = False
                    break
            # Small sleep to yield control and avoid high CPU usage
            time.sleep(0.001)
    except Exception as e:
        print(f"\nError in key capture: {e}")
    finally:
        restore_terminal()

def main():
    global lat, lon, running  # Use the global variables
    
    # Register signal handler for SIGINT (Ctrl+C)
    signal.signal(signal.SIGINT, signal_handler)

    # Connection setup
    connection = "udp:127.0.0.1:14550"
    print("Connecting to MAVLink at:", connection)
    master = mavutil.mavlink_connection(connection, source_system=1)

    print("Waiting for heartbeat...")
    if not master.wait_heartbeat(timeout=30):
        print("No heartbeat received. Exiting.")
        return
    print("Heartbeat received from system:", master.target_system)

    # Other GPS parameters remain unchanged
    rate = 10.0
    interval = 1.0 / rate
    time_usec = int(time.time() * 1e6)
    fix_type = 3
    alt = -1130  # Altitude (MSL) [mm]
    eph = 0
    epv = 0
    vel = 0
    vn = 0
    ve = 0
    vd = 0
    cog = 65535
    satellites_visible = 255
    gps_id = 0  # Renamed from 'id' to avoid conflict with built-in id()
    yaw = 36000

    # Start the key capture thread
    key_thread = threading.Thread(target=key_capture, daemon=True)
    key_thread.start()

    print("Starting real-time HIL_GPS message sending.")
    print("Press 'a' to increase latitude, 'd' to decrease latitude, 'w' to increase longitude, and 's' to decrease longitude.")
    print("Press Ctrl+C to exit.")

    try:
        while running:
            time_usec = int(time.time() * 1e6)
            master.mav.hil_gps_send(
                time_usec,
                fix_type,
                lat,
                lon,
                alt,
                eph,
                epv,
                vel,
                vn,
                ve,
                vd,
                cog,
                satellites_visible,
                gps_id,
                yaw,
            )

            msg = master.recv_match(blocking=False, timeout=0.1)
            if msg is None:
                continue

            msg_type = msg.get_type()
            # Check for EKF innovations (names may vary with firmware version)
            if msg_type in ["EKF2_INNOVATIONS", "EKF2_INNOV"]:
                print("EKF Innovations:", msg.to_dict())
            elif msg_type in ["ESTIMATOR_STATUS"]:
                print("Estimator Status:", msg.to_dict())
            elif msg_type in ["SENSOR_BIAS", "ODOMETRY"]:
                print(f"{msg_type}:", msg.to_dict())

            time.sleep(interval)
    except Exception as e:
        print(f"\nError in main loop: {e}")
    finally:
        running = False
        restore_terminal()
        print("\nExiting...")

if __name__ == "__main__":
    main()