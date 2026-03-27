#!/usr/bin/env python3
import time
from pymavlink import mavutil


def main():

    for i in range(0, 3):
        connection_string = "udp:127.0.0.1:14550"
        print("Connecting to MAVLink at:", connection_string)
        master = mavutil.mavlink_connection(connection_string, source_system=1)

        print("Waiting for heartbeat...")
        if not master.wait_heartbeat(timeout=30):
            print("No heartbeat received. Exiting.")
            return
        print("Heartbeat received from system:", master.target_system)

        # --- Optional: Monitor for status messages for a short time ---
        print("Monitoring MAVLink messages for 10 seconds...")
        start = time.time()
        while time.time() - start < 10:
            msg = master.recv_match(blocking=True, timeout=2)
            if msg:
                print(msg)


if __name__ == "__main__":
    main()
