#!/usr/bin/env python3
import time
from pymavlink import mavutil

def main():
    # Use the same UDP connection parameters that work for you.
    connection_string = "udpin:0.0.0.0:14550"
    udp_output = "127.0.0.1:14550"
    print("Connecting to MAVLink using:", connection_string, "with udp_output:", udp_output)
    master = mavutil.mavlink_connection(connection_string, source_system=1, udp_output=udp_output)
    
    print("Waiting for heartbeat...")
    if not master.wait_heartbeat(timeout=30):
        print("No heartbeat received. Exiting.")
        return
    print("Heartbeat received from system:", master.target_system)
    
    # Record the start time to compute relative time_boot_ms.
    start_time = time.time()
    
    print("Publishing LOCAL_POSITION_NED messages at 50 Hz for 20 seconds...")
    duration = 20  # seconds
    while time.time() - start_time < duration:
        # Compute time_boot_ms as elapsed time since start.
        time_boot_ms = int((time.time() - start_time) * 1000)
        
        # Sample values; adjust these as needed.
        x  = -10.662840843200684
        y  = -4.222443580627441
        z  = -6.304474830627441
        vx = -1.1831903457641602
        vy = -1.1139914989471436
        vz =  0.5732097029685974
        
        # Send the LOCAL_POSITION_NED message with relative time.
        master.mav.local_position_ned_send(
            time_boot_ms,
            x, y, z,
            vx, vy, vz
        )
        time.sleep(0.02)  # 50 Hz
    
    print("Finished publishing LOCAL_POSITION_NED messages.")

if __name__ == "__main__":
    main()
