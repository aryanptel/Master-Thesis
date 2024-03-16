from pymavlink import mavutil
import threading
import time

def connect_to_mavproxy(address, port):
    mav = mavutil.mavlink_connection(f'udpin:{address}:{port}')
    mav.wait_heartbeat()
    print(f"Heartbeat from the system (system {mav.target_system}, component {mav.target_component})")
    return mav

def perform_flight_operations(mav):
    print("Mode")
    mav.mav.set_mode_send(mav.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4)
    mode_ack = mav.recv_match(type='COMMAND_ACK', blocking=True)
    print(mode_ack)

    time.sleep(0.5)

    print("Arming")
    mav.mav.command_long_send(mav.target_system, mav.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    arm_ack = mav.recv_match(type='COMMAND_ACK', blocking=True)
    print(arm_ack)

    time.sleep(0.5)

    print("Takeoff")
    initial_altitude = 7
    mav.mav.command_long_send(mav.target_system, mav.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, initial_altitude)
    takeoff_ack = mav.recv_match(type='COMMAND_ACK', blocking=True)
    print(takeoff_ack.result)

    time.sleep(1)

# Define the addresses and ports for each connection
connections = [('127.0.0.1', 15560), ('127.0.0.1', 15570), ('127.0.0.1', 15590)]

# Create and start a thread for each connection
threads = []
for address, port in connections:
    mav = connect_to_mavproxy(address, port)
    thread = threading.Thread(target=perform_flight_operations, args=(mav,))
    thread.start()
    threads.append(thread)

# Wait for all threads to finish
for thread in threads:
    thread.join()

