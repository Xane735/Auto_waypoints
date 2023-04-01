
import time
from pymavlink import mavutil
from pymavlink import mavwp

def mission_start():
    # to Start the mission
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, 300 , 0, 0, 0, 0, 0, 0, 0, 0)
    msn_str = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msn_str)
    print("MISSION STARTING")

def arm():
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, 400, 1, 0, 0, 0, 0, 0, 0, 0)
    is_arm = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(is_arm)
    time.sleep(2)
    print("DRONE IS ARMED")

def disarm():
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, 400 ,0, 0, 0, 0, 0, 0, 0, 0)
    is_disarm = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(is_disarm)
    time.sleep(2)
    print("DRONE IS DISARMED")

def write_pixhawk():
    # Enter the WayPoint file path 
    with open("waypoints.waypoints") as f:
        next(f)     # Skip the first line
        for line in f:
            fields = line.strip().split("\t")

    #line_num = 0
    #for line in f:                   ******* USE THIS CODE IF ABOVE DOESNT WORK *******
        #if line_num == 0:
            #line_num += 1
            #continue  # Skip the first line

            # Defining each field in the file
            seq = int(fields[0])
            current = int(fields[1])
            frame = int(fields[2])
            command = int(fields[3])
            param_1 = int(fields[4])
            param_2 = int(fields[5])
            param_3 = int(fields[6])
            param_4 = int(fields[7])
            latitude = float(fields[8])
            longitude = float(fields[9])
            altitude = float(fields[10])
            print(f"Waypoint: ({latitude}, {longitude}, {altitude})") # Just for debugging, can be removed right after
            # Writing the parameters into the pixhawk 
            waypoint = the_connection.mavlink.MAVLink_mission_item_message(
                0,         # target system
                0,         # target component
                seq,       
                frame,     
                command,   
                current,  
                0,         # autocontinue
                param_1,   
                param_2,   
                param_3,   
                param_4,   
                longitude,  
                latitude,   
                altitude)
            the_connection.send(waypoint)
            print("Waypoint is written into the pixhawk")
    print("ALL WAYPOINTS WRITTEN INTO PIXHAWK")

def auto_mode():
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, 176, 0, 1, 3, 0, 0, 0, 0, 0)
    auto = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(auto)
    print("DRONE SET TO AUTO MODE")

# to start a connection listening on a UDP port
the_connection = mavutil.mavlink_connection('udpin:localhost:14551')

# Waiting for first heartbeat message 
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

# Connecting to the autopilot
master = mavutil.mavlink_connection('udpout:localhost:14550')

# Connecting to Pixhawk
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
write_pixhawk()
arm()
auto_mode()
mission_start()

# Waiting for the mission to end
while True:
    msg = master.recv_match(type='MISSION_ITEM_REACHED', blocking=True)
    if not msg:
        continue
    else:
        print("MISSION COMPLETED SUCESSFULLY")
        break
disarm()

# Ending connection
the_connection.close()