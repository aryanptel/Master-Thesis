#In this case, we are not using multi-thereading but port forwarding using mavproxy.
#Addtionally, we are implementing an algorithm, discussed in my MS thesis (please read theoretical details from there.).



from pymavlink import mavutil
import matplotlib.pyplot as plt
import time
import numpy as np
import math

#heartbeat and connection
master = mavutil.mavlink_connection('udpin:127.0.0.1:15560')
print(master.wait_heartbeat())

slave1 =  mavutil.mavlink_connection('udpin:127.0.0.1:15590')
print(slave1.wait_heartbeat())

slave2 =  mavutil.mavlink_connection('udpin:127.0.0.1:15570')
print(slave2.wait_heartbeat())



def square(array):
    sq_arry = [x ** 2 for x in array]
    return np.array(sq_arry)

import math

#converting GPS coordinate to reference coordinates (local coordinate frame).
def latlon_to_xy(ref_lat, ref_lon, lat, lon):
    # Radius of the Earth in meters
    R = 6371000  
    
    # Convert latitude and longitude from degrees to radians
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    ref_lat_rad = math.radians(ref_lat)
    ref_lon_rad = math.radians(ref_lon)
    
    # differences between latitudes and longitudes wrt reference coordinates
    dlat = lat_rad - ref_lat_rad
    dlon = lon_rad - ref_lon_rad

    #converting to x and y
    x = R * dlon * math.cos(ref_lat_rad)
    y = R * dlat
    
    return x, y


#MAIN ALGORITHM 

#set reference points
#use master's initial position as refrence positions

#init position master, here we are using master initial position as the local coordinate frames origin.

init_pos_master = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)  #LOCAL_POSITION_NED   #GLOBAL_POSITION_INIT
xmg = init_pos_master.lat * 10**-7
ymg = init_pos_master.lon * 10**-7
#zm = init_pos_master.z
print("master", xmg,ymg)
ref_lat, ref_lon = xmg , ymg
xm, ym = latlon_to_xy(ref_lat, ref_lon, xmg, ymg)
print("master init: ", xm, ym)



#SETTING MAX SPEED for the master drone.
'''master.mav.param_set_send(
    master.target_system,
    master.target_component,
    b'WPNAV_SPEED',
    100,  # Desired maximum velocity in centimeters per second (cm/s)
    mavutil.mavlink.MAV_PARAM_TYPE_INT32
)'''


#INITIAL POSITIONS in global frame slave 1
init_pos_slave1 = slave1.recv_match(type='GLOBAL_POSITION_INT', blocking=True)  #LOCAL_POSITION_NED   #GLOBAL_POSITION_INIT
x1g = init_pos_slave1.lat * 10**-7
y1g = init_pos_slave1.lon * 10**-7
#z1 = init_pos_slave1.z
print("global slave1", x1g, y1g)

#initial position in refrence frame.
x1, y1 = latlon_to_xy(ref_lat, ref_lon, x1g, y1g)
print("slave1 init: ",x1, y1)


#INITIAL POSITIONS in global frame slave 2
init_pos_slave2 = slave2.recv_match(type='GLOBAL_POSITION_INT', blocking=True)  #LOCAL_POSITION_NED   #GLOBAL_POSITION_INIT
x2g = init_pos_slave2.lat * 10**-7
y2g = init_pos_slave2.lon * 10**-7
#z1 = init_pos_slave1.z
print("global slave2", x2g, y2g)

#initial position in refrence frame.

x2, y2 = latlon_to_xy(ref_lat, ref_lon, x2g, y2g)
print("slave2 init: ",x2, y2)


#to stop and verify the initial positions. Press enter after making sure that everthing is fine.
input("wait")


################################
################################
################################

time_series = 1000

xm_values = np.zeros(time_series)
ym_values = np.zeros(time_series)
#zm_values = np.zeros(time_series)

x1_values = np.zeros(time_series)
y1_values = np.zeros(time_series)
#z1_values = np.zeros(time_series)


abs_x1 = abs(x1_values - xm_values)
abs_y1= abs(y1_values - ym_values)
#abs_z1= abs(z1_values - zm_values)

norm1= np.sqrt(square(abs_x1) + square(abs_y1))

x2_values = np.zeros(time_series)
y2_values = np.zeros(time_series)
#z1_values = np.zeros(time_series)


abs_x2 = abs(x2_values - xm_values)
abs_y2= abs(y2_values - ym_values)
#abs_z1= abs(z1_values - zm_values)

norm2 = np.sqrt(square(abs_x2) + square(abs_y2))


#LIVE PLOTTING

plt.ion()
fig1, ax1 = plt.subplots()
line1, = ax1.plot(norm1, label='norm1', lw=0.8)
line2, = ax1.plot(norm2, label='norm2', lw=0.8)
ax1.set_ylim(0,50)
ax1.legend()



xm_values[0] = xm
ym_values[0] = ym
#zm_values[0] = zm

x1_values[0] = x1
y1_values[0] = y1
#z1_values[0] = z1


x2_values[0] = x2
y2_values[0] = y2
#z1_values[0] = z1


#movement using the velocity control master
vx_m = 0
vy_m = 0
vz_m = 0
ya_w = 0
ya_wrate = 0


#INITIAL VELOCITIES

vx_1 = 0
vy_1 = 0
vz_1 = 0

vx_2 = 0
vy_2 = 0
vz_2 = 0


#ODE POSITIONS PARAMTERS

#By changing the values of a_i and b_i, one can make various formation.

'''
#for straight line with both drones at the distance of 5m

a1 = -5
b1 = 0
#c1 = 0

a2 = -5
b2 = 0
#c1 = 0

'''

a1 = -5
b1 = 5
#c1 = 0

a2 = -5
b2 = -5
#c1 = 0



for i in range(1,time_series):

    ##############################################################################################################################################
    #master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10,
    #                master.target_system, master.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b110111000111), 0,0,0, vx_m,vy_m,vz_m, 0,0,0, ya_w, ya_wrate))  #FORMAT: coordinate_frame, input type, x, y, z, vx, vy, vz, ax,ay,az, yaw, yaw_rate

    #receiving master drone's coordinates
    msg_m = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)  #LOCAL_POSITION_NED   #GLOBAL_POSITION_INIT
    
    #print(xm, " ", ym," ", zm ," ", zm )
    #print(vx_m, " ", vy_m," ", vz_m ," ", zm )
    
    xmg =  msg_m.lat * 10**-7
    ymg =  msg_m.lon * 10**-7
    xm, ym = latlon_to_xy(ref_lat, ref_lon, xmg, ymg)
    
    #zm =  msg_m.z
    vx_m =  msg_m.vx
    vy_m =  msg_m.vy
    #vz_m =  msg_m.vz
    print("Master Coordinates: ", xm, ym)
    print("Master Velocity: " , vx_m, vy_m)
    
    vx_m = 0
    vy_m = 0

    damping_factor = 0.3

    coup_x = 0.5
    coup_y = 0.5
    #coup_z = 0.1
    ##############################################################################################################################################

    tmp1 = vx_1
    tmp2 = vy_1
    #tmp3 = vz_1
    #coupling equations 
    vx_1 = vx_m + coup_x*(xm - x1 - a1) + damping_factor*(vx_m - tmp1)

    #print("velocity: ", vx_1, vx_m)
    vy_1 = vy_m + coup_y*(ym - y1 - b1) + damping_factor*(vy_m - tmp2)
    #One can also couple z-coordinates, but never tested this during landing, so if you are testing it on real drones.
    #Please do this caution.
    
    #vz_1 = vz_m + coup_z*(zm - z1 - c1) + damping_factor*(vz_m - tmp3)
    
    #sending coordinates to slave drones. for z, replace 0 with -vz_1 
    slave1.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10,
                   slave1.target_system, slave1.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b110111000111), 0,0,0,vy_1,vx_1,0, 0,0,0, ya_w, ya_wrate))  #FORMAT: coordinate_frame, input type, x, y, z, vx, vy, vz, ax,ay,az, yaw, yaw_rate
    msg1 = slave1.recv_match(type='GLOBAL_POSITION_INT', blocking=True)  #LOCAL_POSITION_NED   #GLOBAL_POSITION_INIT
    
    x1g =  msg1.lat * 10**-7
    y1g =  msg1.lon * 10**-7
    x1, y1 = latlon_to_xy(ref_lat, ref_lon, x1g, y1g)
    #z1 =  msg1.z

    print("Slave Coordinates: ","x", x1,"y", y1)
    print("slave v: ", vx_1, vy_1)


    ##############################################################################################################################################


    tmp1 = vx_2
    tmp2 = vy_2
    tmp3 = vz_2

    vx_2 = vx_m + coup_x*(xm - x2 - a2) + damping_factor*(vx_m - tmp1)
    vy_2 = vy_m + coup_y*(ym - y2 - b2) + damping_factor*(vy_m - tmp2)
    #vz_2 = vz_m + coup_z*(zm - z2 - c2) + damping_factor*(vz_m - tmp3)


    slave2.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10,
                    slave2.target_system, slave2.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, int(0b110111000111), 0,0,0, vy_2,vx_2,0, 0,0,0, ya_w, ya_wrate))  #FORMAT: coordinate_frame, input type, x, y, z, vx, vy, vz, ax,ay,az, yaw, yaw_rate
    msg2 = slave2.recv_match(type='GLOBAL_POSITION_INT', blocking=True)  #LOCAL_POSITION_NED   #GLOBAL_POSITION_INT

    
    x2g =  msg2.lat * 10**-7
    y2g =  msg2.lon * 10**-7
    x2, y2 = latlon_to_xy(ref_lat, ref_lon, x2g, y2g)
    #z1 =  msg1.z

    print("Slave Coordinates: ","x", x2,"y", y2)
    print("slave v: ", vx_2, vy_2)


    print("*****************************************************************")
        ##############################################################################################################################################

    #plotting real-time
    line1.set_ydata(norm1)
    line2.set_ydata(norm2)

    fig1.canvas.draw()
    fig1.canvas.flush_events()

    xm_values[i] = xm
    ym_values[i] = ym
    #zm_values[i] = zm

    x1_values[i] = x1
    y1_values[i] = y1
    #z1_values[i] = z1

    x2_values[i] = x2
    y2_values[i] = y2
    #z1_values[i] = z1

    abs_x1 = abs(x1_values - xm_values)
    abs_y1 = abs(y1_values - ym_values)
    #abs_z1 = abs(z1_values - zm_values)

    abs_x2 = abs(x2_values - xm_values)
    abs_y2 = abs(y2_values - ym_values)
    #abs_z1 = abs(z1_values - zm_values)

    norm1 = np.sqrt(square(abs_x1) + square(abs_y1))
    norm2 = np.sqrt(square(abs_x2) + square(abs_y2))

    time.sleep(0.1)

input("wait")
