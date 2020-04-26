#!/usr/bin/env python
""" Drone Pilot - Control of MRUAV """
""" simulation-slung-load.py: Script that simulates neural network output."""

__author__ = "Aldo Vargas"
__copyright__ = "Copyright 2016 Altax.net"

__license__ = "GPL"
__version__ = "1.0"
__maintainer__ = "Aldo Vargas"
__email__ = "alduxvm@gmail.com"
__status__ = "Development"

import time, datetime, csv, threading
from modules.utils import *
from modules.pyMultiwii import MultiWii
import modules.UDPserver as udp

import pandas as pd
import numpy as np
import modules.pyrenn as prn

# Load data
#data = pd.read_csv('sim-data/sl-neural-bptt-500.csv',sep=',',header=0)
#data = pd.read_csv('modules/sim-data/sl-c-inf15-1.csv',sep=',',header=0)
data = pd.read_csv('modules/sim-data/step-control-nofilter2.csv',sep=',',header=0)

# Main configuration
logging = True
update_rate = 0.01 # 100 hz loop cycle
vehicle_weight = 0.84 # Kg
sl_weight = 0.1 # Kg
u0 = 1000 # Zero throttle command
uh = 1360 # Hover throttle command
kt = vehicle_weight * g / (uh-u0)
kt_sl = (vehicle_weight + sl_weight) * g / (uh-u0)
ky = 500 / pi # Yaw controller gain

# MRUAV initialization
#vehicle = MultiWii("/dev/ttyUSB0")

# Position coordinates [x, y, x] 
desiredPos = {'x':0.0, 'y':0.0, 'z':1.8} # Set at the beginning (for now...)
currentPos = {'x':0.0, 'y':0.0, 'z':0.0} # It will be updated using UDP
sl_currentPos = {'x':0.0, 'y':0.0, 'z':0.0} # It will be updated using UDP

# Velocity
velocities = {'x':0.0, 'y':0.0, 'z':0.0, 'fx':0.0, 'fy':0.0, 'fz':0.0}

# Initialize RC commands and pitch/roll to be sent to the MultiWii 
rcCMD = [1500,1500,1500,1000]
desiredRoll = desiredPitch = desiredYaw = 1500
desiredThrottle = 1000

# Controller PID's gains (Gains are considered the same for pitch and roll)
p_gains =  {'kp': 2.61, 'ki':0.57, 'kd':3.41, 'iMax':2, 'filter_bandwidth':50} # Position Controller gains
h_gains =  {'kp': 4.64, 'ki':1.37, 'kd':4.55, 'iMax':2, 'filter_bandwidth':50} # Height Controller gains
y_gains =  {'kp': 1.0,  'ki':0.0,  'kd':0.0,  'iMax':2, 'filter_bandwidth':50} # Yaw Controller gains
sl_gains = {'kp': 0.3,  'ki':0.0,  'kd':0.0,  'iMax':2, 'filter_bandwidth':10} # Slung load controller gains (slower than position)

# PID modules initialization
rollPID =   PID(p_gains['kp'], p_gains['ki'], p_gains['kd'], p_gains['filter_bandwidth'], 0, 0, update_rate, p_gains['iMax'], -p_gains['iMax'])
pitchPID =  PID(p_gains['kp'], p_gains['ki'], p_gains['kd'], p_gains['filter_bandwidth'], 0, 0, update_rate, p_gains['iMax'], -p_gains['iMax'])
heightPID = PID(h_gains['kp'], h_gains['ki'], h_gains['kd'], h_gains['filter_bandwidth'], 0, 0, update_rate, h_gains['iMax'], -h_gains['iMax'])
yawPID =    PID(y_gains['kp'], y_gains['ki'], y_gains['kd'], y_gains['filter_bandwidth'], 0, 0, update_rate, y_gains['iMax'], -y_gains['iMax'])
rPIDvalue = pPIDvalue = yPIDvalue = hPIDvalue = 0.0

# PID for slung load
slx_posPID = PID(sl_gains['kp'], sl_gains['ki'], sl_gains['kd'], sl_gains['filter_bandwidth'], 0, 0, update_rate, sl_gains['iMax'], -sl_gains['iMax'])
sly_posPID = PID(sl_gains['kp'], sl_gains['ki'], sl_gains['kd'], sl_gains['filter_bandwidth'], 0, 0, update_rate, sl_gains['iMax'], -sl_gains['iMax'])
sl_xPIDvalue = sl_yPIDvalue = 0.0

# Filters initialization
f_yaw   = low_pass(20,update_rate)
f_pitch = low_pass(20,update_rate)
f_roll  = low_pass(20,update_rate)

# Desired positions filters
f_desx  = low_pass(10,update_rate)
f_desy  = low_pass(10,update_rate)

# Calculate velocities
vel_x = velocity(20,update_rate)
vel_y = velocity(20,update_rate)
vel_z = velocity(20,update_rate)

# Function to update commands and attitude to be called by a thread
def control():
    global vehicle, rcCMD
    global rollPID, pitchPID, heightPID, yawPID
    global desiredPos, currentPos, velocities
    global desiredRoll, desiredPitch, desiredThrottle
    global rPIDvalue, pPIDvalue, yPIDvalue
    global f_yaw, f_pitch, f_roll, f_desx, f_desy
    global vel_x, vel_y, vel_z
    global sl_currentPos, sl_xPIDvalue, sl_yPIDvalue, slx_posPID, sly_posPID

    # Start simulation messages
    print "\n\nStarting simulation!!\n"
    print "Simulation size -> %d" % (data.shape[0])
    print "Estimated completion time -> %0.2f minutes" % ((data.shape[0]*update_rate)/60)
    time.sleep(3)

    # Start Neural network
    network = 'modules/networks/sl_rtrl_step_control.csv'

    print "Starting \'%s\' neural network..." % (network)
    net = prn.loadNN(network)
    # Array of inputs to network 
    memory = 100
    inputs = np.zeros((10,memory))
    print "Neural network active!"
    time.sleep(1)

    #while True:
    #    if udp.active:
    #        print "UDP server is active..."
    #        break
    #    else:
    #        print "Waiting for UDP server to be active..."
    #    time.sleep(0.5)

    try:
        if logging:
            st = datetime.datetime.fromtimestamp(time.time()).strftime('%m_%d_%H-%M-%S')+".csv"
            f = open("logs/ground-test-"+st, "w")
            logger = csv.writer(f)
            # V -> vehicle | P -> pilot (joystick) | D -> desired position 
            # M -> motion capture | C -> commanded controls | sl -> Second marker | Mode 
            logger.writerow(('timestamp','Vroll','Vpitch','Vyaw','Proll','Ppitch','Pyaw','Pthrottle', \
                             'x','y','z','Dx','Dy','Dz','Mroll','Mpitch','Myaw','Mode','Croll','Cpitch','Cyaw','Cthrottle', \
                             'slx','sly','slz','slr','slp','sly', \
                             'vel_x', 'vel_fx', 'vel_y', 'vel_fy', 'vel_z', 'vel_fz', \
                             'NNslX','NNslY','NNslZ' ))

        # Index for scrolling the data csv file
        data_index = 0

        while True:
            # Variable to time the loop
            current = time.time()
            elapsed = 0

            # Update joystick commands from UDP communication, order (roll, pitch, yaw, throttle)
            #for channel in range(0, 4):
                #rcCMD[channel] = int(udp.message[channel])
            rcCMD[0] = int(data['Proll'][data_index])
            rcCMD[1] = int(data['Ppitch'][data_index])
            rcCMD[2] = int(data['Pyaw'][data_index])
            rcCMD[3] = int(data['Pthrottle'][data_index])

            # Coordinate map from Optitrack in the MAST Lab: X, Y, Z. NED: If going up, Z is negative. 
            ######### WALL ########
            #Door      | x+       |
            #          |          |
            #          |       y+ |
            #---------------------|
            # y-       |          |
            #          |          |
            #        x-|          |
            #######################
            # Update current position of the vehicle
            currentPos['x'] = data['x'][data_index]#udp.message[5]
            currentPos['y'] = data['y'][data_index]#udp.message[6]
            currentPos['z'] = data['z'][data_index]#-udp.message[7]

            # Update position of the slung load
            sl_currentPos['x'] = data['slx'][data_index]#udp.message[8]
            sl_currentPos['y'] = data['slx'][data_index]#udp.message[9]

            # Get velocities of the vehicle
            velocities['x'],velocities['fx'] = vel_x.get_velocity(currentPos['x'])
            velocities['y'],velocities['fy'] = vel_y.get_velocity(currentPos['y'])
            velocities['z'],velocities['fz'] = vel_z.get_velocity(currentPos['z'])

            # Update vehicle Attitude 
            #vehicle.getData(MultiWii.ATTITUDE)

            # Slung load PID calculation, the relative position of the vehicle vs the slung load
            sl_xPIDvalue = slx_posPID.update(sl_currentPos['x'] - currentPos['x'])
            sl_yPIDvalue = sly_posPID.update(sl_currentPos['y'] - currentPos['y'])

            # Desired position changed using slung load movements
            if int(data['Mode'][data_index]) == 1:#udp.message[4] == 1:  
                desiredPos['x'] = 0.0
                desiredPos['y'] = 0.0
            if int(data['Mode'][data_index]) == 2:#udp.message[4] == 2:
                desiredPos['x'] = limit(f_desx.update(sl_xPIDvalue), -1.0, 1.0)
                desiredPos['y'] = limit(f_desy.update(sl_yPIDvalue), -1.0, 1.0)

            # Filter new values before using them
            heading = f_yaw.update(data['Myaw'][data_index])#udp.message[12])

            # PID updating, Roll is for Y and Pitch for X, Z is negative
            rPIDvalue = rollPID.update(  desiredPos['y'] - currentPos['y'])
            pPIDvalue = pitchPID.update( desiredPos['x'] - currentPos['x'])
            hPIDvalue = heightPID.update(desiredPos['z'] - currentPos['z'])
            yPIDvalue = yawPID.update(0.0 - heading)
            
            # Heading must be in radians, MultiWii heading comes in degrees, optitrack in radians
            sinYaw = sin(heading)
            cosYaw = cos(heading)

            # Conversion from desired accelerations to desired angle commands
            desiredRoll  = toPWM(degrees( (rPIDvalue * cosYaw + pPIDvalue * sinYaw) * (1 / g) ),1)
            desiredPitch = toPWM(degrees( (pPIDvalue * cosYaw - rPIDvalue * sinYaw) * (1 / g) ),1)
            desiredThrottle = ((hPIDvalue + g) * vehicle_weight) / (cos(f_pitch.update(radians(data['Vroll'][data_index])))*cos(f_roll.update(radians(data['Vpitch'][data_index]))))
            if int(data['Mode'][data_index]) == 2:
                desiredThrottle = round((desiredThrottle / kt_sl) + u0, 0)
            else:
                desiredThrottle = round((desiredThrottle / kt) + u0, 0)
            desiredYaw = round(1500 - (yPIDvalue * ky), 0)

            # Limit commands for safety
            if int(data['Mode'][data_index]) == 1:
                rcCMD[0] = limit(desiredRoll,1200,1800)
                rcCMD[1] = limit(desiredPitch,1200,1800)
                rcCMD[2] = limit(desiredYaw,1000,2000)
                rcCMD[3] = limit(desiredThrottle,1000,2000)
                slx_posPID.resetIntegrator()
                sly_posPID.resetIntegrator()
                mode = 'Auto'
            elif int(data['Mode'][data_index]) == 2:
                #rcCMD[0] = limit(desiredRoll,1200,1800)
                #rcCMD[1] = limit(desiredPitch,1200,1800)
                #rcCMD[2] = limit(desiredYaw,1000,2000)
                #rcCMD[3] = limit(desiredThrottle,1000,2000)
                mode = 'SlungLoad'
            else:
                # Prevent integrators/derivators to increase if they are not in use
                rollPID.resetIntegrator()
                pitchPID.resetIntegrator()
                heightPID.resetIntegrator()
                yawPID.resetIntegrator()
                slx_posPID.resetIntegrator()
                sly_posPID.resetIntegrator()
                mode = 'Manual'
            rcCMD = [limit(n,1000,2000) for n in rcCMD]

            # Neural network update
            # Order of the inputs -> vehicle roll, vehicle pitch, vehicle yaw, x, y, z, roll, pitch, yaw, throttle
            inputs = np.delete(inputs,0,1) # Delete the oldest element on the array
            # Add new column of data to the front of the stack
            inputs = np.column_stack((inputs, \
                [data['Vroll'][data_index], data['Vpitch'][data_index], data['Vyaw'][data_index], \
                 currentPos['x'], currentPos['y'], currentPos['z'], \
                 rcCMD[0], rcCMD[1], rcCMD[2], rcCMD[3] ]))
            # Get output of neural network
            outputs = prn.NNOut(inputs,net)

            # Send commands to vehicle
            #vehicle.sendCMD(8,MultiWii.SET_RAW_RC,rcCMD)
            udp.message[0] = data['Proll'][data_index]
            udp.message[1] = data['Ppitch'][data_index]
            udp.message[2] = data['Pyaw'][data_index]
            udp.message[3] = data['Pthrottle'][data_index]
            udp.message[4] = data['Mode'][data_index]

            udp.message[11] = data['Mroll'][data_index]
            udp.message[13] = data['Mpitch'][data_index]
            udp.message[12] = data['Myaw'][data_index]

            udp.message[8] = data['slx'][data_index]
            udp.message[9] = data['sly'][data_index]
            udp.message[10] = data['slz'][data_index]
            udp.message[14] = data['slr'][data_index]
            udp.message[15] = data['slp'][data_index]
            udp.message[16] = data['sly'][data_index]

            row =   (time.time(), \
                    data['Vroll'][data_index], data['Vpitch'][data_index], data['Vyaw'][data_index], \
                    udp.message[0], udp.message[1], udp.message[2], udp.message[3], \
                    currentPos['x'], currentPos['y'], currentPos['z'], desiredPos['x'], desiredPos['y'], desiredPos['z'], \
                    udp.message[11], udp.message[13], udp.message[12], \
                    udp.message[4], \
                    rcCMD[0], rcCMD[1], rcCMD[2], rcCMD[3], \
                    udp.message[8], udp.message[9], udp.message[10], udp.message[14],udp.message[15], udp.message[16], \
                    velocities['x'], velocities['fx'], velocities['y'], velocities['fy'], velocities['z'], velocities['fz'], \
                    # The output we care about is the latest....
                    outputs[0,memory-1], outputs[1,memory-1], outputs[2,memory-1] )

            if logging:
                logger.writerow(row)

            if mode == 'Manual':
                print "[%d] Mode: %s | X: %0.3f | Y: %0.3f | Z: %0.3f | Heading: %0.3f" % (data_index, mode, currentPos['x'], currentPos['y'], currentPos['z'], heading)                
            if mode == 'Auto':
                print "[%d] Mode: %s | X: %0.3f | Y: %0.3f | Z: %0.3f" % (data_index, mode, currentPos['x'], currentPos['y'], currentPos['z'])
            elif mode == 'SlungLoad':
                print "[%d] Mode: %s | SL_X: %0.3f | SL_Y: %0.3f" % (data_index, mode, sl_currentPos['x'], sl_currentPos['y'])                

            # Wait until the update_rate is completed 
            while elapsed < update_rate:
                elapsed = time.time() - current
            
            # Update data index and kill the thread when its done
            data_index = data_index + 1
            if data_index == data.shape[0]:
                print "\n\nSimulation terminated!\n"
                break
    except Exception,error:
        print "Error in control thread: "+str(error)

if __name__ == "__main__":
    try:
        logThread = threading.Thread(target=control)
        logThread.daemon=False
        logThread.start()
        #udp.startTwisted()
    except Exception,error:
        print "Error on main: "+str(error)
        #vehicle.ser.close()
    except KeyboardInterrupt:
        print "Keyboard Interrupt, exiting."
        exit()