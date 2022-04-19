import sim
import numpy as np

runSim = True

#Speed of the two wheels that we're going to control with this script
leftVelocity = 0
rightVelocity = 0

sim.simxFinish(-1) 
clientID=sim.simxStart('127.0.0.1',19997,True,True,5000,5) 

# The values in this array describe the distance to the closest object (wall) in 8 directions of the epuck
proxSensor_reading = np.nan

# The values in this array describe one packet of 15 auxiliary values:
# the minimum of {intensity, red, green, blue, depth value},
# the maximum of {intensity, red, green, blue, depth value},
# and the average of {intensity, red, green, blue, depth value} of the left vision sensor
leftSensor_colorReading = np.nan

# The values in this array describe one packet of 15 auxiliary values:
# the minimum of {intensity, red, green, blue, depth value},
# the maximum of {intensity, red, green, blue, depth value},
# and the average of {intensity, red, green, blue, depth value} of the right vision sensor
rightSensor_colorReading = np.nan

# The values in this array describe the average {red,green,blue} value of the left vision sensor
avg_colors_left = np.nan

# The values in this array describe the average {red,green,blue} value of the left vision sensor
avg_colors_right = np.nan

# This value displays how much power is left in the battery
batteryValue = np.nan

#Set the initial velocity of the epuck to 0
velocities = sim.simxPackFloats([0, 0])
sim.simxSetStringSignal(clientID=clientID, signalName="Motors", signalValue=velocities, operationMode=sim.simx_opmode_blocking)


if clientID != -1:
    
    while runSim:
        
        #We send the desired velocities for both motors to the simulator by packing the floats into something that can be sent over a string signal
        velocities = sim.simxPackFloats([leftVelocity, rightVelocity])
        sim.simxSetStringSignal(clientID=clientID, signalName="Motors", signalValue=velocities, operationMode=sim.simx_opmode_blocking)
       
        #We read out all string signal we are able to retrieve from the robot
        prox_code, prox_value = sim.simxGetStringSignal(clientID=clientID, signalName="ProxSensors", operationMode=sim.simx_opmode_blocking)
        battery_code, battery_value = sim.simxGetStringSignal(clientID=clientID, signalName="BatteryValue", operationMode=sim.simx_opmode_blocking)
        leftSensor_code, leftSensor_value = sim.simxGetStringSignal(clientID=clientID, signalName="LeftSensor", operationMode=sim.simx_opmode_blocking)
        rightSensor_code, rightSensor_value = sim.simxGetStringSignal(clientID=clientID, signalName="RightSensor", operationMode=sim.simx_opmode_blocking)
        
        #We need to unpack some of the values to be able to read them and then we store them in variables to be able to work with them
        if prox_code == 0 and battery_code == 0 and leftSensor_code == 0 and rightSensor_code == 0: 
            proxSensor_reading = sim.simxUnpackFloats(prox_value)
            batteryValue = sim.simxUnpackFloats(battery_value)[0]
            leftSensor_colorReading = sim.simxUnpackFloats(leftSensor_value)
            rightSensor_colorReading = sim.simxUnpackFloats(rightSensor_value)
            avg_colors_left = [leftSensor_colorReading[11],leftSensor_colorReading[12],leftSensor_colorReading[13]]
            avg_colors_right = [rightSensor_colorReading[11],rightSensor_colorReading[12],rightSensor_colorReading[13]]
        
        
        # Here you should put your control loop in which u set the velocities of the left and right motor to an
        # appropiate value given the task you want it to perform
        
        # Below you can find an example of the epuck driving forward until it is close to an object
        if proxSensor_reading[3] > 0.04:
            leftVelocity = 3
            rightVelocity = 3
        else:
            leftVelocity = -1
            rightVelocity = -1
       
    #Return the velocities to the epuc
    velocities = sim.simxPackFloats([leftVelocity, rightVelocity])
    sim.simxSetStringSignal(clientID=clientID, signalName="Motors", signalValue=velocities, operationMode=sim.simx_opmode_blocking)



    #End connection
    sim.simxGetPingTime(clientID)
    sim.simxFinish(clientID)

else:
    print ('Failed connecting to remote API server')
    
print ('Program ended')