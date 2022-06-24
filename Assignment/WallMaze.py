import sim
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

runSim = True

# Speed of the two wheels that we're going to control with this script
leftVelocity = 0
rightVelocity = 0
# Seconds (time for X-axis for plotting proximity sensors)
seconds_array = []
sensor_array = []

if_black = 0

sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

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

# The values in this array describe the average {red,green,blue} value of the right vision sensor
avg_colors_right = np.nan

# This value displays how much power is left in the battery
batteryValue = np.nan

# Set the initial velocity of the epuck to 0
velocities = sim.simxPackFloats([0, 0])
sim.simxSetStringSignal(clientID=clientID, signalName="Motors", signalValue=velocities,
                        operationMode=sim.simx_opmode_blocking)

if clientID != -1:
    while runSim:
        # We send the desired velocities for both motors to the simulator by packing the floats into something that can be sent over a string signal
        velocities = sim.simxPackFloats([leftVelocity, rightVelocity])
        sim.simxSetStringSignal(clientID=clientID, signalName="Motors", signalValue=velocities,
                                operationMode=sim.simx_opmode_blocking)

        # We read out all string signal we are able to retrieve from the robot
        prox_code, prox_value = sim.simxGetStringSignal(clientID=clientID, signalName="ProxSensors",
                                                        operationMode=sim.simx_opmode_blocking)
        battery_code, battery_value = sim.simxGetStringSignal(clientID=clientID, signalName="BatteryValue",
                                                              operationMode=sim.simx_opmode_blocking)
        leftSensor_code, leftSensor_value = sim.simxGetStringSignal(clientID=clientID, signalName="LeftSensor",
                                                                    operationMode=sim.simx_opmode_blocking)
        rightSensor_code, rightSensor_value = sim.simxGetStringSignal(clientID=clientID, signalName="RightSensor",
                                                                      operationMode=sim.simx_opmode_blocking)
        # We need to unpack some values to be able to read them, and then we store them in variables to be able to work with them
        if prox_code == 0 and battery_code == 0 and leftSensor_code == 0 and rightSensor_code == 0:
            proxSensor_reading = sim.simxUnpackFloats(prox_value)
            batteryValue = sim.simxUnpackFloats(battery_value)[0]
            leftSensor_colorReading = sim.simxUnpackFloats(leftSensor_value)
            rightSensor_colorReading = sim.simxUnpackFloats(rightSensor_value)
            avg_colors_left = [leftSensor_colorReading[11], leftSensor_colorReading[12], leftSensor_colorReading[13]]
            avg_colors_right = [rightSensor_colorReading[11], rightSensor_colorReading[12],
                                rightSensor_colorReading[13]]

        # When facing front to the wall and 'pushing' it, goes back, adjusting to the wall
        # > IT IS INDEPENDENT OF THE STRUCTURE OF ELSE-IF CHAIN
        if proxSensor_reading[4] > 0.05 and proxSensor_reading[2] < 0.025:
            leftVelocity = -0.9
            rightVelocity = -0.4

        # When sensing black at certain threshold of occurrences spins
        # to the left and goes back following second side of the maze
        # if avg_colors_right[0] == 0 and avg_colors_left[0] == 0:
        #     if_black += 1
        #     print(if_black)
        #     leftVelocity = -4
        #     rightVelocity = 4
        # When sensing yellow at all (checkpoint station), slows down until the color disappears
        elif avg_colors_right[0] > avg_colors_right[2] and avg_colors_right[1] > avg_colors_right[2]:
            if avg_colors_right[0] > 0.5 and avg_colors_right[1] > 0.5:
                leftVelocity = 0.001
                rightVelocity = 0.001
        # When sensing red at all (end of charging), speeds up the motors bit to the wall and leaves the station
        elif avg_colors_right[0] > avg_colors_right[1] + avg_colors_right[2]:
            leftVelocity = 1.2
            rightVelocity = 1
        # When sensing green at all (charging station), slows down until the color disappears
        elif avg_colors_right[1] > avg_colors_right[0] + avg_colors_right[2]:
            leftVelocity = 0.01
            rightVelocity = 0.01
        # When sensing blue at all (end of checkpoint), speeds up the motors bit to the wall and leaves the station
        elif avg_colors_right[2] > avg_colors_right[0] + avg_colors_right[1]:
            leftVelocity = 1.2
            rightVelocity = 1
        # If the sensor on the front left side senses less than sensor on the right side -> corner to the left
        # Spins to the left side (around 90 degrees)
        elif proxSensor_reading[2] < proxSensor_reading[4]:
            leftVelocity = -4
            rightVelocity = 4
        # If the sensor on the front right side senses more
        # than sensor on the left side (that doesn't sense anything) -> corner to the right
        # Spins to the right side with circular motion, so it can spin even full 180 turn
        elif proxSensor_reading[4] == 0.05000000074505806 and proxSensor_reading[6] > 0.03285243362188339:
            leftVelocity = 3.2
            rightVelocity = 1
        # If sum of the signals of the sensor from the front left and front right is less than back right spin back
        # Situation where robot goes almost its left side to the right wall -> no working sensors there
        elif proxSensor_reading[2] + proxSensor_reading[4] < proxSensor_reading[6]:
            leftVelocity = -2
            rightVelocity = -0.2
        # If the difference between sensor on the right back side and front left side is less than -0.0198
        # it may also indicate precisely corner to the right
        # or any blocking situation when it nees to turn more to the right
        elif proxSensor_reading[7] - proxSensor_reading[3] < -0.0198:
            leftVelocity = 2
            rightVelocity = 0.8
        # If sensor on the back right senses less than front right, goes right
        elif proxSensor_reading[6] < proxSensor_reading[4]:
            leftVelocity = 3
            rightVelocity = 2
        # If sensor on the back right senses more than front right, goes left
        elif proxSensor_reading[6] > proxSensor_reading[4]:
            leftVelocity = 2
            rightVelocity = 3
        else:
            leftVelocity = 0.1
            rightVelocity = 0.3

    # Return the velocities to the epuck
    velocities = sim.simxPackFloats([leftVelocity, rightVelocity])
    sim.simxSetStringSignal(clientID=clientID, signalName="Motors", signalValue=velocities,
                            operationMode=sim.simx_opmode_blocking)
    # End connection
    sim.simxGetPingTime(clientID)
    sim.simxFinish(clientID)
else:
    print('Failed connecting to remote API server')
print('Program ended')