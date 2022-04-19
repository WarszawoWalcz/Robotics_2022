import sim
import numpy as np
import time
import colorsys
import matplotlib.pyplot as plt

sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
this_time = time.time()


# FUNCTIONS TO INTERFACE WITH THE ROBOT
def set_speed(speed_l, speed_r):
    velocities = sim.simxPackFloats([speed_l, speed_r])
    sim.simxSetStringSignal(clientID=clientID, signalName="motors", signalValue=velocities,
                            operationMode=sim.simx_opmode_blocking)


def get_image_sensor():
    return_code, return_value = sim.simxGetStringSignal(clientID=clientID, signalName="Sensors",
                                                        operationMode=sim.simx_opmode_blocking)
    if return_code == 0:
        image = sim.simxUnpackFloats(return_value)
        res = int(np.sqrt(len(image) / 3))
        return image_correction(image, res)
    else:
        return return_code


# HELPER FUNCTIONS
def image_correction(image, res):
    """
    This function can be applied to images coming directly out of CoppeliaSim.
    It turns the 1-dimensional array into a more useful res*res*3 array, with the first
    two dimensions corresponding to the coordinates of a pixel and the third dimension to the
    RGB values. Aspect ratio of the image is assumed to be square (1x1).

    :param image: the image as a 1D array
    :param res: the resolution of the image, e.g. 64
    :return: an array of shape res*res*3
    """

    image = [int(x * 255) for x in image]
    image = np.array(image).reshape((res, res, 3))
    image = np.flip(m=image, axis=0)
    return image


def show_image(image):
    plt.imshow(image)
    plt.show()


# MAIN CONTROL LOOP
if clientID != -1:
    print('Connected')
    while sim.simxGetConnectionId(clientID) != -1:
        # your code goes here
        set_speed(1, -1)

        r_values = []

        for i in get_image_sensor():
            for j in i:
                r_values.append(j[0])
        print(np.mean(r_values))
    sim.simxGetPingTime(clientID)
    sim.simxFinish(clientID)
else:
    print('Failed connecting to remote API server')
    sys.exit('Could not connect')

print('Program ended')
