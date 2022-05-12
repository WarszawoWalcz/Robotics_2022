import colorsys

from click import pass_obj
import sim
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import random
import struct


sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)


# FUNCTIONS TO INTERFACE WITH THE ROBOT
def compress():
    sim.simxSetIntegerSignal(clientID=clientID, signalName="compress", signalValue=1,
                             operationMode=sim.simx_opmode_blocking)


def set_speed(speed_l, speed_r):
    sim.simxSetStringSignal(clientID=clientID, signalName="motors", signalValue=sim.simxPackFloats([speed_l, speed_r]),
                            operationMode=sim.simx_opmode_blocking)


def get_battery():
    _, battery = sim.simxGetStringSignal(clientID=clientID, signalName="battery",
                                   operationMode=sim.simx_opmode_blocking)
    return (bytes(battery)).decode()

def get_bumper_sensor():
    # Bumper reading as 3-dimensional force vector
    bumper_force_vector = [0, 0, 0]
    return_code, bumper_force_vector_packed = sim.simxGetStringSignal(clientID=clientID, signalName="bumper_sensor",
                                                                      operationMode=sim.simx_opmode_blocking)
    if return_code == 0:
        bumper_force_vector = sim.simxUnpackFloats(bumper_force_vector_packed)
    return bumper_force_vector


def get_sonar_sensor():
    # Sonar reading as distance to closest object detected by it, -1 if no data
    sonar_dist = -1
    return_code, sonar_dist_packed = sim.simxGetStringSignal(clientID=clientID, signalName="sonar_sensor",
                                                             operationMode=sim.simx_opmode_blocking)
    if return_code == 0:
        sonar_dist = sim.simxUnpackFloats(sonar_dist_packed)
    return sonar_dist


def get_image_small_cam():
    # Image from the small camera
    return_code, return_value = sim.simxGetStringSignal(clientID=clientID, signalName="small_cam_image",
                                                        operationMode=sim.simx_opmode_blocking)
    if return_code == 0:
        image = sim.simxUnpackFloats(return_value)
        res = int(np.sqrt(len(image) / 3))
        return image_correction(image, res)
    else:
        return return_code


def get_image_top_cam():
    # Image from the top camera
    return_code, return_value = sim.simxGetStringSignal(clientID=clientID, signalName="top_cam_image",
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

def drive_random():
    l = random.random() * 10
    r = random.random() * 10
    set_speed(l,r)

def green_image(image):
    green = image[:, :, 1]
    green = (green > 235).astype(int)
    return green
   
def red_image(image,object):
    if object == "box":
        red = image[:, :, 0]
        green = image[:,:,1]
        print("red: ", np.unique(red.flatten()))
        print("green: ", np.unique(green.flatten()))
        red = ((red > 190)&(green > 223)).astype(int)
        print("box")
    elif object == "bin":
        green = image[:,:,1]
        print("red: " , np.unique(red.flatten()))
        print("green: ", np.unique(green.flatten()))
        red = (red > 220).astype(int)
        print("bin")
    return red

def yellow_image(image):
    return ((green_image(image) + red_image(image,"bin"))/2).astype(int)
    
def blue_image(image):
    blue = image[:, :, 2]
    blue = (blue > 235).astype(int)
    return blue


# END OF FUNCTIONS
def sense(request):
    if request == "bumper_pressure":
        return get_bumper_sensor()
    elif request == "object distance":
        return get_sonar_sensor()
    elif request == "top_view":
        return get_image_top_cam()
    elif request == "close_view":
        return get_image_small_cam()
    elif request == "battery_lev":
        return get_battery()
    
def object_against_bumper(view):
    if np.all(red_image(view,"box")) or np.all(green_image(view)):
        return True
    return False

def find_object_vision(image):
    red = red_image(image,"box")
    if np.any(red.flatten()):
        return True
    green = green_image(image)
    if np.any(green.flatten()):
        return True
    return False

       
def decide():
    battery = float(sense("battery_lev"))
    if battery < 0.1:
        print("charge")
        return "charge"
    else: 
        print("battery OK")
    found_object = object_against_bumper(sense("close_view"))
    if found_object:
        print("feel object against bumper")
        return "feel object against bumper"
    object_in_vision = find_object_vision(get_image_small_cam())
    # print(get_image_small_cam()[:,:,0])
    if object_in_vision:
        print("object in vision")
    

# MAIN CONTROL LOOP
if clientID != -1:
    print('Connected')
    while True:
        # your code goes here
        drive_random()
        decide()
        # print()
        # decide()
        # show_image(get_image_small_cam())
        # show_image(get_image_top_cam())
    # End connection
    sim.simxGetPingTime(clientID)
    sim.simxFinish(clientID)
else:
    print('Failed connecting to remote API server')
print('Program ended')
