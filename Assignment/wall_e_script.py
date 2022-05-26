import colorsys

from click import pass_obj
import sim
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import random
import struct
import time


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
    # set speed of wheels randomly
    l = random.random() * 30
    r = random.random() * 20
    set_speed(l,r)

def green_image(image):
    # determine which pixels in view are green
    green = image[:,:,1]
    red = image[:,:,0]
    blue = image[:,:,2]
    return ((green > 148) & (red < 55) & (blue<20))

def blue_image(image):
    # determine which pixels in view are blue
    blue = image[:,:,2]
    red = image[:,:,0]
    return (blue > 140)& (red < 10)
   
def red_image(image,object):
    # determine which pixels in view are red, depending wether we detect red boxes or red bin
    if object == "box":
        #determine which pixels in view are of red box
        red = image[:, :, 0]
        green = image[:,:,1] 
        red_matches = (red < 115) & (red > 90)
        green_matches = (green > 35) & (green < 60)  
        return((red_matches & green_matches))
    elif object == "bin":
        #determine which pixels in view are of red bin
        red = image[:, :, 0]
        green = image[:,:,1]
        red_matches = (red > 120) 
        green_matches = (green < 10)  
        return((red_matches & green_matches))

def yellow_image(image):
    # determine which pixels in view are yellow
    green = image[:,:,1]
    red = image[:,:,0]
    return ((green>240) & (red>240))

def purple_image(image):
    #determine which pixels in view are purple
    green = image[:,:,1]
    red = image[:,:,0]
    blue = image[:,:,2]
    return ((red > 120) & (red < 140) & (green > 120) & (green < 140) & (blue>140) & (blue<165))
    
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
    
def wall(image):
    wall = purple_image(image)
    if np.any(wall):
        # determine direction to turn from wall
        threshold = 1500
        left_side = np.sum(wall[:,:int(wall.shape[0]/2)]) 
        right_side =np.sum(wall[:,int(wall.shape[0]/2):])
        if (left_side >1500) or (right_side > 1500):
            if (left_side > right_side):
                return True,"to right"
            else:
                return True, "to_left"
        else:
            return False, "_"
    else:
        return False, "_"
     
def towards_objective(image):
    # determine direction to follow towards box
    left_side = np.sum(image[:,:int(image.shape[0]/2)])  
    right_side = np.sum(image[:,int(image.shape[0]/2):])     
    if left_side > right_side:
        n = 1
        proportion = (left_side -right_side)/(left_side + right_side)*n
        set_speed(1,1*proportion)
        time.sleep(0.5)
        set_speed(0.1,0.1)
        return
    else:
        n = 1
        proportion = (right_side - left_side)/(left_side + right_side)*n
        set_speed(1*proportion,1)
        time.sleep(0.5)
        set_speed(0.1,0.1)
        return
        
    
def find_bin(image,colour):
    if colour == "blue":
        bin_image = blue_image(image)
        blue_bin = np.any(bin_image)
        return blue_bin,bin_image
    elif colour == "red":
        bin_image = red_image(image,"bin")
        red_bin = np.any(bin_image)
        return red_bin,bin_image

def find_objective(objective):
    if objective == "station":
        image = sense("top_view")
        yellow_view = yellow_image(image)
        station = np.any(yellow_view)
        return station
    elif objective == "box_view":
        image = sense("close_view")
        red_box = red_image(image,"box")
        if np.any(red_box):
            return (True,"red",red_box)
        green_box = green_image(image)
        if  np.any(green_box):
            return (True,"green",green_box)
        return (False,"")
    elif objective == "box_posession":
        image = sense("close_view")
        if np.count_nonzero((red_image(image,"box")).astype(int).flatten()) > (60*60):
            print("red against bumper")
            return True, "red"
        elif np.count_nonzero((green_image(image)).astype(int).flatten()) > (60*60):
            print("green against bumper")
            return True, "green"
        return False, "_"
    elif objective == "red bin":
        image = get_image_top_cam()
        found,bin_image = find_bin(image,"red")
        return found,"bin_image",bin_image
    elif objective == "blue bin":
        image = get_image_top_cam()
        found,bin_image = find_bin(image,"blue")
        return found,"bin_image",bin_image
        
          
def decide():
    close_to_wall, direction = wall(sense("top_view"))
    if close_to_wall:
        print("close to wall")
        if direction == "to_left":
            print(direction)
            set_speed(-2,7)
            time.sleep(1.5)
            set_speed(1,1)
            return
        else:
            print(direction)
            set_speed(7,-2)
            time.sleep(1.5)
            set_speed(1,1)  
            return  
    low_battery = float(sense("battery_lev")) < 0.1
    if low_battery:
        print("battery low")
        found_station,station_image = find_objective("station")
        if found_station:
            #go to station
            print("station in vision")
            towards_objective(station_image)
            return
        else:
            # find station
            print("searching station")
            drive_random()
            time.sleep(0.5)
            return
    print("battery OK")
    box, box_colour = find_objective("box_posession")
    no_box = (not box)
    if (no_box):
        print("no box in posession")
        box_in_vision, colour,box_image = find_objective("box_view")
        if box_in_vision:
            print("box in vision: ", box_colour, " , approaching it")
            towards_objective(box_image)
        else:
            print("searching box")
            # go search for a box by driving randomly
            drive_random()
            time.sleep(0.5)
            return
    print("box in posession")
    if box_colour == "red": 
        bin_in_vision, bin_image = find_objective("red bin")
        if bin_in_vision:
            print("red bin in vision")
            print("approaching bin")
            towards_objective(bin_image)
    elif box_colour == "green":
        bin_in_vision, bin_image = find_objective("blue bin")
        if bin_in_vision:
            print("blue bin detected")
            print("approaching bin")
            towards_objective(bin_image)
    
    
    

# MAIN CONTROL LOOP
if clientID != -1:
    print('Connected')
    i = 0
    while (i<40):
        print("___interation "+ str(i) + "____")
        # your code goes here
        # drive_random()
        decide()
        i+=1
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
