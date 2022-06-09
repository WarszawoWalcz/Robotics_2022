import colorsys

from click import pass_obj
from regex import R
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
    n = 10
    set_speed(20*n,10*n)

def green_image(image):
    # determine which pixels in view are green
    green = image[:,:,1]
    red = image[:,:,0]
    blue = image[:,:,2]
    return ((green > 140) & (red < 75) & (blue<25))

def blue_image(image):
    # determine which pixels in view are blue
    blue = image[:,:,2]
    red = image[:,:,0]
    return((blue > 120) & (red < 20))
   
def red_image(image,object):
    # determine which pixels in view are red, depending wether we detect red boxes or red bin
    if object == "box":
        #determine which pixels in view are of red box
        red = image[:, :, 0]
        green = image[:,:,1] 
        red_matches = (red < 140) & (red > 90)
        green_matches = (green > 30) & (green < 75)  
        return((red_matches & green_matches))
    elif object == "bin":
        #determine which pixels in view are of red bin
        red = image[:, :, 0]
        green = image[:,:,1]
        red_matches = (red > 110) 
        green_matches = (green < 20)  
        return((red_matches & green_matches))

def yellow_image(image):
    # determine which pixels in view are yellow
    green = image[:,:,1]
    red = image[:,:,0]
    blue = image[:,:,2]
    return ((green>220) & (red>220) & (blue<80))

def purple_image(image):
    #determine which pixels in view are purple
    green = image[:,:,1]
    red = image[:,:,0]
    blue = image[:,:,2]
    return ((red > 100) & (red < 160) & (green > 100) & (green < 160) & (blue>100) & (blue<190))
    
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

def go_from_wall(direction):
    if direction == "to_left":
        print(direction)
        set_speed(-100,70)
        time.sleep(1.5)
    elif direction == "to_right":
        print(direction)
        set_speed(70,-100)
        time.sleep(1.5)
    elif direction == "backwards":
        turn_around()
    return
    
def wall(wall_image):
    # determine direction to turn from wall
    threshold = 1000
    left_side = np.sum(wall_image[:,:int(wall_image.shape[0]/2)]) 
    right_side = np.sum(wall_image[:,int(wall_image.shape[0]/2):])
    if (left_side >threshold) or (right_side > threshold):
        if (left_side > right_side):
            return True,"to right"
        elif(right_side > left_side):
            return True, "to_left"
        elif(right_side == left_side):
            return True, "backwards"
    return False, "_"

def detect_wall():
    close_to_wall__top_cam, direction_top_cam = wall(purple_image(sense("top_view")))
    close_to_wall__small_cam,direction_small_cam = wall(purple_image(sense("close_view")))
    if close_to_wall__small_cam:
        return (True,direction_small_cam)
    return(close_to_wall__top_cam,direction_top_cam)
     
def towards_objective(image,close=False):
    # determine direction to follow towards box
    left_side = np.sum(image[:,:int(image.shape[0]/2)])  
    right_side = np.sum(image[:,int(image.shape[0]/2):])  
    n = 10
    if close:
        i = 0.05
    else:
        i = 1
    proportion = (np.abs((left_side -right_side))/(left_side + right_side))*n   
    if proportion < 5:
        set_speed(400*i,400*i)
        return
    if left_side > right_side:
        set_speed(100*i,(100+2*proportion)*i)
    else:
        set_speed((100+2*proportion)*i,(100)*i)
    return
           
def bin_image(image,colour):
    if colour == "blue":
        bin_image = blue_image(image)
    elif colour == "red":
        bin_image = red_image(image,"bin")
    proximity = np.count_nonzero(bin_image)
    bin_detected = np.any(bin_image)    
    return bin_detected,bin_image,proximity

def find_station():
    image = sense("top_view")
    yellow_view = yellow_image(image)
    station = np.any(yellow_view)
    return (station,yellow_view)

def at_station(image):
    return (np.all(image[40:,:]))
        

def find_box():
    image = sense("close_view")
    green_box = green_image(image)
    if  np.any(green_box):
        return (True,"green",green_box)
    red_box = red_image(image,"box")
    if np.any(red_box):
        return (True,"red",red_box)
    return (False,"_ ","_")


def detect_box_posession():
    image = sense("close_view")
    if np.count_nonzero((red_image(image,"box")).astype(int).flatten()) > (60*60):
        return (True, "red")
    elif np.count_nonzero((green_image(image)).astype(int).flatten()) > (60*60):
        return (True, "green")
    return (False, "_")
    
def find_bin(objective,threshold = (40*40)):
    colour = objective.split(" bin")[0]
    if colour == "red":
        bin_detected,image,proximity = bin_image(get_image_top_cam(),"red")
    elif colour == "blue":
        bin_detected,image,proximity = bin_image(get_image_top_cam(),"blue")
    if proximity > threshold:
        close = True
    else:
        close = False
    return bin_detected, image,close

def find_objective(objective):
    if objective == "station":
        return find_station()
    elif objective == "box_view":
        return find_box()
    elif objective == "box_posession":
        return detect_box_posession()
    elif "bin" in objective:
        return find_bin(objective)
 
def evation_layer():
### add: OR CLOSE TO BIN ##
    close_to_wall,direction = detect_wall()
    if close_to_wall:
        print("close to wall, going to: ", direction)
        go_from_wall(direction)
        return True
    box, _ = find_objective("box_posession")
    if box:
        #ignore, we do actually want to get close to bin
        return False 
    _,_,close_to_blue_bin = find_bin("blue")
    _,_,close_to_red_bin = find_bin("red")
    if close_to_blue_bin or close_to_red_bin:
        print("too close to bin")
        turn_around()
        return  True     
    return False
         
def battery_layer():
    low_battery = float(sense("battery_lev")) < 0.1
    if low_battery:
        print("battery low")
        found_station,station_image = find_objective("station")
        if found_station:
            #go to station
            print("station in vision")
            if at_station(station_image):
                print("at station")
                while(float(sense("battery_lev"))< 0.99) :
                    set_speed(0,0)
            towards_objective(station_image)
            return True
        else:
            # find station
            print("searching station")
            drive_random()
            time.sleep(1)
            return True
    print("battery OK")
    return False

def box_posession_layer():
    box, box_colour = find_objective("box_posession")
    no_box = (not box)
    if (no_box):
        print("no box in posession")
        box_in_vision, colour,box_image = find_objective("box_view")
        if box_in_vision:
            print("box in vision: ", colour, " , approaching it")
            towards_objective(box_image)
            #if we have box in posession, continue driving for 0.2 second to ensure we grab it
            box, box_colour = find_objective("box_posession")
            if box:
                print("box in possession after finding")
                time.sleep(0.2)
            return True,None
        else:
            print("searching box")
            # go search for a box by driving randomly
            drive_random()
            return True,None
    print("box in posession")
    return False,box_colour
           
def bin_layer(box_colour):
    bin = str(["red","blue"][box_colour == "green"])
    objective = str(bin + " bin" )
    bin_in_vision, bin_image,close = find_objective(objective)
    if bin_in_vision:
        print(bin + " bin in vision")
        if close:
            print("close")
            towards_objective(bin_image,True)
            time.sleep(0.3)
            box, _ = find_objective("box_posession")
            if not box:
                print("dropped box :D")
                turn_around()
        else:
            towards_objective(bin_image)
        time.sleep(0.5)
    else:
        print("searching "+ bin+ " bin")
        drive_random()
        time.sleep(1)
    return         
                 
def decide():
    if evation_layer() == True:
        return
    if battery_layer() == True:
        return
    box_posession,box_colour = box_posession_layer()
    if box_posession== True:
        return
    bin_layer(box_colour)
    return
 
def turn_around():
    set_speed(-100,-100)
    time.sleep(5)
    set_speed(1,1)
    print("turning around")
    return
    

# MAIN CONTROL LOOP
if clientID != -1:
    print('Connected')
    i = 0
    while (True):
        print("___interation "+ str(i) + "____")
        decide()
    # End connection
    sim.simxGetPingTime(clientID)
    sim.simxFinish(clientID)
else:
    print('Failed connecting to remote API server')
print('Program ended')
