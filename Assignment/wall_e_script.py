import colorsys
from turtle import right

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

# GENERAL 
    
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

# DETECTIONS
   
def wall(wall_image):
    """Determine direction to which to turn to away from wall. Based on our image of the wall.
    threshold for "too close to the wall" is adaptively based on the camera resolution.

    Args:
        wall_image (numpy array): image representing presence of a wall
    Returns:
        _type_: _description_
    """    
    # determine direction to turn from wall
    threshold = [1300,500][wall_image.shape[0]<64]
    left_side = np.sum(wall_image[:,:int(wall_image.shape[0]/2)]) 
    right_side = np.sum(wall_image[:,int(wall_image.shape[0]/2):])
    if (left_side >threshold) or (right_side > threshold):
        if (left_side/right_side) > 2:
            return True,"to right"
        elif(right_side/left_side)>2:
            return True, "to left"
        else:
            return True, "backwards"
    return False, "_"

def detect_wall():
    """detect whether we are close to a wall. checks both top cam and small cam

    Returns:
        (Boolean,wall image): Boolean representing whether we are close to a wall, image representing location of the wall.
    """    
    close_to_wall__top_cam, direction_top_cam = wall(purple_image(sense("top_view")))
    close_to_wall__small_cam,direction_small_cam = wall(purple_image(sense("close_view")))
    if close_to_wall__small_cam:
        return (True,direction_small_cam)
    return(close_to_wall__top_cam,direction_top_cam)

def green_image(image):
    """  determine which pixels in view are green

    Args:
        image (numpy array): array of camera view of dimensions res*res*3

    Returns:
        numpy array: array of dimensions res*res with binary presentation of green pixels
    """    
    green = image[:,:,1]
    red = image[:,:,0]
    blue = image[:,:,2]
    return ((green > 140) & (red < 75) & (blue<25))

def blue_image(image):
    """   determine which pixels in view are blue

    Args:
        image (numpy array): array of camera view of dimensions res*res*3

    Returns:
        numpy array: array of dimensions res*res with binary presentation of blue pixels
    """    
    blue = image[:,:,2]
    red = image[:,:,0]
    return((blue > 120) & (red < 20))
   
def red_image(image,object):
    """determine which pixels in view are red. Definition of red depends on whether we detect red boxes or red bin

    Args:
        image (numpy array): some senor image, of shape res*res*3
        object (numpy array): object to detect. Either "box" or "bin"
        
    Raises:
        ValueError: invalid argument
    
    Returns:
        numpy array: array dimensions res*res  with binary presentation of pixels representing bin or box
    """    
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
    else:
        raise ValueError("wrong object type. Choose either box or bin")

def yellow_image(image):
    """   determine which pixels in view are yellow

    Args:
        image (numpy array): array of camera view of dimensions res*res*3

    Returns:
        numpy array: array of dimensions res*res with binary presentation of yellow pixels
    """    
    green = image[:,:,1]
    red = image[:,:,0]
    blue = image[:,:,2]
    return ((green>220) & (red>220) & (blue<80))

def purple_image(image):
    """   determine which pixels in view are purple

    Args:
        image (numpy array): array of camera view of dimensions res*res*3

    Returns:
        numpy array: array of dimensions res*res with binary presentation of purple pixels
    """    
    green = image[:,:,1]
    red = image[:,:,0]
    blue = image[:,:,2]
    return ((red > 100) & (red < 160) & (green > 100) & (green < 160) & (blue>100) & (blue<190))
    

def sense(request):
    """returns observation of environment or robot's own status as specified by request 

    Args:
        request (string): information to be requested.

    Returns:
        numpy array or integer: array for sensors and camera's. integer for battery level
        
    Raises:
        ValueError: invalid argument.
    """    
    if request == "bumper_pressure":
        return get_bumper_sensor()
    elif request == "top_view":
        return get_image_top_cam()
    elif request == "close_view":
        return get_image_small_cam()
    elif request == "battery_lev":
        return get_battery()
    else:
        raise ValueError("Invalid Request type. should be of bumper_pressure/top_view//close_view/battery_lev")
    
           
def bin_image(image,colour):
    """Obtain binary presentation in array of where we detect the bin of specified colour

    Args:
        image (numpy array): of resolution res*res*3
        colour (string): bin colour of interest

    Raises:
        ValueError: in case of incorrect bin colour

    Returns:
        numpy array: or resolution res*res with binary presentation of bin location
    """    
    if colour == "blue":
        bin_image = blue_image(image)
    elif colour == "red":
        bin_image = red_image(image,"bin")
    else:
        raise ValueError("Bin colour error. Choose blue or red")
    proximity = np.count_nonzero(bin_image)
    bin_detected = np.any(bin_image)    
    return bin_detected,bin_image,proximity

def find_station():
    """Obtain binary presentation of location of charging station

    Returns:
        (Boolean,image): Boolean representing whether there is a station view. Image displaying station location 
    """    
    image = sense("top_view")
    yellow_view = yellow_image(image)
    station = np.any(yellow_view)
    return (station,yellow_view)

def at_station(image):
    """determine whether robot is at station 

    Args:
        image (numpy array): with binary presentation of station square location

    Returns:
        Boolean: Robot is at station
    """    
    row_nrs = [40,50][sense("close_view").shape[0] < 64]   
    threshold = int(row_nrs/(64/image.shape[0]))
    return (np.all(image[threshold:,:]))
        

def find_box():
    """detect whether any box is in vision

    Returns:
        (Boolean,numpy array): Boolean representing wether a box is in view. array displaying box location.
    """    
    
    image = sense("close_view")
    green_box = green_image(image)
    if  np.any(green_box):
        return (True,"green",green_box)
    red_box = red_image(image,"box")
    if np.any(red_box):
        return (True,"red",red_box)
    return (False,"_ ","_")


def detect_box_posession():
    """detect wether we have a box in posession ( between the arms )

    Returns:
        (Boolean,string): Boolean representing whether there is a box in posession. String expressing which colour box it is
    """    
    image = sense("close_view")
    threshold = 60 / (64/image.shape[0])
    if np.count_nonzero((red_image(image,"box")).astype(int).flatten()) > (threshold**2):
        return (True, "red")
    elif np.count_nonzero((green_image(image)).astype(int).flatten()) > (threshold**2):
        return (True, "green")
    return (False, "_")
    
def find_bin(objective):
    """detect any bin in our vision of the box we have in posession.
    Closeness to bin is calculated adaptively to the the diffferent image dimensions

    Args:
        objective (string): colour  of the box we have in posession influences bin colour we search for

    Returns:
        (boolean,image,close): Boolean representing whether there is a bin in vision. Image representing bin location. 
        Close representing whether we are close to the bin
    """    
    colour = objective.split(" bin")[0]
    if colour == "red":
        bin_detected,image,proximity = bin_image(get_image_top_cam(),"red")
    elif colour == "blue":
        bin_detected,image,proximity = bin_image(get_image_top_cam(),"blue")
    threshold = 40/(64/image.shape[0])
    if proximity > (threshold**2):
        close = True
    else:
        close = False
    return bin_detected, image,close

def find_objective(objective):
    """detect object of interest in vision

    Args:
        objective (string): object of interest to detect

    Returns:
        call to child function: combination of boolean ( whether objective is detected) and array with binary presentation of objective location
    """    
    if objective == "station":
        return find_station()
    elif objective == "box_view":
        return find_box()
    elif objective == "box_posession":
        return detect_box_posession()
    elif "bin" in objective:
        return find_bin(objective)
    

# ACTIONS
def drive_random():
    """  set speed of wheels randomly. adopt driving beaviour to scenery.
    """
    r_speed,l_speed = [(20,10),(12,8)][sense("close_view").shape[0] < 64]    
    n = 5
    set_speed(r_speed*n,l_speed*n)
    
     
def turn_around():
    """turns around. Turning time is adapted according to scene.
    """    
    print("turning around")
    set_speed(-100,-150)
    sleep_time = [3,0.5][sense("close_view").shape[0] < 64]
    time.sleep(sleep_time)
    set_speed(1,1)
    return
    

def go_from_wall(direction):
    """In the case we are too close to a wall we can go awai from it in specified direction

    Args:
        direction (string): direction to turn away to

    Raises:
        ValueError: invalid argument.
    """    
    if direction == "to left":
        print(direction)
        set_speed(-100,70)
        time.sleep(1.5)
    elif direction == "to right":
        print(direction)
        set_speed(70,-100)
        time.sleep(1.5)
    elif direction == "backwards":
        turn_around()
    else:
        raise ValueError("invalid direction. Should be to right or to left")
    return

     
def towards_objective(image,close=False):
    """drive towards objective depicted in image. Chooses lower speed  when robot is close to the objective.
    speed towards objective is adopted to scene.

    Args:
        image (numpy array): of resolution res*res. contains binary representation of objective's location
        close (bool, optional): Whether robot is close to objective. Defaults to False.
    """    
    left_side = np.sum(image[:,:int(image.shape[0]/2)])  
    right_side = np.sum(image[:,int(image.shape[0]/2):])  
    n = 15
    if close:
        i = 0.05
    else:
        i = [1,0.5][sense("close_view").shape[0] < 64]
    proportion = (np.abs((left_side -right_side))/(left_side + right_side))*n   
    if proportion < 5:
        set_speed(400*i,400*i)
        return
    if left_side > right_side:
        set_speed(100*i,(100+2*proportion)*i)
    else:
        set_speed((100+2*proportion)*i,(100)*i)
    return

## CONTROL STRUCTURE
 
def evation_layer():
    """high abstraction level, entrance of subsumption layer. Evades walls and unintended proximity to bins.
    Either does not take action and goes to next layer or takes action and goes to new iteration

    Returns:
        Boolean (optionally): whether we take action or not
    """
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
    """high abstraction level, entrance of subsumption layer.Ensures sufficient battery by checking battery level and taking action if necessary.
    battery threshold adapted to scenery.
    Either does not take action and goes to next layer or takes action and goes to new iteration

    Returns:
        Boolean (optionally): whether we take action or not
    """    
    threshold = [0.15,0.4][sense("close_view").shape[0] < 64]
    low_battery = float(sense("battery_lev")) < threshold
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
    """high abstraction level, entrance of subsumption layer. Ensures we have a box by checking we have one or finding one.
    Either does not take action and goes to next layer or takes action and goes to new iteration

    Returns:
        Boolean (optionally): whether we take action or not
    """    
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
    """high abstraction level, entrance of subsumption layer. Ensures we go to bin by either finding it or heading in its direction.
    Either does not take action and goes to next layer or takes action and goes to new iteration

    Returns:
        Boolean (optionally): whether we take action or not
    """    
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
    """ Highest level. Dictates the subsumption architecture. 
    Directs to next iteration if acton was taken in some layer. Goes to next layer if no action was taken.
    """    
    if evation_layer() == True:
        return
    if battery_layer() == True:
        return
    box_posession,box_colour = box_posession_layer()
    if box_posession== True:
        return
    bin_layer(box_colour)
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
