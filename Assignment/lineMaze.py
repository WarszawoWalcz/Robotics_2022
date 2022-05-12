from functools import update_wrapper
from turtle import right
import sim
import numpy as np
import colorsys
import matplotlib.pyplot as plt
import random

sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)


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
        res = int(np.sqrt(len(image)/3))
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

def gray_scale(rgb):
    r = rgb[:,:,0] 
    g = rgb[:,:,1]
    b = rgb[:,:,2]
    return 0.2989 * r + 0.5870 * g + 0.1140 * b

def black_image(image):
    gray = gray_scale(image)
    black = (gray < 30).astype(int)
    return black

def red_image(image):
    red = image[:,:,0]
    red = (red  > 235).astype(int)
    return red

def blue_image(image):
    blue = image[:,:,2]
    blue = (blue > 235).astype(int)
    return blue

def show_image(image):
    plt.imshow(image)
    plt.show()

def E(error):
    Kp = 0.01
    Ki = 0.005
    Kd = 0.000001
    if len(error) <2:
        PID_error = ( Kp * error[-1] ) +(Ki * np.trapz(error) ) + (Kd *  0)
    else:
        PID_error = ( Kp * error[-1] ) +(Ki * np.trapz(error) ) + (Kd *  np.gradient(error[-2:])[-1])
    return PID_error

def sense(image):
    red = red_image(image)
    blue = blue_image(image)
    black = black_image(image)
    
    sense_red = np.any(red.flatten())
    sense_blue = np.any(blue.flatten())
    sense_black = np.any(black.flatten())
    
    if sense_blue:
        print("blue")
        print(blue)
        return (True,0)
    
    elif sense_black:
        print("black")
        print(black)
        state,error = detect_line_direction(black)
        return (state,error)
    
    elif sense_red:
        print("red")
        print(red)
        if np.all(red.flatten()):
            state = "red"
            return(state, 64)
        else: 
            print("edge of red")
            error = np.count_nonzero(red.flatten() == 0)
            state = "edge of red"
            return(state, error)
    else:
        print("blank")
        state = "blank"
        return(state, 64)

def decide(state, error):
    if state == "red":
        decision = "random"
        adjustment = None
        errors.append(error)
    elif state == "straight on line":
        decision = "follow line"
        adjustment = None
        errors.append(error)
    elif state == "blank":
        errors.append(error)
        PID_error = E(errors)
        adjustment = PID_error
        decision = "random"
    elif state == "edge of red":
        errors.append(error)
        PID_error = E(errors)
        decision = "random"
        adjustment = PID_error
    elif state == True:
        decision = "stay"
        adjustment = None
        errors.append(error)
    elif state == "corner to right":
        decision = "take right corner"
        adjustment = None
        errors.append(0)
    elif state == "corner to left":
        decision = "take left corner"
        adjustment = None
    elif state == "on center of line":
        decision = "follow"
        adjustment = None
        errors.append(0)
    else:
        #adjust to either right or left
        errors.append(error)
        print("error is: ", error)
        PID_error = E(errors)
        if state == "on left side of line":
            decision = "shift right"
            adjustment = PID_error
        else:
            decision = "shift left"
            adjustment =  PID_error
            
    return (decision,adjustment)
    
def act(decision,adjustment):
    if decision == "random":
        drive_random()
    elif decision == "follow line":
        follow_line()
    elif decision == "return":
        do_return(adjustment)
    elif decision == "stay":
        print("staying")
        set_speed(0,0)
    elif decision == "follow":
        follow_line()
    elif decision == "shift right":
        shift("l", adjustment)
    elif decision == "shift left":
        shift("r", adjustment)
    elif decision == "take left corner":
        turn("l")
    elif decision == "take right corner":
        turn("r")
      
def detect_turn(image):
    previous_right = 100
    previous_left = 100
    for j in image:
        n_black_r = 0
        for i in j:
            if i ==0:
                return n_black_r
            else:
                n_black_r +=1
        n_black_l = 0
        for i in j.reverse():
            if i == 0:
                return n_black_l
            else:
                n_black_l += 1
        if (n_black_l - previous_left) >2 and (previous_left > 0):
            return "turn to left"
        elif (n_black_r - previous_right) > 2:
            return "turn to right"
    
        else:
            previous_left = n_black_l
            previous_right = n_black_r

    return "no corner"                 
            
def detect_line_direction(image):
    straight = detect_straight(image)
    if not straight: 
        left = np.array([image[i][:int((image.shape[1]/2))] for i in range(image.shape[0])])
        right = np.array([image[i][int((image.shape[1]/2)):] for i in range(image.shape[0])])
        n_left = np.count_nonzero(left)
        n_right = np.count_nonzero(right)
        if n_left > n_right:
            print("on right side of line")
            error = np.abs(n_left - n_right)
            return ("on right side of line",(error))
        elif n_right > n_left:
            print("on left side of line")
            error = np.abs(n_left - n_right)
            return ("on left side of line",error)
        else:
            corner = detect_turn(image)
            print("on center of line")
            return ("on center of line",0)
    else:
        print("straight on line")
        return ("straight on line",0)

def detect_straight(image):
    if np.count_nonzero(image[0] ) == 10:
        if np.all(image[0] == image[-1]):
            print("detected straight line")
            return True
    return False

def detect_line(image):
    # we checkout the front row and the last row of the sensor
    # the k values calculate how far the 1's are located from the right or left border
    # k_rs stores this for both the most front and  most back row of the sensor image. same goes for k_ls for the left side
    k_rs= k_ls = []
    focus_image = image[[0,image.shape[0]-1],:]
    # j is either the front or the back row of the image.
    # we iterate over a row starting from the left side of the camera
    for j in focus_image:
        # the k values calculate how far the 1's are located from the right or left border
        k_right = k_left = 0
        for i in range(j.shape[0]):
            #i is an element in the row
            if i == 0:
                k_left +=1
            else:
                continue
        for l in j[i+1:]:
            if l == 0:
                k_right +=1
            else:
                continue
         
        k_rs.append(k_right)
        k_ls.append(k_left)
        
    if (k_ls[0] == 0) and (np.count_nonzero(focus_image[0] > 10)):
        # there is a turn to the left
        return ("turn to the left",None)
    
    elif (k_rs[0]) == 0 and (np.count_nonzero(focus_image[0] > 10)):
        #there is a turn to the right
        return ("turn to the right",None)    
    
    elif k_rs[0] == k_ls[0]:
        # we are at the center of the line
        return ("center of line",None)
    
    elif k_rs[0] > k_ls[0]:
        # we are on the right side of the line
        return ("right of line",(k_rs,k_ls))

    else:
        # we are on the left side of the line
        return ("left of line",(k_rs,k_ls))

def follow_line():
    print("following line")
    set_speed(1,1)
    return

def turn(side):
    if side == "l":
        print("turning left")
        set_speed(0,1.5)
        return
    elif side == "r":
        print("turning right")
        set_speed(1.5,0) 
        return 
    
def do_return(adjustment):
    print("returning back with adjustment", adjustment)
    set_speed(-adjustment,-adjustment)
    return

def shift(side,adjustment):
    if side == "l":
        print("shifting left with adjustment ", adjustment)
        set_speed(0,adjustment)
        return
    else:
        print("shifting right with adjustment ", adjustment)
        set_speed(adjustment,0)
        return
def drive_random():
    right = random.random() * 2
    left = random.random() *2
    print("driving random with speeds ", left, " and ", right)
    set_speed(left,right)
    return
    
# MAIN CONTROL LOOP
if clientID != -1:
    print('Connected')
    i = 0
    errors = []
    Goal = False
    set_speed(0,0)
    while (not Goal):
        image = get_image_sensor()
        state,error = sense(image)
        if state == True:
            Goal = True
        decision,adjustment = decide(state,error)
        act(decision,adjustment)
        i+=1

    # End connection
    sim.simxGetPingTime(clientID)
    sim.simxFinish(clientID)
else:
    print('Failed connecting to remote API server')
print('Program ended')