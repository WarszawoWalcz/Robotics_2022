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


def gray_scale(rgb):
    r = rgb[:, :, 0]
    g = rgb[:, :, 1]
    b = rgb[:, :, 2]
    return 0.2989 * r + 0.5870 * g + 0.1140 * b


def black_image(image):
    gray = gray_scale(image)
    black = (gray < 30).astype(int)
    return black


def red_image(image):
    red = image[:, :, 0]
    red = (red > 235).astype(int)
    return red


def blue_image(image):
    blue = image[:, :, 2]
    blue = (blue > 235).astype(int)
    return blue


def show_image(image):
    plt.imshow(image)
    plt.show()


errors_left = []
errors_right = []


def E(error):
    Kp = 0.03
    Ki = 1e-10
    Kd = 0.015
    if len(error) <2:
        PID_error = Kp * error[-1] + Ki * np.trapz(error) + Kd * 0
        print("absolute: ", error[-1], " integral: ", np.trapz(error))
    else:
        PID_error = Kp * error[-1] + Ki * np.trapz(error) + Kd * np.gradient(error[-2:])[-1]
        print("absolute: ", error[-1], " integral: ", np.trapz(error), " gradient: ", np.gradient(error[-2:])[-1])

    return PID_error


def sense(image):
    red = red_image(image)
    blue = blue_image(image)
    black = black_image(image)

    sense_red = np.any(red.flatten())
    sense_blue = np.any(blue.flatten())
    sense_black = np.any(black.flatten())

    return (sense_red, sense_blue, sense_black), (red, blue, black)


def detect_line_direction(black):
    n_pixels_back_right = np.count_nonzero(black[:4,:3])
    n_pixels_back_left = np.count_nonzero(black[:4,-3:])
    n_pixels_front_right = np.count_nonzero(black[-4:,:3])
    n_pixels_front_left = np.count_nonzero(black[-4:,-3:])
    n_pixels_front = np.count_nonzero(black[-14:,:])
    
    if n_pixels_front == 0:
        return "return", (n_pixels_back_left,n_pixels_back_right)
    if (n_pixels_front_left > 0):
        if not (n_pixels_front_right>0):
            return "turn left", (n_pixels_front_left,n_pixels_front_right)
        else:
            if(n_pixels_front_left >= n_pixels_front_right):
                return "turn left", (n_pixels_front_left,n_pixels_front_right)
            else:
                return "turn right", (n_pixels_front_left,2*n_pixels_front_right) 
    elif n_pixels_front_right > 0:
        return "turn right", (n_pixels_front_left,2*n_pixels_front_right)
    elif n_pixels_back_left > 0:
        return "turn left", (n_pixels_back_left,n_pixels_back_right)
    elif n_pixels_back_right > 0:
        return "turn right", (n_pixels_back_left,n_pixels_back_right)
    else:
        return "straight", (n_pixels_back_left,n_pixels_back_right)
    

def decide(state):
    ((sense_red, sense_blue, sense_black), (red, blue, black)) = state
    if sense_blue:
        print("blue")
        print(blue)
        if atDisk(blue):
            Goal = True
            action = "stay"
            error = (0,0)
            return action,error
        
    elif sense_black:
        print("black")
        action, error = detect_line_direction(black)
        if action != "straight":
            print(black)

    elif sense_red:
        print("red")
        # print(red)
        action = "straight"
        error = (0,0)
        if not atDisk(red):
            print("Leaving the START point")
   
    else:
        print("blank")
        action = "circle back"
        error = (0,0)

    return action,error

def act(action,error):
    errors_left.append(error[0])
    errors_right.append(error[1])
    if action == "stay":
        set_speed(0,0)
    elif action == "turn left":
        print("turning left. error left: ", errors_left[-1])
        PID_error = E(errors_left)
        turn("l", PID_error)
    elif action == "turn right":
        print("turning right. error right: ", errors_right[-1])
        PID_error = E(errors_right)
        turn("r", PID_error)
    elif action == "straight":
       straight() 
    elif action == "circle back":
        backward_circle()
    elif action == "return":
        rotate()

def straight():
    print("go straight")
    set_speed(0.3,0.3)

def rotate():
    set_speed(-5,5)
    
def turn(direction, magnitude):
    if direction == "l":
        print("slowing left by ", magnitude)
        set_speed(0.4 - magnitude,0.6)
    else:
        print("slowing right by", magnitude)
        set_speed(0.6, 0.4- magnitude)

def atDisk(state):
    if np.all(state):
        return True
    else:
        return False

def backward_circle():
    set_speed(-0.8,1)
    
def drive_random():
    right = random.random()
    left = random.random()
    set_speed(left,right)


# MAIN CONTROL LOOP
Goal = False
if clientID != -1:
    print('Connected')
    i = 0
    while  not Goal:
        i += 1
        image = get_image_sensor()
        state = sense(image)
        action,error = decide(state)
        act(action,error)

    # End connection
    sim.simxGetPingTime(clientID)
    sim.simxFinish(clientID)
else:
    print('Failed connecting to remote API server')
print('Program ended')