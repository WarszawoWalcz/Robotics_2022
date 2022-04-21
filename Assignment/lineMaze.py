import sim
import numpy as np
import colorsys
import matplotlib.pyplot as plt

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
    black = (image < 30).astype(int)
    return black

def show_image(image):
    plt.imshow(image)
    plt.show()

error = []

def E(t):
    Kp = 1
    Ki = 1
    Kd = 1
    PID_error = Kp * error(t) + Ki * np.trapz(error) + Kd *  np.gradient(error[-2:])[-1]
    return PID_error

def sense():
    pass

def decide(state):
    state = None
    if atDisk(state):
        Goal = True
        return 
    else:
        pass

def act(decision):
    if decision ==1:
        #goal
        return
    elif decision ==2:
        #curve
    
        #take turn
        pass

def atDisk(state):
    pass

# MAIN CONTROL LOOP
Goal = False
if clientID != -1:
    print('Connected')
    while (not Goal):
        # state = sense()
        # action = decide(state)
        # act(action)
        r_values = []

        image = get_image_sensor()
        gray = gray_scale(image)
        
        print("gray scale: ")
        print (gray)

        Goal = True

        # for i in get_image_sensor():
        #     for j in i:
               
        #         r_values.append(j[0])
        # print(np.mean(r_values))

    # End connection
    sim.simxGetPingTime(clientID)
    sim.simxFinish(clientID)
else:
    print('Failed connecting to remote API server')
print('Program ended')
