import numpy as np
import rospy
import time
from std_msgs.msg import String
import get_position as location
from simple_pid import PID
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

#get the path to follow
path = get_path()

#Robot parameters
r_wheel: float = 0.063/2 #meters
axle_length: float = 0.5 #meters
epsilon: float = 0.2 #meters
dt: float = 0.1 #timestep for controller
v: float = 0 #initial velocity 

#define inputs 
u_cont = np.zeros(2)
u_l_max = 1.0
U_r_max = 1.0

#initial robot position
x, y = path[0] # assuming the first point in the path is the starting position of robot

#initial chasis angle
theta = 0 

#robot pose
q = np.array([theta, x, y])

#initialize simple pid contoller 
pid = PID(5, 0.2, 0.1, sample_time = dt)

#initialize a kalman filter
kalman = KalmanFilter(dim_x = 3, dim_z = 3)

class Controller():

    def __init__():

        #Robot parameters

        pass

def adjust_position():
    pub = rospy.Publisher()
    pass

def calculate_position() -> np.array:
    #initialize the pose vector:
    f.x = q
    #define state transition matrix
    f.F = None
    #define measurement function 
    f.H = None
    #define convariance matrix
    f.P = None
    #define measurement noise 
    f.R = None
    #define the noise matrix
    f.Q = Q_discrete_white_noise(dim=2, dt=0.1, var=0.13)

    #read the sensor data
    measurement = location.get_position()

    #get the prediction 
    prediction = f.predict()
    #update the filter with measurement
    f.update(measurement)
    
    return f.x
    #define state transition matrix

def calculate_inputs(current_pos: np.array, target_point: np.array) -> np.array: 
    
    # calculate target angle based on given pos coordinates (in rad)
    target_angle = np.arctan2(target_point[1] - current_pos[2], target_point[0] - current_pos[1])

    #!!! angle error is counted from the chassis position, meaning that if the 
    # angle is negative the robot has to turn clockwise and otherwise counter-clockwise
    angle_error = target_angle - theta

    # error calculated by the controller, angle_error/4 describes the weight how much
    # pid should impact the calculation of the velocity vectors 
    pid_error = np.absolute(pid(angle_error/4))

    #normalize the angle to range [-pi, pi]
    if angle_error > np.pi:
        angle_error -= 2*np.pi
    else:
        angle_error += 2*np.pi
    
    #if the velocity vector is not initialized 
    if v == 0 and feedforward == False:
        v = middle_speed
        feedforward = True
    
    #for angles between 0 and pi chassis should rotate clockwise 
    if np.pi > angle_error > 0:
        u_cont[0] = v - (axle_length/2)*angle_error - pid_error
        u_cont[1] = v + (axle_length/2)*angle_error + pid_error
    else:
        u_cont[0] = v + (axle_length/2)*angle_error + pid_error
        u_cont[1] = v - (axle_length/2)*angle_error - pid_error

    return u_cont

    def adjust_position(controls: np.array) -> None:
        pass

if __name__ == '__main__':

    #variable for feedforward control
    feedforward = False
    
    #go through each point in the path in order
    for coordinate in path:
        #for each point check if the distance between the middle point of the
        #chassis is close enough to the coordinate, if not run the control loop
        while np.linalg.norm(q[1:] - coordinate) > epsilon: 
            #Regelung 
            #first calculate the required inputs
            u_cont = calculate_inputs(current_pos, target_pos)
            #pass the controller inputs to the motors
            adjust_position(u_cont)
            #get new approximation for the position to 
            q = location.get_location()
