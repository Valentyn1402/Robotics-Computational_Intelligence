import numpy as np
import rospy
import time
from std_msgs.msg import String
import get_position as location
from simple_pid import PID
from motor.msg import MotorPWM
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

class Controller():

    def __init__(self, path: np.array, theta: float = 0, epsilon: float = 0.2, dt: float = 0.2):

        #Robot parameters
        self.r_wheel: float = 0.063/2 #meters
        self.axle_length: float = 0.5 #meters
        self.v: float = 0 #initial velocity 
        self.max_speed = 0.838 # m/s -> at maximum input 

        #Controller parameters
        self.epsilon = epsilon #meters
        self.dt = dt
        self.path = path

        #initialize simple pid contoller 
        self.pid = PID(5, 0.2, 0.1, sample_time = dt)

        #initialize a kalman filter
        self.kalman = KalmanFilter(dim_x = 3, dim_z = 3)
    
        #define inputs 
        self.u_cont = np.zeros(2)
        self.u_max = 1.0 #maximum input
        self.u_min = -1.0 #minimum input

        #initial robot position
        x, y = path[0] # assuming the first point in the path is the starting position of robot

        #initial chasis angle
        self.theta = theta 

        #robot pose
        self.pose = np.array([theta, x, y])

        #create a rospy subscriber which sends motor_pwm messages 
        self.rospy.init_node("position_listener", anonymous=True)

        #create a publisher node to 
        self.pub = rospy.Publisher("/motor/pwm_cmd", MotorPWM, queue_size=10)

    def calculate_position(self) -> np.array:
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

    def adjust_position(self, u_cont: np.array) -> None:

        if not rospy.is_shutdown():
            msg = MotorPWM()
            msg.pwm_left = u_cont[0]
            msg.pwm_right = u_cont[1]
            self.pub.publish(msg)

    def calculate_inputs(self, current_pos: np.array, target_point: np.array) -> np.array: 
    
        # calculate target angle based on given pos coordinates (in rad)
        target_angle = np.arctan2(target_point[1] - current_pos[2], target_point[0] - current_pos[1])

        #!!! angle error is counted from the chassis position, meaning that if the 
        # angle is negative the robot has to turn clockwise and otherwise counter-clockwise
        angle_error = target_angle - self.theta

        # error calculated by the controller, angle_error/4 describes the weight how much
        # pid should impact the calculation of the velocity vectors 
        pid_error = np.absolute(self.pid(angle_error/4))

        #normalize the angle to range [-pi, pi]
        if angle_error > np.pi:
            angle_error -= 2*np.pi
        else:
            angle_error += 2*np.pi
        
        #if the velocity vector is not initialized 
        if v == 0 and feedforward == False:
            v = self.max_speed/2
            u = self.u_max/2
            feedforward = True
        
        #for angles between 0 and pi chassis should rotate clockwise 
        if np.pi > angle_error > 0:
            self.u_cont[0] = u - (self.axle_length/2)*angle_error - pid_error
            self.u_cont[1] = u + (self.axle_length/2)*angle_error + pid_error
        else:
            self.u_cont[0] = u + (self.axle_length/2)*angle_error + pid_error
            self.u_cont[1] = u - (self.axle_length/2)*angle_error - pid_error

        return self.u_cont

    def drive_path(self) -> None:
        #variable for feedforward control
        feedforward = False
    
        #go through each point in the path in order
        for coordinate in self.path:
            #for each point check if the distance between the middle point of the
            #chassis is close enough to the coordinate, if not run the control loop
            while np.linalg.norm(q[1:] - coordinate) > self.epsilon: 
                #Regelung 
                #first calculate the required inputs
                u_cont = self.calculate_inputs(q, coordinate)
                #pass the controller inputs to the motors
                self.adjust_position(u_cont)
                #get new approximation for the position to 
                q = location.get_location()

