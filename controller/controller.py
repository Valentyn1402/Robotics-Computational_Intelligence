#!/usr/bin/env python3

import sys 
import numpy as np
import rospy
import time
from pathlib import Path
from std_msgs.msg import String
from apriltag_ros.msg import AprilTagDetectionArray
from motor.msg import MotorPWM
# from maze_solver.main import get_path
from get_position import MazeListener

# from filterpy.kalman import KalmanFilter
# from filterpy.common import Q_discrete_white_noise
# from simple_pid import PID


class PIDController():
    def __init__(self, kp, ki, kd, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.prev_error = 0
        self.integral = 0

    def compute(self, error):
        self.integral += error*self.dt
        derivative = (error - self.prev_error)/self.dt
        self.prev_error = error
        return np.absolute(self.kp*error + self.ki*self.integral + self.kd*derivative)


class Controller():

    def __init__(self, path: np.array, pub, theta: float = 0, epsilon: float = 0.2, dt: float = 0.1):


        #create a maze instance
        self.maze = MazeListener()

        #Robot parameters
        self.r_wheel: float = 0.063/2 #meters
        self.axle_length: float = 0.5 #meters
        self.v: float = 0 #initial velocity 
        self.feedforward = False
        self.max_speed = 0.838 # m/s -> at maximum input 


        #Controller parameters
        self.epsilon = epsilon #meters
        self.dt = dt
        self.path = path

        #initialize simple pid contoller 
        self.pid = PIDController(3, 0.2, 0.01, self.dt)

        #initialize a kalman filter
        # self.kalman = KalmanFilter(dim_x = 3, dim_z = 3)
    
        #define inputs 
        self.u_cont = np.zeros(2)
        self.u_prev = np.zeros(2)
        self.u_max = 0.5 #maximum input
        self.u_min = -0.5 #minimum input
        self.u = 0

        #initial robot position
        self.pos = path[0] # assuming the first point in the path is the starting position of robot

        self.x, self.y = self.pos 

        #initial chasis angle
        self.theta = theta 

        #robot pose
        self.pose = np.array([self.theta, self.x, self.y])

        #create a publisher node to 
        self.pub =  pub#rospy.Publisher("/motor/pwm_cmd", MotorPWM, queue_size=10) 
        rospy.loginfo("Publisher created: /motor/pwm_cmd")
        self.rate = rospy.Rate(120)

        print("Created a publisher to /motor/pwm_cmd")
    
        #rospy.spin()
    

    # def calculate_position(self) -> np.array:
    #     #initialize the pose vector:
    #     f.x = q
    #     #define state transition matrix
    #     f.F = None
    #     #define measurement function 
    #     f.H = None
    #     #define convariance matrix
    #     f.P = None
    #     #define measurement noise 
    #     f.R = None
    #     #define the noise matrix
    #     f.Q = Q_discrete_white_noise(dim=2, dt=0.1, var=0.13)

    #     #read the sensor data
    #     measurement = location.get_position()

    #     #get the prediction 
    #     prediction = f.predict()
    #     #update the filter with measurement
    #     f.update(measurement)
        
    #     return f.x
    #     #define state transition matrix

    def adjust_position(self) -> None:

        print("I in adjust position")
        msg = MotorPWM()
        msg.pwm_left = self.u_cont[0]
        msg.pwm_right = self.u_cont[1]
        self.pub.publish(msg)
        print("I send a message: " + str(self.u_cont))
        # if not rospy.is_shutdown():
        #     print("I am in if")
        #     msg = MotorPWM()
        #     msg.pwm_left = u_cont[0]
        #     msg.pwm_right = u_cont[1]
        #     self.pub.publish(msg)
        # else:
        #     print("Im in else")

    def calculate_inputs(self, current_pos: np.array, target_point: np.array ) -> np.array: 
    
        # calculate target angle based on given pos coordinates (in rad)
        target_angle = np.arctan2(target_point[1] - current_pos[1], target_point[0] - current_pos[0])

        print("Current target angle: " + str(target_angle))

        #!!! angle error is counted from the chassis position, meaning that if the 
        # angle is negative the robot has to turn clockwise and otherwise counter-clockwise
        angle_error = target_angle - self.theta

        print("Current angle error:" + str(angle_error))

        # error calculated by the controller, angle_error/4 describes the weight how much
        # pid should impact the calculation of the velocity vectors 
        pid_error = self.pid.compute(angle_error/16)

        #normalize the angle to range [-pi, pi]
        if angle_error > np.pi:
            angle_error -= 2*np.pi
        else:
            angle_error += 2*np.pi
        
        #if the velocity vector is not initialized 
        if self.u_cont[0] == 0 and self.u_cont[0] == 0 and self.feedforward == False:
            self.u_cont[0] = -self.u_max/2
            self.u_cont[1] = -self.u_max/2 
            self.u = (self.u_cont[0] + self.u_cont[1])/2
            self.feedforward = True

        self.u = (self.u_cont[0] + self.u_cont[1])/2
        #for angles between 0 and pi chassis should rotate counter clockwise 
        if np.pi > angle_error > 0:

            #first calculate the error 
            error_neg = (self.axle_length/16)*angle_error - pid_error
            error_pos = (self.axle_length/16)*angle_error + pid_error

            if self.u_cont[0] > 0 and (self.u_cont[0] - error_neg) > 0:
                if (self.u_cont[0] - error_neg) > self.u_max:
                    self.u_cont[0] = self.u_max
                else:
                    self.u_cont[0] = self.u_cont[0] - error_neg

            elif self.u_cont[0] < 0 and (self.u_cont[0] - error_neg) < 0:
                if (self.u_cont[0] - error_neg) < self.u_min:
                    self.u_cont[0] = self.u_min
                else:
                    self.u_cont[0] = self.u_cont[0] - error_neg

            if self.u_cont[1] > 0 and (self.u_cont[1] + error_pos) > 0:
                if (self.u_cont[1] + error_pos) > self.u_max:
                    self.u_cont[1] = self.u_max
                else:
                    self.u_cont[1] = self.u_cont[1] + error_pos

            elif self.u_cont[1] < 0 and (self.u_cont[1] + error_pos) < 0:
                if (self.u_cont[1] + error_pos) < self.u_min:
                    self.u_cont[1] = self.u_min
                else:
                    self.u_cont[1] = self.u_cont[1] + error_pos

        elif 0 > angle_error > -np.pi:
        #rotate clockwise 
            print("my angle is negative")
             #first calculate the error 
            error_neg = (self.axle_length/16)*angle_error - pid_error
            error_pos = (self.axle_length/16)*angle_error + pid_error

            if self.u_cont[1] > 0 and (self.u_cont[1] - error_neg) > 0:
                if (self.u_cont[1] - error_neg) > self.u_max:
                    self.u_cont[1] = self.u_max
                else:
                    self.u_cont[1] = self.u_cont[1] - error_neg

            elif self.u_cont[1] < 0 and (self.u_cont[1] - error_neg) < 0:
                if (self.u_cont[1] - error_neg) < self.u_min:
                    self.u_cont[1] = self.u_min
                else:
                    self.u_cont[1] = self.u_cont[1] - error_neg

            if self.u_cont[0] > 0 and (self.u_cont[0] + error_pos) > 0:
                if (self.u_cont[0] + error_pos) > self.u_max:
                    self.u_cont[0] = self.u_max
                else:
                    self.u_cont[0] = self.u_cont[0] + error_pos

            elif self.u_cont[0] < 0 and (self.u_cont[0] + error_pos) < 0:
                if (self.u_cont[0] + error_pos) < self.u_min:
                    self.u_cont[0] = self.u_min
                else:
                    self.u_cont[0] = self.u_cont[0] + error_pos

        return self.u_cont
    
    def regelung(self, position, coordinate) -> None:
        #Regelung 
        #first calculate the required inputs
        print(self.calculate_inputs(position, coordinate))
        #update position based on perfect model
        self.u = (self.u_cont[0] + self.u_cont[1])/2
        #update the angle
        self.theta = ((self.u_cont[0]*self.max_speed - self.u_cont[1]*self.max_speed)/self.axle_length)*self.dt
        #pass the controller inputs to the motors
        self.adjust_position()
        #get new approximation for the position to 

        if position is not None:

            print("given position in x: " + str(position[0]) + " given position in y: " + str(position[1]))
            self.pos = position[0:2]

        print("robot position in x: " + str(self.pos[0]) + " robot position in y: " + str(self.pos[1]))
            
        print("control inputs: " + str(self.u_cont))

        print("robot position in x: " + str(self.pos[0]) + " robot position in y: " + str(self.pos[1]))
        # local_pos = self.pos


    def drive_path(self, location) -> None:

        print("driving path")
    
        #go through each point in the path in order
        for coordinate in self.path:
            print(coordinate)
            #for each point check if the distance between the middle point of the
            #chassis is close enough to the coordinate, if not run the control loop
            while np.linalg.norm(self.pos - coordinate) > self.epsilon: 
                #Regelung 
                #first calculate the required inputs
                print(self.calculate_inputs(self.pose, coordinate))
                #update location based on perfect model
                self.u = (self.u_cont[0] + self.u_cont[1])/2
                #update the angle
                self.theta = ((self.u_cont[0]*self.max_speed - self.u_cont[1]*self.max_speed)/self.axle_length)*self.dt
                #pass the controller inputs to the motors
                self.adjust_position()
                #get new approximation for the location to 
                print("given location in x: " + str(location[0]) + " given location in y: " + str(location[1]))

                self.pos = location[0:2]
                print("control inputs: " + str(self.u_cont))

                print("robot position in x: " + str(self.pos[0]) + " robot position in y: " + str(self.pos[1]))
                # local_pos = self.pos
                time.sleep(self.dt)

        #when goal is reached rosshutdown
        stop_msg = MotorPWM()
        stop_msg.pwm_left = 0
        stop_msg.pwm_left = 0
        self.pub.publish(stop_msg)

        rospy.loginfo("Motorshutdown")

def main():
     #initialize rospy node
    rospy.init_node("controller_node", anonymous=True)

    #create a publisher node to 
    pub = rospy.Publisher("/motor/pwm_cmd", MotorPWM, queue_size=10) 
    rospy.loginfo("Publisher created: /motor/pwm_cmd")

    #create a path 
    path = np.array([[0.131, 0.195], [0.131, 0.445], [0.131, 0.695], [0.445, 0.695], [0.445, 0.945]])
    print("initialized path array \n")

    listener = MazeListener()
    #create controller instance 
    controller = Controller(path, pub, theta=np.pi/2)
    print("initialized controller instance \n")
    # tags = Path("/home/jetson/Downloads/tags.yaml")

    robot_position = controller.pos

    print("driving path")

    #go through each point in the path in order
    for coordinate in path:

        print("Going towards following cordinate:" + " in x: " + str(coordinate[0]) + " in y: " + str(coordinate[1]))
        #for each point check if the distance between the middle point of the
        #chassis is close enough to the coordinate, if not run the control loop
        while np.linalg.norm(robot_position[0:2] - coordinate) > controller.epsilon: 
            print("Going towards following cordinate:" + " in x: " + str(coordinate[0]) + " in y: " + str(coordinate[1]))
            print("The angle of the robot is: " + str(controller.theta))
            position = listener.get_position()
            controller.regelung(position[0:2], coordinate)
            print("The angle of the robot is: " + str(controller.theta))
            if position is not None:
                controller.pos = position
                robot_position = position

            time.sleep(controller.dt)
                
    #when goal is reached rosshutdown
    stop_msg = MotorPWM()
    stop_msg.pwm_left = 0
    stop_msg.pwm_left = 0
    pub.publish(stop_msg)

    rospy.loginfo("Motorshutdown")
# path = get_path(tags, (0, 0), (0, 3)

# rospy.spin()

if __name__ == "__main__":
    main()


    