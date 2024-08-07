#!/usr/bin/env python3

from controller import Controller
from maze_solver import get_path
from motor.msg import MotorPWM
from pathlib import Path
from get_position import MazeListener
import sys 
import os
import numpy as np
import rospy
import time

if __name__ == "__main__":

    #initialize rospy node
    rospy.init_node("controller_node", anonymous=True)

    #create a publisher node to the /motor/pwm_cmd topic
    pub = rospy.Publisher("/motor/pwm_cmd", MotorPWM, queue_size=10) 
    rospy.loginfo("Publisher created: /motor/pwm_cmd")

    #retrieve the path generated by get_path()
    tags = Path("/home/jetson/Downloads/tags.yaml")
    path = get_path(tags, (0, 0), (1, 3))
    print("path received")

    #falls import nicht funktioniert einfach selber path definieren 
    #create a path 
    path = np.array([[0.131, 0.195], [0.131, 0.445], [0.131, 0.695], [0.445, 0.695], [0.445, 0.945]])
    print("initialized path array \n")

    #create mazelistener instance 
    listener = MazeListener()
    print("created MazeListener instance")

    #create controller instance 
    controller = Controller(path, theta=np.pi/2)
    print("initialized Controller instance \n")

    #set the initial robot position as start point 
    robot_position = path[0]

    #go through each coordinate in the path (in order)
    for coordinate in path:
        #make sure that the node is still running 
        if rospy.is_shutdown():
            break
        else: 
            print("Going towards following cordinate:" + " in x: " + str(coordinate[0]) \
            + " in y: " + str(coordinate[1]))
            #for each point check if the distance between the middle point of the
            #chassis is close enough to the coordinate, if not run the control loop
            while np.linalg.norm(robot_position[0:2] - coordinate) > controller.epsilon: 
                print("still driving towards following cordinate:" + " in x: " + str(coordinate[0]) + " in y: " + str(coordinate[1]))
                if rospy.is_shutdown():
                    break
                position = listener.get_position()
                print("Robot position is:" + " in x: " + str(position[0]) + " in y: " + str(position[1]))               
                print("The angle of the robot is: " + str(controller.theta))
                controller.regelung(position, coordinate)
                if listener.get_position() is not None:
                    robot_position = listener.get_position()
                    controller.pos = listener.get_position()

                time.sleep(controller.dt)
