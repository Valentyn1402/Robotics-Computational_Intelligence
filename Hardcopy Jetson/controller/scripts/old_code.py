#!/usr/bin/env python3

import rospy
import numpy as np
import tf
from apriltag_ros.msg import AprilTagDetectionArray
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_matrix
from tf.transformations import euler_from_quaternion
# from scipy.spatial.transform import Rotation as R

class MazeListener:
    def __init__(self):
        rospy.loginfo("Initializing Maze_Listener")
        # rospy.init_node('apriltag_position_listener', anonymous=True)
        self.listener = tf.TransformListener()
        self.tag_detected = False
        self.robot_world_position = None  # 初始化为 None
        self.angular_v = None

        self.quaternion = None

        rospy.Subscriber("/imu/data_raw", Imu, self.rotation)

        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.callback)

    def rotation(self, data):
        if data is not None:
            self.angular_v = (data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z)

            #get euler angles 
            rospy.loginfo("Angular velocities: %s" % str(self.angular_v[1]))
        else:
            self.angular_v = None

    def get_rotation(self):
        rate = rospy.Rate(10) 
        while not rospy.is_shutdown():
            if self.angular_v is None:
                rospy.loginfo("Waiting for angular detection...")
                rate.sleep()
            else:
                return self.angular_v[1]

    def callback(self, data):
        if len(data.detections) > 0:
            try:
                # 等待直到变换可用
                self.listener.waitForTransform('/maze', '/mono_cam', rospy.Time(0), rospy.Duration(1.0))
                (trans, rot) = self.listener.lookupTransform('/maze', '/mono_cam', rospy.Time(0))
                #define point for mono cam
                point_mono_cam = np.array([-0.07, 0, 0, 1])
                #define rotations matrix
                rotation_matrix = quaternion_matrix(rot)
                #add the translation component to the transformation matrix
                transformation_matrix = np.identity(4)
                transformation_matrix[:3, 3] = trans
                transformation_matrix[:3, :3] = rotation_matrix[:3, :3]
                # Transform the point
                point_maze = np.dot(transformation_matrix, point_mono_cam)
                self.robot_world_position = [point_maze[0], point_maze[1], point_maze[2]]
                self.tag_detected = True
                rospy.loginfo("Jetbot_frame_origin in world frame: (%.2f, %.2f, %.2f)" %
                              (trans[0], trans[1], trans[2]))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.loginfo("tf transformation not available")
                self.tag_detected = False
        else:
            self.tag_detected = False
            rospy.loginfo("No tag detected")
            self.robot_world_position = None  # 没有检测到时置为 None

    def get_position(self):
        rate = rospy.Rate(10)  # 设置循环频率，例如每秒10次
        while not rospy.is_shutdown():
            if self.tag_detected:
                rospy.loginfo("Current position: (%.2f, %.2f, %.2f)" % tuple(self.robot_world_position))
                return np.array(self.robot_world_position)  # 返回坐标数组
            else:
                rospy.loginfo("Waiting for tag detection...")
            rate.sleep()
        
        return None  # 如果 ROS 被关闭，则返回 None

if __name__ == '__main__':
    try:
        listener = MazeListener()
        position = listener.get_position()
        if position is not None:
            rospy.loginfo("Position: (%.2f, %.2f)" % (position[0], position[1]))
        else:
            rospy.loginfo("Failed to get position.")
    except rospy.ROSInterruptException:
        pass

