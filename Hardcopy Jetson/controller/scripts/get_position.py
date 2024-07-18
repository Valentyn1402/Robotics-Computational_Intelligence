

#!/usr/bin/env python3

import rospy
import numpy as np
from apriltag_ros.msg import AprilTagDetectionArray
import tf

class MazeListener:
    def __init__(self):
        rospy.loginfo("Initializing Maze_Listener")
        # rospy.init_node('apriltag_position_listener', anonymous=True)
        self.listener = tf.TransformListener()
        self.tag_detected = False
        self.robot_world_position = None  # 初始化为 None

        rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.callback)

    def callback(self, data):
        if len(data.detections) > 0:
            try:
                # 等待直到变换可用
                self.listener.waitForTransform('/maze', '/mono_cam', rospy.Time(0), rospy.Duration(1.0))
                (trans, rot) = self.listener.lookupTransform('/maze', '/mono_cam', rospy.Time(0))
                self.robot_world_position = [float(trans[0]), float(trans[1]), float(trans[2])]
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

