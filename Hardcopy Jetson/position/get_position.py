#!/usr/bin/env python3

import rospy
import tf

class MazeListener:
    def __init__(self):
        rospy.loginfo("Initializing Maze_Listener")
        rospy.init_node('apriltag_position_listener', anonymous=True)
        self.listener = tf.TransformListener()

    def get_position(self):
        rate = rospy.Rate(10)  # 设置循环频率，例如每秒10次
        while not rospy.is_shutdown():
            try:
                # 等待直到变换可用
                self.listener.waitForTransform('/maze', '/mono_cam', rospy.Time(0), rospy.Duration(1.0))
                (trans, rot) = self.listener.lookupTransform('/maze', '/mono_cam', rospy.Time(0))
                self.robot_world_position = [trans[0], trans[1], trans[2]]
                rospy.loginfo("Jetbot_frame_origin in world frame: (%.2f, %.2f, %.2f)" %
                              (trans[0], trans[1], trans[2]))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.loginfo("tf transformation not available")
            
            rate.sleep()  # 暂停一段时间，以避免循环过快导致的资源浪费和负载

if __name__ == '__main__':
    try:
        listener = MazeListener()
        listener.get_position()
    except rospy.ROSInterruptException:
        pass
