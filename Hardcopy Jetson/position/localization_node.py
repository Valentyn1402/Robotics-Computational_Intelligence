#!/usr/bin/env python3

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped, TransformStamped
import tf2_ros
import tf

class AprilTagLocalization:
    def __init__(self):
        rospy.init_node('apriltag_localization')
       
        # Subscribe to the AprilTag detection topic
        self.apriltag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)
       
        # Publisher for the robot's pose
        self.pose_pub = rospy.Publisher('/robot_pose', PoseStamped, queue_size=10)
       
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
       
        rospy.loginfo("AprilTag Localization Node Initialized")

    def tag_callback(self, data):
        if len(data.detections) == 0:
            return
       
        for detection in data.detections:
            tag_id = detection.id[0]
            tag_pose = detection.pose.pose.pose
           
            # Create a PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "camera_link"
            pose_msg.pose = tag_pose
           
            # Publish the robot's pose
            self.pose_pub.publish(pose_msg)
           
            # Broadcast the transform
            transform = TransformStamped()
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = "camera_link"
            transform.child_frame_id = f"tag_{tag_id}"
            transform.transform.translation.x = tag_pose.position.x
            transform.transform.translation.y = tag_pose.position.y
            transform.transform.translation.z = tag_pose.position.z
            transform.transform.rotation = tag_pose.orientation
           
            self.tf_broadcaster.sendTransform(transform)
           
            rospy.loginfo(f"Published pose for tag {tag_id}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = AprilTagLocalization()
    node.run()
