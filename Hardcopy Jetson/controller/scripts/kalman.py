import get_position as location
import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

#initialize a kalman filter
kalman = KalmanFilter(dim_x = 3, dim_z = 3)


if __name__ == '__main__':
    try:
        location.get_position()
    except rospy.ROSInterruptException:
        pass


def adjust_position():
    pub = rospy.Publisher()
    pass
