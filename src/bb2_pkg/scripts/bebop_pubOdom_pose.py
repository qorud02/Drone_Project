#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from math import radians, degrees, pi, sqrt
from bb2_pkg.msg import Pos_XYZ_th
from tf.transformations import euler_from_quaternion

class OdomPose:

    def __init__(self):
        rospy.init_node('bb2_pub_pose_odom', anonymous=True)
        rospy.Subscriber('/bebop/odom', Odometry, self.get_odom_cb)
        self.pub = rospy.Publisher('/bb2_pose_odom', Pos_XYZ_th, queue_size=1)

        self.xyzth_now = Pos_XYZ_th()
        self.prev_theta = 0.0
        self.sum_theta = 0.0


    def get_odom_cb(self, msg):
        self.xyzth_now.x = msg.pose.pose.position.x
        self.xyzth_now.y = msg.pose.pose.position.y
        self.xyzth_now.z = msg.pose.pose.position.z

        theta = self.get_theta(msg)
        d_theta = 0.0

        if (theta - self.prev_theta) > 5.0: # 5.0 rad = 286.479 deg
            d_theta = (theta - self.prev_theta) - 2 * pi
        elif (theta - self.prev_theta) > -5.0:
            d_theta = (theta - self.prev_theta) + 2*pi
        else:
            d_theta = theta - self.prev_theta

        self.sum_theta = self.sum_theta + d_theta
        self.prev_theta = theta
        self.xyzth_now.th = self.sum_theta

        self.pub.publish(self.xyzth_now)
        self.print_xyzth(self.xyzth_now)

    def get_theta(self, dat):
        q = (dat.pose.pose.orientation.x, dat.pose.pose.orientation.y,
            dat.pose.pose.orientation.z, dat.pose.pose.orientation.w)

        quart = euler_from_quaternion(q)
        theta = quart[2]        # [0] = roll, [1] = pitch, [2] = yaw

        if theta <0:
            theta = theta + 2*pi
        if theta > 2*pi:
            theta = theta - 2*pi

        return theta

    def print_xyzth(self, msg):
        print "x = %s, y = %s, z = %s, th = %s" %(msg.x, msg.y, msg.z, degrees(msg.th))



if __name__ == '__main__':
    try:
        OdomPose()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
