#!/usr/bin/env python

import roslib, rospy

# Import the ROS messages
from nav_msgs.msg import Odometry

from geometry_msgs.msg import Twist

class Controller():

    def __init__(self):

        # Setup initial variables
        self.dialength = rospy.get_param('~dialength', 0.0)                 # this should be measure from robot center
        # self.robot_base_length = rospy.get_param('~robot_base_length', 0.2) # hardcoded 20cm on y axis
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb, queue_size=2)    

        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

    def odom_cb(self, data):
        pos_x = data.pose.pose.position.x
        quaternion = data.pose.pose.orientation

    def run_circle(self):
        default_angular = 0.8 #rad/s
        related_linear_x = self.dialength * default_angular

        t = Twist()
        t.linear.x = related_linear_x
        t.angular.z = default_angular

        self.cmd_vel_pub.publish(t)

    def run_square(self):
        pass

if __name__=='__main__':
    try:
        # Init the node here
        rospy.init_node('auto_pilot')
        
        circle_control = rospy.get_param('~circle', False)

        controller = Controller()

        # wait until the image has been passed then init the visualizer
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown():
            if circle_control:
                controller.run_circle()
            else:
                controller.run_square()

            rate.sleep()

    except rospy.ROSInterruptException:
        pass