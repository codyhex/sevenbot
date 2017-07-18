#!/usr/bin/env python

import roslib, rospy

# Import the ROS messages
from nav_msgs.msg import Odometry

from geometry_msgs.msg import Twist, Point

from math import radians, copysign, sqrt, pow, pi

class Controller():

    def __init__(self):

        # Setup initial variables
        self.dialength = rospy.get_param('~dialength', 0.0)                 # this should be measure from robot center
        self.linear_speed = rospy.get_param('~linear_speed', 0.2)                 # this should be measure from robot center
        self.angular_speed = rospy.get_param('~angular_speed', 1.0)                 # this should be measure from robot center
        # self.robot_base_length = rospy.get_param('~robot_base_length', 0.2) # hardcoded 20cm on y axis
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb, queue_size=2)    

        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self.angular_tolerance = radians(2.5)

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()

        # Give tf some time to fill its buffer
        rospy.sleep(2)

        # Set the odom frame
        self.odom_frame = '/odom'

        self.old_position = Point()

        # Find out if the robot uses /base_link or /base_footprint
        try:
            self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")  

    def odom_cb(self, data):
        pos_x = data.pose.pose.position.x
        quaternion = data.pose.pose.orientation

    def run_circle(self):
        related_linear_x = self.dialength * self.angular_speed

        t = Twist()
        t.linear.x = related_linear_x
        t.angular.z = self.angular_speed

        self.cmd_vel_pub.publish(t)

    def go_straight(self, distance):
        pass

    def in_place_turn(self, radian):
        pass        

    def run_square(self):
        # simply setting up the square motion with odom data
        # because if using Navigation Stack, robot needs a Lidar
        for i in range(4):
            self.go_straight(1.0)
            self.in_place_turn(1.57)

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