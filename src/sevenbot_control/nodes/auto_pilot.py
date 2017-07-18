#!/usr/bin/env python
# Python Solo Libs
from math import radians, copysign, sqrt, pow, pi
# ROS PKGs
import roslib, rospy, tf
from tf.transformations import quaternion_multiply
# ROS messages
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point, Pose, Quaternion

# Action Libs & Custom Action Msg
from actionlib import SimpleActionClient
import sevenbot_navigation.msg
    
####################### Shorthand ################################
def normalize_angle(angle):
    res = angle
    while res > pi:
        res -= 2.0 * pi
    while res < -pi:
        res += 2.0 * pi
    return res

def isclose(a, b, rel_tol=1e-09, abs_tol=0.0):
    return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

#################################################################

class Controller():

    def __init__(self):

        # Setup initial variables
        self.dialength = rospy.get_param('~dialength', 0.0)                 # this should be measure from robot center
        self.linear_speed = rospy.get_param('~linear_speed', 0.2)                 # this should be measure from robot center
        self.angular_speed = rospy.get_param('~angular_speed', 1.0)                 # this should be measure from robot center

        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        # You should pass your server name in here
        self.client = SimpleActionClient('simple_move_sever', sevenbot_navigation.msg.SimpleMoveAction)

        self.tf_listener = tf.TransformListener()

        # Give tf some time to fill its buffer
        rospy.sleep(2)

        # Set the odom frame
        self.odom_frame = '/odom'

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

    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            trans, quat = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return trans, quat

    def run_circle(self):
        related_linear_x = self.dialength * self.angular_speed

        t = Twist()
        t.linear.x = related_linear_x
        t.angular.z = self.angular_speed

        self.cmd_vel_pub.publish(t)

    def send_simple_move_goal(self, distance, rotation):
        """ make the goal pose base on the distance first,
            this method is lated turning.
            rotation is in theta degree """
        self.client.wait_for_server()

        goal = sevenbot_navigation.msg.SimpleMoveGoal()
        trans, quat = self.get_odom()

        goal.pose.position = Point(*trans)
        goal.pose.orientation = quat

        if not isclose(distance, 0.0):
            goal.pose.position.x += distance  

        if not isclose(rotation, 0.0):
            q = tf.transformations.quaternion_from_euler(0, 0, rotation)
            goal.pose.orientation = Quaternion(*quaternion_multiply(q, goal.pose.orientation))

        print ("sending goal: ")
        print (goal)
        self.client.send_goal(goal)
        # handle feedback later
        
        self.client.wait_for_result()

        print ("get results:")
        print self.client.get_result()

    def run_square(self):
        # simply setting up the square motion with odom data
        # because if using Navigation Stack, robot needs a Lidar
        # for i in range(4):
        #     self.go_straight(1.0)
        #     self.in_place_turn(1.57)

        self.send_simple_move_goal(1.0, 90)

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