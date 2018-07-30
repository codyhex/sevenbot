#!/usr/bin/env python

import roslib, rospy, tf
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, Twist
from actionlib import SimpleActionServer

import sevenbot_navigation.msg
import tf.transformations as tftr
from math import sqrt

def isclose(a, b, rel_tol=1e-09, abs_tol=0.0):
    return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

class SimpleMoveAction():
    """docstring for SimpleMoveAction"""
    def __init__(self, name):

        self.linear_speed = rospy.get_param('~linear_speed', 0.2)   # measure from robot center
        self.angular_speed = rospy.get_param('~angular_speed', 0.5)   
        self.linear_tolerance = 0.05 # theta
        self.angular_tolerance = 10 # theta

        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()

        # Give tf some time to fill its buffer
        rospy.sleep(2)

        # Set the odom frame
        self.odom_frame = '/odom'
        self._action_name = name

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


        self._feedback = sevenbot_navigation.msg.SimpleMoveFeedback()
        self._result = sevenbot_navigation.msg.SimpleMoveResult()
        self._as = SimpleActionServer(self._action_name, sevenbot_navigation.msg.SimpleMoveAction, \
                   execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            trans, quat = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return trans, quat

    def go_straight(self, dist_vec):


        cmd_vel = Twist()
        cmd_vel.linear.x = self.linear_speed

        # Publish the Twist message and sleep 1 cycle         
        self.cmd_vel_pub.publish(cmd_vel)
        # Post-update
        trans, quat = self.get_odom()
        dist_vec.x = self.goal.pose.position.x - trans[0]
        dist_vec.y = self.goal.pose.position.y - trans[1]

        r = rospy.Rate(20)
        r.sleep()

        return dist_vec


    def turn_in_place(self, dist_vec):
        
        cmd_vel = Twist()
        cmd_vel.angular.z = self.angular_speed

        # Publish the Twist message and sleep 1 cycle         
        self.cmd_vel_pub.publish(cmd_vel)

        trans, quat = self.get_odom()
        quat0 = [self.goal.pose.orientation.w, self.goal.pose.orientation.x, self.goal.pose.orientation.y, self.goal.pose.orientation.z]

        dist_vec.z = tftr.euler_from_quaternion(tftr.quaternion_slerp(quat0, quat, fraction=1))[2] # take only the Yaw value

        r = rospy.Rate(20)
        r.sleep()
        return dist_vec

    def execute_cb(self, goal):
        self.goal = goal
        self._result.reached = False
        # How fast will we update the robot's movement?
        dist_vec = Vector3()
        trans, quat = self.get_odom()
        dist_vec.x = self.goal.pose.position.x - trans[0]
        dist_vec.y = self.goal.pose.position.y - trans[1]
        # the goal contains a quat already turned, so this with end up with the error between current and goal
        # dist_vec.z will be the euler angle theta of the error to goal
        quat0 = [self.goal.pose.orientation.w, self.goal.pose.orientation.x, self.goal.pose.orientation.y, self.goal.pose.orientation.z]
        dist_vec.z = tftr.euler_from_quaternion(tftr.quaternion_slerp(quat0, quat, fraction=1))[2]

        print tftr.euler_from_quaternion(quat0)
        print tftr.euler_from_quaternion(tftr.quaternion_slerp(quat0, quat, fraction=1))
        print ('goal z', dist_vec.z)
        distance = sqrt(dist_vec.x**2 + dist_vec.y**2)

        while (distance-self.linear_tolerance)>0.0 and not rospy.is_shutdown():
            dist_vec = self.go_straight(dist_vec)
            distance = sqrt(dist_vec.x**2 + dist_vec.y**2)
            self._feedback.vector = dist_vec
            self._as.publish_feedback(self._feedback)
            # rospy.loginfo('%s: Moving, distance to goal %f ' % (self._action_name, distance))

        # Stop the robot before the rotation
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)
        rospy.sleep(3)

        # Now start turning
        trans, quat = self.get_odom()
        quat0 = [self.goal.pose.orientation.w, self.goal.pose.orientation.x, self.goal.pose.orientation.y, self.goal.pose.orientation.z]

        # update the orientation again to make sure you have the latest value after the straight action
        dist_vec.z = tftr.euler_from_quaternion(tftr.quaternion_slerp(quat0, quat, fraction=1))[2]

        while (dist_vec.z-self.angular_tolerance)>0.0 and not rospy.is_shutdown():
            dist_vec = self.turn_in_place(dist_vec)
            self._feedback.vector = dist_vec
            self._as.publish_feedback(self._feedback)
            rospy.loginfo('%s: Truning to goal %f' % (self._action_name, self._feedback.x, self._feedback.y, self._feedback.z))

        # Goal should be reached after the operations but this is not helpful
        self._result.reached = True
        trans, quat = self.get_odom()
        pose = Pose()
        pose.position = Point(*trans)
        pose.orientation = Quaternion(*quat)

        self._result.pose = pose
        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._as.set_succeeded(self._result)

if __name__ == '__main__':
    # Use the same name for the actionlib client
    rospy.init_node('simple_move_sever')
    # get_name passed in the node name
    # an action server has to live inside a node
    # but a node can contain many servers.
    server = SimpleMoveAction(rospy.get_name())
    rospy.spin()