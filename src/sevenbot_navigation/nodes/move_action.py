#!/usr/bin/env python

import roslib, rospy, tf
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
from actionlib import SimpleActionServer

import sevenbot_navigation.msg

class SimpleMoveAction():
    """docstring for SimpleMoveAction"""
    def __init__(self, name):

        self.linear_speed = rospy.get_param('~linear_speed', 0.2)   # measure from robot center
        self.angular_speed = rospy.get_param('~angular_speed', 1.0)   
        self.old_position = Point()
        self.angular_tolerance = 10 # theta
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

    def execute_cb(self, goal):
        self.goal = goal
        trans, quat = self.get_odom()
        v = Vector3()
        v.x = self.goal.pose.position.x - trans[0]
        v.y = self.goal.pose.position.y - trans[1]
        v.z = -1 # make the quat angle later
        self._feedback = v
        rospy.loginfo('%s: Moving, distance to goal %f  %f, %f' % (self._action_name, self._feedback.x, self._feedback.y, self._feedback.z))

        self._result.reached = True
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