#!/usr/bin/env python3

import rospy
import tf
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import *

class GoForwardAvoid():
    def __init__(self):
        rospy.init_node('nav_test', anonymous=False)
        rospy.on_shutdown(self.shutdown)

	
        self.tf_listener = tf.TransformListener(10)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("wait for the action server to come up")
        self.move_base.wait_for_server(rospy.Duration(5))

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'base_link'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = 5.0
        goal.target_pose.pose.orientation.w = 1.0

        self.move_base.send_goal(goal)

        success = self.move_base.wait_for_result(rospy.Duration(600)) 


        if not success:
            self.move_base.cancel_goal()
            rospy.loginfo("Error")
        else:
            state = self.move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Success")



    def shutdown(self):
        rospy.loginfo("Stop")


if __name__ == '__main__':
    try:
        GoForwardAvoid()
    except rospy.ROSInterruptException:
        rospy.loginfo("Exception thrown")
