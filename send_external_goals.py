#!/usr/bin/env python3
# license removed for brevity
import sys
import time as t
import rospy
import pickle 
import math
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

marker_array = [3]#, 2, 6, 5, 1, 4]

f = open("file2.pkl", "rb")
array = pickle.load(f)
f.close()

def movebase_client(x, y, w):

    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.z = w

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py')
        for i in marker_array:
            print("Looking for marker : ", i)
            result = movebase_client((array[i][3][0]), (array[i][3][1]), array[i][4][2]%(2*math.pi))
            print("Marker Found, Waiting for 15 sec")
            t.sleep(15)
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

