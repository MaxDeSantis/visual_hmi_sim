#! /usr/bin/env python3

import roslib
roslib.load_manifest('visual_hmi')
import rospy
import actionlib

from visual_hmi.msg import SelectGoalLocationAction, SelectGoalLocationGoal

if __name__ == '__main__':
    rospy.init_node('Select_Goal_Client')
    client = actionlib.SimpleActionClient('Select_Goal', SelectGoalLocationAction)
    client.wait_for_server()

    goal = SelectGoalLocationGoal()

    print("SENDING")
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))
