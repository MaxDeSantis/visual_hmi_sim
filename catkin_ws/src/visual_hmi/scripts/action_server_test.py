#! /usr/bin/env python3

import roslib
roslib.load_manifest('visual_hmi')
import rospy
import actionlib

from visual_hmi.msg import SelectGoalLocationAction

class GoalServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('Select_Goal', SelectGoalLocationAction, self.execute, False)

        self.server.start()

    def execute(self, goal):
        self.server.set_succeeded()
        print("server")

    
if __name__ == '__main__':
    rospy.init_node('Select_Goal_Server')
    server = GoalServer()
    rospy.spin()