#! /usr/bin/env python3

import queue
from tkinter import E
from warnings import formatwarning

from cv2 import pointPolygonTest
import roslib
roslib.load_manifest('visual_hmi')
import rospy
import actionlib
import math

from visual_hmi.msg import SelectGoalLocationAction
from visual_hmi.msg import SelectGoalLocationFeedback
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
import tf2_ros
import tf2_geometry_msgs


class GoalServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('Select_Goal', SelectGoalLocationAction, self.Position, False)
        self.server.start()

        #self.rotate_server = actionlib.SimpleActionServer('Rotate', Twist, self.Rotate, False)
        #self.rotate_server.start()

        rospy.Subscriber('/camera/depth/image_raw', Image, self.GetDepthImage)
        rospy.Subscriber('/odom', Odometry, self.GetOdom)
        self.bridge = CvBridge()
        self.ANGLERATIO = 0.0324 * math.pi/180 #rad per px

        self.pointPub = rospy.Publisher('/visual_hmi/point_selected', PointStamped, queue_size=1)
        self.navPub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
    def GetOdom(self, msg):
        pose = PoseStamped()
        pose.pose.orientation = msg.pose.pose.orientation
        try:
            transform = self.tf_buffer.lookup_transform("camera_link", msg.header.frame_id, msg.header.stamp, rospy.Duration(1.0))
            newPose = tf2_geometry_msgs.do_transform_pose(pose, transform)
            self.orientation = newPose.pose.orientation
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("get orientation failed")

    def GetDepthImage(self, msg):
        try:
            depth_im = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
            self.im = depth_im
        except CvBridgeError:
            print("uh oh")
        

    def GetXYZ(self, goalX, goalY):
        print("getting xyz")
        #print(self.im.height, self.image.width, self.image.point_step, self.image.row_step)
        #now, how to transform from camera XY to world XYZ?
        # image in camera_rgb_optical_frame
        depth = self.im[int(goalY), int(goalX)]
        print("DEPTH: ", depth)
        (row, col) = self.im.shape
        hor_angle = (col/2 - goalX)*self.ANGLERATIO
        print("ANGLE: ", hor_angle, hor_angle * 180/math.pi)
        forward = depth * math.cos(hor_angle)
        side = depth * math.sin(hor_angle)
        print("FOR:", forward, " SIDE:", side)

        # Build point
        point = PointStamped()
        point.point.x = forward
        point.point.y = side
        point.point.z = 0
        point.header.frame_id = "camera_link"

        # Build pose
        pointPose = PoseStamped()
        pointPose.pose.position.x = forward
        pointPose.pose.position.y = side
        pointPose.pose.position.z = 0
        pointPose.header.frame_id = "camera_link"
        pointPose.pose.orientation = self.orientation
        
        
        # Attempt to transform point to map frame
        try:
            transform = self.tf_buffer.lookup_transform("map", point.header.frame_id, point.header.stamp, rospy.Duration(1.0))
            pt = tf2_geometry_msgs.do_transform_point(point, transform)
            self.pointPub.publish(pt)

            transform = self.tf_buffer.lookup_transform("map", pointPose.header.frame_id, pointPose.header.stamp, rospy.Duration(1.0))
            navPose = tf2_geometry_msgs.do_transform_pose(pointPose, transform)
            self.navPub.publish(navPose)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("transform fail")
        



    def Position(self, goal):
        self.server.set_succeeded()
        _feedback = SelectGoalLocationFeedback()
        _feedback.feedback = 3
        self.server.publish_feedback(_feedback)
        #print("server execute")

        self.GetXYZ(goal.x, goal.y)

    
if __name__ == '__main__':
    rospy.init_node('Select_Goal_Server')
    server = GoalServer()
    
    rospy.spin()