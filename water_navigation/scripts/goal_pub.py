#!/usr/bin/env python
# -*- coding: utf-8 -*-
from matplotlib.pyplot import twinx
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


class Obstacle_Trans:
    def __init__(self):
        self.person_sub_sim = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pubGoal, queue_size=3)
        self.pub_goal = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)        
        self.is_pubed = False
        self.goal1 = PoseStamped()
        self.goal1.header.frame_id = "map"
        self.goal1.header.stamp = rospy.Time.now()
        self.goal1.pose.position.x = 0.1
        self.goal1.pose.position.y = 0.0
        self.goal1.pose.position.z = 0.0
        self.goal1.pose.orientation.x = 0.0
        self.goal1.pose.orientation.y = 0.0
        self.goal1.pose.orientation.z = 0.0
        self.goal1.pose.orientation.w = 1.0
        
        self.goal2 = PoseStamped()
        self.goal2.header.frame_id = "map"
        self.goal2.header.stamp = rospy.Time.now()
        self.goal2.pose.position.x = 10.0
        self.goal2.pose.position.y = -0.4
        self.goal2.pose.position.z = 0.0
        self.goal2.pose.orientation.x = 0.0
        self.goal2.pose.orientation.y = 0.0
        self.goal2.pose.orientation.z = 0.0
        self.goal2.pose.orientation.w = 1.0        

    def pubGoal(self,msg):
        if (not self.is_pubed) and (msg.pose.pose.position.x<0.3):
            self.goal2.header.stamp = rospy.Time.now()
            self.pub_goal.publish(self.goal2)
            self.is_pubed = True
        if (not self.is_pubed) and (msg.pose.pose.position.x>9.7):
            self.goal1.header.stamp = rospy.Time.now()
            self.pub_goal.publish(self.goal1)
            self.is_pubed = True
        if (msg.pose.pose.position.x>1.0) and (msg.pose.pose.position.x<9.0):
            self.is_pubed = False

if __name__ == '__main__':
    rospy.init_node('dynamic_obstacle_listener', anonymous=True)
    gaze_box = Obstacle_Trans()
    rate = rospy.Rate(20) # 10hz
    while not rospy.is_shutdown():
        rate.sleep()





