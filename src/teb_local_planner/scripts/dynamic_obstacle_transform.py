#!/usr/bin/env python
# -*- coding: utf-8 -*-
from math import atan2, cos, sin, hypot, acos
from pedsim_msgs.msg import TrackedPerson, TrackedPersons
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, PoseArray, Pose
import rospy
import numpy as np

class Obstacle_Trans:
    def __init__(self):
        self.gaze_sub = rospy.Subscriber("/persons", TrackedPersons, self.pubDynamicObstaclesFromPerson, queue_size=3)
        self.pub_obs_from_person = rospy.Publisher('dynamic_obstacles', PoseArray, queue_size=1)        
        self.person_diameter = 0.6
        if rospy.has_param("pedsim_fakemap/person_diameter")==True:
            self.person_diameter = rospy.get_param("pedsim_fakemap/person_diameter")

    def pubDynamicObstaclesFromPerson(self,msg):
        res = PoseArray()
        res.header.frame_id = "odom"
        for person in msg.tracks:
            temp = Pose()
            temp.position.x = person.pose.pose.position.x
            temp.position.y = person.pose.pose.position.y
            temp.position.z = self.person_diameter/2;
            temp.orientation.x = person.twist.twist.linear.x
            temp.orientation.y = person.twist.twist.linear.y
            temp.orientation.z = 0
            temp.orientation.w = 0
            res.poses.append(temp)
        
        self.pub_obs_from_person.publish(res)

if __name__ == '__main__':
    rospy.init_node('dynamic_obstacle_listener', anonymous=True)
    gaze_box = Obstacle_Trans()
    rate = rospy.Rate(20) # 10hz
    while not rospy.is_shutdown():
        rate.sleep()





