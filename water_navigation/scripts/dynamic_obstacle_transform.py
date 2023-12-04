#!/usr/bin/env python
# -*- coding: utf-8 -*-
from math import atan2, cos, sin, hypot, acos
from pedsim_msgs.msg import TrackedPerson, TrackedPersons
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, PoseArray, Pose
from hdl_people_tracking.msg import TrackArray, Track
from geometry_msgs.msg import Point, Vector3
import rospy
import numpy as np

class Obstacle_Trans:
    def __init__(self):
        self.person_sub_sim = rospy.Subscriber("/persons", TrackedPersons, self.pubDynamicObstaclesFromSimPerson, queue_size=3)
        self.person_sub_real = rospy.Subscriber("/hdl_people_tracking_nodelet/tracks", TrackArray, self.pubDynamicObstaclesFromRealPerson, queue_size=3)

        self.pub_obs_from_person = rospy.Publisher('dynamic_obstacles', PoseArray, queue_size=1)        
        self.person_diameter = 0.6
        if rospy.has_param("pedsim_fakemap/person_diameter")==True:
            self.person_diameter = rospy.get_param("pedsim_fakemap/person_diameter")

    def pubDynamicObstaclesFromSimPerson(self,msg):
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

    # # for open space
    # def pubDynamicObstaclesFromRealPerson(self,msg):        
    #     res = PoseArray()
    #     res.header.frame_id = msg.header.frame_id
    #     for person in msg.tracks:
    #         if (len(person.associated) != 0) and (person.pos.x>1.2) and (person.pos.x<5.0) and (person.pos.y>-1.5) and (person.pos.y<1.5):
    #             temp = Pose()
    #             temp.position.x = person.pos.x
    #             temp.position.y = person.pos.y
    #             temp.position.z = self.person_diameter/2;
    #             temp.orientation.x = person.vel.x
    #             temp.orientation.y = person.vel.y
    #             temp.orientation.z = 0
    #             temp.orientation.w = 0
    #             res.poses.append(temp)

    #     self.pub_obs_from_person.publish(res)


    # for long corridor
    def pubDynamicObstaclesFromRealPerson(self,msg):        
        res = PoseArray()
        res.header.frame_id = msg.header.frame_id
        for person in msg.tracks:
            if (len(person.associated) != 0) and (person.pos.x>0) and (person.pos.x<11.0) and (person.pos.y>-1.5) and (person.pos.y<1.5):
                temp = Pose()
                temp.position.x = person.pos.x
                temp.position.y = person.pos.y
                temp.position.z = self.person_diameter/2;
                temp.orientation.x = person.vel.x
                temp.orientation.y = person.vel.y
                temp.orientation.z = 0
                temp.orientation.w = 0
                res.poses.append(temp)

        self.pub_obs_from_person.publish(res)

    # # original / generalizable
    # def pubDynamicObstaclesFromRealPerson(self,msg):        
    #     res = PoseArray()
    #     res.header.frame_id = msg.header.frame_id
    #     for person in msg.tracks:
    #         if len(person.associated) != 0:
    #             temp = Pose()
    #             temp.position.x = person.pos.x
    #             temp.position.y = person.pos.y
    #             temp.position.z = self.person_diameter/2;
    #             temp.orientation.x = person.vel.x
    #             temp.orientation.y = person.vel.y
    #             temp.orientation.z = 0
    #             temp.orientation.w = 0
    #             res.poses.append(temp)

    #     self.pub_obs_from_person.publish(res)



if __name__ == '__main__':
    rospy.init_node('dynamic_obstacle_listener', anonymous=True)
    gaze_box = Obstacle_Trans()
    rate = rospy.Rate(20) # 10hz
    while not rospy.is_shutdown():
        rate.sleep()





