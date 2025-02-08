#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
sys.path.append("../MODEL_SAC")

import time
import rospy
import rospkg 

import math

import copy
import numpy as np

from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import PoseArray
from rl_planner.srv import rl_state, rl_stateRequest, rl_stateResponse
import os
from torch._C import is_anomaly_enabled
import torch
import torch.nn as nn
import argparse
import random
from torch.utils.tensorboard import SummaryWriter
from collections import deque

from utils import hard_update
from sac import SAC
from replay_memory import ReplayMemory

# todo: 结束的可能：与行人碰撞、与静态障碍物碰撞、找不到轨迹、error
class env_gazebo:
    def __init__(self):
        rospy.init_node("rl_interface", anonymous=None)
        current_time = rospy.Time.now().to_sec()
        self.init_parameters()
        self.agent = None
        self.load_model()

        self.people_list_ = []
        self.robot_pos = []
        self.collision_time = rospy.Time.now().to_sec()-1
        self.reach_goal_time = rospy.Time.now().to_sec()-1

        self.current_state_ = None
        self.last_state_ = None
        
        self.current_action_ = [0,0]
        self.last_action_ = [0,0]

        self.last_reward_ = 0
        self.current_reward_ = 0

        self.writer = SummaryWriter('policy_events/')
        self.memory = ReplayMemory(self.args.replay_size, self.args.seed)
        self.updates = 0

        rospy.Service('/rl_state_service', rl_state, self.rl_state_server)
        rospy.Subscriber('/dynamic_obstacles', PoseArray, self.people_callback)
        rospy.Subscriber('/odom', Odometry, self.odometry_callback)
        rospy.Subscriber('/is_reach_goal', Bool, self.goal_callback)
        self.rl_state_static_pub_ = rospy.Publisher('/rl_state_static', MarkerArray, queue_size=1)
        self.rl_state_dynamic_pub_ = rospy.Publisher('/rl_state_dynamic', MarkerArray, queue_size=1)
        self.rl_state_trajectories_pub_ = rospy.Publisher('/rl_state_trajectories', MarkerArray, queue_size=1)

        print("model initialized by {} s".format(rospy.Time.now().to_sec()-current_time))


    def rl_state_server(self, req):
        print("-"*50)
        current_time = rospy.Time.now().to_sec()
        print("    rl state server is cliented")
        self.last_state_ = self.current_state_
        self.last_action_ = self.current_action_
        self.last_reward_ = self.current_reward_
        self.current_state_ = self.analyze_current_state(req)
        self.current_action_ = self.get_action(self.current_state_)
        print("    ======== {} ========".format(self.current_action_))
        done, info, self.current_reward_ = self.get_current_reward()
        if self.last_state_ is not None:
            self.memory.push(self.last_state_, self.last_action_, self.last_reward_, self.current_state_, done)
            # test start
            if len(self.memory)<10:
                for i in range(5*self.args.batch_size):
                    self.memory.push(self.last_state_, self.last_action_, self.last_reward_, self.current_state_, done)
            self.memory.sample(batch_size=self.args.batch_size)
            # test end
        self.train_policy()
        print("    info:", info, "done:", done, "action:", self.current_action_, "reward:", self.current_reward_)
        res = rl_stateResponse()
        res.static_safety_margin = self.current_action_[0]
        res.dynamic_safety_margin = self.current_action_[1]
        # self.publish_states()
        print("    rl state server has been responsed in {} s".format(rospy.Time.now().to_sec()-current_time))
        print("-"*50)
        print("")
        return res
    
    def get_action(self, state):
        action = self.agent.select_action(state)[0]
        action_clip_bound = [[0.0, 0.0], [0.3, 0.3]] #### Action maximum, minimum values 
        cliped_action = np.clip(action, a_min=action_clip_bound[0], a_max=action_clip_bound[1])
        return cliped_action

    def get_current_reward(self):
        done = -1
        info = ""
        reward = -1
        w_collision = 100
        w_safety_margin = 1
        w_change = 1
        if rospy.Time.now().to_sec()-self.collision_time<0.1:
            reward = w_safety_margin*(self.current_action_[0]+self.current_action_[1]) + w_change*(abs(self.current_action_[0]-self.last_action_[0])+abs(self.current_action_[1]-self.last_action_[1])) + -w_collision*1 
            done = 1
            info = "ended: collide"
        elif rospy.Time.now().to_sec()-self.reach_goal_time < 0.1:
            reward = w_safety_margin*(self.current_action_[0]+self.current_action_[1]) + w_change*(abs(self.current_action_[0]-self.last_action_[0])+abs(self.current_action_[1]-self.last_action_[1]))
            done = 1
            info = "ended: reach goal"
        else:
            reward = w_safety_margin*(self.current_action_[0]+self.current_action_[1]) + w_change*(abs(self.current_action_[0]-self.last_action_[0])+abs(self.current_action_[1]-self.last_action_[1]))
            done = 0
            info = "keep moving"
        assert(done is not -1)
        return done, info, reward

    def goal_callback(self):
        self.reach_goal_time = rospy.Time.now().to_sec()

    def init_parameters(self):
        parser = argparse.ArgumentParser(description='PyTorch Soft Actor-Critic Args')
        parser.add_argument('--policy', default="Gaussian",
                            help='Policy Type: Gaussian | Deterministic (default: Gaussian)')
        parser.add_argument('--eval', type=bool, default=True,
                            help='Evaluates a policy a policy every 10 episode (default: True)')
        parser.add_argument('--gamma', type=float, default=0.99, metavar='G',
                            help='discount factor for reward (default: 0.99)')
        parser.add_argument('--tau', type=float, default=0.005, metavar='G',
                            help='target smoothing coefficient(\tau) (default: 0.005)')
        parser.add_argument('--lr', type=float, default=0.0003, metavar='G',
                            help='learning rate (default: 0.0003)')
        parser.add_argument('--alpha', type=float, default=0.2, metavar='G',
                            help='Temperature parameter \alpha determines the relative importance of the entropy\
                                    term against the reward (default: 0.2)')
        parser.add_argument('--automatic_entropy_tuning', type=bool, default=True, metavar='G',
                            help='Automaically adjust \alpha (default: False)')
        parser.add_argument('--seed', type=int, default=123456, metavar='N',
                            help='random seed (default: 123456)')
        parser.add_argument('--batch_size', type=int, default=64, metavar='N',
                            help='batch size (default: 256)')
        parser.add_argument('--updates_per_step', type=int, default=80, metavar='N',
                            help='model updates per simulator step (default: 1)')
        parser.add_argument('--target_update_interval', type=int, default=1, metavar='N',
                            help='Value target update per no. of updates per step (default: 1)')
        parser.add_argument('--replay_size', type=int, default=160000, metavar='N',
                            help='size of replay buffer (default: 10000000)')
        parser.add_argument('--cuda', action="store_true",default=True,
                            help='run on CUDA (default: False)')
        parser.add_argument('--epoch', type=int, default=1,
                            help='Epoch (default: 1)')
        parser.add_argument('--discount_factor', type=float, default=0.97, metavar='N',
                            help='discount_factor for reward (default: 0.97)')                                        
        self.args = parser.parse_args()

    def load_model(self):
        policy_path = 'trained_policy'
        self.agent = SAC(args=self.args)
        if not os.path.exists(policy_path):
            os.makedirs(policy_path)

        file_policy = policy_path + '/new_policy_epi_0' 
        file_critic_1 = policy_path + '/new_critic_1_epi_0'
        file_critic_2 = policy_path + '/new_critic_2_epi_0'

        if os.path.exists(file_policy):
            print('############Loading Policy Model###########')
            print(file_policy)
            if self.args.cuda:
                state_dict = torch.load(file_policy)
            else:
                state_dict = torch.load(file_policy,map_location=torch.device("cuda" if self.args.cuda else "cpu"))
            self.agent.policy.load_state_dict(state_dict)
        else:
            print('############Start policy Training###########')

        if os.path.exists(file_critic_1):
            print('############Loading critic_1 Model###########')
            print(file_critic_1)
            if self.args.cuda:
                state_dict = torch.load(file_critic_1)
            else:
                state_dict = torch.load(file_critic_1,map_location=torch.device("cuda" if self.args.cuda else "cpu"))
            self.agent.critic_1.load_state_dict(state_dict)
            hard_update(self.agent.critic_1_target, self.agent.critic_1)
        else:
            print('############Start critic_1 Training###########')

        if os.path.exists(file_critic_2):
            print('############Loading critic_2 Model###########')
            print(file_critic_2)
            if self.args.cuda:
                state_dict = torch.load(file_critic_2)
            else:
                state_dict = torch.load(file_critic_2,map_location=torch.device("cuda" if self.args.cuda else "cpu"))
            self.agent.critic_2.load_state_dict(state_dict)
            hard_update(self.agent.critic_2_target, self.agent.critic_2)
        else:
            print('############Start critic_2 Training###########')

    def train_policy(self):
        for i in range(self.args.updates_per_step):
            # Update parameters of all the networks
            if len(self.memory) > self.args.batch_size:
                current_time = rospy.Time.now().to_sec()
                critic_1_loss, critic_2_loss, policy_loss, ent_loss, alpha = self.agent.update_parameters(self.memory, self.args.batch_size, self.updates)
                self.writer.add_scalar('loss/critic_1', critic_1_loss, self.updates)
                self.writer.add_scalar('loss/critic_2', critic_2_loss, self.updates)
                self.writer.add_scalar('loss/policy', policy_loss, self.updates)
                self.writer.add_scalar('loss/entropy_loss', ent_loss, self.updates)
                self.writer.add_scalar('entropy_temprature/alpha', alpha, self.updates)
                self.updates += 1
                print("train: {}->{}/{} {}".format(self.updates, i, self.args.updates_per_step, rospy.Time.now().to_sec()-current_time))

    def analyze_current_state(self, req):
        current_state = []

        self.width_ = int(math.sqrt(len(req.map_static_position)))
        assert(self.width_ == 120)
        # 0. map static position
        map_statc_position = [[0 for _ in range(self.width_)] for _ in range(self.width_)]
        for i in range(len(req.map_static_position)):
            x = math.floor(i/self.width_)
            y = i%self.width_
            map_statc_position[x][y] = req.map_static_position[i]
        current_state.append(map_statc_position)
        # 1/2. map dynamic velocity_x && map dynamic velocity_y
        map_dynamic_velocity_x = [[0 for _ in range(self.width_)] for _ in range(self.width_)]
        map_dynamic_velocity_y = [[0 for _ in range(self.width_)] for _ in range(self.width_)]
        assert(self.width_*self.width_ == len(req.map_dynamic_velocity_x))

        for i in range(len(req.map_dynamic_velocity_x)):
            x = math.floor(i/self.width_)
            y = i%self.width_
            map_dynamic_velocity_x[x][y] = req.map_dynamic_velocity_x[i]
            map_dynamic_velocity_y[x][y] = req.map_dynamic_velocity_y[i]
        current_state.append(map_dynamic_velocity_x)
        current_state.append(map_dynamic_velocity_y)
        # 3. [vx, vy]
        current_state.append([req.vx, req.vy])
        # 4. last_static_safety_margin, last_dynamic_safety_margin, 
        current_state.append([req.last_static_safety_margin, req.last_dynamic_safety_margin])
        # 5. trajectories
        trajectory_map_lists = []
        # assert( len(req.trajectories.markers)<=10 )
        for i in range(10):
            trajectory_map_lists.append([[0 for _ in range(self.width_)] for _ in range(self.width_)])
        for i in range(len(req.trajectories.markers)):
            for p in req.trajectories.markers[i].points:
                trajectory_map_lists[-i][int(p.x)][int(p.y)] = 1
        current_state.append(trajectory_map_lists)
        
        return current_state

    def people_callback(self, topic):
        if len(self.robot_pos)==2:
            for person in topic.poses:
                # 机器人的半径：robot_constrains.yaml的robot_radius: 0.3； 人的半径：sim_map.yaml的person_diameter或dynamic_obstacle_transform.py: 
                if math.hypot(self.robot_pos[0]-person.position.x, self.robot_pos[1]-person.position.y)<0.3 + 0.3:
                    self.collision_time = rospy.Time.now().to_sec()
                    break
        print("current time: ", rospy.Time.now().to_sec(), "collision time:", self.collision_time)

    def odometry_callback(self, topic):
        self.robot_pos = [topic.pose.pose.position.x, topic.pose.pose.position.y]

    def publish_states(self):
        # 0. map static position
        marker_array_1 = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "1_map_static_position"
        marker.id = 0
        marker.action = Marker.ADD
        marker.type = Marker.POINTS
        marker.pose.orientation.w = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.scale.x = 0.1  # 箭头轴的宽度
        marker.scale.y = 0.1  # 箭头头部的宽度
        marker.scale.z = 0.1  # 箭头头部的高度
        marker.lifetime = rospy.Duration(1)
        for x in range(self.width_):
            for y in range(self.width_):
                if self.current_state_[0][x][y]:
                    point = Point()
                    point.x = x*0.1 -5.9 + self.robot_pos[0]
                    point.y = y*0.1 -5.9 + self.robot_pos[1]
                    point.z = 0
                    marker.points.append(point)
        if len(marker.points)>0:
            marker_array_1.markers.append(marker)
        self.rl_state_static_pub_.publish(marker_array_1)
        
        # 1/2. map_dynamic_velocity
        marker_array_2 = MarkerArray()
        count = 0
        for x in range(self.width_):
            for y in range(self.width_):
                if self.current_state_[1][x][y]!=0 or self.current_state_[2][x][y]!=0:
                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.header.stamp = rospy.Time.now()
                    marker.action = Marker.ADD
                    marker.type = Marker.ARROW
                    marker.pose.orientation.w = 1.0
                    marker.color.r = 0.0
                    marker.color.g = 0.0
                    marker.color.b = 1.0
                    marker.color.a = 1.0
                    # 设置箭头的尺寸（缩放比例）
                    marker.scale.x = 0.01  # 箭头轴的宽度
                    marker.scale.y = 0.05  # 箭头头部的宽度
                    marker.scale.z = 0.05  # 箭头头部的高度
                    marker.lifetime = rospy.Duration(1)
                    marker.id = count
                    marker.ns = str(count)
                    count = count+1
                    start_point = Point()
                    start_point.x = x*0.1 -5.9 + self.robot_pos[0]
                    start_point.y = y*0.1 -5.9 + self.robot_pos[1]
                    start_point.z = 0.1
                    marker.points.append(start_point)                    
                    end_point = Point()
                    end_point.x = start_point.x + self.current_state_[1][x][y]
                    end_point.y = start_point.y + self.current_state_[2][x][y]
                    end_point.z = 0.1
                    marker.points.append(end_point)
                    marker_array_2.markers.append(marker)
        self.rl_state_dynamic_pub_.publish(marker_array_2)

        # 5. trajectories
        marker_array_3 = MarkerArray()
        trajectory_maps = self.current_state_[5]
        color_list = [
            [0.754, 0.234, 0.674],
            [0.934, 0.123, 0.549],
            [0.385, 0.872, 0.430],
            [0.612, 0.295, 0.714],
            [0.123, 0.801, 0.562],
            [0.879, 0.430, 0.342],
            [0.678, 0.120, 0.543],
            [0.234, 0.901, 0.675],
            [0.542, 0.231, 0.908],
            [0.749, 0.657, 0.842]
        ]
        count = 0
        for i in range(len(trajectory_maps)):
            traj_map = trajectory_maps[i]
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.action = Marker.ADD
            marker.type = Marker.POINTS
            marker.pose.orientation.w = 1.0
            marker.color.r = color_list[i][0]
            marker.color.g = color_list[i][1]
            marker.color.b = color_list[i][2]
            marker.color.a = 1.0
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.lifetime = rospy.Duration(1)
            marker.id = count
            marker.ns = str(count)
            count = count+1
            for x in range(self.width_):
                for y in range(self.width_):
                    if traj_map[x][y]==1:
                        poi = Point()
                        poi.x = x*0.1 -5.9 + self.robot_pos[0]
                        poi.y = y*0.1 -5.9 + self.robot_pos[1]
                        poi.z = 0.1
                        marker.points.append(poi)
                        marker_array_3.markers.append(marker)
        self.rl_state_trajectories_pub_.publish(marker_array_3)

def main():
    env = env_gazebo()
    rospy.spin()  # 等待调用


if __name__ == '__main__':
    main()