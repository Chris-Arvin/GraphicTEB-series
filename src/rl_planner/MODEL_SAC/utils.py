import numpy as np
import bisect
import torch
from torch.autograd import Variable
import math
import random

env1_index = 0
env2_index = 0
env3_index = 0
env4_index = 0
env5_index = 0
env6_index = 0
env7_index = 0
env8_index = 0
env9_index = 0
env10_index = 0

# def get_start_and_goal_point(index):
#     return env9_point() 

def get_start_and_goal_point(index):
    idx = index%9
    return weaving_scenario([0, 12-3*idx])


def weaving_scenario(center=[0,0]):
    start = [center[0]-3.3, center[1], 0]
    target = [center[0]+3.3, center[1]]
    return start, target

def dynamic_scenario(center=[0,0]):
    start = [center[0]-3.3, center[1], 0]
    target = [center[0]+3.3, center[1]]
    return start, target



def get_all_points():
    res = []
    return res


def soft_update(target, source, tau):
    for target_param, param in zip(target.parameters(), source.parameters()):
        target_param.data.copy_(target_param.data * (1.0 - tau) + param.data * tau)

def hard_update(target, source):
    for target_param, param in zip(target.parameters(), source.parameters()):
        target_param.data.copy_(param.data)
