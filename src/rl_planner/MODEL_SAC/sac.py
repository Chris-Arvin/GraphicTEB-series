import torch
import logging
import os
from torch.autograd import Variable
from torch.nn import functional as F
import numpy as np
import socket
from torch.utils.data.sampler import BatchSampler, SubsetRandomSampler
from net import GaussianPolicy, QNetwork_1, QNetwork_2
from utils import soft_update, hard_update
from torch.optim import Adam


### SAC
class SAC(object):
    def __init__(self, args):

        self.gamma = args.gamma
        self.tau = args.tau
        self.alpha = args.alpha

        self.policy_type = args.policy
        self.target_update_interval = args.target_update_interval
        self.automatic_entropy_tuning = args.automatic_entropy_tuning

        self.device = torch.device("cuda" if args.cuda else "cpu")

        self.critic_1 = QNetwork_1().to(device=self.device)
        self.critic_1_optim = Adam(self.critic_1.parameters(), lr=args.lr)
        self.critic_1_target = QNetwork_1().to(self.device)
        hard_update(self.critic_1_target, self.critic_1)

        self.critic_2 = QNetwork_2().to(device=self.device)
        self.critic_2_optim = Adam(self.critic_2.parameters(), lr=args.lr)
        self.critic_2_target = QNetwork_2().to(self.device)
        hard_update(self.critic_2_target, self.critic_2)

        assert(self.policy_type == "Gaussian")
        if self.automatic_entropy_tuning is True:
            self.target_entropy = -torch.prod(torch.Tensor(2).to(self.device)).item()
            self.log_alpha = torch.zeros(1, requires_grad=True, device=self.device)
            self.alpha_optim = Adam([self.log_alpha], lr=args.lr)

        self.policy = GaussianPolicy().to(self.device)
        self.policy_optim = Adam(self.policy.parameters(), lr=args.lr)


    def select_action(self, state_list, evaluate=False):
        frame_map_static_position = [[state_list[0]]]
        frame_map_dynamic_velocity_x = [[state_list[1]]]
        frame_map_dynamic_velocity_y = [[state_list[2]]]
        frame_robot_velocity = [state_list[3]]
        frame_safety_margins = [state_list[4]]
        frame_map_trajectories = [state_list[5]]

        frame_map_static_position = np.asfarray(frame_map_static_position)
        frame_map_dynamic_velocity_x = np.asfarray(frame_map_dynamic_velocity_x)
        frame_map_dynamic_velocity_y = np.asfarray(frame_map_dynamic_velocity_y)
        frame_robot_velocity = np.asfarray(frame_robot_velocity)
        frame_safety_margins = np.asfarray(frame_safety_margins)
        frame_map_trajectories = np.asfarray(frame_map_trajectories)

        frame_map_static_position = Variable(torch.from_numpy(frame_map_static_position)).float().to(self.device)
        frame_map_dynamic_velocity_x = Variable(torch.from_numpy(frame_map_dynamic_velocity_x)).float().to(self.device)
        frame_map_dynamic_velocity_y = Variable(torch.from_numpy(frame_map_dynamic_velocity_y)).float().to(self.device)
        frame_robot_velocity = Variable(torch.from_numpy(frame_robot_velocity)).float().to(self.device)
        frame_safety_margins = Variable(torch.from_numpy(frame_safety_margins)).float().to(self.device)
        frame_map_trajectories = Variable(torch.from_numpy(frame_map_trajectories)).float().to(self.device)

        # print("***")
        # print(frame_map_static_position.shape)
        # print(frame_map_dynamic_velocity_x.shape)
        # print(frame_map_dynamic_velocity_y.shape)
        # print(frame_robot_velocity.shape)
        # print(frame_safety_margins.shape)
        # print(frame_map_trajectories.shape)
        # print("***")

        #state = torch.FloatTensor(state).to(self.device).unsqueeze(0)
        if evaluate is False:
            action, _, _ = self.policy.sample(frame_map_static_position, frame_map_dynamic_velocity_x, frame_map_dynamic_velocity_y, frame_robot_velocity, frame_safety_margins, frame_map_trajectories)
        else:
            _, _, action = self.policy.sample(frame_map_static_position, frame_map_dynamic_velocity_x, frame_map_dynamic_velocity_y, frame_robot_velocity, frame_safety_margins, frame_map_trajectories)
        # print(":::", action.shape, action)
        #return action.detach().cpu().numpy()[0]
        return action.data.cpu().numpy()

    def update_parameters(self, memory, batch_size, updates):
        # Sample a batch from memory
        frame_map_static_position, frame_map_dynamic_velocity_x, frame_map_dynamic_velocity_y, frame_robot_velocity, frame_safety_margins, frame_map_trajectories, frame_action, frame_reward, next_frame_map_static_position, next_frame_map_dynamic_velocity_x, next_frame_map_dynamic_velocity_y, next_frame_robot_velocity, next_frame_safety_margins, next_frame_map_trajectories, next_frame_done = memory.sample(batch_size=batch_size)
        
        frame_map_static_position = torch.FloatTensor(frame_map_static_position).to(self.device)
        frame_map_dynamic_velocity_x = torch.FloatTensor(frame_map_dynamic_velocity_x).to(self.device)
        frame_map_dynamic_velocity_y = torch.FloatTensor(frame_map_dynamic_velocity_y).to(self.device)
        frame_robot_velocity = torch.FloatTensor(frame_robot_velocity).to(self.device)
        frame_safety_margins = torch.FloatTensor(frame_safety_margins).to(self.device)
        frame_map_trajectories = torch.FloatTensor(frame_map_trajectories).to(self.device)
        
        frame_action = torch.FloatTensor(frame_action).to(self.device)
        frame_reward = torch.FloatTensor(frame_reward).to(self.device)

        next_frame_map_static_position = torch.FloatTensor(next_frame_map_static_position).to(self.device)
        next_frame_map_dynamic_velocity_x = torch.FloatTensor(next_frame_map_dynamic_velocity_x).to(self.device)
        next_frame_map_dynamic_velocity_y = torch.FloatTensor(next_frame_map_dynamic_velocity_y).to(self.device)
        next_frame_robot_velocity = torch.FloatTensor(next_frame_robot_velocity).to(self.device)
        next_frame_safety_margins = torch.FloatTensor(next_frame_safety_margins).to(self.device)
        next_frame_map_trajectories = torch.FloatTensor(next_frame_map_trajectories).to(self.device)

        next_frame_done = torch.FloatTensor(next_frame_done).to(self.device)

        # update critic network
        with torch.no_grad():
            next_state_action, next_state_log_pi, _ = self.policy.sample(frame_map_static_position, frame_map_dynamic_velocity_x, frame_map_dynamic_velocity_y, frame_robot_velocity, frame_safety_margins, frame_map_trajectories)
            qf1_next_target = self.critic_1_target(next_frame_map_static_position, next_frame_map_dynamic_velocity_x, next_frame_map_dynamic_velocity_y, next_frame_robot_velocity, next_frame_safety_margins, next_frame_map_trajectories, next_state_action)
            qf2_next_target = self.critic_2_target(next_frame_map_static_position, next_frame_map_dynamic_velocity_x, next_frame_map_dynamic_velocity_y, next_frame_robot_velocity, next_frame_safety_margins, next_frame_map_trajectories, next_state_action)
            # print("===== ", qf1_next_target.shape, qf2_next_target.shape, next_state_log_pi.shape)
            min_qf_next_target = torch.min(qf1_next_target, qf2_next_target) - self.alpha * next_state_log_pi
            next_q_value = frame_reward + (1 - next_frame_done) * self.gamma * (min_qf_next_target)

        qf1 = self.critic_1(frame_map_static_position, frame_map_dynamic_velocity_x, frame_map_dynamic_velocity_y, frame_robot_velocity, frame_safety_margins, frame_map_trajectories, frame_action)  # Two Q-functions to mitigate positive bias in the policy improvement step
        qf2 = self.critic_2(frame_map_static_position, frame_map_dynamic_velocity_x, frame_map_dynamic_velocity_y, frame_robot_velocity, frame_safety_margins, frame_map_trajectories, frame_action)  # Two Q-functions to mitigate positive bias in the policy improvement step
        # print(qf1.shape)
        # print(next_q_value.shape)
        qf1_loss = F.mse_loss(qf1, next_q_value)  
        qf2_loss = F.mse_loss(qf2, next_q_value)  

        self.critic_1_optim.zero_grad()
        qf1_loss.backward()
        self.critic_1_optim.step()
        self.critic_2_optim.zero_grad()
        qf2_loss.backward()
        self.critic_2_optim.step()

        # update actor network
        pi, log_pi, _ = self.policy.sample(frame_map_static_position, frame_map_dynamic_velocity_x, frame_map_dynamic_velocity_y, frame_robot_velocity, frame_safety_margins, frame_map_trajectories)

        qf1_pi = self.critic_1(frame_map_static_position, frame_map_dynamic_velocity_x, frame_map_dynamic_velocity_y, frame_robot_velocity, frame_safety_margins, frame_map_trajectories, pi)
        qf2_pi = self.critic_2(frame_map_static_position, frame_map_dynamic_velocity_x, frame_map_dynamic_velocity_y, frame_robot_velocity, frame_safety_margins, frame_map_trajectories, pi)
    
        min_qf_pi = torch.min(qf1_pi, qf2_pi)

        policy_loss = ((self.alpha * log_pi) - min_qf_pi).mean() 

        self.policy_optim.zero_grad()
        policy_loss.backward()
        self.policy_optim.step()
        
        if self.automatic_entropy_tuning:
            alpha_loss = -(self.log_alpha * (log_pi + self.target_entropy).detach()).mean()

            self.alpha_optim.zero_grad()
            alpha_loss.backward()
            self.alpha_optim.step()

            self.alpha = self.log_alpha.exp()
            alpha_tlogs = self.alpha.clone() # For TensorboardX logs
        else:
            alpha_loss = torch.tensor(0.).to(self.device)
            alpha_tlogs = torch.tensor(self.alpha) # For TensorboardX logs


        if updates % self.target_update_interval == 0:

            soft_update(self.critic_1_target, self.critic_1, self.tau)
            soft_update(self.critic_2_target, self.critic_2, self.tau)

        return qf1_loss.item(), qf2_loss.item(), policy_loss.item(), alpha_loss.item(), alpha_tlogs.item()



