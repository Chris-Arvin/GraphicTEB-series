import random
import numpy as np

class ReplayMemory:
    def __init__(self, capacity, seed):
        random.seed(seed)
        self.capacity = capacity
        self.buffer = []
        self.position = 0

    #def list_push(self, state_list, action_list, reward_list, next_state_list, done_list):
    #    s_list, goal_list, speed_list = [], [], []

    def push(self, current_state, action, reward, next_state, done):
        if len(self.buffer) < self.capacity:
            self.buffer.append(None)
        self.buffer[self.position] = [[current_state[0]], [current_state[1]], [current_state[2]], current_state[3], current_state[4], current_state[5], action, [reward], [next_state[0]], [next_state[1]], [next_state[2]], next_state[3], next_state[4], next_state[5], [done]]
        self.position = (self.position + 1) % self.capacity 

    def sample(self, batch_size):
        batch = random.sample(self.buffer, batch_size)
        map_static_position, map_dynamic_velocity_x, map_dynamic_velocity_y, robot_velocity, safety_margins, map_trajectories, action, reward, next_map_static_position, next_map_dynamic_velocity_x, next_map_dynamic_velocity_y, next_robot_velocity, next_safety_margins, next_map_trajectories, done = zip(*batch)
        frame_map_static_position = np.array(map_static_position)
        # print("-----")
        frame_map_dynamic_velocity_x = np.array(map_dynamic_velocity_x)
        frame_map_dynamic_velocity_y = np.array(map_dynamic_velocity_y)
        frame_robot_velocity = np.array(robot_velocity)
        frame_safety_margins = np.array(safety_margins)
        frame_map_trajectories = np.array(map_trajectories)
        # print(frame_map_static_position.shape)
        # print(frame_map_dynamic_velocity_x.shape)
        # print(frame_map_dynamic_velocity_y.shape)
        # print(frame_robot_velocity.shape)
        # print(frame_safety_margins.shape)
        # print(frame_map_trajectories.shape)

        frame_action = np.array(action)
        frame_reward = np.array(reward)
        # print(frame_action.shape)
        # print(frame_reward.shape)

        next_frame_map_static_position = np.array(next_map_static_position)
        next_frame_map_dynamic_velocity_x = np.array(next_map_dynamic_velocity_x)
        next_frame_map_dynamic_velocity_y = np.array(next_map_dynamic_velocity_y)
        next_frame_robot_velocity = np.array(next_robot_velocity)
        next_frame_safety_margins = np.array(next_safety_margins)
        next_frame_map_trajectories = np.array(next_map_trajectories)
        # print(next_frame_map_static_position.shape)
        # print(next_frame_map_dynamic_velocity_x.shape)
        # print(next_frame_map_dynamic_velocity_y.shape)
        # print(next_frame_robot_velocity.shape)
        # print(next_frame_safety_margins.shape)
        # print(next_frame_map_trajectories.shape)
        
        next_frame_done = np.array(done)
        # print(next_frame_done.shape)
        # print("-----")

        return frame_map_static_position, frame_map_dynamic_velocity_x, frame_map_dynamic_velocity_y, frame_robot_velocity, frame_safety_margins, frame_map_trajectories, frame_action, frame_reward, next_frame_map_static_position, next_frame_map_dynamic_velocity_x, next_frame_map_dynamic_velocity_y, next_frame_robot_velocity, next_frame_safety_margins, next_frame_map_trajectories, next_frame_done


    def __len__(self):
        return len(self.buffer)
