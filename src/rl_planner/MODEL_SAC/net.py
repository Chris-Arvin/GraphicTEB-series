import math
import numpy as np
import torch
import torch.nn as nn
from torch.nn import init
from torch.nn import functional as F
from torch.distributions import Normal


LOG_SIG_MAX = 2
LOG_SIG_MIN = -20
epsilon = 1e-6

# Initialize Policy weights
def weights_init_(m):
    if isinstance(m, nn.Linear):
        torch.nn.init.xavier_uniform_(m.weight, gain=1)
        torch.nn.init.constant_(m.bias, 0)


width = 120

class QNetwork_1(nn.Module):
    def __init__(self):
        super(QNetwork_1, self).__init__()
        # 1. 处理trajectories
        self.downsampling = nn.MaxPool2d(kernel_size=2, stride=2)
        self.trajectory_lstm = nn.LSTM(input_size=60*60, hidden_size=512, num_layers=1, batch_first=True)

        # 2. 同时处理3个map
        # 2.1 in_channels为3的conv2d：
        self.conv2d_all_map = nn.Conv2d(in_channels=3, out_channels=8, kernel_size=7, stride=1, padding='same')
        # 2.2 bottleneck        
        self.bottleneck_all_map1 = nn.Sequential(
            # 1x1 卷积：降维
            nn.Conv2d(in_channels=8, out_channels=4, kernel_size=1, stride=1, padding='same', bias=False),
            nn.BatchNorm2d(num_features=4),
            nn.ReLU(inplace=True),
            # 3x3 卷积：提取特征
            nn.Conv2d(in_channels=4, out_channels=4, kernel_size=3, stride=1, padding='same', bias=False),
            nn.BatchNorm2d(num_features=4),
            nn.ReLU(inplace=True),
            # 1x1 卷积：恢复通道数
            nn.Conv2d(in_channels=4, out_channels=8, kernel_size=1, stride=1, padding='same', bias=False), 
            nn.BatchNorm2d(num_features=8),
            nn.ReLU(inplace=True)
        )
        # 2.3 bottleneck        
        self.bottleneck_all_map2 = nn.Sequential(
            # 1x1 卷积：降维
            nn.Conv2d(in_channels=8, out_channels=4, kernel_size=1, stride=1, padding='same', bias=False),
            nn.BatchNorm2d(num_features=4),
            nn.ReLU(inplace=True),
            # 3x3 卷积：提取特征
            nn.Conv2d(in_channels=4, out_channels=4, kernel_size=3, stride=1, padding='same', bias=False),
            nn.BatchNorm2d(num_features=4),
            nn.ReLU(inplace=True),
            # 1x1 卷积：恢复通道数
            nn.Conv2d(in_channels=4, out_channels=8, kernel_size=1, stride=1, padding='same', bias=False), 
            nn.BatchNorm2d(num_features=8),
            nn.ReLU(inplace=True)
        )
        
        # 3. 处理动态map
        # 3.1 in_channels为2的conv2d：
        self.conv2d_dynamic_map = nn.Conv2d(in_channels=2, out_channels=8, kernel_size=7, stride=1, padding='same')
        # 3.2 bottleneck        
        self.bottleneck_dynamic_map1 = nn.Sequential(
            # 1x1 卷积：降维
            nn.Conv2d(in_channels=8, out_channels=4, kernel_size=1, stride=1, padding='same', bias=False),
            nn.BatchNorm2d(num_features=4),
            nn.ReLU(inplace=True),
            # 3x3 卷积：提取特征
            nn.Conv2d(in_channels=4, out_channels=4, kernel_size=3, stride=1, padding='same', bias=False),
            nn.BatchNorm2d(num_features=4),
            nn.ReLU(inplace=True),
            # 1x1 卷积：恢复通道数
            nn.Conv2d(in_channels=4, out_channels=8, kernel_size=1, stride=1, padding='same', bias=False), 
            nn.BatchNorm2d(num_features=8),
            nn.ReLU(inplace=True)
        )
        # 3.3 bottleneck        
        self.bottleneck_dynamic_map2 = nn.Sequential(
            # 1x1 卷积：降维
            nn.Conv2d(in_channels=8, out_channels=4, kernel_size=1, stride=1, padding='same', bias=False),
            nn.BatchNorm2d(num_features=4),
            nn.ReLU(inplace=True),
            # 3x3 卷积：提取特征
            nn.Conv2d(in_channels=4, out_channels=4, kernel_size=3, stride=1, padding='same', bias=False),
            nn.BatchNorm2d(num_features=4),
            nn.ReLU(inplace=True),
            # 1x1 卷积：恢复通道数
            nn.Conv2d(in_channels=4, out_channels=8, kernel_size=1, stride=1, padding='same', bias=False), 
            nn.BatchNorm2d(num_features=8),
            nn.ReLU(inplace=True)
        )

        # 4. 合并处理all_map 和 trajectory_trigger、safety_margins和current_velocities
        self.merge_all_map_with_others1 = nn.Linear(8 * int(width/4) * int(width/4) + 512, 1024)
        self.merge_all_map_with_others2 = nn.Linear(1024+4, 512)

        # 5. 合并处理dynamic_map 和 trajectory_trigger、safety_margins和current_velocities
        self.merge_dynamic_map_with_others1 = nn.Linear(8 * int(width/4) * int(width/4) + 512, 1024)
        self.merge_dynamic_map_with_others2 = nn.Linear(1024+4, 512)

        # 6. 合并两个merge
        self.flatten = nn.Flatten(start_dim=1)
        self.merge_all_data1 = nn.Linear(1024, 512)
        self.merge_all_data2 = nn.Linear(512+2, 256)
        self.merge_all_data3 = nn.Linear(256,64)
        self.merge_all_data4 = nn.Linear(64,1)

        self.apply(weights_init_)


    def forward(self, frame_map_static_position, frame_map_dynamic_velocity_x, frame_map_dynamic_velocity_y, frame_robot_velocity, frame_safety_margins, frame_map_trajectories, frame_action):
        # 1. 处理frame_map_trajectories，得到一个trigger: {batch_size, channel=1, hidden_size=256}
        frame_map_trajectories = self.downsampling(frame_map_trajectories)
        frame_map_trajectories = frame_map_trajectories.view(frame_map_trajectories.shape[0], 10, -1)
        output, (h_n, c_n) = frame_map_trajectories = self.trajectory_lstm(frame_map_trajectories)
        output = torch.permute(output,[1,0,2])
        trigger = output[-1]
        # print("111", trigger.shape)

        # 2. 处理整个map
        merged_all_map = torch.cat([frame_map_static_position, frame_map_dynamic_velocity_x, frame_map_dynamic_velocity_y], dim=1)
        merged_all_map = self.conv2d_all_map(merged_all_map)
        merged_all_map = self.bottleneck_all_map1(merged_all_map) + merged_all_map
        merged_all_map = F.avg_pool2d(merged_all_map, kernel_size=2, stride=2)
        merged_all_map = self.bottleneck_all_map2(merged_all_map) + merged_all_map
        merged_all_map = F.avg_pool2d(merged_all_map, kernel_size=2, stride=2)
        merged_all_map = merged_all_map.view(merged_all_map.shape[0], 1, -1)
        merged_all_map = self.flatten(merged_all_map)
        # print("111", merged_all_map.shape)

        # 3. 处理动态map
        merged_dynamic_map = torch.cat([frame_map_dynamic_velocity_x, frame_map_dynamic_velocity_y], dim=1)
        merged_dynamic_map = self.conv2d_dynamic_map(merged_dynamic_map)
        merged_dynamic_map = self.bottleneck_dynamic_map1(merged_dynamic_map) + merged_dynamic_map
        merged_dynamic_map = F.avg_pool2d(merged_dynamic_map, kernel_size=2, stride=2)
        merged_dynamic_map = self.bottleneck_dynamic_map2(merged_dynamic_map) + merged_dynamic_map
        merged_dynamic_map = F.avg_pool2d(merged_dynamic_map, kernel_size=2, stride=2)
        merged_dynamic_map = merged_dynamic_map.view(merged_dynamic_map.shape[0], 1, -1)
        merged_dynamic_map = self.flatten(merged_dynamic_map)
        # print("111", merged_dynamic_map.shape)

        # 4. 合并处理all_map 和 trajectory_trigger、safety_margins和current_velocities
        marged_data1 = torch.cat([merged_all_map, trigger], dim=1)
        marged_data1 = F.relu(self.merge_all_map_with_others1(marged_data1), inplace=True)
        merged_data1 = torch.cat([marged_data1, frame_robot_velocity, frame_safety_margins], dim=1)
        merged_data1 = F.relu(self.merge_all_map_with_others2(merged_data1), inplace=True)

        # 5. 合并处理dynamic_map 和 trajectory_trigger、safety_margins和current_velocities
        marged_data2 = torch.cat([merged_dynamic_map, trigger], dim=1)
        marged_data2 = F.relu(self.merge_dynamic_map_with_others1(marged_data2), inplace=True)
        merged_data2 = torch.cat([marged_data2, frame_robot_velocity, frame_safety_margins], dim=1)
        merged_data2 = F.relu(self.merge_dynamic_map_with_others2(merged_data2), inplace=True)

        # 6. 合并两个merge
        merged_data3 = torch.cat([merged_data1, merged_data2], dim=1)
        merged_data3 = F.relu(self.merge_all_data1(merged_data3), inplace=True)
        merged_data3 = torch.cat([merged_data3, frame_action], dim=1)
        merged_data3 = F.relu(self.merge_all_data2(merged_data3), inplace=True)
        merged_data3 = F.relu(self.merge_all_data3(merged_data3), inplace=True)
        merged_data3 = self.merge_all_data4(merged_data3)


        return merged_data3

class QNetwork_2(nn.Module):
    def __init__(self):
        super(QNetwork_2, self).__init__()
        # 1. 处理trajectories
        self.downsampling = nn.MaxPool2d(kernel_size=2, stride=2)
        self.trajectory_lstm = nn.LSTM(input_size=60*60, hidden_size=512, num_layers=1, batch_first=True)

        # 2. 同时处理3个map
        # 2.1 in_channels为3的conv2d：
        self.conv2d_all_map = nn.Conv2d(in_channels=3, out_channels=8, kernel_size=7, stride=1, padding='same')
        # 2.2 bottleneck        
        self.bottleneck_all_map1 = nn.Sequential(
            # 1x1 卷积：降维
            nn.Conv2d(in_channels=8, out_channels=4, kernel_size=1, stride=1, padding='same', bias=False),
            nn.BatchNorm2d(num_features=4),
            nn.ReLU(inplace=True),
            # 3x3 卷积：提取特征
            nn.Conv2d(in_channels=4, out_channels=4, kernel_size=3, stride=1, padding='same', bias=False),
            nn.BatchNorm2d(num_features=4),
            nn.ReLU(inplace=True),
            # 1x1 卷积：恢复通道数
            nn.Conv2d(in_channels=4, out_channels=8, kernel_size=1, stride=1, padding='same', bias=False), 
            nn.BatchNorm2d(num_features=8),
            nn.ReLU(inplace=True)
        )
        # 2.3 bottleneck        
        self.bottleneck_all_map2 = nn.Sequential(
            # 1x1 卷积：降维
            nn.Conv2d(in_channels=8, out_channels=4, kernel_size=1, stride=1, padding='same', bias=False),
            nn.BatchNorm2d(num_features=4),
            nn.ReLU(inplace=True),
            # 3x3 卷积：提取特征
            nn.Conv2d(in_channels=4, out_channels=4, kernel_size=3, stride=1, padding='same', bias=False),
            nn.BatchNorm2d(num_features=4),
            nn.ReLU(inplace=True),
            # 1x1 卷积：恢复通道数
            nn.Conv2d(in_channels=4, out_channels=8, kernel_size=1, stride=1, padding='same', bias=False), 
            nn.BatchNorm2d(num_features=8),
            nn.ReLU(inplace=True)
        )
        
        # 3. 处理动态map
        # 3.1 in_channels为2的conv2d：
        self.conv2d_dynamic_map = nn.Conv2d(in_channels=2, out_channels=8, kernel_size=7, stride=1, padding='same')
        # 3.2 bottleneck        
        self.bottleneck_dynamic_map1 = nn.Sequential(
            # 1x1 卷积：降维
            nn.Conv2d(in_channels=8, out_channels=4, kernel_size=1, stride=1, padding='same', bias=False),
            nn.BatchNorm2d(num_features=4),
            nn.ReLU(inplace=True),
            # 3x3 卷积：提取特征
            nn.Conv2d(in_channels=4, out_channels=4, kernel_size=3, stride=1, padding='same', bias=False),
            nn.BatchNorm2d(num_features=4),
            nn.ReLU(inplace=True),
            # 1x1 卷积：恢复通道数
            nn.Conv2d(in_channels=4, out_channels=8, kernel_size=1, stride=1, padding='same', bias=False), 
            nn.BatchNorm2d(num_features=8),
            nn.ReLU(inplace=True)
        )
        # 3.3 bottleneck        
        self.bottleneck_dynamic_map2 = nn.Sequential(
            # 1x1 卷积：降维
            nn.Conv2d(in_channels=8, out_channels=4, kernel_size=1, stride=1, padding='same', bias=False),
            nn.BatchNorm2d(num_features=4),
            nn.ReLU(inplace=True),
            # 3x3 卷积：提取特征
            nn.Conv2d(in_channels=4, out_channels=4, kernel_size=3, stride=1, padding='same', bias=False),
            nn.BatchNorm2d(num_features=4),
            nn.ReLU(inplace=True),
            # 1x1 卷积：恢复通道数
            nn.Conv2d(in_channels=4, out_channels=8, kernel_size=1, stride=1, padding='same', bias=False), 
            nn.BatchNorm2d(num_features=8),
            nn.ReLU(inplace=True)
        )

        # 4. 合并处理all_map 和 trajectory_trigger、safety_margins和current_velocities
        self.merge_all_map_with_others1 = nn.Linear(8 * int(width/4) * int(width/4) + 512, 1024)
        self.merge_all_map_with_others2 = nn.Linear(1024+4, 512)

        # 5. 合并处理dynamic_map 和 trajectory_trigger、safety_margins和current_velocities
        self.merge_dynamic_map_with_others1 = nn.Linear(8 * int(width/4) * int(width/4) + 512, 1024)
        self.merge_dynamic_map_with_others2 = nn.Linear(1024+4, 512)

        # 6. 合并两个merge
        self.flatten = nn.Flatten(start_dim=1)
        self.merge_all_data1 = nn.Linear(1024, 512)
        self.merge_all_data2 = nn.Linear(512+2, 256)
        self.merge_all_data3 = nn.Linear(256,64)
        self.merge_all_data4 = nn.Linear(64,1)
        self.apply(weights_init_)

    def forward(self, frame_map_static_position, frame_map_dynamic_velocity_x, frame_map_dynamic_velocity_y, frame_robot_velocity, frame_safety_margins, frame_map_trajectories, frame_action):
        # 1. 处理frame_map_trajectories，得到一个trigger: {batch_size, channel=1, hidden_size=256}
        frame_map_trajectories = self.downsampling(frame_map_trajectories)
        frame_map_trajectories = frame_map_trajectories.view(frame_map_trajectories.shape[0], 10, -1)
        output, (h_n, c_n) = frame_map_trajectories = self.trajectory_lstm(frame_map_trajectories)
        output = torch.permute(output,[1,0,2])
        trigger = output[-1]
        # print("222", trigger.shape)

        # 2. 处理整个map
        merged_all_map = torch.cat([frame_map_static_position, frame_map_dynamic_velocity_x, frame_map_dynamic_velocity_y], dim=1)
        merged_all_map = self.conv2d_all_map(merged_all_map)
        merged_all_map = self.bottleneck_all_map1(merged_all_map) + merged_all_map
        merged_all_map = F.avg_pool2d(merged_all_map, kernel_size=2, stride=2)
        merged_all_map = self.bottleneck_all_map2(merged_all_map) + merged_all_map
        merged_all_map = F.avg_pool2d(merged_all_map, kernel_size=2, stride=2)
        merged_all_map = merged_all_map.view(merged_all_map.shape[0], 1, -1)
        merged_all_map = self.flatten(merged_all_map)
        # print("222", merged_all_map.shape)

        # 3. 处理动态map
        merged_dynamic_map = torch.cat([frame_map_dynamic_velocity_x, frame_map_dynamic_velocity_y], dim=1)
        merged_dynamic_map = self.conv2d_dynamic_map(merged_dynamic_map)
        merged_dynamic_map = self.bottleneck_dynamic_map1(merged_dynamic_map) + merged_dynamic_map
        merged_dynamic_map = F.avg_pool2d(merged_dynamic_map, kernel_size=2, stride=2)
        merged_dynamic_map = self.bottleneck_dynamic_map2(merged_dynamic_map) + merged_dynamic_map
        merged_dynamic_map = F.avg_pool2d(merged_dynamic_map, kernel_size=2, stride=2)
        merged_dynamic_map = merged_dynamic_map.view(merged_dynamic_map.shape[0], 1, -1)
        merged_dynamic_map = self.flatten(merged_dynamic_map)
        # print("222", merged_dynamic_map.shape)

        # 4. 合并处理all_map 和 trajectory_trigger、safety_margins和current_velocities
        marged_data1 = torch.cat([merged_all_map, trigger], dim=1)
        marged_data1 = F.relu(self.merge_all_map_with_others1(marged_data1), inplace=True)
        merged_data1 = torch.cat([marged_data1, frame_robot_velocity, frame_safety_margins], dim=1)
        merged_data1 = F.relu(self.merge_all_map_with_others2(merged_data1), inplace=True)

        # 5. 合并处理dynamic_map 和 trajectory_trigger、safety_margins和current_velocities
        marged_data2 = torch.cat([merged_dynamic_map, trigger], dim=1)
        marged_data2 = F.relu(self.merge_dynamic_map_with_others1(marged_data2), inplace=True)
        merged_data2 = torch.cat([marged_data2, frame_robot_velocity, frame_safety_margins], dim=1)
        merged_data2 = F.relu(self.merge_dynamic_map_with_others2(merged_data2), inplace=True)

        # 6. 合并两个merge
        merged_data3 = torch.cat([merged_data1, merged_data2], dim=1)
        merged_data3 = F.relu(self.merge_all_data1(merged_data3), inplace=True)
        merged_data3 = torch.cat([merged_data3, frame_action], dim=1)
        merged_data3 = F.relu(self.merge_all_data2(merged_data3), inplace=True)
        merged_data3 = F.relu(self.merge_all_data3(merged_data3), inplace=True)
        merged_data3 = self.merge_all_data4(merged_data3)


        return merged_data3


class GaussianPolicy(nn.Module):
    def __init__(self):
        super(GaussianPolicy, self).__init__()
        # 1. 处理trajectories
        self.downsampling = nn.MaxPool2d(kernel_size=2, stride=2)
        self.trajectory_lstm = nn.LSTM(input_size=60*60, hidden_size=512, num_layers=1, batch_first=True)

        # 2. 同时处理3个map
        # 2.1 in_channels为3的conv2d：
        self.conv2d_all_map = nn.Conv2d(in_channels=3, out_channels=8, kernel_size=7, stride=1, padding='same')
        # 2.2 bottleneck        
        self.bottleneck_all_map1 = nn.Sequential(
            # 1x1 卷积：降维
            nn.Conv2d(in_channels=8, out_channels=4, kernel_size=1, stride=1, padding='same', bias=False),
            nn.BatchNorm2d(num_features=4),
            nn.ReLU(inplace=True),
            # 3x3 卷积：提取特征
            nn.Conv2d(in_channels=4, out_channels=4, kernel_size=3, stride=1, padding='same', bias=False),
            nn.BatchNorm2d(num_features=4),
            nn.ReLU(inplace=True),
            # 1x1 卷积：恢复通道数
            nn.Conv2d(in_channels=4, out_channels=8, kernel_size=1, stride=1, padding='same', bias=False), 
            nn.BatchNorm2d(num_features=8),
            nn.ReLU(inplace=True)
        )
        # 2.3 bottleneck        
        self.bottleneck_all_map2 = nn.Sequential(
            # 1x1 卷积：降维
            nn.Conv2d(in_channels=8, out_channels=4, kernel_size=1, stride=1, padding='same', bias=False),
            nn.BatchNorm2d(num_features=4),
            nn.ReLU(inplace=True),
            # 3x3 卷积：提取特征
            nn.Conv2d(in_channels=4, out_channels=4, kernel_size=3, stride=1, padding='same', bias=False),
            nn.BatchNorm2d(num_features=4),
            nn.ReLU(inplace=True),
            # 1x1 卷积：恢复通道数
            nn.Conv2d(in_channels=4, out_channels=8, kernel_size=1, stride=1, padding='same', bias=False), 
            nn.BatchNorm2d(num_features=8),
            nn.ReLU(inplace=True)
        )
        
        # 3. 处理动态map
        # 3.1 in_channels为2的conv2d：
        self.conv2d_dynamic_map = nn.Conv2d(in_channels=2, out_channels=8, kernel_size=7, stride=1, padding='same')
        # 3.2 bottleneck        
        self.bottleneck_dynamic_map1 = nn.Sequential(
            # 1x1 卷积：降维
            nn.Conv2d(in_channels=8, out_channels=4, kernel_size=1, stride=1, padding='same', bias=False),
            nn.BatchNorm2d(num_features=4),
            nn.ReLU(inplace=True),
            # 3x3 卷积：提取特征
            nn.Conv2d(in_channels=4, out_channels=4, kernel_size=3, stride=1, padding='same', bias=False),
            nn.BatchNorm2d(num_features=4),
            nn.ReLU(inplace=True),
            # 1x1 卷积：恢复通道数
            nn.Conv2d(in_channels=4, out_channels=8, kernel_size=1, stride=1, padding='same', bias=False), 
            nn.BatchNorm2d(num_features=8),
            nn.ReLU(inplace=True)
        )
        # 3.3 bottleneck        
        self.bottleneck_dynamic_map2 = nn.Sequential(
            # 1x1 卷积：降维
            nn.Conv2d(in_channels=8, out_channels=4, kernel_size=1, stride=1, padding='same', bias=False),
            nn.BatchNorm2d(num_features=4),
            nn.ReLU(inplace=True),
            # 3x3 卷积：提取特征
            nn.Conv2d(in_channels=4, out_channels=4, kernel_size=3, stride=1, padding='same', bias=False),
            nn.BatchNorm2d(num_features=4),
            nn.ReLU(inplace=True),
            # 1x1 卷积：恢复通道数
            nn.Conv2d(in_channels=4, out_channels=8, kernel_size=1, stride=1, padding='same', bias=False), 
            nn.BatchNorm2d(num_features=8),
            nn.ReLU(inplace=True)
        )

        # 4. 合并处理all_map 和 trajectory_trigger、safety_margins和current_velocities
        self.flatten = nn.Flatten(start_dim=1)
        self.merge_all_map_with_others1 = nn.Linear(8 * int(width/4) * int(width/4) + 512, 1024)
        self.merge_all_map_with_others2 = nn.Linear(1024+4, 512)

        # 5. 合并处理dynamic_map 和 trajectory_trigger、safety_margins和current_velocities
        self.merge_dynamic_map_with_others1 = nn.Linear(8 * int(width/4) * int(width/4) + 512, 1024)
        self.merge_dynamic_map_with_others2 = nn.Linear(1024+4, 512)

        # 6. 合并两个merge
        self.merge_all_data1 = nn.Linear(1024, 512)
        self.merge_all_data2 = nn.Linear(512, 256)
        self.merge_all_data3 = nn.Linear(256,64)

        self.mean_linear = nn.Linear(64, 2) # Different from PPO
        self.log_std_linear = nn.Linear(64, 2)


        self.apply(weights_init_)

        # action rescaling
        scale = [0.1, 0.1]
        bias = [0.1, 0.1]
        self.action_scale = torch.FloatTensor(scale)
        self.action_bias = torch.FloatTensor(bias)

        print("self.action_scale: {}, self.action_bias: {}".format(self.action_scale, self.action_bias))

    def forward(self, frame_map_static_position, frame_map_dynamic_velocity_x, frame_map_dynamic_velocity_y, frame_robot_velocity, frame_safety_margins, frame_map_trajectories):
        # 1. 处理frame_map_trajectories，得到一个trigger: {batch_size, channel=1, hidden_size=256}
        frame_map_trajectories = self.downsampling(frame_map_trajectories)
        frame_map_trajectories = frame_map_trajectories.view(frame_map_trajectories.shape[0], 10, -1)
        output, (h_n, c_n) = frame_map_trajectories = self.trajectory_lstm(frame_map_trajectories)
        output = torch.permute(output,[1,0,2])
        trigger = output[-1]
        # print("333", trigger.shape)

        # 2. 处理整个map
        merged_all_map = torch.cat([frame_map_static_position, frame_map_dynamic_velocity_x, frame_map_dynamic_velocity_y], dim=1)
        merged_all_map = self.conv2d_all_map(merged_all_map)
        merged_all_map = self.bottleneck_all_map1(merged_all_map) + merged_all_map
        merged_all_map = F.avg_pool2d(merged_all_map, kernel_size=2, stride=2)
        merged_all_map = self.bottleneck_all_map2(merged_all_map) + merged_all_map
        merged_all_map = F.avg_pool2d(merged_all_map, kernel_size=2, stride=2)
        merged_all_map = merged_all_map.view(merged_all_map.shape[0], 1, -1)
        merged_all_map = self.flatten(merged_all_map)
        # print("333", merged_all_map.shape)

        # 3. 处理动态map
        merged_dynamic_map = torch.cat([frame_map_dynamic_velocity_x, frame_map_dynamic_velocity_y], dim=1)
        merged_dynamic_map = self.conv2d_dynamic_map(merged_dynamic_map)
        merged_dynamic_map = self.bottleneck_dynamic_map1(merged_dynamic_map) + merged_dynamic_map
        merged_dynamic_map = F.avg_pool2d(merged_dynamic_map, kernel_size=2, stride=2)
        merged_dynamic_map = self.bottleneck_dynamic_map2(merged_dynamic_map) + merged_dynamic_map
        merged_dynamic_map = F.avg_pool2d(merged_dynamic_map, kernel_size=2, stride=2)
        merged_dynamic_map = merged_dynamic_map.view(merged_dynamic_map.shape[0], 1, -1)
        merged_dynamic_map = self.flatten(merged_dynamic_map)
        # print("333", merged_dynamic_map.shape)

        # 4. 合并处理all_map 和 trajectory_trigger、safety_margins和current_velocities
        marged_data1 = torch.cat([merged_all_map, trigger], dim=1)
        marged_data1 = F.relu(self.merge_all_map_with_others1(marged_data1), inplace=True)
        merged_data1 = torch.cat([marged_data1, frame_robot_velocity, frame_safety_margins], dim=1)
        merged_data1 = F.relu(self.merge_all_map_with_others2(merged_data1), inplace=True)

        # 5. 合并处理dynamic_map 和 trajectory_trigger、safety_margins和current_velocities
        marged_data2 = torch.cat([merged_dynamic_map, trigger], dim=1)
        marged_data2 = F.relu(self.merge_dynamic_map_with_others1(marged_data2), inplace=True)
        merged_data2 = torch.cat([marged_data2, frame_robot_velocity, frame_safety_margins], dim=1)
        merged_data2 = F.relu(self.merge_dynamic_map_with_others2(merged_data2), inplace=True)

        # 6. 合并两个merge
        merged_data3 = torch.cat([merged_data1, merged_data2], dim=1)
        merged_data3 = F.relu(self.merge_all_data1(merged_data3), inplace=True)
        merged_data3 = F.relu(self.merge_all_data2(merged_data3), inplace=True)
        merged_data3 = F.relu(self.merge_all_data3(merged_data3), inplace=True)

        mean = self.mean_linear(merged_data3)
        log_std = self.log_std_linear(merged_data3)        
        log_std = torch.clamp(log_std, min=LOG_SIG_MIN, max=LOG_SIG_MAX)
        # print("shape: {}, {}".format(mean.shape, log_std.shape))
        return mean, log_std

    def sample(self, frame_map_static_position, frame_map_dynamic_velocity_x, frame_map_dynamic_velocity_y, frame_robot_velocity, frame_safety_margins, frame_map_trajectories):
        mean, log_std = self.forward(frame_map_static_position, frame_map_dynamic_velocity_x, frame_map_dynamic_velocity_y, frame_robot_velocity, frame_safety_margins, frame_map_trajectories)
        std = log_std.exp()
        normal = Normal(mean, std)
        x_t = normal.rsample()  # for reparameterization trick (mean + std * N(0,1))
        y_t = torch.tanh(x_t)

        action = y_t * self.action_scale + self.action_bias
        log_prob = normal.log_prob(x_t)
        # Enforcing Action Bound
        log_prob -= torch.log(self.action_scale * (1 - y_t.pow(2)) + epsilon)

        log_prob = log_prob.sum(1, keepdim=True)
        mean = torch.tanh(mean) * self.action_scale  + self.action_bias
        # print("shape: {}, {}, {}".format(action.shape, log_prob.shape, mean.shape))
        return action, log_prob, mean

    def to(self, device):
        self.action_scale = self.action_scale.to(device)
        self.action_bias = self.action_bias.to(device)
        return super(GaussianPolicy, self).to(device)


if __name__ == '__main__':
    from torch.autograd import Variable


