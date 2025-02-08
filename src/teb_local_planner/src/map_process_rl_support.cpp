#include <teb_local_planner/map_process.h>

rl_planner::rl_stateRequest mapProcess::getState(){
    rl_planner::rl_stateRequest req;

    // 1. static obstacle
    std::vector<int> map_static_obstacle_list;
    for (int x=0; x<map_obs_occupied_without_ignore_.size(); x++){
        for (int y=0; y<map_obs_occupied_without_ignore_.front().size(); y++){
            if(map_obs_occupied_without_ignore_[x][y])
                map_static_obstacle_list.push_back(1);
            else
                map_static_obstacle_list.push_back(0);
        }
    }
    req.map_static_position = map_static_obstacle_list;

    // 2. dynamic obstacle
    std::vector<float> map_dynamic_velocity_x;
    std::vector<float> map_dynamic_velocity_y;
    for (int x=0; x<width_; x++){
        for (int y=0; y<height_; y++){
            map_dynamic_velocity_x.push_back(map_vel_x_[x][y]);
            map_dynamic_velocity_y.push_back(map_vel_y_[x][y]);
        }
    }
    req.map_dynamic_velocity_x = map_dynamic_velocity_x;
    req.map_dynamic_velocity_y = map_dynamic_velocity_y;

    // 3. 机器人当前速度(vx,vy)
    req.vx = velocity_in_map_.first;
    req.vy = velocity_in_map_.second;

    // 4. 搜索到的轨迹
    visualization_msgs::MarkerArray trajectories;
    for (auto traj:homo_paths_add_to_TEB_){
        visualization_msgs::Marker temp;
        for (auto p:traj){
            int mx, my;
            world2map(mx,my,p[0],p[1]);
            geometry_msgs::Point poi; poi.x = mx; poi.y = my;
            temp.points.push_back(poi);
        }
        trajectories.markers.push_back(temp);
    }
    req.trajectories = trajectories;
    return req;
}

// void getStateAndReward();