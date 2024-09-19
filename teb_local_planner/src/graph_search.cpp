/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Christoph Rösmann, Franz Albers
 *********************************************************************/

#include <teb_local_planner/graph_search.h>
#include <teb_local_planner/homotopy_class_planner.h>

namespace teb_local_planner
{

void graphicProcess::updateDynamicObstacle(std::vector<std::vector<double>> dynamic_obstacle){
  dynamic_obstacle_ = dynamic_obstacle;
}

void graphicProcess::createGraph(const PoseSE2& start, const PoseSE2& goal, const std::vector<geometry_msgs::PoseStamped>& local_plan, costmap_2d::Costmap2D* costmap2d, double dist_to_obst, double obstacle_heading_threshold, const geometry_msgs::Twist* start_velocity, bool free_goal_vel, std::pair<double,double> global_goal)
{
  // Clear existing graph and paths
  clearGraph();
  /**
   *  Case1: there is enough trajectories produced by last time
   * **/
  if((int)hcp_->getTrajectoryContainer().size() >= cfg_->hcp.max_number_classes)
    return;
  /**
   *  Case2: mapProcess to extract all non-homo paths
   * **/
  // search non-homo paths with Graphic operations
  auto t = ros::Time::now();
  map_cv_obj_ = mapProcess();
  ROS_INFO("++++++ -1. Time: [%f] s", (ros::Time::now() - t).toSec() );
  map_cv_obj_.findGoalLineGroups(costmap2d, local_plan, global_goal);
  ROS_INFO("++++++ 0. Time: [%f] s", (ros::Time::now() - t).toSec() );
  map_cv_obj_.initialize(costmap2d, start, goal, local_plan, dynamic_obstacle_, global_goal);
  ROS_INFO("++++++ 1. Time: [%f] s", (ros::Time::now() - t).toSec() );
  map_cv_obj_.setParams(cfg_->hcp.max_path_explore_number_for_GraphicTEB, cfg_->hcp.max_path_remained_for_GraphicTEB, cfg_->hcp.graphic_is_hallway, cfg_->hcp.is_cos_limitation, cfg_->hcp.is_father_can_visit_limitation, cfg_->hcp.epsilon_for_early_stop );
  ROS_INFO("++++++ 2. Time: [%f] s", (ros::Time::now() - t).toSec() );
  map_cv_obj_.clusterObs();
  ROS_INFO("++++++ 3. Time: [%f] s", (ros::Time::now() - t).toSec() );
  map_cv_obj_.borderIdentify();
  ROS_INFO("++++++ 4. Time: [%f] s", (ros::Time::now() - t).toSec() );
  map_cv_obj_.cornerIdentify();
  ROS_INFO("++++++ 5. Time: [%f] s", (ros::Time::now() - t).toSec() );
  map_cv_obj_.createPathBetweenObs();
  ROS_INFO("++++++ 6. Time: [%f] s", (ros::Time::now() - t).toSec() );
  hcp_->updateObstacles(map_cv_obj_.getStaticObsIdentificationInWorld(), dynamic_obstacle_);
  hcp_->updateObsMap(map_cv_obj_.getMapObsLabeled());
  ROS_INFO("++++++ 7. Time: [%f] s", (ros::Time::now() - t).toSec() );
}

void graphicProcess::addPaths(const PoseSE2& start, const PoseSE2& goal, const std::vector<geometry_msgs::PoseStamped>& local_plan, costmap_2d::Costmap2D* costmap2d, double dist_to_obst, double obstacle_heading_threshold, const geometry_msgs::Twist* start_velocity, bool free_goal_vel){

  //todo 优化过程中，为了保证同伦性，goal仅可在当前goal line滑动；以其他goal line为目标的homo traj是存在的，只不过是需要借助obstacle group做中转~
  //todo homo生成的normal path如果压到了地图边界，要被删掉哦！！  
  std::vector<std::pair<std::pair<double, double>, std::pair<double, double>>> goal_endpoints_list;
  auto paths = map_cv_obj_.findHomoPaths(goal_endpoints_list);
  // add non-homo paths to TEB processor
  for (int i=0; i<paths.size(); i++){
    auto path = paths[i];
    auto getGoalLineEnds = goal_endpoints_list[i];
    if (hcp_->addAndInitNewTeb(path.begin(), path.end(), boost::bind(getSelf, _1), start.theta(), goal.theta(), start_velocity, free_goal_vel, false, getGoalLineEnds))
      map_cv_obj_.add_to_homo_paths_pruned(path);
  }
  ROS_INFO("    Add [%d] non-homo trajectories to TEB", map_cv_obj_.getHomoPathsPruned().size());
}


} // end namespace
