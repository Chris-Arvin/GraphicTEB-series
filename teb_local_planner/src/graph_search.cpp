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

/**
 * @param: 
    * g即graph_；
    * visited保存的是深搜过程中扩展到过的节点
 * 
 * **/
void GraphSearchInterface::DepthFirst(HcGraph& g, std::vector<HcGraphVertexType>& visited, const HcGraphVertexType& goal, double start_orientation,
                                      double goal_orientation, const geometry_msgs::Twist* start_velocity, bool free_goal_vel)
{
  // std::cout<<visited.size()<<", "<<(int)hcp_->getTrajectoryContainer().size()<<std::endl;
  // see http://www.technical-recipes.com/2011/a-recursive-algorithm-to-find-all-paths-between-two-given-nodes/ for details on finding all simple paths
  // 搜索到max_number_classes这么多同伦轨迹 就已经足够了
  if ((int)hcp_->getTrajectoryContainer().size() >= cfg_->hcp.max_number_classes)
    return; // We do not need to search for further possible alternative homotopy classes.
  
  // 拿出来最后一个点，如果该点是goal点，则形成新的teb轨迹，并检测是否形成新同伦类
  HcGraphVertexType back = visited.back();
  /// Examine adjacent nodes
  HcGraphAdjecencyIterator it, end;
  for ( boost::tie(it,end) = boost::adjacent_vertices(back,g); it!=end; ++it)
  {
    if ( std::find(visited.begin(), visited.end(), *it)!=visited.end() )
      continue; // already visited

    if ( *it == goal ) // goal reached
    {
      visited.push_back(*it);
      // Add new TEB, if this path belongs to a new homotopy class
      hcp_->addAndInitNewTeb(visited.begin(), visited.end(), boost::bind(getVector2dFromHcGraph, _1, boost::cref(graph_)),
                             start_orientation, goal_orientation, start_velocity, free_goal_vel);

      visited.pop_back();
      break;
    }
  }

  // 扩展visited中的最后一个点的临接点
  /// Recursion for all adjacent vertices
  for ( boost::tie(it,end) = boost::adjacent_vertices(back,g); it!=end; ++it)
  {
    if ( std::find(visited.begin(), visited.end(), *it)!=visited.end() || *it == goal)
      continue; // already visited || goal reached


    visited.push_back(*it);

    // recursion step
    DepthFirst(g, visited, goal, start_orientation, goal_orientation, start_velocity, free_goal_vel);

    visited.pop_back();
  }
}


// 在创建图时，适用了轨迹同伦。teb_换了个名字叫hcp_
void lrKeyPointGraph::createGraph(const PoseSE2& start, const PoseSE2& goal, costmap_2d::Costmap2D* costmap2d, double dist_to_obst, double obstacle_heading_threshold, const geometry_msgs::Twist* start_velocity, bool free_goal_vel)
{
  // Clear existing graph and paths
  clearGraph();
  if((int)hcp_->getTrajectoryContainer().size() >= cfg_->hcp.max_number_classes)
    return;
  // Direction-vector between start and goal and normal-vector:
  Eigen::Vector2d diff = goal.position()-start.position();

  // 已经到目标点附近了，直接做直线离散就可以了
  if (diff.norm()<cfg_->goal_tolerance.xy_goal_tolerance)
  {
    ROS_DEBUG("HomotopyClassPlanner::createProbRoadmapGraph(): xy-goal-tolerance already reached.");
    if (hcp_->getTrajectoryContainer().empty())
    {
      ROS_INFO("HomotopyClassPlanner::createProbRoadmapGraph(): Initializing a small straight line to just correct orientation errors.");
      hcp_->addAndInitNewTeb(start, goal, start_velocity, free_goal_vel);
    }
    return;
  }

  // normal是一个与 从起点指向终点的向量 垂直 的向量
  Eigen::Vector2d normal(-diff[1],diff[0]); // normal-vector
  normal.normalize();
  normal = normal*dist_to_obst; // scale with obstacle_distance; 默认为机器人的半径

  // Insert Vertices
  HcGraphVertexType start_vtx = boost::add_vertex(graph_); // start vertex
  graph_[start_vtx].pos = start.position();
  diff.normalize();

  // store nearest obstacle keypoints -> only used if limit_obstacle_heading is enabled
  std::pair<HcGraphVertexType,HcGraphVertexType> nearest_obstacle; // both vertices are stored
  double min_dist = DBL_MAX;

  if (hcp_->obstacles()!=NULL)
  {
    // 把所有点添加到graph_中
    for (ObstContainer::const_iterator it_obst = hcp_->obstacles()->begin(); it_obst != hcp_->obstacles()->end(); ++it_obst)
    {
      // check if obstacle is placed in front of start point
      Eigen::Vector2d start2obst = (*it_obst)->getCentroid() - start.position();
      double dist = start2obst.norm();
      // cos theta<0.1 说明夹角很大，舍弃
      if (start2obst.dot(diff)/dist<0.1)
        continue;

      // 把障碍物点的中心 沿着normal和-normal推离一段距离
      // Add Keypoints
      HcGraphVertexType u = boost::add_vertex(graph_);
      graph_[u].pos = (*it_obst)->getCentroid() + normal;
      HcGraphVertexType v = boost::add_vertex(graph_);
      graph_[v].pos = (*it_obst)->getCentroid() - normal;
      
      // store nearest obstacle
      // obstacle_heading_threshold 默认为0.45
      if (obstacle_heading_threshold && dist<min_dist)
      {
        min_dist = dist;
        nearest_obstacle.first = u;
        nearest_obstacle.second = v;
      }
    }
  }

  HcGraphVertexType goal_vtx = boost::add_vertex(graph_); // goal vertex
  graph_[goal_vtx].pos = goal.position();

  // Insert Edges
  HcGraphVertexIterator it_i, end_i, it_j, end_j;
  // boost::tie可以类比为pair，boost::vertices也是pair， .first返回graph的vertices的起点的指针， .second返回终点
  for (boost::tie(it_i,end_i) = boost::vertices(graph_); it_i!=end_i-1; ++it_i) // ignore goal in this loop
  {
    for (boost::tie(it_j,end_j) = boost::vertices(graph_); it_j!=end_j; ++it_j) // check all forward connections
    {
      if (it_i==it_j)
        continue;
      // TODO: make use of knowing in which order obstacles are inserted and that for each obstacle 2 vertices are added,
      // therefore we must only check one of them.
      Eigen::Vector2d distij = graph_[*it_j].pos-graph_[*it_i].pos;
      distij.normalize();
      // Check if the direction is backwards:
      // 如果某两个节点间的方向 和 起点与终点连线的方向 间的差值过大，则舍弃
      if (distij.dot(diff)<=obstacle_heading_threshold)
        continue;


      // Check start angle to nearest obstacle
      // 检测起点，如果和离得最近的障碍物间的夹角很小，忽略
      if (obstacle_heading_threshold && *it_i==start_vtx && min_dist!=DBL_MAX)
      {
        if (*it_j == nearest_obstacle.first || *it_j == nearest_obstacle.second)
        {
          Eigen::Vector2d keypoint_dist = graph_[*it_j].pos-start.position();
          keypoint_dist.normalize();
          Eigen::Vector2d start_orient_vec( cos(start.theta()), sin(start.theta()) ); // already normalized
          // check angle
          if (start_orient_vec.dot(keypoint_dist) <= obstacle_heading_threshold)
          {
            ROS_DEBUG("createGraph() - deleted edge: limit_obstacle_heading");
            continue;
          }
        }
      }

      // Collision Check
      
      // 如果该边撞击障碍物了，则不要该边
      if (hcp_->obstacles()!=NULL)
      {
        bool collision = false;
        for (ObstContainer::const_iterator it_obst = hcp_->obstacles()->begin(); it_obst != hcp_->obstacles()->end(); ++it_obst)
        {
          if ( (*it_obst)->checkLineIntersection(graph_[*it_i].pos,graph_[*it_j].pos, 0.5*dist_to_obst) )
          {
            collision = true;
            break;
          }
        }
        if (collision)
          continue;
      }

      // Create Edge
      boost::add_edge(*it_i,*it_j,graph_);
    }
  }


  // Find all paths between start and goal!
  std::vector<HcGraphVertexType> visited;
  visited.push_back(start_vtx);
  DepthFirst(graph_,visited,goal_vtx, start.theta(), goal.theta(), start_velocity, free_goal_vel);
}


// 这个是被默认调用的，用PRM采样点
void ProbRoadmapGraph::createGraph(const PoseSE2& start, const PoseSE2& goal, costmap_2d::Costmap2D* costmap2d, double dist_to_obst, double obstacle_heading_threshold, const geometry_msgs::Twist* start_velocity, bool free_goal_vel)
{
  // Clear existing graph and paths
  clearGraph();
  if((int)hcp_->getTrajectoryContainer().size() >= cfg_->hcp.max_number_classes)
    return;
  // Direction-vector between start and goal and normal-vector:
  Eigen::Vector2d diff = goal.position()-start.position();
  double start_goal_dist = diff.norm();
  // 到达目标点附近
  if (start_goal_dist<cfg_->goal_tolerance.xy_goal_tolerance)
  {
    ROS_DEBUG("HomotopyClassPlanner::createProbRoadmapGraph(): xy-goal-tolerance already reached.");
    if (hcp_->getTrajectoryContainer().empty())
    {
      ROS_INFO("HomotopyClassPlanner::createProbRoadmapGraph(): Initializing a small straight line to just correct orientation errors.");
      hcp_->addAndInitNewTeb(start, goal, start_velocity, free_goal_vel);
    }
    return;
  }
  // 以机器人为中心，在大小为start_goal_dist * area_width的矩形框内采样
  Eigen::Vector2d normal(-diff.coeffRef(1),diff.coeffRef(0)); // normal-vector
  normal.normalize();

  // Now sample vertices between start, goal and a specified width between both sides
  // Let's start with a square area between start and goal (maybe change it later to something like a circle or whatever)

  double area_width = cfg_->hcp.roadmap_graph_area_width;   //.yaml中 默认为5

  boost::random::uniform_real_distribution<double> distribution_x(0, start_goal_dist * cfg_->hcp.roadmap_graph_area_length_scale);    //.yaml中默认为1
  boost::random::uniform_real_distribution<double> distribution_y(0, area_width);

  double phi = atan2(diff.coeffRef(1),diff.coeffRef(0)); // rotate area by this angle
  Eigen::Rotation2D<double> rot_phi(phi);

  Eigen::Vector2d area_origin;
  if (cfg_->hcp.roadmap_graph_area_length_scale != 1.0)
    area_origin = start.position() + 0.5*(1.0-cfg_->hcp.roadmap_graph_area_length_scale)*start_goal_dist*diff.normalized() - 0.5*area_width*normal; // bottom left corner of the origin
  else
    area_origin = start.position() - 0.5*area_width*normal; // bottom left corner of the origin

  // Insert Vertices
  HcGraphVertexType start_vtx = boost::add_vertex(graph_); // start vertex
  graph_[start_vtx].pos = start.position();
  diff.normalize(); // normalize in place


  // Start sampling
  // hcp.roadmap_graph_no_samples默认为15，即只采样15个点。。
  for (int i=0; i < cfg_->hcp.roadmap_graph_no_samples; ++i)
  {
    Eigen::Vector2d sample;
//     bool coll_free;
//     do // sample as long as a collision free sample is found
//     {
      // Sample coordinates
      sample = area_origin + rot_phi*Eigen::Vector2d(distribution_x(rnd_generator_), distribution_y(rnd_generator_));

      // Test for collision
      // we do not care for collision checking here to improve efficiency, since we perform resampling repeatedly.
      // occupied vertices are ignored in the edge insertion state since they always violate the edge-obstacle collision check.
//       coll_free = true;
//       for (ObstContainer::const_iterator it_obst = obstacles_->begin(); it_obst != obstacles_->end(); ++it_obst)
//       {
//         if ( (*it_obst)->checkCollision(sample, dist_to_obst)) // TODO really keep dist_to_obst here?
//         {
//           coll_free = false;
//           break;
//         }
//       }
//
//     } while (!coll_free && ros::ok());

    // Add new vertex
    HcGraphVertexType v = boost::add_vertex(graph_);
    graph_[v].pos = sample;
  }

  // Now add goal vertex
  HcGraphVertexType goal_vtx = boost::add_vertex(graph_); // goal vertex
  graph_[goal_vtx].pos = goal.position();


  // Insert Edges
  HcGraphVertexIterator it_i, end_i, it_j, end_j;
  for (boost::tie(it_i,end_i) = boost::vertices(graph_); it_i!=boost::prior(end_i); ++it_i) // ignore goal in this loop
  {
    for (boost::tie(it_j,end_j) = boost::vertices(graph_); it_j!=end_j; ++it_j) // check all forward connections
    {
      if (it_i==it_j) // same vertex found
        continue;

      Eigen::Vector2d distij = graph_[*it_j].pos-graph_[*it_i].pos;
      distij.normalize(); // normalize in place

      // Check if the direction is backwards:
      // 仅当两个点间的距离比较近时，该后续点 才会被考虑
      if (distij.dot(diff)<=obstacle_heading_threshold)
          continue; // diff is already normalized


      // Collision Check
      bool collision = false;
      for (ObstContainer::const_iterator it_obst = hcp_->obstacles()->begin(); it_obst != hcp_->obstacles()->end(); ++it_obst)
      {
        if ( (*it_obst)->checkLineIntersection(graph_[*it_i].pos,graph_[*it_j].pos, dist_to_obst) )
        {
          collision = true;
          break;
        }
      }
      if (collision)
        continue;

      // Create Edge
      boost::add_edge(*it_i,*it_j,graph_);
    }
  }

  /// Find all paths between start and goal!
  std::vector<HcGraphVertexType> visited;
  visited.push_back(start_vtx);
  DepthFirst(graph_,visited,goal_vtx, start.theta(), goal.theta(), start_velocity, free_goal_vel);
}

void graphicProcess::updateDynamicObstacle(std::vector<std::vector<double>> dynamic_obstacle){
  dynamic_obstacle_ = dynamic_obstacle;
}

void graphicProcess::createGraph(const PoseSE2& start, const PoseSE2& goal, costmap_2d::Costmap2D* costmap2d, double dist_to_obst, double obstacle_heading_threshold, const geometry_msgs::Twist* start_velocity, bool free_goal_vel)
{
  // Clear existing graph and paths
  clearGraph();
  /**
   *  Case1: there is enough trajectories produced by last time
   * **/
  if((int)hcp_->getTrajectoryContainer().size() >= cfg_->hcp.max_number_classes)
    return;
  /**
   *  Case2: near the goal
   * **/
  Eigen::Vector2d diff = goal.position()-start.position();
  double start_goal_dist = diff.norm();
  if (start_goal_dist<cfg_->goal_tolerance.xy_goal_tolerance)
  {
    ROS_DEBUG("HomotopyClassPlanner::createProbRoadmapGraph(): xy-goal-tolerance already reached.");
    if (hcp_->getTrajectoryContainer().empty())
    {
      ROS_INFO("HomotopyClassPlanner::createProbRoadmapGraph(): Initializing a small straight line to just correct orientation errors.");
      hcp_->addAndInitNewTeb(start, goal, start_velocity, free_goal_vel);
    }
    return;
  }
  /**
   *  Case3: mapProcess to extract all non-homo paths
   * **/
  // search non-homo paths with Graphic operations
  auto t = ros::Time::now();
  map_cv_obj_ = mapProcess(costmap2d, start, goal, dynamic_obstacle_);
  map_cv_obj_.clusterObs();
  map_cv_obj_.borderIdentify();
  map_cv_obj_.cornerIdentify();
  map_cv_obj_.createPathBetweenObs();
  auto paths = map_cv_obj_.findHomoPaths(start, goal, cfg_->hcp.max_path_explore_number_for_GraphicTEB, cfg_->hcp.max_path_remained_for_GraphicTEB, cfg_->hcp.graphic_is_limitation, cfg_->hcp.graphic_is_hallway);
  ROS_INFO("Time for finding [%d] non-homo paths: [%f] s", paths.size(), (ros::Time::now() - t).toSec() );
  // add non-homo paths to TEB processor
  hcp_->updateObstacles(map_cv_obj_.getStaticObsIdentificationInWorld(), map_cv_obj_.getDynamicObsIdentificationInWorld());
  hcp_->updateObsMap(map_cv_obj_.getMapObsLabeled());
  for (auto path:paths){
    if (hcp_->addAndInitNewTeb(path.begin(), path.end(), boost::bind(getSelf, _1), start.theta(), goal.theta(), start_velocity, free_goal_vel))
      map_cv_obj_.add_to_homo_paths_pruned(path);
  }
}


} // end namespace
