/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
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
 * Author: Christoph Rösmann
 *********************************************************************/

#include <teb_local_planner/visualization.h>
#include <teb_local_planner/optimal_planner.h>
#include <teb_local_planner/FeedbackMsg.h>

namespace teb_local_planner
{

TebVisualization::TebVisualization() : initialized_(false)
{
}

TebVisualization::TebVisualization(ros::NodeHandle& nh, const TebConfig& cfg) : initialized_(false)
{
  initialize(nh, cfg);
}

void TebVisualization::initialize(ros::NodeHandle& nh, const TebConfig& cfg)
{
  if (initialized_)
    ROS_WARN("TebVisualization already initialized. Reinitalizing...");
  
  // set config
  cfg_ = &cfg;
  
  // register topics
  global_plan_pub_ = nh.advertise<nav_msgs::Path>("global_plan", 1);
  local_plan_pub_ = nh.advertise<nav_msgs::Path>("local_plan",1);
  teb_poses_pub_ = nh.advertise<geometry_msgs::PoseArray>("teb_poses", 100);
  teb_poses_origin_pub_ = nh.advertise<geometry_msgs::PoseArray>("teb_poses_origin", 100);
  teb_marker_pub_ = nh.advertise<visualization_msgs::Marker>("teb_markers", 1000);
  feedback_pub_ = nh.advertise<teb_local_planner::FeedbackMsg>("teb_feedback", 10);  
  
  // register publichser for graphicTEB
  pub_corner = nh.advertise<visualization_msgs::MarkerArray>("/corners",1);
  pub_obs_group = nh.advertise<visualization_msgs::MarkerArray>("/groups",1);
  pub_border = nh.advertise<visualization_msgs::MarkerArray>("/borders",1);
  pub_connect_inner_obs = nh.advertise<visualization_msgs::MarkerArray>("/connects_inner_obs",1);
  pub_connect_between_obs_adjecent = nh.advertise<visualization_msgs::MarkerArray>("/connects_outer_obs_adjecent",1);
  pub_connect_between_obs_all = nh.advertise<visualization_msgs::MarkerArray>("/connects_outer_obs_all",1);
  pub_homo_paths = nh.advertise<visualization_msgs::MarkerArray>("/homo_paths",1);
  pub_corridor_map = nh.advertise<visualization_msgs::MarkerArray>("/corridor_linetripe",1);

  initialized_ = true; 
}



void TebVisualization::publishGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan) const
{
  if ( printErrorWhenNotInitialized() ) return;
  base_local_planner::publishPlan(global_plan, global_plan_pub_); 
}

void TebVisualization::publishLocalPlan(const std::vector<geometry_msgs::PoseStamped>& local_plan) const
{
  if ( printErrorWhenNotInitialized() )
    return;
  base_local_planner::publishPlan(local_plan, local_plan_pub_); 
}

void TebVisualization::publishLocalPlanAndPoses(const TimedElasticBand& teb, const TimedElasticBand& teb_origin) const
{
  if ( printErrorWhenNotInitialized() )
    return;
  
    // create path msg
    nav_msgs::Path teb_path;
    teb_path.header.frame_id = cfg_->map_frame;
    teb_path.header.stamp = ros::Time::now();
    
    // create pose_array (along trajectory)
    geometry_msgs::PoseArray teb_poses;
    teb_poses.header.frame_id = teb_path.header.frame_id;
    teb_poses.header.stamp = teb_path.header.stamp;
    
    // fill path msgs with teb configurations
    for (int i=0; i < teb.sizePoses(); i++)
    {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = teb_path.header.frame_id;
      pose.header.stamp = teb_path.header.stamp;
      pose.pose.position.x = teb.Pose(i).x();
      pose.pose.position.y = teb.Pose(i).y();
      // pose.pose.position.z = cfg_->hcp.visualize_with_time_as_z_axis_scale*teb.getSumOfTimeDiffsUpToIdx(i);
      pose.pose.position.z = 0.1;
      pose.pose.orientation = tf::createQuaternionMsgFromYaw(teb.Pose(i).theta());
      teb_path.poses.push_back(pose);
      teb_poses.poses.push_back(pose.pose);
    }

    geometry_msgs::PoseArray teb_poses_origin;
    teb_poses_origin.header.frame_id = teb_path.header.frame_id;
    teb_poses_origin.header.stamp = teb_path.header.stamp;
    
    // fill path msgs with teb configurations
    for (int i=0; i < teb_origin.sizePoses(); i++)
    {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = teb_path.header.frame_id;
      pose.header.stamp = teb_path.header.stamp;
      pose.pose.position.x = teb_origin.Pose(i).x();
      pose.pose.position.y = teb_origin.Pose(i).y();
      pose.pose.position.z = 0;
      // pose.pose.position.z = cfg_->hcp.visualize_with_time_as_z_axis_scale*teb_origin.getSumOfTimeDiffsUpToIdx(i);
      pose.pose.orientation = tf::createQuaternionMsgFromYaw(teb_origin.Pose(i).theta());
      teb_poses_origin.poses.push_back(pose.pose);
    }

    local_plan_pub_.publish(teb_path);
    teb_poses_origin_pub_.publish(teb_poses_origin);
    teb_poses_pub_.publish(teb_poses);
}



void TebVisualization::publishRobotFootprintModel(const PoseSE2& current_pose, const BaseRobotFootprintModel& robot_model, const std::string& ns,
                                                  const std_msgs::ColorRGBA &color)
{
  if ( printErrorWhenNotInitialized() )
    return;
  
  std::vector<visualization_msgs::Marker> markers;
  robot_model.visualizeRobot(current_pose, markers, color);
  if (markers.empty())
    return;
  
  int idx = 1000000;  // avoid overshadowing by obstacles
  for (std::vector<visualization_msgs::Marker>::iterator marker_it = markers.begin(); marker_it != markers.end(); ++marker_it, ++idx)
  {
    marker_it->header.frame_id = cfg_->map_frame;
    marker_it->header.stamp = ros::Time::now();
    marker_it->action = visualization_msgs::Marker::ADD;
    marker_it->ns = ns;
    marker_it->id = idx;
    marker_it->lifetime = ros::Duration(2.0);
    teb_marker_pub_.publish(*marker_it);
  }
  
}

void TebVisualization::publishInfeasibleRobotPose(const PoseSE2& current_pose, const BaseRobotFootprintModel& robot_model)
{
  publishRobotFootprintModel(current_pose, robot_model, "InfeasibleRobotPoses", toColorMsg(0.5, 0.8, 0.0, 0.0));
}


void TebVisualization::publishObstacles(const ObstContainer& obstacles) const
{
  if ( obstacles.empty() || printErrorWhenNotInitialized() )
    return;
  
  // Visualize point obstacles
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = cfg_->map_frame;
    marker.header.stamp = ros::Time::now();
    marker.ns = "PointObstacles";
    marker.id = 0;
    marker.pose.orientation.x=0;
    marker.pose.orientation.y=0;
    marker.pose.orientation.z=0;
    marker.pose.orientation.w=1;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(2.0);
    
    for (ObstContainer::const_iterator obst = obstacles.begin(); obst != obstacles.end(); ++obst)
    {
      boost::shared_ptr<PointObstacle> pobst = boost::dynamic_pointer_cast<PointObstacle>(*obst);      
      if (!pobst)
        continue;

      if (cfg_->hcp.visualize_with_time_as_z_axis_scale < 0.001)
      {
        geometry_msgs::Point point;
        point.x = pobst->x();
        point.y = pobst->y();
        point.z = 0;
        marker.points.push_back(point);
      }
      else // Spatiotemporally point obstacles become a line
      {
        marker.type = visualization_msgs::Marker::LINE_LIST;
        geometry_msgs::Point start;
        start.x = pobst->x();
        start.y = pobst->y();
        start.z = 0;
        marker.points.push_back(start);

        geometry_msgs::Point end;
        double t = 20;
        Eigen::Vector2d pred;
        pobst->predictCentroidConstantVelocity(t, pred);
        end.x = pred[0];
        end.y = pred[1];
        end.z = cfg_->hcp.visualize_with_time_as_z_axis_scale*t;
        marker.points.push_back(end);
      }
    }
    
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    if (marker.points.size()!=0)
      teb_marker_pub_.publish( marker );
  }
  
  // Visualize circular obstacles
  {
    std::size_t idx = 0;
    for (ObstContainer::const_iterator obst = obstacles.begin(); obst != obstacles.end(); ++obst)
    {
      boost::shared_ptr<CircularObstacle> pobst = boost::dynamic_pointer_cast<CircularObstacle>(*obst);
      if (!pobst)
        continue;

      visualization_msgs::Marker marker;
      marker.header.frame_id = cfg_->map_frame;
      marker.header.stamp = ros::Time::now();
      marker.ns = "CircularObstacles";
      marker.id = idx++;
      marker.pose.orientation.x=0;
      marker.pose.orientation.y=0;
      marker.pose.orientation.z=0;
      marker.pose.orientation.w=1;
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.action = visualization_msgs::Marker::ADD;
      marker.lifetime = ros::Duration(2.0);
      const double& x = pobst->x();
      const double& y = pobst->y();
      const double& r = pobst->radius();
      for (double theta=0; theta<2*3.14; theta+=0.3){
        geometry_msgs::Point point;
        point.x = pobst->x() + r*cos(theta);
        point.y = pobst->y() + r*sin(theta);
        point.z = 0;
        marker.points.push_back(point);
      }
      geometry_msgs::Point point;
      point.x = pobst->x() + r*cos(0);
      point.y = pobst->y() + r*sin(0);
      point.z = 0;
      marker.points.push_back(point);
            
      marker.scale.x = 0.02;
      marker.scale.y = 0.02;      
      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      if (marker.points.size()!=0)
        teb_marker_pub_.publish( marker );
    }
  }

  // Visualize line obstacles
  {
    std::size_t idx = 0;
    for (ObstContainer::const_iterator obst = obstacles.begin(); obst != obstacles.end(); ++obst)
    {	
      boost::shared_ptr<LineObstacle> pobst = boost::dynamic_pointer_cast<LineObstacle>(*obst);   
      if (!pobst)
        continue;
      
      visualization_msgs::Marker marker;
      marker.header.frame_id = cfg_->map_frame;
      marker.header.stamp = ros::Time::now();
      marker.ns = "LineObstacles";
      marker.id = idx++;
      marker.pose.orientation.x=0;
      marker.pose.orientation.y=0;
      marker.pose.orientation.z=0;
      marker.pose.orientation.w=1;
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.action = visualization_msgs::Marker::ADD;
      marker.lifetime = ros::Duration(2.0);
      geometry_msgs::Point start;
      start.x = pobst->start().x();
      start.y = pobst->start().y();
      start.z = 0;
      marker.points.push_back(start);
      geometry_msgs::Point end;
      end.x = pobst->end().x();
      end.y = pobst->end().y();
      end.z = 0;
      marker.points.push_back(end);
  
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      if (marker.points.size()!=0)  
        teb_marker_pub_.publish( marker );     
    }
  }
  

  // Visualize polygon obstacles
  {
    std::size_t idx = 0;
    for (ObstContainer::const_iterator obst = obstacles.begin(); obst != obstacles.end(); ++obst)
    {	
      boost::shared_ptr<PolygonObstacle> pobst = boost::dynamic_pointer_cast<PolygonObstacle>(*obst);   
      if (!pobst)
				continue;
      
      visualization_msgs::Marker marker;
      marker.header.frame_id = cfg_->map_frame;
      marker.header.stamp = ros::Time::now();
      marker.ns = "PolyObstacles";
      marker.id = idx++;
      marker.pose.orientation.x=0;
      marker.pose.orientation.y=0;
      marker.pose.orientation.z=0;
      marker.pose.orientation.w=1;
      marker.type = visualization_msgs::Marker::LINE_STRIP;
      marker.action = visualization_msgs::Marker::ADD;
      marker.lifetime = ros::Duration(2.0);
      
      for (Point2dContainer::const_iterator vertex = pobst->vertices().begin(); vertex != pobst->vertices().end(); ++vertex)
      {
        geometry_msgs::Point point;
        point.x = vertex->x();
        point.y = vertex->y();
        point.z = 0;
        marker.points.push_back(point);
      }
      
      // Also add last point to close the polygon
      // but only if polygon has more than 2 points (it is not a line)
      if (pobst->vertices().size() > 2)
      {
        geometry_msgs::Point point;
        point.x = pobst->vertices().front().x();
        point.y = pobst->vertices().front().y();
        point.z = 0;
        marker.points.push_back(point);
      }
      marker.scale.x = 0.02;
      marker.scale.y = 0.02;
      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      if (marker.points.size()!=0)
        teb_marker_pub_.publish( marker );     
    }
  }
}


void TebVisualization::publishViaPoints(const std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >& via_points, const std::string& ns) const
{
  if ( via_points.empty() || printErrorWhenNotInitialized() )
    return;
  
  visualization_msgs::Marker marker;
  marker.header.frame_id = cfg_->map_frame;
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.id = 0;
  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration(2.0);
  
  for (std::size_t i=0; i < via_points.size(); ++i)
  {
    geometry_msgs::Point point;
    point.x = via_points[i].x();
    point.y = via_points[i].y();
    point.z = 0;
    marker.points.push_back(point);
  }
  
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;

  teb_marker_pub_.publish( marker );
}

// Note: Reforam: diffenert height
void TebVisualization::publishTebContainer(const TebOptPlannerContainer& teb_planner, const std::string& ns)
{
if ( printErrorWhenNotInitialized() )
    return;
  
  visualization_msgs::Marker marker;
  marker.header.frame_id = cfg_->map_frame;
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.action = visualization_msgs::Marker::DELETEALL;
  teb_marker_pub_.publish( marker );
  marker.points.clear();
  ros::Duration(0.001).sleep();

  marker.id = 0;
  marker.pose.orientation.x=0;
  marker.pose.orientation.y=0;
  marker.pose.orientation.z=0;
  marker.pose.orientation.w=1;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  
  // Iterate through teb pose sequence
  double height = 0;
  for( TebOptPlannerContainer::const_iterator it_teb = teb_planner.begin(); it_teb != teb_planner.end(); ++it_teb )
  {	
    ++height;
    // iterate single poses
    PoseSequence::const_iterator it_pose = it_teb->get()->teb().poses().begin();
    PoseSequence::const_iterator it_pose_end = it_teb->get()->teb().poses().end();
    std::advance(it_pose_end, -1); // since we are interested in line segments, reduce end iterator by one.
    

    while (it_pose != it_pose_end)
    {
      geometry_msgs::Point point_start;
      point_start.x = (*it_pose)->x();
      point_start.y = (*it_pose)->y();
      point_start.z = height*0.05;
      marker.points.push_back(point_start);

      geometry_msgs::Point point_end;
      point_end.x = (*boost::next(it_pose))->x();
      point_end.y = (*boost::next(it_pose))->y();
      point_end.z = height*0.05;
      marker.points.push_back(point_end);
      ++it_pose;
    }
  }
  marker.scale.x = 0.01;
  marker.color.a = 1.0;
  marker.color.r = 0.5;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  teb_marker_pub_.publish( marker );
}

void TebVisualization::publishFeedbackMessage(const std::vector< boost::shared_ptr<TebOptimalPlanner> >& teb_planners,
                                              unsigned int selected_trajectory_idx, const ObstContainer& obstacles)
{
  FeedbackMsg msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = cfg_->map_frame;
  msg.selected_trajectory_idx = selected_trajectory_idx;
  
  
  msg.trajectories.resize(teb_planners.size());
  
  // Iterate through teb pose sequence
  std::size_t idx_traj = 0;
  for( TebOptPlannerContainer::const_iterator it_teb = teb_planners.begin(); it_teb != teb_planners.end(); ++it_teb, ++idx_traj )
  {   
    msg.trajectories[idx_traj].header = msg.header;
    it_teb->get()->getFullTrajectory(msg.trajectories[idx_traj].trajectory);
  }
  
  // add obstacles
  msg.obstacles_msg.obstacles.resize(obstacles.size());
  for (std::size_t i=0; i<obstacles.size(); ++i)
  {
    msg.obstacles_msg.header = msg.header;

    // copy polygon
    msg.obstacles_msg.obstacles[i].header = msg.header;
    obstacles[i]->toPolygonMsg(msg.obstacles_msg.obstacles[i].polygon);

    // copy id
    msg.obstacles_msg.obstacles[i].id = i; // TODO: we do not have any id stored yet

    // orientation
    //msg.obstacles_msg.obstacles[i].orientation =; // TODO

    // copy velocities
    obstacles[i]->toTwistWithCovarianceMsg(msg.obstacles_msg.obstacles[i].velocities);
  }
  
  feedback_pub_.publish(msg);
}

void TebVisualization::publishFeedbackMessage(const TebOptimalPlanner& teb_planner, const ObstContainer& obstacles)
{
  FeedbackMsg msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = cfg_->map_frame;
  msg.selected_trajectory_idx = 0;
  
  msg.trajectories.resize(1);
  msg.trajectories.front().header = msg.header;
  teb_planner.getFullTrajectory(msg.trajectories.front().trajectory);
 
  // add obstacles
  msg.obstacles_msg.obstacles.resize(obstacles.size());
  for (std::size_t i=0; i<obstacles.size(); ++i)
  {
    msg.obstacles_msg.header = msg.header;

    // copy polygon
    msg.obstacles_msg.obstacles[i].header = msg.header;
    obstacles[i]->toPolygonMsg(msg.obstacles_msg.obstacles[i].polygon);

    // copy id
    msg.obstacles_msg.obstacles[i].id = i; // TODO: we do not have any id stored yet

    // orientation
    //msg.obstacles_msg.obstacles[i].orientation =; // TODO

    // copy velocities
    obstacles[i]->toTwistWithCovarianceMsg(msg.obstacles_msg.obstacles[i].velocities);
  }
  
  feedback_pub_.publish(msg);
}

void TebVisualization::pubMarker(int wx, int wy, char color, ros::Publisher publisher, mapProcess* map_obj){
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
 
    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    // marker.ns = "basic_shapes";
    marker.id = 0;
 
    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = 2;  //points
 
    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;
 
    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = wx;
    marker.pose.position.y = wy;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
 
    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
 
    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    if (color == 'r'){
      marker.color.r = 1.0f;
    }
    else if (color == 'g'){
      marker.color.g = 1.0f;
    }
    else if (color == 'b'){
      marker.color.b = 1.0f;
    }
 
    marker.lifetime = ros::Duration();
    publisher.publish(marker);
}

void TebVisualization::pubMarkerArray(std::vector<Point2D> points, char color, ros::Publisher publisher, mapProcess* map_obj){
    visualization_msgs::MarkerArray markerArray;
    int i=0;
    for (auto point:points){
      visualization_msgs::Marker marker;
      double wx=point.x;
      double wy=point.y;
      map_obj->map2world(point.x,point.y,wx,wy);
      // Set the frame ID and timestamp.  See the TF tutorials for information on these.
      marker.header.frame_id = "/map";
      marker.header.stamp = ros::Time::now();
  
      // Set the namespace and id for this marker.  This serves to create a unique ID
      // Any marker sent with the same namespace and id will overwrite the old one
      // marker.ns = "basic_shapes";
      marker.id = i++;
  
      // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
      marker.type = 2;  //points
  
      // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
      marker.action = visualization_msgs::Marker::ADD;
  
      // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
      marker.pose.position.x = wx;
      marker.pose.position.y = wy;
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
  
      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;
  
      // Set the color -- be sure to set alpha to something non-zero!
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      if (color == 'r'){
        marker.color.r = 1.0f;
      }
      else if (color == 'g'){
        marker.color.g = 1.0f;
      }
      else if (color == 'b'){
        marker.color.b = 1.0f;
      }
      marker.lifetime = ros::Duration();
      markerArray.markers.push_back(marker);
    }
    publisher.publish(markerArray);    
}


void TebVisualization::pubMarkerArray(std::vector<std::vector<Point2D>> points_list, char color, ros::Publisher publisher, mapProcess* map_obj){
    visualization_msgs::MarkerArray markerArray;
    int i=0;
    for (auto points:points_list){
      for (auto point:points){
        visualization_msgs::Marker marker;
        double wx=point.x;
        double wy=point.y;
        map_obj->map2world(point.x,point.y,wx,wy);
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();
    
        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        // marker.ns = "basic_shapes";
        marker.id = i++;
    
        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        marker.type = 2;  //points
    
        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::ADD;
    
        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = wx;
        marker.pose.position.y = wy;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
    
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
    
        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        if (color == 'r'){
          marker.color.r = 1.0f;
        }
        else if (color == 'g'){
          marker.color.g = 1.0f;
        }
        else if (color == 'b'){
          marker.color.b = 1.0f;
        }
        marker.lifetime = ros::Duration();
        markerArray.markers.push_back(marker);
      }
    }
    publisher.publish(markerArray);    
}


void TebVisualization::pubMarkerArray(std::map<int, std::vector<Point2D>> points_list, char color, ros::Publisher publisher, mapProcess* map_obj){
    std::default_random_engine e;
    std::uniform_real_distribution<float> u(0.0,1.0); // 左闭右闭区间
    e.seed(time(0));

    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker marker_del;
    marker_del.action = visualization_msgs::Marker::DELETEALL;
    markerArray.markers.emplace_back(marker_del);    
    int num=0;
    for (auto points:points_list){
      double height = 0;
      if (color=='g')
        height = 0.05;
      else if (color=='b')
        height = 0.15;
      for (auto point:points.second){
        visualization_msgs::Marker marker;
        double wx=point.x;
        double wy=point.y;
        map_obj->map2world(point.x,point.y,wx,wy);
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();
    
        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        // marker.ns = "basic_shapes";
        marker.id = num++;
    
        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        marker.type = 2;  //points
    
        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::ADD;
    
        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = wx;
        marker.pose.position.y = wy;
        // marker.pose.position.z = height++*0.05;
        marker.pose.position.z = height;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
    
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
    
        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        if (color == 'r'){
          marker.color.r = 1.0f;
        }
        else if (color == 'g'){
          marker.color.g = 1.0f;
        }
        else if (color == 'b'){
          marker.color.b = 1.0f;
        }
        marker.lifetime = ros::Duration();
        markerArray.markers.push_back(marker);
      }
    }
    publisher.publish(markerArray);    
}

void TebVisualization::pubMarkerArray(std::map<int, std::vector<Point2D>> points_list, ros::Publisher publisher, mapProcess* map_obj){
    std::default_random_engine e;
    std::uniform_real_distribution<float> u(0.0,1.0); // 左闭右闭区间
    e.seed(time(0));
    int num = 0;
    visualization_msgs::MarkerArray markerArray;
    visualization_msgs::Marker marker_del;
    marker_del.action = visualization_msgs::Marker::DELETEALL;
    markerArray.markers.emplace_back(marker_del);
    for (auto points:points_list){
      float color[3] = {1,0,0};
      // for (int j=0; j<3; j++) {
      //     color[j] = u(e);
      // }
      double height = 0;
      for (auto point:points.second){
        visualization_msgs::Marker marker;
        double wx=point.x;
        double wy=point.y;
        map_obj->map2world(point.x,point.y,wx,wy);
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();
    
        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        // marker.ns = "basic_shapes";
        marker.id = num++;
    
        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        marker.type = 2;  //points
    
        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::ADD;
    
        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = wx;
        marker.pose.position.y = wy;
        marker.pose.position.z = 0;
        // marker.pose.position.z = height++*0.05;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
    
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
    
        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = color[0];
        marker.color.g = color[1];
        marker.color.b = color[2];
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration();
        markerArray.markers.push_back(marker);
      }
    }
    publisher.publish(markerArray);    
}



void TebVisualization::pubLinList(std::vector<std::vector<std::pair<Point2D,Point2D>>> edges_graph, ros::Publisher publisher, mapProcess* map_obj)
{
  visualization_msgs::MarkerArray markerArray;
  visualization_msgs::Marker marker_del;
  marker_del.action = visualization_msgs::Marker::DELETEALL;
  markerArray.markers.emplace_back(marker_del);  
  int i=0;
  double wx;
  double wy;
  std::default_random_engine e;
  std::uniform_real_distribution<float> u(0.0,1.0); // 左闭右闭区间
  e.seed(time(0));

  double height = 0;
  for (int j=0; j<map_obj->getLabelList().size(); j++){
    for (int k=j+1; k<map_obj->getLabelList().size(); k++){
      float color[3] = {0,0,0};
      for (int j=0; j<3; j++) {
          color[j] = u(e);
      }
      auto p = edges_graph[j][k];
      visualization_msgs::Marker marker;
      marker.header.frame_id = "/map";
      marker.header.stamp = ros::Time::now();
      // marker.ns = "points_and_lins";
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.id = i++;
      marker.type = visualization_msgs::Marker::LINE_LIST;
      marker.scale.x = 0.1;
      marker.color.r = color[0];
      marker.color.g = color[1];
      marker.color.b = color[2];
      marker.color.a = 1.0;

      geometry_msgs::Point poi;
      map_obj->map2world(p.first.x,p.first.y,wx,wy);
      poi.x = wx;
      poi.y = wy;
      // poi.z = 0;
      poi.z = height*0.1;
      marker.points.push_back(poi);

      map_obj->map2world(p.second.x,p.second.y,wx,wy);
      poi.x = wx;
      poi.y = wy;
      // poi.z = 0;
      // poi.z = height++*0.1;
      poi.z = height*0.1;
      marker.points.push_back(poi);
      markerArray.markers.push_back(marker);
    }
  }
  publisher.publish(markerArray);
}

void TebVisualization::pubLinList(std::vector<std::vector<std::vector<int>>> covers_graph, ros::Publisher publisher, mapProcess* map_obj)
{
  visualization_msgs::MarkerArray markerArray;
  visualization_msgs::Marker marker_del;
  marker_del.action = visualization_msgs::Marker::DELETEALL;
  markerArray.markers.emplace_back(marker_del);  
  int num=0;
  double wx;
  double wy;
  Point2D p1,p2;
  std::default_random_engine e;
  std::uniform_real_distribution<float> u(0.0,1.0); // 左闭右闭区间
  e.seed(time(0));
  
  float height = 0;

  for (int j=0; j<map_obj->getLabelList().size(); j++){
    for (int k=j+1; k<map_obj->getLabelList().size(); k++){
      if (map_obj->getCoversGraph()[j][k].size()==0)
        continue;
      height+=0.1;  // 讲不同的piecewise投影到不同的高度
      float color[3] = {0,0,0};
      for (int j=0; j<3; j++) {
          color[j] = u(e);
      }
      // 把起点和终点包含在p_list中
      int start_obs_ID = map_obj->getLabelList()[j];
      int end_obs_ID = map_obj->getLabelList()[k];
      auto p_list = covers_graph[j][k];
      p_list.insert(p_list.begin(),start_obs_ID);
      p_list.push_back(end_obs_ID);

      for (int i=0; i<p_list.size()-1; i++){
        auto temp = map_obj->getPathBetweenObs(p_list[i], p_list[i+1]);
        p1 = temp.first;
        p2 = temp.second;

        visualization_msgs::Marker marker;
        marker.header.frame_id = "/map";
        marker.header.stamp = ros::Time::now();
        // marker.ns = "points_and_lins";
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.id = num++;
        marker.type = visualization_msgs::Marker::LINE_LIST;
        marker.scale.x = 0.1;
        marker.color.r = color[0];
        marker.color.g = color[1];
        marker.color.b = color[2];
        marker.color.a = 1.0;

        geometry_msgs::Point poi;
        map_obj->map2world(p1.x,p1.y,wx,wy);
        poi.x = wx;
        poi.y = wy;
        poi.z = 0;
        poi.z = height;
        marker.points.push_back(poi);

        map_obj->map2world(p2.x,p2.y,wx,wy);
        poi.x = wx;
        poi.y = wy;
        poi.z = height;
        marker.points.push_back(poi);
        markerArray.markers.push_back(marker);
      }
    }
  }
  publisher.publish(markerArray);
}


void TebVisualization::pubHomoPaths(const std::vector<std::vector<Eigen::Vector2d>>& paths, ros::Publisher publisher){
  if (paths.empty())
    return;

  visualization_msgs::MarkerArray markerArray;
  visualization_msgs::Marker marker_del;
  marker_del.action = visualization_msgs::Marker::DELETEALL;
  markerArray.markers.emplace_back(marker_del);  
  int num=0;
  Point2D p1,p2;
  std::default_random_engine e;
  std::uniform_real_distribution<float> u(0.0,1.0); // 左闭右闭区间
  e.seed(time(0));

  float height = 0;


  for (auto path:paths){
    height+=0.1;  // 将不同的piecewise投影到不同的高度
    float color[3] = {0,0,0};
    for (int j=0; j<3; j++) {
        color[j] = u(e);
    }    
    for (auto p:path){
      visualization_msgs::Marker marker;
      marker.header.frame_id = "/map";
      marker.header.stamp = ros::Time::now();
      // marker.ns = "points_and_lins";
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.id = num++;
      marker.type = 2;
      marker.scale.x = 0.1;
      marker.scale.y = 0.1;
      marker.scale.z = 0.1;      
      marker.color.r = color[0];
      marker.color.g = color[1];
      marker.color.b = color[2];
      marker.color.a = 1.0;

      marker.pose.position.x = p.x();
      marker.pose.position.y = p.y();
      marker.pose.position.z = height;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;

      marker.lifetime = ros::Duration();
      markerArray.markers.push_back(marker);
    }
  }
  publisher.publish(markerArray);
}


std_msgs::ColorRGBA TebVisualization::toColorMsg(double a, double r, double g, double b)
{
  std_msgs::ColorRGBA color;
  color.a = a;
  color.r = r;
  color.g = g;
  color.b = b;
  return color;
}

bool TebVisualization::printErrorWhenNotInitialized() const
{
  if (!initialized_)
  {
    ROS_ERROR("TebVisualization class not initialized. You must call initialize or an appropriate constructor");
    return true;
  }
  return false;
}

} // namespace teb_local_planner
