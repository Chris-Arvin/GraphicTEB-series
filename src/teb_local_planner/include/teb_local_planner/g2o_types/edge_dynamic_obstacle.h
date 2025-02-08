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
 * Notes:
 * The following class is derived from a class defined by the
 * g2o-framework. g2o is licensed under the terms of the BSD License.
 * Refer to the base class source for detailed licensing information.
 *
 * Author: Christoph Rösmann, Franz Albers
 *********************************************************************/

#ifndef EDGE_DYNAMICOBSTACLE_H
#define EDGE_DYNAMICOBSTACLE_H

#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/vertex_timediff.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>
#include <teb_local_planner/obstacles.h>
#include <teb_local_planner/teb_config.h>
#include <teb_local_planner/robot_footprint_model.h>

namespace teb_local_planner
{
  
/**
 * @class EdgeDynamicObstacle
 * @brief Edge defining the cost function for keeping a distance from dynamic (moving) obstacles.
 * 
 * The edge depends on two vertices \f$ \mathbf{s}_i, \Delta T_i \f$ and minimizes: \n
 * \f$ \min \textrm{penaltyBelow}( dist2obstacle) \cdot weight \f$. \n
 * \e dist2obstacle denotes the minimum distance to the obstacle trajectory (spatial and temporal). \n
 * \e weight can be set using setInformation(). \n
 * \e penaltyBelow denotes the penalty function, see penaltyBoundFromBelow(). \n
 * @see TebOptimalPlanner::AddEdgesDynamicObstacles
 * @remarks Do not forget to call setTebConfig(), setVertexIdx() and 
 * @warning Experimental
 */  
class EdgeDynamicObstacle : public BaseTebUnaryEdge<2, const Obstacle*, VertexPose>
{
public:
  
  /**
   * @brief Construct edge.
   */    
  EdgeDynamicObstacle() : t_(0), safety_margin_(0)
  {
  }
  
  /**
   * @brief Construct edge and specify the time for its associated pose (neccessary for computeError).
   * @param t_ Estimated time until current pose is reached
   */      
  EdgeDynamicObstacle(double t) : t_(t), safety_margin_(0)
  {
  }
  
  /**
   * @brief Actual cost function
   */   
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_ && _measurement && robot_model_, "You must call setTebConfig(), setObstacle() and setRobotModel() on EdgeDynamicObstacle()");
    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);
    // new relationship: 
    auto human_vel = _measurement->getCentroidVelocity();
    auto new_human_t = _measurement->getCentroid() + current_alpha_*_measurement->getCentroidVelocity()*t_;
    auto new_traj_waypoint_at_t = bandpt->pose().position();
    auto new_human_t_to_traj_t = new_traj_waypoint_at_t - new_human_t;
    auto new_relationship_along_v = (new_human_t_to_traj_t.x()*human_vel.x() + new_human_t_to_traj_t.y()*human_vel.y())/human_vel.norm();

    _error[0] = 0;
    if (is_detour_front_){
      if (old_relationship_along_v_ >0 && fabs(old_relationship_vertice_v_)<=radius_human_plus_robot_inflation_ + safety_margin_){
        _error[1] = std::max(radius_human_plus_robot_inflation_ + safety_margin_ - new_relationship_along_v, 0.0);
      }
      else{
        _error[1] = std::max(radius_human_plus_robot_inflation_ + safety_margin_ - new_human_t_to_traj_t.norm(), 0.0);  
      }
    }
    else{
      if (old_relationship_along_v_ <0 && fabs(old_relationship_vertice_v_)<=radius_human_plus_robot_inflation_ + safety_margin_){
        _error[1] = std::max(radius_human_plus_robot_inflation_ + safety_margin_ + new_relationship_along_v, 0.0);
      }
      else{
        // _error[1] = 0;
        _error[1] = std::max(radius_human_plus_robot_inflation_ + safety_margin_ - new_human_t_to_traj_t.norm(), 0.0);  
      }
    }

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "EdgeDynamicObstacle::computeError() _error[0]=%f\n",_error[0]);
  }
  
  
  /**
   * @brief Set Obstacle for the underlying cost function
   * @param obstacle Const pointer to an Obstacle or derived Obstacle
   */     
  void setObstacle(const Obstacle* obstacle)
  {
    _measurement = obstacle;
  }
  
  /**
   * @brief Set pointer to the robot model
   * @param robot_model Robot model required for distance calculation
   */
  void setRobotModel(const BaseRobotFootprintModel* robot_model)
  {
    robot_model_ = robot_model;
  }

  void setSafetyMargin(double safety_margin)
  {
    safety_margin_ = safety_margin;
  }

  /**
   * @brief Set all parameters at once
   * @param cfg TebConfig class
   * @param robot_model Robot model required for distance calculation
   * @param obstacle 2D position vector containing the position of the obstacle
   */
  void setParameters(const TebConfig& cfg, const BaseRobotFootprintModel* robot_model, const Obstacle* obstacle)
  {
    cfg_ = &cfg;
    robot_model_ = robot_model;
    _measurement = obstacle;
  }

  void setDetourDir(const bool& is_detour_front){
    is_detour_front_ = is_detour_front;
  }

  void setAlpha(const double current_alpha, const double old_alpha){
    current_alpha_ = current_alpha;
    old_alpha_ = old_alpha;    
  }

  void setAnchor(double anchor_x, double anchor_y)
  {
    anchor_ = Eigen::Vector2d(anchor_x, anchor_y);
  }

  void setGoal(double goal_x, double goal_y){
    goal_ = Eigen::Vector2d(goal_x, goal_y);
  }

  void complete(){
    // 使用dynamic_cast来转换，使得父类可以调用子类函数。【必须保证动态障碍物智能是circular类】
    CircularObstacle* temp_measurement = dynamic_cast<CircularObstacle*>(const_cast<Obstacle*>(_measurement));
    auto human_vel = temp_measurement->getCentroidVelocity();
    radius_human_plus_robot_inflation_ = temp_measurement->radius()+cfg_->obstacles.dynamic_obstacle_inflation_dist + cfg_->optim.penalty_epsilon; // human radius + extended_robot_radius + penalty_epsilon + safety margin
    // old relationship:
    auto old_human_t = temp_measurement->getCentroid() + old_alpha_*temp_measurement->getCentroidVelocity()*t_;
    auto old_traj_waypoint_at_t = anchor_;
    auto old_human_t_to_traj_t = old_traj_waypoint_at_t - old_human_t;
    old_relationship_along_v_ = (old_human_t_to_traj_t.x()*human_vel.x() + old_human_t_to_traj_t.y()*human_vel.y())/human_vel.norm();
    old_relationship_vertice_v_ = (old_human_t_to_traj_t.x()*human_vel.y() - old_human_t_to_traj_t.y()*human_vel.x())/human_vel.norm();
  }


protected:
  
  const BaseRobotFootprintModel* robot_model_; //!< Store pointer to robot_model
  double t_; //!< Estimated time until current pose is reached
  bool is_detour_front_;
  double current_alpha_, old_alpha_;
  Eigen::Vector2d anchor_;
  Eigen::Vector2d goal_;
  double radius_human_plus_robot_inflation_, old_relationship_along_v_, old_relationship_vertice_v_;
  double safety_margin_;
  
public: 
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
    
 
    

} // end namespace

#endif
