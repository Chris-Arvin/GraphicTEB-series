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
 * Author: Christoph Rösmann
 *********************************************************************/
#ifndef EDGE_OBSTACLE_H_
#define EDGE_OBSTACLE_H_

#include <teb_local_planner/obstacles.h>
#include <teb_local_planner/robot_footprint_model.h>
#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include <teb_local_planner/teb_config.h>



namespace teb_local_planner
{
/**
 * @class EdgeInflatedObstacle
 * @brief Edge defining the cost function for keeping a minimum distance from inflated obstacles.
 * 
 * The edge depends on a single vertex \f$ \mathbf{s}_i \f$ and minimizes: \n
 * \f$ \min \textrm{penaltyBelow}( dist2point, min_obstacle_dist ) \cdot weight_inflation \f$. \n
 * Additional, a second penalty is provided with \n
 * \f$ \min \textrm{penaltyBelow}( dist2point, inflation_dist ) \cdot weight_inflation \f$.
 * It is assumed that inflation_dist > min_obstacle_dist and weight_inflation << weight_inflation.
 * \e dist2point denotes the minimum distance to the point obstacle. \n
 * \e penaltyBelow denotes the penalty function, see penaltyBoundFromBelow() \n
 * @see TebOptimalPlanner::AddEdgesObstacles, TebOptimalPlanner::EdgeObstacle
 * @remarks Do not forget to call setTebConfig() and setObstacle()
 */     
class EdgeInflatedObstacle : public BaseTebUnaryEdge<2, const Obstacle*, VertexPose>
{
public:
    
  /**
   * @brief Construct edge.
   */    
  EdgeInflatedObstacle() 
  {
    _measurement = NULL;
    safety_margin_ = 0;
  }
 
  /**
   * @brief Actual cost function
   */    
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_ && _measurement && robot_model_, "You must call setTebConfig(), setObstacle() and setRobotModel() on EdgeInflatedObstacle()");
    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);

    double dist = robot_model_->calculateDistance(bandpt->pose(), _measurement);
    _error[0] = penaltyBoundFromBelow(dist, cfg_->obstacles.min_obstacle_dist, cfg_->optim.penalty_epsilon+safety_margin_, bandpt->pose(),costmap2d_, obs_map_labeled_);
    _error[1] = penaltyBoundFromBelow(dist, cfg_->obstacles.inflation_dist, 0.0, bandpt->pose(),costmap2d_, obs_map_labeled_);
    if (cfg_->optim.obstacle_cost_exponent != 1.0 && cfg_->obstacles.min_obstacle_dist > 0.0)   //default:4, 0.25
    {
      // Optional non-linear cost. Note the max cost (before weighting) is
      // the same as the straight line version and that all other costs are
      // below the straight line (for positive exponent), so it may be
      // necessary to increase weight_obstacle and/or the inflation_weight
      // when using larger exponents.
      _error[0] = cfg_->obstacles.min_obstacle_dist * std::pow(_error[0] / cfg_->obstacles.min_obstacle_dist, cfg_->optim.obstacle_cost_exponent);
    }

    ROS_ASSERT_MSG(std::isfinite(_error[0]) && std::isfinite(_error[1]), "EdgeInflatedObstacle::computeError() _error[0]=%f, _error[1]=%f\n",_error[0], _error[1]);
  }

  /**
   * @brief Set pointer to associated obstacle for the underlying cost function 
   * @param obstacle 2D position vector containing the position of the obstacle
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
    
  /**
   * @brief Set all parameters at once
   * @param cfg TebConfig class
   * @param robot_model Robot model required for distance calculation
   * @param obstacle 2D position vector containing the position of the obstacle
   */ 
  void setParameters(const TebConfig& cfg, const BaseRobotFootprintModel* robot_model, const Obstacle* obstacle, double safety_margin)
  {
    cfg_ = &cfg;
    robot_model_ = robot_model;
    _measurement = obstacle;
    safety_margin_ = safety_margin;
  }

  void setCostmap(costmap_2d::Costmap2D* costmap){costmap2d_=costmap;};

  void setObsMapLabeled(const std::vector<std::vector<int>>& obs_map_labeled){obs_map_labeled_=obs_map_labeled;};

protected:

  const BaseRobotFootprintModel* robot_model_; //!< Store pointer to robot_model
  costmap_2d::Costmap2D* costmap2d_;
  std::vector<std::vector<int>> obs_map_labeled_;
  double safety_margin_;
public:         
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
    

} // end namespace

#endif
