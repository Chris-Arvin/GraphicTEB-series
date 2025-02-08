/*********************************************************************
 *
 * Software License Agreement (BSD License)

 * Author: Qianyi Zhang
 *********************************************************************/
#ifndef EDGE_GOAL_LINE_H_
#define EDGE_GOAL_LINE_H_

#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>

#include "g2o/core/base_unary_edge.h"

namespace teb_local_planner
{

/**
 * @class GoalLine
 * @brief Edge defining the cost function for pushing a configuration towards a via point
 * 
 * The edge depends on a single vertex \f$ \mathbf{s}_i \f$ and minimizes: \n
 * \f$ \min  dist2point \cdot weight \f$. \n
 * \e dist2point denotes the distance to the via point. \n
 * \e weight can be set using setInformation(). \n
 * @see TebOptimalPlanner::AddEdgesGoalLine
 * @remarks Do not forget to call setTebConfig() and setViaPoint()
 */     
class GoalLine : public BaseTebUnaryEdge<1, const Eigen::Vector2d*, VertexPose>
{
public:
    
  /**
   * @brief Construct edge.
   */    
  GoalLine() 
  {
    _measurement = NULL;
  }
 
  /**
   * @brief Actual cost function
   */    
  void computeError()
  {
    ROS_ASSERT_MSG(cfg_ && _measurement, "You must call setTebConfig(), setPoint() on GoalLine()");
    const VertexPose* bandpt = static_cast<const VertexPose*>(_vertices[0]);

    // double p1end1_dis = pow(bandpt->position().x() - end_points_.first.first,2) + pow(bandpt->position().y() - end_points_.first.second,2);
    // double p2end2_dis = pow(bandpt->position().x() - end_points_.second.first,2) + pow(bandpt->position().y() - end_points_.second.second,2);
    // double end1end2_dis = pow(end_points_.first.first - end_points_.second.first,2) + pow(end_points_.first.second - end_points_.second.second,2);
    double p1end1_dis = hypot(bandpt->position().x() - end_points_.first.first, bandpt->position().y() - end_points_.first.second);
    double p2end2_dis = hypot(bandpt->position().x() - end_points_.second.first, bandpt->position().y() - end_points_.second.second);
    double end1end2_dis = hypot(end_points_.first.first - end_points_.second.first, end_points_.first.second - end_points_.second.second);

    // _error[0] = std::max(0.0, p1end1_dis + p2end2_dis - end1end2_dis-0.02);
    _error[0] = std::max(0.0, p1end1_dis + p2end2_dis - end1end2_dis);

    ROS_ASSERT_MSG(std::isfinite(_error[0]), "GoalLine::computeError() _error[0]=%f\n",_error[0]);
  }

  /**
   * @brief Set pointer to associated via point for the underlying cost function 
   * @param goal_line 2D position vector containing the position of the via point
   */ 
  void setViaPoint(const Eigen::Vector2d* goal_line)
  {
    _measurement = goal_line;
  }
    
  /**
   * @brief Set all parameters at once
   * @param cfg TebConfig class
   * @param goal_line 2D position vector containing the position of the via point
   */ 
  void setParameters(const TebConfig& cfg, const std::pair<std::pair<double,double>, std::pair<double,double>> end_points)
  {
    cfg_ = &cfg;
    end_points_ = end_points;
  }
  
public: 	
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  std::pair<std::pair<double,double>, std::pair<double,double>> end_points_;
};
  
    

} // end namespace

#endif
