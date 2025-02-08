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
 * Author: Christoph Rösmann
 *********************************************************************/

#include <teb_local_planner/homotopy_class_planner.h>
#include <teb_local_planner/h_signature.h>

namespace teb_local_planner
{
  
  
template<typename BidirIter, typename Fun>
HSignature3dPtr HomotopyClassPlanner::calculateEquivalenceClass(BidirIter path_start, BidirIter path_end, Fun fun_cplx_point, const ObstContainer* obstacles,
                                                                    boost::optional<TimeDiffSequence::iterator> timediff_start, boost::optional<TimeDiffSequence::iterator> timediff_end, bool is_xyt)
{
  HSignature3d* H = new HSignature3d(*cfg_);
  H->calculateHSignature(path_start, path_end, fun_cplx_point, obstacles, timediff_start, timediff_end, is_xyt);
  return HSignature3dPtr(H);
}

template<typename BidirIter, typename Fun>
TebOptimalPlannerPtr HomotopyClassPlanner::addAndInitNewTeb(BidirIter path_start, BidirIter path_end, Fun fun_position, std::vector<double>& signature_value, double start_orientation, double goal_orientation, const geometry_msgs::Twist* start_velocity, bool free_goal_vel, bool ix_xyt, std::pair<std::pair<double,double>, std::pair<double,double>> goal_endpoints)
{
  if(tebs_.size() >= cfg_->hcp.max_number_classes)
    return TebOptimalPlannerPtr();

  TebOptimalPlannerPtr candidate = TebOptimalPlannerPtr( new TebOptimalPlanner(*cfg_, obstacles_, robot_model_));
  candidate->teb().initTrajectoryToGoal(path_start, path_end, fun_position, cfg_->robot.max_vel_x, cfg_->robot.max_vel_theta,
                                 cfg_->robot.acc_lim_x, cfg_->robot.acc_lim_theta, start_orientation, goal_orientation, cfg_->trajectory.min_samples,
                                 cfg_->trajectory.allow_init_with_backwards_motion);
  if (start_velocity)
    candidate->setVelocityStart(*start_velocity);
  HSignature3dPtr H = calculateEquivalenceClass(candidate->teb().poses().begin(), candidate->teb().poses().end(), getCplxFromVertexPosePtr, obstacles_,
                                                    candidate->teb().timediffs().begin(), candidate->teb().timediffs().end(), ix_xyt);
  candidate->setHsignature3DValue(H->getSignature3DValue());
  signature_value = H->getSignature3DValue();
  // std::cout<<"self: ["<<H->getSignature3DValue().size()<<"]"<<std::endl;
  if (free_goal_vel)
    candidate->setVelocityGoalFree();

  if(addEquivalenceClassIfNew(H))
  {
    candidate->setEndLines(goal_endpoints);
    tebs_.push_back(candidate);
    return tebs_.back();
  }
  // 为具有相同H-signature的轨迹添加最后一点，以实现持续更新
  else{
    for(auto iter=tebs_.begin(); iter!=tebs_.end(); ++iter){
      // std::cout<<"old: "<<iter->get()->getHsignature3DValue().size()<<std::endl;
      if (H->isEqual(iter->get()->getHsignature3DValue())){
        if (iter->get()->getEndpointAppended()==false){
          iter->get()->setEndpointAppended();
          iter->get()->setEndLines(goal_endpoints);
          PoseSE2 temp = PoseSE2(candidate->teb().poses().back()->x(), candidate->teb().poses().back()->y(), candidate->teb().poses().back()->theta());
          iter->get()->teb().BackPose() = temp;
          break;
        }
      }
    }
  }
  // If the candidate constitutes no new equivalence class, return a null pointer
  return TebOptimalPlannerPtr();
}
} // namespace teb_local_planner

