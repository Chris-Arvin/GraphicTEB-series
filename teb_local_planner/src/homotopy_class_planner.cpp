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

#include <teb_local_planner/homotopy_class_planner.h>

#include <limits>

namespace teb_local_planner
{

HomotopyClassPlanner::HomotopyClassPlanner() : cfg_(NULL), obstacles_(NULL), via_points_(NULL), robot_model_(new PointRobotFootprint()), initial_plan_(NULL), initialized_(false)
{
}

HomotopyClassPlanner::HomotopyClassPlanner(costmap_2d::Costmap2D* costmap2d, const TebConfig& cfg, ObstContainer* obstacles, RobotFootprintModelPtr robot_model,
                                           TebVisualizationPtr visual, const ViaPointContainer* via_points) : initial_plan_(NULL)
{
  initialize(costmap2d, cfg, obstacles, robot_model, visual, via_points);
}

HomotopyClassPlanner::~HomotopyClassPlanner()
{
}

void HomotopyClassPlanner::initialize(costmap_2d::Costmap2D* costmap2d, const TebConfig& cfg, ObstContainer* obstacles, RobotFootprintModelPtr robot_model,
                                      TebVisualizationPtr visual, const ViaPointContainer* via_points)
{
  costmap2d_ = costmap2d;
  cfg_ = &cfg;
  obstacles_ = obstacles;
  via_points_ = via_points;
  robot_model_ = robot_model;
  // 这个参数，在obstacle covert和PRM中选择。默认为0，即使用PRM
  if (cfg_->hcp.graphic_exploration){ //使用graphic teb
    graph_search_ = boost::shared_ptr<GraphSearchInterface>(new graphicProcess(costmap2d_, *cfg_, this));
  }
  else if (cfg_->hcp.simple_exploration){ //使用障碍物拟合
    graph_search_ = boost::shared_ptr<GraphSearchInterface>(new lrKeyPointGraph(*cfg_, this));
  }
  else{ // 使用随机采样
    graph_search_ = boost::shared_ptr<GraphSearchInterface>(new ProbRoadmapGraph(*cfg_, this));
  }

  std::random_device rd;
  random_.seed(rd());

  initialized_ = true;

  setVisualization(visual);
}

void HomotopyClassPlanner::updateCostmap(costmap_2d::Costmap2D* costmap)
{
  costmap2d_ = costmap;
}

void HomotopyClassPlanner::updateObstacles(ObstContainer* obstacles){
  obstacles_ = obstacles;
}

void HomotopyClassPlanner::updateObsMap(const std::vector<std::vector<int>>& obs_map_labeled){
  obs_map_labeled_ = obs_map_labeled;
}

void HomotopyClassPlanner::updateCorridorMap(const std::vector<std::vector<double>>& corridorMap){
  corridorMap_ = corridorMap;
}

void HomotopyClassPlanner::updateDynamicObs(std::vector<std::vector<double>> dynamic_obstacles){
  dynamic_obstacles_ = dynamic_obstacles;
}

void HomotopyClassPlanner::updateObstacles(const std::map<int, std::vector<std::pair<double,double>>>& static_obstacle, std::vector<std::vector<double>> dynamic_obstacle){
  obstacles_->clear();
  for (auto obs_group:static_obstacle){
    PolygonObstacle* polyobst = new PolygonObstacle;
    for (auto p:obs_group.second){
      polyobst->pushBackVertex(p.first,p.second);
    }
    polyobst->finalizePolygon();
    obstacles_->push_back(ObstaclePtr(polyobst));
  }
  for(auto obs:dynamic_obstacle){
    obstacles_->push_back(ObstaclePtr(new CircularObstacle(obs[0], obs[1], obs[4])));
    obstacles_->back()->setCentroidVelocity(Eigen::Vector2d(obs[2],obs[3]));
  }
}



void HomotopyClassPlanner::updateRobotModel(RobotFootprintModelPtr robot_model )
{
  robot_model_ = robot_model;
}

void HomotopyClassPlanner::setVisualization(TebVisualizationPtr visualization)
{
  visualization_ = visualization;
}



bool HomotopyClassPlanner::plan(const std::vector<geometry_msgs::PoseStamped>& initial_plan, const geometry_msgs::Twist* start_vel, bool free_goal_vel)
{
  ROS_ASSERT_MSG(initialized_, "Call initialize() first.");

  // store initial plan for further initializations (must be valid for the lifetime of this object or clearPlanner() is called!)
  // initial_plan_ = &initial_plan;
  initial_plan_ = nullptr;

  PoseSE2 start(initial_plan.front().pose);
  PoseSE2 goal(initial_plan.back().pose);

  return plan(start, goal, start_vel, free_goal_vel);
}


bool HomotopyClassPlanner::plan(const tf::Pose& start, const tf::Pose& goal, const geometry_msgs::Twist* start_vel, bool free_goal_vel)
{
  ROS_ASSERT_MSG(initialized_, "Call initialize() first.");
  PoseSE2 start_pose(start);
  PoseSE2 goal_pose(goal);
  return plan(start_pose, goal_pose, start_vel, free_goal_vel);
}

// 程序入口
bool HomotopyClassPlanner::plan(const PoseSE2& start, const PoseSE2& goal, const geometry_msgs::Twist* start_vel, bool free_goal_vel)
{
  ROS_ASSERT_MSG(initialized_, "Call initialize() first.");
  // Update old TEBs with new start, goal and velocity
  /**
   * teb_是一个vector，由<optimal_planner中维护的单个teb的对象>组成
   * 遍历teb_，比较上一时刻的teb的结果和局部目标点间的"位置差"和"角度差"，如果差值较大，则清空上一时刻的结果
   * 如果差别不大，则删掉机器人已经cover的viapoint
   * **/
  updateAllTEBs(&start, &goal, start_vel);

  // Init new TEBs based on newly explored homotopy classes
  /**
   * 计算每条轨迹的h-signature，删除重复的轨迹
   * 调用createGraph，创建图 
     * NOTE: tebs_是在下面这个函数里被append的，在graph_searcher的类中，定义了hcp_这个指针来指向tebs_
     * NOTE: createGraph有两个重载，第一个是PRM（默认），第二个是costmap convert
   * **/
  exploreEquivalenceClassesAndInitTebs(start, goal, cfg_->obstacles.min_obstacle_dist, start_vel, free_goal_vel);

  // update via-points if activated
  /**
   *  添加必须要经过的viapoints
   * **/
  updateReferenceTrajectoryViaPoints(cfg_->hcp.viapoints_all_candidates);

  auto t = ros::Time::now();
  // Optimize all trajectories in alternative homotopy classes
  /** 
   * 对teb_中的每条轨迹：调用TebOptimalPlanner::optimizeTEB来优化求解
   * **/
  optimizeAllTEBs(cfg_->optim.no_inner_iterations, cfg_->optim.no_outer_iterations);
  ROS_INFO("5.Time to optimize all paths: [%f] s", (ros::Time::now() - t).toSec());

  // Select which candidate (based on alternative homotopy classes) should be used
  /**
   *  遍历所有同伦轨迹，选择cost最小的那个：tricks：
      * trick1：正在执行的轨迹，被选择的概率更大
      * tirck2：要求轨迹转换的时间间隔大于某个参数
   * **/
  selectBestTeb();
  initial_plan_ = nullptr; // clear pointer to any previous initial plan (any previous plan is useless regarding the h-signature);
  return true;
}

bool HomotopyClassPlanner::getVelocityCommand(double& vx, double& vy, double& omega, int look_ahead_poses) const
{
  TebOptimalPlannerConstPtr best_teb = bestTeb();
  if (!best_teb)
  {
    vx = 0;
    vy = 0;
    omega = 0;
    return false;
  }

  return best_teb->getVelocityCommand(vx, vy, omega, look_ahead_poses);
}




void HomotopyClassPlanner::visualize()
{
  if (visualization_)
  {
    // Visualize graph
    if (cfg_->hcp.visualize_hc_graph && graph_search_)
      visualization_->publishGraph(graph_search_->graph_);

    // Visualize active tebs as marker
    visualization_->publishTebContainer(tebs_);
    
    // Visualize best teb and feedback message if desired
    TebOptimalPlannerConstPtr best_teb = bestTeb();
    TebOptimalPlannerConstPtr best_teb_origin = bestTebOrigin();
    if (best_teb)
    {
      visualization_->publishLocalPlanAndPoses(best_teb->teb(), best_teb_origin->teb());

      if (best_teb->teb().sizePoses() > 0) //TODO maybe store current pose (start) within plan method as class field.
        visualization_->publishRobotFootprintModel(best_teb->teb().Pose(0), *robot_model_);

      // feedback message
      if (cfg_->trajectory.publish_feedback)
      {
        int best_idx = bestTebIdx();
        if (best_idx>=0)
          visualization_->publishFeedbackMessage(tebs_, (unsigned int) best_idx, *obstacles_);
      }
    }

    if(cfg_->hcp.visualize_graphic_exploration && cfg_->hcp.graphic_exploration){
      auto map_obj = graph_search_->getMapProcess();
      visualization_->pubMarkerArray(map_obj->getObsList(), visualization_->pub_obs_group,map_obj);
      visualization_->pubMarkerArray(map_obj->getBorderList(), 'g', visualization_->pub_border, map_obj);
      visualization_->pubMarkerArray(map_obj->getCornerList(), 'b', visualization_->pub_corner, map_obj);
      visualization_->pubLinList(map_obj->getEdgesGraph(), visualization_->pub_connect_between_obs_adjecent, map_obj);
      visualization_->pubLinList(map_obj->getCoversGraph(), visualization_->pub_connect_between_obs_all, map_obj);
      visualization_->pubHomoPaths(map_obj->getHomoPathsPruned(), visualization_->pub_homo_paths);
    }
  }
  else ROS_DEBUG("Ignoring HomotopyClassPlanner::visualize() call, since no visualization class was instantiated before.");
}



bool HomotopyClassPlanner::hasEquivalenceClass(const EquivalenceClassPtr& eq_class) const
{
  // iterate existing h-signatures and check if there is an existing H-Signature similar the candidate
  for (const std::pair<EquivalenceClassPtr, bool>& eqrel : equivalence_classes_)
  {
     if (eq_class->isEqual(*eqrel.first))
        return true; // Found! Homotopy class already exists, therefore nothing added
  }
  return false;
}

// lock默认false
bool HomotopyClassPlanner::addEquivalenceClassIfNew(const EquivalenceClassPtr& eq_class, bool lock)
{
  if (!eq_class)
    return false;

  if (!eq_class->isValid())
  {
    ROS_WARN("HomotopyClassPlanner: Ignoring invalid H-signature");
    return false;
  }

  // 如果新轨迹和 已与轨迹的h-signature相同，则该轨迹必须满足：1.该轨迹的h-signature和最优轨迹重合；2.现有最优轨迹的数量小于阈值max_number_plans_in_current_class
  // 但实际上，max_number_plans_in_current_class默认为1，即，不允许h-signature相同的轨迹存在。
  if (hasEquivalenceClass(eq_class))
  {
    // Allow up to configured number of Tebs that are in the same homotopy
    // class as the current (best) Teb to avoid being stuck in a local minimum
    if (!isInBestTebClass(eq_class) || numTebsInBestTebClass() >= cfg_->hcp.max_number_plans_in_current_class)
      return false;
  }

  // Homotopy class not found -> Add to class-list, return that the h-signature is new
  equivalence_classes_.push_back(std::make_pair(eq_class,lock));
  return true;
}


void HomotopyClassPlanner::renewAndAnalyzeOldTebs(bool delete_detours)
{
  // auto t = ros::Time::now();
  // clear old h-signatures (since they could be changed due to new obstacle positions.
  rob_radius_ = 0.2;
  obs_radius_ = 0.3;
  equivalence_classes_.clear();

  // Adding the equivalence class of the latest best_teb_ first
  // 把上一时刻得到的best_teb_添加到当前的同伦参考中
  TebOptPlannerContainer::iterator it_best_teb = best_teb_ ? std::find(tebs_.begin(), tebs_.end(), best_teb_) : tebs_.end();
  bool has_best_teb = it_best_teb != tebs_.end();
  bool is_collision = false;
  // 如果上一时刻的best_teb被检测到碰撞，则删除该轨迹
  if (has_best_teb){
    double time = 0;
    int i=0;
    auto traj = tebs_.front()->teb().poses();
    for (int i=0; i<traj.size()-1; i++){
      if (is_collision)
        break;
      for (auto obs: *obstacles_){
        if (is_collision)
          break;
        // 处理动态障碍物
        if (obs->isDynamic()){
          auto po = obs->getCentroid() + time*obs->getCentroidVelocity();
          if (std::hypot(po.x()-traj[i]->x(),po.y()-traj[i]->y())<rob_radius_+obs_radius_){
            is_collision = true;
          }
        }
        // 处理静态障碍物
        else{
          if (obs->checkCollision(Eigen::Vector2d(traj[i]->x(),traj[i]->y()),rob_radius_)){
            is_collision = true;
          }
        }
      }
      time += tebs_.front()->teb().TimeDiff(i);
    }
  }
  if (has_best_teb && (!is_collision))
  {
    std::iter_swap(tebs_.begin(), it_best_teb);  // Putting the last best teb at the beginning of the container
    // 按照17年的RAS来计算
    best_teb_eq_class_ = calculateEquivalenceClass(best_teb_->teb().poses().begin(),
      best_teb_->teb().poses().end(), getCplxFromVertexPosePtr , obstacles_,
      best_teb_->teb().timediffs().begin(), best_teb_->teb().timediffs().end());
    // 如果这条轨迹的h-signature没有重复过中，则保存该signature
    addEquivalenceClassIfNew(best_teb_eq_class_);
  }
  // std::cout<<"     ininer1:"<<(ros::Time::now() - t).toSec()<<std::endl;
  // t = ros::Time::now();
  // Collect h-signatures for all existing TEBs and store them together with the corresponding iterator / pointer:
//   typedef std::list< std::pair<TebOptPlannerContainer::iterator, std::complex<long double> > > TebCandidateType;
//   TebCandidateType teb_candidates;

  TebOptPlannerContainer::iterator it_teb = (has_best_teb&& (!is_collision)) ? std::next(tebs_.begin(), 1) : tebs_.begin();
  // 如果开启了graphic，那么前面这里就不用做signature的检测了，等会在graph_search里做就好
  if (cfg_->hcp.graphic_exploration){
    while(it_teb != tebs_.end())
      {
          it_teb = tebs_.erase(it_teb);
      }
  }
  // get new homotopy classes and delete multiple TEBs per homotopy class. Skips the best teb if available (added before).
  else{
    while(it_teb != tebs_.end())
    {
      // calculate equivalence class for the current candidate
      EquivalenceClassPtr equivalence_class = calculateEquivalenceClass(it_teb->get()->teb().poses().begin(), it_teb->get()->teb().poses().end(), getCplxFromVertexPosePtr , obstacles_,
                                                                        it_teb->get()->teb().timediffs().begin(), it_teb->get()->teb().timediffs().end());

  //     teb_candidates.push_back(std::make_pair(it_teb,H));

      // WORKAROUND until the commented code below works
      // Here we do not compare cost values. Just first come first serve...
      // 仅保留非重复h_signature的轨迹
      bool new_flag = addEquivalenceClassIfNew(equivalence_class);
      if (!new_flag)
      {
        it_teb = tebs_.erase(it_teb);   // 注意一下，这一行写的挺有意思的。使用erase之后，it_teb指向的原地址就被清空且释放了，it_teb也变成了野指针，这时如果不管不顾，直接it_teb++，那么新的it_teb指得可能是个乱的地方，会报错。但erase本身会返回"被删除元素的下一个元素的指针"，配合continue使用，直接达到了先原地erase，再++的作用。
        continue;
      }

      ++it_teb;
    }
  }
  // std::cout<<"     ininer2:"<<(ros::Time::now() - t).toSec()<<std::endl;
  // t = ros::Time::now();  
  // 默认为true，删除有大转角的轨迹
  if(delete_detours)
    deletePlansDetouringBackwards(cfg_->hcp.detours_orientation_tolerance, cfg_->hcp.length_start_orientation_vector);

  // Find multiple candidates and delete the one with higher cost
  // TODO: this code needs to be adpated. Erasing tebs from the teb container_ could make iteratores stored in the candidate list invalid!
//   TebCandidateType::reverse_iterator cand_i = teb_candidates.rbegin();
//   int test_idx = 0;
//   while (cand_i != teb_candidates.rend())
//   {
//
//     TebCandidateType::reverse_iterator cand_j = std::find_if(boost::next(cand_i),teb_candidates.rend(), boost::bind(compareH,_1,cand_i->second));
//     if (cand_j != teb_candidates.rend() && cand_j != cand_i)
//     {
//         TebOptimalPlannerPtr pt1 = *(cand_j->first);
//         TebOptimalPlannerPtr pt2 = *(cand_i->first);
//         assert(pt1);
//         assert(pt2);
//       if ( cand_j->first->get()->getCurrentCost().sum() > cand_i->first->get()->getCurrentCost().sum() )
//       {
// 	// found one that has higher cost, therefore erase cand_j
// 	tebs_.erase(cand_j->first);
// 	teb_candidates.erase(cand_j);
//       }
//       else   // otherwise erase cand_i
//       {
// 	tebs_.erase(cand_i->first);
// 	cand_i = teb_candidates.erase(cand_i);
//       }
//     }
//     else
//     {
//         ROS_WARN_STREAM("increase cand_i");
//         ++cand_i;
//     }
//   }

  // now add the h-signatures to the internal lookup-table (but only if there is no existing duplicate)
//   for (TebCandidateType::iterator cand=teb_candidates.begin(); cand!=teb_candidates.end(); ++cand)
//   {
//     bool new_flag = addNewHSignatureIfNew(cand->second, cfg_->hcp.h_signature_threshold);
//     if (!new_flag)
//     {
// //       ROS_ERROR_STREAM("getAndFilterHomotopyClassesTEB() - This schould not be happen.");
//       tebs_.erase(cand->first);
//     }
//   }

}

void HomotopyClassPlanner::updateReferenceTrajectoryViaPoints(bool all_trajectories)
{
  std::cout<<"               tebs: "<<tebs_.size()<<std::endl;
  if ( (!all_trajectories && !initial_plan_) || !via_points_ || via_points_->empty() || cfg_->optim.weight_viapoint <= 0)
    return;

  if(equivalence_classes_.size() < tebs_.size())
  {
    ROS_ERROR("HomotopyClassPlanner::updateReferenceTrajectoryWithViaPoints(): Number of h-signatures does not match number of trajectories.");
    return;
  }

  if (all_trajectories)
  {
    // enable via-points for all tebs
    for (std::size_t i=0; i < equivalence_classes_.size(); ++i)
    {
        tebs_[i]->setViaPoints(via_points_);
    }
  }
  else
  {
    // enable via-points for teb in the same hommotopy class as the initial_plan and deactivate it for all other ones
    for (std::size_t i=0; i < equivalence_classes_.size(); ++i)
    {
      if(initial_plan_eq_class_->isEqual(*equivalence_classes_[i].first))
        tebs_[i]->setViaPoints(via_points_);
      else
        tebs_[i]->setViaPoints(NULL);
    }
  }
}


void HomotopyClassPlanner::exploreEquivalenceClassesAndInitTebs(const PoseSE2& start, const PoseSE2& goal, double dist_to_obst, const geometry_msgs::Twist* start_vel, bool free_goal_vel)
{
  // first process old trajectories
  // 计算每个轨迹的h-signature，删除重复h-signature的轨迹
  renewAndAnalyzeOldTebs(cfg_->hcp.delete_detours_backwards);
  // 随机删除一些轨迹，但默认参数是不调用这个的~
  randomlyDropTebs();
  // inject initial plan if available and not yet captured
  // initial_plan_对应的是move_base给过来的global plan。
  // 目前已经有了很多teb轨迹和h-signature；这里只是做了一次冗余处理，把全局路径加进去。
  // initial_plan_teb_仅会通过addAndInitNewTeb来赋值，如果本轮规划cycle中没有全局路径(initial_plan_)，则initial_plan_teb_也不会被更新；getInitialPlanTeb()是在当前cycle中找 和 上一cycle中根据全局路径产生的轨迹 的h-signature 相同的轨迹
  if (initial_plan_)
  {
    initial_plan_teb_ = addAndInitNewTeb(*initial_plan_, start_vel, free_goal_vel);
  }
  else
  {
    initial_plan_teb_.reset();
    initial_plan_teb_ = getInitialPlanTEB(); // this method searches for initial_plan_eq_class_ in the teb container (-> if !initial_plan_teb_)
  }
  // now explore new homotopy classes and initialize tebs if new ones are found. The appropriate createGraph method is chosen via polymorphism.
  // 目前，已经做完初始化了，接下来通过深搜来获取所有同伦路径
  graph_search_->updateDynamicObstacle(dynamic_obstacles_);
  graph_search_->createGraph(start, goal, costmap2d_, dist_to_obst, cfg_->hcp.obstacle_heading_threshold, start_vel, free_goal_vel);
}


TebOptimalPlannerPtr HomotopyClassPlanner::addAndInitNewTeb(const PoseSE2& start, const PoseSE2& goal, const geometry_msgs::Twist* start_velocity, bool free_goal_vel)
{
  if(tebs_.size() >= cfg_->hcp.max_number_classes)
    return TebOptimalPlannerPtr();
  TebOptimalPlannerPtr candidate =  TebOptimalPlannerPtr( new TebOptimalPlanner(*cfg_, obstacles_, robot_model_, visualization_));

  candidate->teb().initTrajectoryToGoal(start, goal, 0, cfg_->robot.max_vel_x, cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion);

  if (start_velocity)
    candidate->setVelocityStart(*start_velocity);

  EquivalenceClassPtr H = calculateEquivalenceClass(candidate->teb().poses().begin(), candidate->teb().poses().end(), getCplxFromVertexPosePtr, obstacles_,
                                                    candidate->teb().timediffs().begin(), candidate->teb().timediffs().end());
  if (free_goal_vel)
    candidate->setVelocityGoalFree(); 

  if(addEquivalenceClassIfNew(H))
  {
    tebs_.push_back(candidate);
    return tebs_.back();
  }

  // If the candidate constitutes no new equivalence class, return a null pointer
  return TebOptimalPlannerPtr();
}


bool HomotopyClassPlanner::isInBestTebClass(const EquivalenceClassPtr& eq_class) const
{
  bool answer = false;
  if (best_teb_eq_class_)
    answer = best_teb_eq_class_->isEqual(*eq_class);
  return answer;
}

int HomotopyClassPlanner::numTebsInClass(const EquivalenceClassPtr& eq_class) const
{
  int count = 0;
  for (const std::pair<EquivalenceClassPtr, bool>& eqrel : equivalence_classes_)
  {
    if (eq_class->isEqual(*eqrel.first))
      ++count;
  }
  return count;
}

int HomotopyClassPlanner::numTebsInBestTebClass() const
{
  int count = 0;
  if (best_teb_eq_class_)
    count = numTebsInClass(best_teb_eq_class_);
  return count;
}

TebOptimalPlannerPtr HomotopyClassPlanner::addAndInitNewTeb(const std::vector<geometry_msgs::PoseStamped>& initial_plan, const geometry_msgs::Twist* start_velocity, bool free_goal_vel)
{
  if(tebs_.size() >= cfg_->hcp.max_number_classes)
    return TebOptimalPlannerPtr();
  TebOptimalPlannerPtr candidate = TebOptimalPlannerPtr( new TebOptimalPlanner(*cfg_, obstacles_, robot_model_, visualization_));

  candidate->teb().initTrajectoryToGoal(initial_plan, cfg_->robot.max_vel_x, cfg_->robot.max_vel_theta,
    cfg_->trajectory.global_plan_overwrite_orientation, cfg_->trajectory.min_samples, cfg_->trajectory.allow_init_with_backwards_motion);

  if (start_velocity)
    candidate->setVelocityStart(*start_velocity);

  if (free_goal_vel)
    candidate->setVelocityGoalFree();

  // store the h signature of the initial plan to enable searching a matching teb later.
  initial_plan_eq_class_ = calculateEquivalenceClass(candidate->teb().poses().begin(), candidate->teb().poses().end(), getCplxFromVertexPosePtr, obstacles_,
                                                     candidate->teb().timediffs().begin(), candidate->teb().timediffs().end());

  if(addEquivalenceClassIfNew(initial_plan_eq_class_, true)) // also prevent candidate from deletion
  {
    tebs_.push_back(candidate);
    return tebs_.back();
  }

  // If the candidate constitutes no new equivalence class, return a null pointer
  return TebOptimalPlannerPtr();
}

void HomotopyClassPlanner::updateAllTEBs(const PoseSE2* start, const PoseSE2* goal, const geometry_msgs::Twist* start_velocity)
{
  // If new goal is too far away, clear all existing trajectories to let them reinitialize later.
  // Since all Tebs are sharing the same fixed goal pose, just take the first candidate:
  // 如果现有的teb_的vector不为空，且局部目标点和已与teb_的最终点的距离大于1 或 角度大于90°，则清空teb_和所有的同伦类
  if (!tebs_.empty()
      && ((goal->position() - tebs_.front()->teb().BackPose().position()).norm() >= cfg_->trajectory.force_reinit_new_goal_dist
        || fabs(g2o::normalize_theta(goal->theta() - tebs_.front()->teb().BackPose().theta())) >= cfg_->trajectory.force_reinit_new_goal_angular))
  {
      ROS_DEBUG("New goal: distance to existing goal is higher than the specified threshold. Reinitalizing trajectories.");
      tebs_.clear();
      equivalence_classes_.clear();
  }

  // hot-start from previous solutions
  // 遍历teb_删除已经cover的轨迹
  for (TebOptPlannerContainer::iterator it_teb = tebs_.begin(); it_teb != tebs_.end(); ++it_teb)
  {
    it_teb->get()->teb().updateAndPruneTEB(*start, *goal);
    if (start_velocity)
      it_teb->get()->setVelocityStart(*start_velocity);
  }
}


void HomotopyClassPlanner::optimizeAllTEBs(int iter_innerloop, int iter_outerloop)
{
  // 如果用grahpicTEB，则需要在优化点到障碍物的距离时，用到costmap和mapObsLabeled
  if (cfg_->hcp.graphic_exploration){
    for (TebOptPlannerContainer::iterator it_teb = tebs_.begin(); it_teb != tebs_.end(); ++it_teb){
      it_teb->get()->setCostmap(costmap2d_);
      it_teb->get()->setObsMapLabeled(obs_map_labeled_);
    }
  }

  // optimize TEBs in parallel since they are independend of each other
  if (cfg_->hcp.enable_multithreading)
  {
    // Must prevent .join_all() from throwing exception if interruption was
    // requested, as this can lead to multiple threads operating on the same
    // TEB, which leads to SIGSEGV
    boost::this_thread::disable_interruption di;

    boost::thread_group teb_threads;
    // tebs_实际上是optimal_planner的一个vector
    for (TebOptPlannerContainer::iterator it_teb = tebs_.begin(); it_teb != tebs_.end(); ++it_teb)
    {
      teb_threads.create_thread( boost::bind(&TebOptimalPlanner::optimizeTEB, it_teb->get(), iter_innerloop, iter_outerloop,
                                             true, cfg_->hcp.selection_obst_cost_scale, cfg_->hcp.selection_viapoint_cost_scale,
                                             cfg_->hcp.selection_alternative_time_cost) );
    }
    teb_threads.join_all();
  }
  else
  {
    for (TebOptPlannerContainer::iterator it_teb = tebs_.begin(); it_teb != tebs_.end(); ++it_teb)
    {
      it_teb->get()->optimizeTEB(iter_innerloop,iter_outerloop, true, cfg_->hcp.selection_obst_cost_scale,
                                 cfg_->hcp.selection_viapoint_cost_scale, cfg_->hcp.selection_alternative_time_cost); // compute cost as well inside optimizeTEB (last argument = true)
    }
  }
}

// equivalence_classes_是一个被typedef后的vector，保存的是pair<h-signature的类，is_block>
// 这个函数会在没有收到全局路径规划时被调用，通过对比上一时刻“根据全局路径产生的轨迹”，找到当前时刻中 和该轨迹的h-signature相同的轨迹。
TebOptimalPlannerPtr HomotopyClassPlanner::getInitialPlanTEB()
{
    // first check stored teb object
    if (initial_plan_teb_)
    {
        // check if the teb is still part of the teb container
        if ( std::find(tebs_.begin(), tebs_.end(), initial_plan_teb_ ) != tebs_.end() )
            return initial_plan_teb_;
        else
        {
            initial_plan_teb_.reset(); // reset pointer for next call
            ROS_DEBUG("initial teb not found, trying to find a match according to the cached equivalence class");
        }
    }

    // reset the locked state for equivalence classes // TODO: this might be adapted if not only the plan containing the initial plan is locked!
    for (int i=0; i<equivalence_classes_.size(); ++i)
    {
        equivalence_classes_[i].second = false;
    }

    // otherwise check if the stored reference equivalence class exist in the list of known classes
    if (initial_plan_eq_class_ && initial_plan_eq_class_->isValid())
    {
         if (equivalence_classes_.size() == tebs_.size())
         {
            for (int i=0; i<equivalence_classes_.size(); ++i)
            {
                if (equivalence_classes_[i].first->isEqual(*initial_plan_eq_class_))
                {
                    equivalence_classes_[i].second = true;
                    return tebs_[i];
                }
            }
         }
         else
             ROS_ERROR("HomotopyClassPlanner::getInitialPlanTEB(): number of equivalence classes (%lu) and number of trajectories (%lu) does not match.", equivalence_classes_.size(), tebs_.size());
    }
    else
        ROS_DEBUG("HomotopyClassPlanner::getInitialPlanTEB(): initial TEB not found in the set of available trajectories.");

    return TebOptimalPlannerPtr();
}

void HomotopyClassPlanner::randomlyDropTebs()
{
  // selection_dropping_probability默认为0，不调用
  if (cfg_->hcp.selection_dropping_probability == 0.0)
  {
    return;
  }
  // interate both vectors in parallel
  auto it_eqrel = equivalence_classes_.begin();
  auto it_teb = tebs_.begin();
  while (it_teb != tebs_.end() && it_eqrel != equivalence_classes_.end())
  {
    if (it_teb->get() != best_teb_.get()  // Always preserve the "best" teb
        && (random_() <= cfg_->hcp.selection_dropping_probability * random_.max()))
    {
      it_teb = tebs_.erase(it_teb);
      it_eqrel = equivalence_classes_.erase(it_eqrel);
    }
    else
    {
      ++it_teb;
      ++it_eqrel;
    }
  }
}

TebOptimalPlannerPtr HomotopyClassPlanner::selectBestTeb()
{
    double min_cost = std::numeric_limits<double>::max(); // maximum cost
    double min_cost_last_best = std::numeric_limits<double>::max();
    double min_cost_initial_plan_teb = std::numeric_limits<double>::max();
    TebOptimalPlannerPtr initial_plan_teb = getInitialPlanTEB();

    // check if last best_teb is still a valid candidate
    if (best_teb_ && std::find(tebs_.begin(), tebs_.end(), best_teb_) != tebs_.end())
    {
        // get cost of this candidate
        // 默认selection_cost_hysteresis=1.0
        min_cost_last_best = best_teb_->getCurrentCost() * cfg_->hcp.selection_cost_hysteresis; // small hysteresis
        last_best_teb_ = best_teb_;
    }
    else
    {
      last_best_teb_.reset();
    }

    // 上一时刻已经被选择的轨迹，具有相对更小的cost（保证连贯性）
    if (initial_plan_teb) // the validity was already checked in getInitialPlanTEB()
    {
        // get cost of this candidate
        // 默认selection_prefer_initial_plan=0.9
        min_cost_initial_plan_teb = initial_plan_teb->getCurrentCost() * cfg_->hcp.selection_prefer_initial_plan; // small hysteresis
    }


    best_teb_.reset(); // reset pointer

    for (TebOptPlannerContainer::iterator it_teb = tebs_.begin(); it_teb != tebs_.end(); ++it_teb)
    {
        // check if the related TEB leaves the local costmap region
//      if (tebs_.size()>1 && !(*it_teb)->teb().isTrajectoryInsideRegion(20, -1, 1))
//      {
//          ROS_INFO("HomotopyClassPlanner::selectBestTeb(): skipping trajectories that are not inside the local costmap");
//          continue;
//      }

        double teb_cost;
        std::cout<<it_teb->get()->getCurrentCost()<<std::endl;

        if (*it_teb == last_best_teb_)
            teb_cost = min_cost_last_best; // skip already known cost value of the last best_teb
        else if (*it_teb == initial_plan_teb)
            teb_cost = min_cost_initial_plan_teb;
        else
            teb_cost = it_teb->get()->getCurrentCost();

        if (teb_cost < min_cost)
        {
          // check if this candidate is currently not selected
          best_teb_ = *it_teb;
          min_cost = teb_cost;
        }
     }
     std::cout<<"min_cost: "<<min_cost<<std::endl;

    min_cost = std::numeric_limits<double>::max();
    for (TebOptPlannerContainer::iterator it_teb = tebs_.begin(); it_teb != tebs_.end(); ++it_teb)
    {
        double teb_cost = it_teb->get()->getCurrentCostOrigin();

        if (teb_cost < min_cost)
        {
          // check if this candidate is currently not selected
          best_teb_calculated_by_origin_ = *it_teb;
          min_cost = teb_cost;
        }
     }


    // check if we are allowed to change
    // switching_blocking_period（默认为0.0），这个参数要求 每次switch轨迹时要有一定的时间间隔，防止机器人抖动
    if (last_best_teb_ && best_teb_ != last_best_teb_)
    {
      ros::Time now = ros::Time::now();
      if ((now-last_eq_class_switching_time_).toSec() > cfg_->hcp.switching_blocking_period)
      {
        last_eq_class_switching_time_ = now;
      }
      else
      {
        ROS_DEBUG("HomotopyClassPlanner::selectBestTeb(): Switching equivalence classes blocked (check parameter switching_blocking_period.");
        // block switching, so revert best_teb_
        best_teb_ = last_best_teb_;
      }

    }


    return best_teb_;
}

int HomotopyClassPlanner::bestTebIdx() const
{
  if (tebs_.size() == 1)
    return 0;

  if (!best_teb_)
    return -1;

  int idx = 0;
  for (TebOptPlannerContainer::const_iterator it_teb = tebs_.begin(); it_teb != tebs_.end(); ++it_teb, ++idx)
  {
    if (*it_teb == best_teb_)
      return idx;
  }
  return -1;
}

bool HomotopyClassPlanner::isTrajectoryFeasible(base_local_planner::CostmapModel* costmap_model, const std::vector<geometry_msgs::Point>& footprint_spec,
                                                double inscribed_radius, double circumscribed_radius, int look_ahead_idx)
{
  bool feasible = false;
  while(ros::ok() && !feasible && tebs_.size() > 0)
  {
    TebOptimalPlannerPtr best = findBestTeb();
    if (!best)
    {
      ROS_ERROR("Couldn't retrieve the best plan");
      return false;
    }
    feasible = best->isTrajectoryFeasible(costmap_model, footprint_spec, inscribed_radius, circumscribed_radius, look_ahead_idx);
    if(!feasible)
    {
      removeTeb(best);
      if(last_best_teb_ && (last_best_teb_ == best)) // Same plan as before.
        return feasible;                             // Not failing could result in oscillations between trajectories.
    }
  }
  return feasible;
}

TebOptimalPlannerPtr HomotopyClassPlanner::findBestTeb()
{
  if(tebs_.empty())
    return TebOptimalPlannerPtr();
  if (!best_teb_ || std::find(tebs_.begin(), tebs_.end(), best_teb_) == tebs_.end())
    best_teb_ = selectBestTeb();
  return best_teb_;
}

TebOptPlannerContainer::iterator HomotopyClassPlanner::removeTeb(TebOptimalPlannerPtr& teb)
{
  TebOptPlannerContainer::iterator return_iterator = tebs_.end();
  if(equivalence_classes_.size() != tebs_.size())
  {
      ROS_ERROR("removeTeb: size of eq classes != size of tebs");
      return return_iterator;
  }
  auto it_eq_classes = equivalence_classes_.begin();
  for(auto it = tebs_.begin(); it != tebs_.end(); ++it)
  {
    if(*it == teb)
    {
      return_iterator = tebs_.erase(it);
      equivalence_classes_.erase(it_eq_classes);
      break;
    }
    ++it_eq_classes;
  }
  return return_iterator;
}

void HomotopyClassPlanner::setPreferredTurningDir(RotType dir)
{
  // set preferred turning dir for all TEBs
  for (TebOptPlannerContainer::const_iterator it_teb = tebs_.begin(); it_teb != tebs_.end(); ++it_teb)
  {
    (*it_teb)->setPreferredTurningDir(dir);
  }
}

bool HomotopyClassPlanner::hasDiverged() const
{
  // Early return if there is no best trajectory initialized
  if (!best_teb_)
    return false;

  return best_teb_->hasDiverged();
}

void HomotopyClassPlanner::computeCurrentCost(std::vector<double>& cost, double obst_cost_scale, double viapoint_cost_scale, bool alternative_time_cost)
{
  for (TebOptPlannerContainer::iterator it_teb = tebs_.begin(); it_teb != tebs_.end(); ++it_teb)
  {
    it_teb->get()->computeCurrentCost(cost, obst_cost_scale, viapoint_cost_scale, alternative_time_cost);
  }
}

void HomotopyClassPlanner::deletePlansDetouringBackwards(const double orient_threshold,
  const double len_orientation_vector)
{
  if (tebs_.size() < 2 || !best_teb_ || std::find(tebs_.begin(), tebs_.end(), best_teb_) == tebs_.end() ||
    best_teb_->teb().sizePoses() < 2)
  {
    return;  // A moving direction wasn't chosen yet
  }
  double current_movement_orientation;
  double best_plan_duration = std::max(best_teb_->teb().getSumOfAllTimeDiffs(), 1.0);
  if(!computeStartOrientation(best_teb_, len_orientation_vector, current_movement_orientation))
    return;  // The plan is shorter than len_orientation_vector
  for(auto it_teb = tebs_.begin(); it_teb != tebs_.end();)
  {
    if(*it_teb == best_teb_)  // The current plan should not be considered a detour
    {
      ++it_teb;
      continue;
    }
    if((*it_teb)->teb().sizePoses() < 2)
    {
      ROS_DEBUG("Discarding a plan with less than 2 poses");
      it_teb = removeTeb(*it_teb);
      continue;
    }
    double plan_orientation;
    if(!computeStartOrientation(*it_teb, len_orientation_vector, plan_orientation))
    {
      ROS_DEBUG("Failed to compute the start orientation for one of the tebs, likely close to the target");
      it_teb = removeTeb(*it_teb);
      continue;
    }
    if(fabs(g2o::normalize_theta(plan_orientation - current_movement_orientation)) > orient_threshold)
    {
      it_teb = removeTeb(*it_teb);  // Plan detouring backwards
      continue;
    }
    if(!it_teb->get()->isOptimized())
    {
      ROS_DEBUG("Removing a teb because it's not optimized");
      it_teb = removeTeb(*it_teb);  // Deletes tebs that cannot be optimized (last optim call failed)
      continue;
    }
    if(it_teb->get()->teb().getSumOfAllTimeDiffs() / best_plan_duration > cfg_->hcp.max_ratio_detours_duration_best_duration)
    {
      ROS_DEBUG("Removing a teb because it's duration is much longer than that of the best teb");
      it_teb = removeTeb(*it_teb);
      continue;
    }
    ++it_teb;
  }
}

bool HomotopyClassPlanner::computeStartOrientation(const TebOptimalPlannerPtr plan, const double len_orientation_vector,
  double& orientation)
{
  VertexPose start_pose = plan->teb().Pose(0);
  bool second_pose_found = false;
  Eigen::Vector2d start_vector;
  for(auto& pose : plan->teb().poses())
  {
    start_vector = start_pose.position() - pose->position();
    if(start_vector.norm() > len_orientation_vector)
    {
      second_pose_found = true;
      break;
    }
  }
  if(!second_pose_found)  // The current plan is too short to make assumptions on the start orientation
    return false;
  orientation = std::atan2(start_vector[1], start_vector[0]);
  return true;
}


} // end namespace
