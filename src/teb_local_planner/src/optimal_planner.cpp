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

#include <teb_local_planner/optimal_planner.h>

// g2o custom edges and vertices for the TEB planner
#include <teb_local_planner/g2o_types/edge_velocity.h>
#include <teb_local_planner/g2o_types/edge_velocity_obstacle_ratio.h>
#include <teb_local_planner/g2o_types/edge_acceleration.h>
#include <teb_local_planner/g2o_types/edge_kinematics.h>
#include <teb_local_planner/g2o_types/edge_time_optimal.h>
#include <teb_local_planner/g2o_types/edge_shortest_path.h>
#include <teb_local_planner/g2o_types/edge_obstacle.h>
#include <teb_local_planner/g2o_types/edge_dynamic_obstacle.h>
#include <teb_local_planner/g2o_types/edge_goal_line.h>
#include <teb_local_planner/g2o_types/edge_prefer_rotdir.h>

#include <memory>
#include <limits>


namespace teb_local_planner
{

// ============== Implementation ===================

TebOptimalPlanner::TebOptimalPlanner() : cfg_(NULL), obstacles_(NULL), via_points_(NULL), cost_(HUGE_VAL), prefer_rotdir_(RotType::none),
                                         robot_model_(new PointRobotFootprint()), initialized_(false), optimized_(false)
{    
}
  
TebOptimalPlanner::TebOptimalPlanner(const TebConfig& cfg, ObstContainer* obstacles, RobotFootprintModelPtr robot_model, TebVisualizationPtr visual, const ViaPointContainer* via_points)
{    
  initialize(cfg, obstacles, robot_model, visual, via_points);
}

TebOptimalPlanner::~TebOptimalPlanner()
{
  clearGraph();
  // free dynamically allocated memory
  //if (optimizer_) 
  //  g2o::Factory::destroy();
  //g2o::OptimizationAlgorithmFactory::destroy();
  //g2o::HyperGraphActionLibrary::destroy();
}

void TebOptimalPlanner::updateRobotModel(RobotFootprintModelPtr robot_model)
{
  robot_model_ = robot_model;
}

void TebOptimalPlanner::initialize(const TebConfig& cfg, ObstContainer* obstacles, RobotFootprintModelPtr robot_model, TebVisualizationPtr visual, const ViaPointContainer* via_points)
{    
  // init optimizer (set solver and block ordering settings)
  optimizer_ = initOptimizer();
  
  cfg_ = &cfg;
  obstacles_ = obstacles;
  robot_model_ = robot_model;
  via_points_ = via_points;
  cost_ = HUGE_VAL;
  prefer_rotdir_ = RotType::none;
  setVisualization(visual);
  
  vel_start_.first = true;
  vel_start_.second.linear.x = 0;
  vel_start_.second.linear.y = 0;
  vel_start_.second.angular.z = 0;

  vel_goal_.first = true;
  vel_goal_.second.linear.x = 0;
  vel_goal_.second.linear.y = 0;
  vel_goal_.second.angular.z = 0;
  initialized_ = true;
  endpoint_appended_ = true;
}


void TebOptimalPlanner::setVisualization(TebVisualizationPtr visualization)
{
  visualization_ = visualization;
}

void TebOptimalPlanner::visualize()
{
  if (!visualization_)
    return;
 
  visualization_->publishLocalPlanAndPoses(teb_);
  
  if (teb_.sizePoses() > 0)
    visualization_->publishRobotFootprintModel(teb_.Pose(0), *robot_model_);
  
  if (cfg_->trajectory.publish_feedback)
    visualization_->publishFeedbackMessage(*this, *obstacles_);
 
}


/*
 * registers custom vertices and edges in g2o framework
 */
void TebOptimalPlanner::registerG2OTypes()
{
  g2o::Factory* factory = g2o::Factory::instance();
  factory->registerType("VERTEX_POSE", new g2o::HyperGraphElementCreator<VertexPose>);
  factory->registerType("VERTEX_TIMEDIFF", new g2o::HyperGraphElementCreator<VertexTimeDiff>);

  factory->registerType("EDGE_TIME_OPTIMAL", new g2o::HyperGraphElementCreator<EdgeTimeOptimal>);
  factory->registerType("EDGE_SHORTEST_PATH", new g2o::HyperGraphElementCreator<EdgeShortestPath>);
  factory->registerType("EDGE_VELOCITY", new g2o::HyperGraphElementCreator<EdgeVelocity>);
  factory->registerType("EDGE_VELOCITY_HOLONOMIC", new g2o::HyperGraphElementCreator<EdgeVelocityHolonomic>);
  factory->registerType("EDGE_ACCELERATION", new g2o::HyperGraphElementCreator<EdgeAcceleration>);
  factory->registerType("EDGE_ACCELERATION_START", new g2o::HyperGraphElementCreator<EdgeAccelerationStart>);
  factory->registerType("EDGE_ACCELERATION_GOAL", new g2o::HyperGraphElementCreator<EdgeAccelerationGoal>);
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC", new g2o::HyperGraphElementCreator<EdgeAccelerationHolonomic>);
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC_START", new g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicStart>);
  factory->registerType("EDGE_ACCELERATION_HOLONOMIC_GOAL", new g2o::HyperGraphElementCreator<EdgeAccelerationHolonomicGoal>);
  factory->registerType("EDGE_KINEMATICS_DIFF_DRIVE", new g2o::HyperGraphElementCreator<EdgeKinematicsDiffDrive>);
  factory->registerType("EDGE_KINEMATICS_CARLIKE", new g2o::HyperGraphElementCreator<EdgeKinematicsCarlike>);
  factory->registerType("EDGE_INFLATED_OBSTACLE", new g2o::HyperGraphElementCreator<EdgeInflatedObstacle>);
  factory->registerType("EDGE_DYNAMIC_OBSTACLE", new g2o::HyperGraphElementCreator<EdgeDynamicObstacle>);
  factory->registerType("EDGE_GOAL_LINE", new g2o::HyperGraphElementCreator<GoalLine>);
  factory->registerType("EDGE_PREFER_ROTDIR", new g2o::HyperGraphElementCreator<EdgePreferRotDir>);
  return;
}

/*
 * initialize g2o optimizer. Set solver settings here.
 * Return: pointer to new SparseOptimizer Object.
 */
boost::shared_ptr<g2o::SparseOptimizer> TebOptimalPlanner::initOptimizer()
{
  // Call register_g2o_types once, even for multiple TebOptimalPlanner instances (thread-safe)
  static boost::once_flag flag = BOOST_ONCE_INIT;
  boost::call_once(&registerG2OTypes, flag);  

  // allocating the optimizer
  boost::shared_ptr<g2o::SparseOptimizer> optimizer = boost::make_shared<g2o::SparseOptimizer>();
  std::unique_ptr<TEBLinearSolver> linear_solver(new TEBLinearSolver()); // see typedef in optimization.h
  linear_solver->setBlockOrdering(true);
  std::unique_ptr<TEBBlockSolver> block_solver(new TEBBlockSolver(std::move(linear_solver)));
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

  optimizer->setAlgorithm(solver);
  
  optimizer->initMultiThreading(); // required for >Eigen 3.1
  
  return optimizer;
}

// 优化器的入口
bool TebOptimalPlanner::optimizeTEB(int iterations_innerloop, int iterations_outerloop, bool compute_cost_afterwards,
                                    double obst_cost_scale, double goal_line_cost_scale, bool alternative_time_cost)
{
  if (cfg_->optim.optimization_activate==false) 
    return false;
  
  bool success = false;
  optimized_ = false;
  
  double weight_multiplier = 1.0;

  // TODO(roesmann): we introduced the non-fast mode with the support of dynamic obstacles
  //                (which leads to better results in terms of x-y-t homotopy planning).
  //                 however, we have not tested this mode intensively yet, so we keep
  //                 the legacy fast mode as default until we finish our tests.
  bool fast_mode = !cfg_->obstacles.include_dynamic_obstacles;

  // 初始化对应轨迹和每个动态障碍物（人）的关系
  detour_dynamic_obstacle_method_.clear(); // 区分轨迹和各个人的关系。true表示从人前方绕行。长度应等于动态障碍物数量
  dynamic_obstacle_old_alpha_.clear();        // 记录每个人的alpha值
  dynamic_obstacle_current_alpha_.clear();    // 记录每个人的alpha值
  teb_within_optimization_process_.clear();
  if (cfg_->hcp.visualize_graphic_optimizatoin_process){
    std::vector<Eigen::Vector2d> current_teb;
    for(int i=0; i<teb_.sizePoses(); ++i)
      current_teb.push_back(teb_.Pose(i).position());
    teb_within_optimization_process_.push_back(current_teb);
  }
  teb_.autoResize(cfg_->trajectory.dt_ref, cfg_->trajectory.dt_hysteresis, cfg_->trajectory.min_samples, cfg_->trajectory.max_samples, fast_mode);
  // std::cout<<"== "<<teb_.sizePoses()<<std::endl;
  if (cfg_->hcp.visualize_graphic_optimizatoin_process){
    std::vector<Eigen::Vector2d> current_teb;
    for(int i=0; i<teb_.sizePoses(); ++i)
      current_teb.push_back(teb_.Pose(i).position());
    teb_within_optimization_process_.push_back(current_teb);
  }

  // 根据相邻点的时间间隔（距离），对路径进行remove和insert
  for (ObstContainer::const_iterator obst = obstacles_->begin(); obst != obstacles_->end(); ++obst)
  {
    if (!(*obst)->isDynamic())
      continue;

    // 计算 以obst的中心指向其速度方向的向量 和 以obst的中心指向teb_包含的点的数值
    double max_cos_angle = -1.1;
    double len_of_the_connection = 1.0;
    Eigen::Vector2d obs_vel = (*obst)->getCentroidVelocity();
    for (int i=0; i<teb_.sizePoses(); i++)
    {
      // 计算obst和teb_的相对位置
      Eigen::Vector2d obs2traj = teb_.Pose(i).position() - (*obst)->getCentroid();
      // 计算两个向量的cos值
      double cos_angle = obs2traj.dot(obs_vel) / (obs2traj.norm() * obs_vel.norm());
      if (cos_angle>max_cos_angle){
        max_cos_angle = cos_angle;
        len_of_the_connection = obs2traj.norm();
      }
    }
    // 如果是从人前方绕行，则max_cos_angle应该为1。但考虑到轨迹点的连续性，此处使用轨迹点到人速度的距离<点间最大时间 * v
    // std::cout<<"== ("<<teb_.sizePoses()<<"): ("<<max_cos_angle<<"): ("<<len_of_the_connection*sqrt(1-pow(max_cos_angle, 2))<<","<<cfg_->trajectory.dt_ref*0.5<<"): ";
    if (max_cos_angle > 0 && len_of_the_connection*sqrt(1-pow(max_cos_angle, 2)) <= cfg_->trajectory.dt_ref*cfg_->robot.max_vel_x){
     detour_dynamic_obstacle_method_.push_back(true);
      dynamic_obstacle_old_alpha_.push_back(0);
      dynamic_obstacle_current_alpha_.push_back(0);
    }
    else{
      detour_dynamic_obstacle_method_.push_back(false);
      dynamic_obstacle_old_alpha_.push_back(1);
      dynamic_obstacle_current_alpha_.push_back(1);
    }
    // std::cout<<detour_dynamic_obstacle_method_.back()<<std::endl;
  }

  // 外部循环优化iteractions_outerloop次
  int outer_loop_count = 0;
  int all_alpha_complete_count = 999;
  double dt_build=0, dt_optimize=0;
  while(true)
  {
    optimized_ = false;
    outer_loop_count ++;
    if (cfg_->trajectory.teb_autosize)
    {
      // 根据相邻点的时间间隔（距离），对路径进行remove和insert
      teb_.autoResize(cfg_->trajectory.dt_ref, cfg_->trajectory.dt_hysteresis, cfg_->trajectory.min_samples, cfg_->trajectory.max_samples, fast_mode);
      if (cfg_->hcp.visualize_graphic_optimizatoin_process){
        std::vector<Eigen::Vector2d> current_teb;
        for(int j=0; j<teb_.sizePoses(); ++j)
          current_teb.push_back(teb_.Pose(j).position());
        teb_within_optimization_process_.push_back(current_teb);      
      }
    }
    // ROS_INFO("length: [%d], [%f]", teb_.sizePoses(), teb_.getAccumulatedDistance());
    auto old_length = teb_.getAccumulatedDistance();
    // 增大alpha
    int obs_idx = 0;
    for (ObstContainer::const_iterator obst = obstacles_->begin(); obst != obstacles_->end(); ++obst)
    {
      if (!(*obst)->isDynamic())
        continue;
      // 保存old alpha
      if (detour_dynamic_obstacle_method_[obs_idx]==true){
        dynamic_obstacle_old_alpha_[obs_idx] = dynamic_obstacle_current_alpha_[obs_idx];
        // 更新新alpha，步长是 0.1
        double tim_of_nearest_waypoint=0;
        double nearest_dis = 99999999.9;
        CircularObstacle* temp_obst = dynamic_cast<CircularObstacle*>(const_cast<Obstacle*>(obst->get()));
        // double increament_dis = 0.05;
        double increament_dis = 0.1;
        for (int i=1; i < teb_.sizePoses() - 1; ++i){
          double tim = teb_.getSumOfTimeDiffsUpToIdx(i);
          Eigen::Vector2d pos = temp_obst->getCentroid() + temp_obst->getCentroidVelocity()*tim*dynamic_obstacle_old_alpha_[obs_idx];
          double dis = (teb_.PoseVertex(i)->position() - pos).norm();
          if (dis < nearest_dis){
            nearest_dis = dis;
            tim_of_nearest_waypoint = tim;
          }
        }
        double dv = increament_dis / tim_of_nearest_waypoint;
        double d_alpha = dv / temp_obst->getCentroidVelocity().norm();
        dynamic_obstacle_current_alpha_[obs_idx] = dynamic_obstacle_current_alpha_[obs_idx] + d_alpha;
        // std::cout<<dynamic_obstacle_current_alpha_[obs_idx]<<std::endl;
        dynamic_obstacle_current_alpha_[obs_idx] = std::min(1.0, dynamic_obstacle_current_alpha_[obs_idx]);
        // ROS_INFO("iterationIndex: %d, obsID: %d, d_alpha: %f, alpha: [%f/%f]", outer_loop_count, obs_idx, d_alpha, dynamic_obstacle_current_alpha_[obs_idx], 1.0);
      }
      obs_idx++;
    }
    auto t1 = ros::Time::now();
    // auto t = ros::Time::now();
    success = buildGraph(dynamic_obstacle_current_alpha_, dynamic_obstacle_old_alpha_, detour_dynamic_obstacle_method_, weight_multiplier);
    // ROS_INFO("-time to build graph: [%f]", (ros::Time::now() - t).toSec());
    if (!success) 
    {
        clearGraph();
        return false;
    }
    auto t2 = ros::Time::now();
    dt_build += (t2-t1).toSec();
    // t = ros::Time::now();
    success = optimizeGraph(iterations_innerloop, false);
    // ROS_INFO("+time to optimize graph: [%f]", (ros::Time::now() - t).toSec());
    if (!success) 
    {
        clearGraph();
        return false;
    }
    auto t3 = ros::Time::now();
    dt_optimize += (t3-t2).toSec();
    // 提前结束optimization的条件：轨迹长度基本不发生变化
    bool early_stop = false;
    auto new_length = teb_.getAccumulatedDistance();
    // // ROS_INFO("[%d] current path length: [%f], [%f]", i, old_length, new_length);
    if (obs_idx==0 && fabs(new_length - old_length) < cfg_->hcp.epsilon_for_early_stop){
      // ROS_INFO("early stop convergence: [%d]/[%d]", outer_loop_count, iterations_outerloop);
      early_stop = true;
    }
    // 提前结束optimization的条件：所有人的alpha均达到1
    if (all_alpha_complete_count==999){
      bool all_alpha_complete = true;
      for(auto p:dynamic_obstacle_current_alpha_){
        if (p!=1.0){
          all_alpha_complete = false;
          break;
        }
      }
      if (all_alpha_complete)
        all_alpha_complete_count = outer_loop_count;
    }
    // 迭代结束的有两个情景
    // Case1（错误退出）： 轨迹超出了最大距离
    if (teb_.getAccumulatedDistance() > cfg_->hcp.length_limition_during_optimization){
      ROS_INFO("        Outer_loop fail at: %d because it is too long", outer_loop_count);
      optimized_ = false;
      return false;
    }
    // Case2（错误退出）： 超出了最大迭代次数
    if (outer_loop_count>=cfg_->optim.no_outer_iterations){
      ROS_INFO("        Outer_loop fail at: %d because it reaches the max iteration time", outer_loop_count);
      optimized_ = false;
      return false;
    }    
    // Case3（成功退出）： 迭代次数超过最小次数 且 所有人的alpha都到1
    // if(outer_loop_count>cfg_->optim.least_no_outer_iterations &&( outer_loop_count > all_alpha_complete_count+20)){
    if(outer_loop_count > all_alpha_complete_count+cfg_->optim.least_no_outer_iterations){
      if (compute_cost_afterwards){
        computeCurrentCost(obst_cost_scale, goal_line_cost_scale, alternative_time_cost);
      }
      ROS_INFO("        Outer_loop success at: %d normally", outer_loop_count);
      clearGraph();
      optimized_ = true;
      // ROS_INFO("Time for build graph: [%f] s; Time for optimize graph: [%f]", dt_build, dt_optimize);
      return true;
    }
    // Case4 (成功退出)： 相邻optimization间的轨迹变化很小 且 （无动态障碍物 或 所有的alpha都到1了）
    if (early_stop && (obs_idx==0 || outer_loop_count>all_alpha_complete_count+cfg_->optim.least_no_outer_iterations)){
      if (compute_cost_afterwards){
        computeCurrentCost(obst_cost_scale, goal_line_cost_scale, alternative_time_cost);
      }
      ROS_INFO("        Outer_loop success at: [%d] because of early stop convergence", outer_loop_count);
      clearGraph();
      optimized_ = true;
      return true;
    }

    clearGraph();
    weight_multiplier *= cfg_->optim.weight_adapt_factor; //yaml里默认为2
  }
  return true;
}

void TebOptimalPlanner::setVelocityStart(const geometry_msgs::Twist& vel_start)
{
  vel_start_.first = true;
  vel_start_.second.linear.x = vel_start.linear.x;
  vel_start_.second.linear.y = vel_start.linear.y;
  vel_start_.second.angular.z = vel_start.angular.z;
}

void TebOptimalPlanner::setVelocityGoal(const geometry_msgs::Twist& vel_goal)
{
  vel_goal_.first = true;
  vel_goal_.second = vel_goal;
}

bool TebOptimalPlanner::plan(const std::vector<geometry_msgs::PoseStamped>& initial_plan, const geometry_msgs::Twist* start_vel, bool free_goal_vel, std::pair<double,double> global_goal)
{    
  return false;
}


bool TebOptimalPlanner::plan(const tf::Pose& start, const tf::Pose& goal, const geometry_msgs::Twist* start_vel, bool free_goal_vel, std::pair<double,double> global_goal)
{
  return false;
}

bool TebOptimalPlanner::plan(const PoseSE2& start, const PoseSE2& goal, const geometry_msgs::Twist* start_vel, bool free_goal_vel, std::pair<double,double> global_goal)
{	
  return false;
}


bool TebOptimalPlanner::buildGraph(std::vector<double>& dynamic_obstacle_current_alpha, std::vector<double>& dynamic_obstacle_old_alpha, const std::vector<bool>& detour_dynamic_obstacle_method, double weight_multiplier)
{
  // auto t=ros::Time::now();
  // auto t_start=ros::Time::now();
  if (!optimizer_->edges().empty() || !optimizer_->vertices().empty())
  {
    ROS_WARN("Cannot build graph, because it is not empty. Call graphClear()!");
    return false;
  }

  optimizer_->setComputeBatchStatistics(cfg_->recovery.divergence_detection_enable);
  // ROS_INFO("1 Time to build: %f", (ros::Time::now()-t).toSec());
  // t=ros::Time::now();
  // add TEB vertices。即把所有的点和dt添加到优化器中
  AddTEBVertices();
  // ROS_INFO("2 Time to build: %f", (ros::Time::now()-t).toSec());
  // t=ros::Time::now();  
  // add Edges (local cost functions)
  AddEdgesObstacles(weight_multiplier);
  // ROS_INFO("3 Time to build: %f", (ros::Time::now()-t).toSec());
  // t=ros::Time::now();  
  // 对动态障碍物，添加time戳，在xyt空间下，teb考虑同一time时的避障
  AddEdgesDynamicObstacles(dynamic_obstacle_current_alpha, dynamic_obstacle_old_alpha, detour_dynamic_obstacle_method, 1.0);
  // ROS_INFO("4 Time to build: %f", (ros::Time::now()-t).toSec());
  // t=ros::Time::now();
  AddEdgesGoalLine();

  AddEdgesVelocity();
  
  AddEdgesAcceleration();

  AddEdgesTimeOptimal();	

  AddEdgesShortestPath();
  if (cfg_->robot.min_turning_radius == 0 || cfg_->optim.weight_kinematics_turning_radius == 0)
    AddEdgesKinematicsDiffDrive(); // we have a differential drive robot
  else
    AddEdgesKinematicsCarlike(); // we have a carlike robot since the turning radius is bounded from below.

  AddEdgesPreferRotDir();

  if (cfg_->optim.weight_velocity_obstacle_ratio > 0)
    AddEdgesVelocityObstacleRatio();
  // ROS_INFO("Time to build: %f", (ros::Time::now()-t_start).toSec());
  return true;  
}

bool TebOptimalPlanner::optimizeGraph(int no_iterations,bool clear_after)
{
  if (cfg_->robot.max_vel_x<0.01)
  {
    ROS_WARN("optimizeGraph(): Robot Max Velocity is smaller than 0.01m/s. Optimizing aborted...");
    if (clear_after) clearGraph();
    return false;	
  }
  
  if (!teb_.isInit() || teb_.sizePoses() < cfg_->trajectory.min_samples)
  {
    ROS_WARN("optimizeGraph(): TEB is empty or has too less elements. Skipping optimization.");
    if (clear_after) clearGraph();
    return false;	
  }
  optimizer_->setVerbose(cfg_->optim.optimization_verbose);
  optimizer_->initializeOptimization();
  int iter = optimizer_->optimize(no_iterations);
  // Save Hessian for visualization
  //  g2o::OptimizationAlgorithmLevenberg* lm = dynamic_cast<g2o::OptimizationAlgorithmLevenberg*> (optimizer_->solver());
  //  lm->solver()->saveHessian("~/MasterThesis/Matlab/Hessian.txt");

  if(!iter)
  {
	ROS_ERROR("optimizeGraph(): Optimization failed! iter=%i", iter);
	return false;
  }

  if (clear_after) clearGraph();	
    
  return true;
}

void TebOptimalPlanner::clearGraph()
{
  // clear optimizer states
  if (optimizer_)
  {
    // we will delete all edges but keep the vertices.
    // before doing so, we will delete the link from the vertices to the edges.
    auto& vertices = optimizer_->vertices();
    for(auto& v : vertices)
      v.second->edges().clear();

    optimizer_->vertices().clear();  // necessary, because optimizer->clear deletes pointer-targets (therefore it deletes TEB states!)
    optimizer_->clear();
  }
}



void TebOptimalPlanner::AddTEBVertices()
{
  // add vertices to graph
  ROS_DEBUG_COND(cfg_->optim.optimization_verbose, "Adding TEB vertices ...");
  unsigned int id_counter = 0; // used for vertices ids
  obstacles_per_vertex_.resize(teb_.sizePoses());
  auto iter_obstacle = obstacles_per_vertex_.begin();
  for (int i=0; i<teb_.sizePoses(); ++i)
  {
    teb_.PoseVertex(i)->setId(id_counter++);
    optimizer_->addVertex(teb_.PoseVertex(i));  //poseVertex是获得index为i对应的pose点，在initTraj时被建立
    if (teb_.sizeTimeDiffs()!=0 && i<teb_.sizeTimeDiffs())
    {
      teb_.TimeDiffVertex(i)->setId(id_counter++);
      optimizer_->addVertex(teb_.TimeDiffVertex(i));  //timeDiffVertex是获得index为i对应的dt点，在initTraj时被建立
    }
    iter_obstacle->clear();
    (iter_obstacle++)->reserve(obstacles_->size());
  }
}

// 先遍历teb_中的所有路径点；对每个路径点，再遍历所有的障碍物点，找到离该路径点最近的左侧和右侧的障碍物
void TebOptimalPlanner::AddEdgesObstacles(double weight_multiplier)
{
  if (cfg_->optim.weight_obstacle==0 || weight_multiplier==0 || obstacles_==nullptr )
    return; // if weight equals zero skip adding edges!
    
  
  bool inflated = cfg_->obstacles.inflation_dist > cfg_->obstacles.min_obstacle_dist;   // default: 0.4>0.3

  Eigen::Matrix<double,1,1> information;
  information.fill(cfg_->optim.weight_obstacle * weight_multiplier);
  
  Eigen::Matrix<double,2,2> information_inflated;
  information_inflated(0,0) = cfg_->optim.weight_obstacle * weight_multiplier;  // 默认：100x1
  information_inflated(1,1) = cfg_->optim.weight_inflation; // 默认0.2
  information_inflated(0,1) = information_inflated(1,0) = 0;

  // 这里定义了一个临时函数，默认已用的谁是[ ]里面，外界要给的是()里的
  auto create_edge_Graphic = [inflated, &information, &information_inflated, this] (int index, const Obstacle* obstacle, costmap_2d::Costmap2D* costmap2d, const std::vector<std::vector<int>> & obs_map_labeled, const double& safey_margin) {
    EdgeInflatedObstacle* dist_bandpt_obst = new EdgeInflatedObstacle;
    dist_bandpt_obst->setCostmap(costmap2d_);
    dist_bandpt_obst->setObsMapLabeled(obs_map_labeled_);
    dist_bandpt_obst->setVertex(0,teb_.PoseVertex(index));
    dist_bandpt_obst->setInformation(information_inflated);
    dist_bandpt_obst->setParameters(*cfg_, robot_model_.get(), obstacle, safey_margin);
    optimizer_->addEdge(dist_bandpt_obst);
  };
  
  int width = obs_map_labeled_.size();
  int height = obs_map_labeled_.front().size();
  int half_dis = int(cfg_->obstacles.min_obstacle_dist*3/costmap2d_->getResolution());

  // 遍历所有路径点
  for (int i = 1; i < teb_.sizePoses() - 1; ++i)
  {    
    // 把teb_.Pose(i)转化到map坐标下：
    uint mx, my;
    costmap2d_->worldToMap(teb_.Pose(i).position().x(), teb_.Pose(i).position().y(), mx, my);
    std::vector<int> index_list;
    // 考虑(mx,my)周围half_dis距离的地图点
    for (int x = mx-half_dis; x < mx+half_dis; ++x){
      for (int y = my-half_dis; y < my+half_dis; ++y){
        if(x<0 || x>=width || y<0 || y>=height) continue;
        if(obs_map_labeled_[x][y] != 0  &&  std::find(index_list.begin(), index_list.end(), obs_map_labeled_[x][y]) == index_list.end()){
          index_list.push_back(obs_map_labeled_[x][y]);
        }
      }
    }

    for (auto idx:index_list){
      for (const ObstaclePtr obst : *obstacles_){ 
        if(obst->getIndex()==idx){
          if (safety_margins_.find(idx) != safety_margins_.end()){
            create_edge_Graphic(i, obst.get(),costmap2d_,obs_map_labeled_, safety_margins_[idx]);
          }
          else
            create_edge_Graphic(i, obst.get(),costmap2d_,obs_map_labeled_, 0);
        }
      }
    }
  }
}

// 动态障碍物就没有静态那么麻烦了，不用酸左右或者是不是最近了，统统添加就好了。。
// 这里也体现了一个点是，只要把障碍物设置成dynamic，那么teb会在xyt空间，针对同一个time，进行距离的计算了。。
void TebOptimalPlanner::AddEdgesDynamicObstacles(std::vector<double>& dynamic_obstacle_current_alpha, std::vector<double>& dynamic_obstacle_old_alpha, const std::vector<bool>& detour_dynamic_obstacle_method, double weight_multiplier)
{
  if (cfg_->optim.weight_dynamic_obstacle==0 && cfg_->optim.weight_dynamic_obstacle_inflation==0 || weight_multiplier==0 || obstacles_==NULL )
    return; // if weight equals zero skip adding edges!

  Eigen::Matrix<double,2,2> information;
  information(0,0) = cfg_->optim.weight_dynamic_obstacle * weight_multiplier;
  information(1,1) = cfg_->optim.weight_dynamic_obstacle_inflation;
  information(0,1) = information(1,0) = 0;
  
  int dynamic_obstacle_index = -1;
  for (ObstContainer::const_iterator obst = obstacles_->begin(); obst != obstacles_->end(); ++obst)
  {
    if (!(*obst)->isDynamic())
      continue;
    ++dynamic_obstacle_index;
    // Skip first and last pose, as they are fixed
    double time = teb_.TimeDiff(0);
    for (int i=1; i < teb_.sizePoses() - 1; ++i)
    {
      Eigen::Vector2d obs_position;
      obst->get()->predictCentroidConstantVelocity(time, obs_position);
      if (hypot(teb_.PoseVertex(i)->x()-obs_position.x(), teb_.PoseVertex(i)->y()-obs_position.y()) > 1.5)    // 1.5是机器人半径+人半径+margin
        continue;
      EdgeDynamicObstacle* dynobst_edge = new EdgeDynamicObstacle(time);
      dynobst_edge->setVertex(0,teb_.PoseVertex(i));
      dynobst_edge->setAnchor(teb_.PoseVertex(i)->x(), teb_.PoseVertex(i)->y());
      // std::cout<<"a a a "<<detour_dynamic_obstacle_method[dynamic_obstacle_index]<<", "<<i<<": "<<teb_.PoseVertex(i)->x()<<","<<teb_.PoseVertex(i)->y()<<"; "<<obst->get()->getCentroid().x()<<","<<obst->get()->getCentroid().y()<<std::endl;
      dynobst_edge->setInformation(information);
      if (safety_margins_.find(safety_margins_[obst->get()->getIndex()])!=safety_margins_.end())
        dynobst_edge->setSafetyMargin(safety_margins_[obst->get()->getIndex()]);
      dynobst_edge->setDetourDir(detour_dynamic_obstacle_method[dynamic_obstacle_index]); //设置绕行策略和alpha
      dynobst_edge->setAlpha(dynamic_obstacle_current_alpha[dynamic_obstacle_index], dynamic_obstacle_old_alpha[dynamic_obstacle_index]);
      dynobst_edge->setParameters(*cfg_, robot_model_.get(), obst->get());
      dynobst_edge->setGoal(teb_.PoseVertex(teb_.sizePoses() - 1)->x(), teb_.PoseVertex(teb_.sizePoses() - 1)->y());
      dynobst_edge->complete();
      optimizer_->addEdge(dynobst_edge);
      time += teb_.TimeDiff(i); // we do not need to check the time diff bounds, since we iterate to "< sizePoses()-1".
    }
  }
}

void TebOptimalPlanner::AddEdgesGoalLine()
{
  assert( (cfg_->optim.weight_goal_line!=0) );
  Eigen::Matrix<double,1,1> information;
  information.fill(cfg_->optim.weight_goal_line);    // 默认：1
  
  GoalLine* edge_goal_line = new GoalLine;
  edge_goal_line->setVertex(0,teb_.PoseVertex(teb_.sizePoses()-1));
  edge_goal_line->setInformation(information);
  edge_goal_line->setParameters(*cfg_, goal_endpoints_);
  optimizer_->addEdge(edge_goal_line);   
}

void TebOptimalPlanner::AddEdgesVelocity()
{
  if (cfg_->robot.max_vel_y == 0) // non-holonomic robot
  {
    if ( cfg_->optim.weight_max_vel_x==0 && cfg_->optim.weight_max_vel_theta==0)
      return; // if weight equals zero skip adding edges!

    int n = teb_.sizePoses();
    Eigen::Matrix<double,2,2> information;
    information(0,0) = cfg_->optim.weight_max_vel_x;
    information(1,1) = cfg_->optim.weight_max_vel_theta;
    information(0,1) = 0.0;
    information(1,0) = 0.0;

    for (int i=0; i < n - 1; ++i)
    {
      EdgeVelocity* velocity_edge = new EdgeVelocity;
      velocity_edge->setVertex(0,teb_.PoseVertex(i));
      velocity_edge->setVertex(1,teb_.PoseVertex(i+1));
      velocity_edge->setVertex(2,teb_.TimeDiffVertex(i));
      velocity_edge->setInformation(information);
      velocity_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(velocity_edge);
    }
  }
  else // holonomic-robot
  {
    if ( cfg_->optim.weight_max_vel_x==0 && cfg_->optim.weight_max_vel_y==0 && cfg_->optim.weight_max_vel_theta==0)
      return; // if weight equals zero skip adding edges!
      
    int n = teb_.sizePoses();
    Eigen::Matrix<double,3,3> information;
    information.fill(0);
    information(0,0) = cfg_->optim.weight_max_vel_x;
    information(1,1) = cfg_->optim.weight_max_vel_y;
    information(2,2) = cfg_->optim.weight_max_vel_theta;

    for (int i=0; i < n - 1; ++i)
    {
      EdgeVelocityHolonomic* velocity_edge = new EdgeVelocityHolonomic;
      velocity_edge->setVertex(0,teb_.PoseVertex(i));
      velocity_edge->setVertex(1,teb_.PoseVertex(i+1));
      velocity_edge->setVertex(2,teb_.TimeDiffVertex(i));
      velocity_edge->setInformation(information);
      velocity_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(velocity_edge);
    } 
    
  }
}

void TebOptimalPlanner::AddEdgesAcceleration()
{
  if (cfg_->optim.weight_acc_lim_x==0  && cfg_->optim.weight_acc_lim_theta==0) 
    return; // if weight equals zero skip adding edges!

  int n = teb_.sizePoses();  
    
  if (cfg_->robot.max_vel_y == 0 || cfg_->robot.acc_lim_y == 0) // non-holonomic robot
  {
    Eigen::Matrix<double,2,2> information;
    information.fill(0);
    information(0,0) = cfg_->optim.weight_acc_lim_x;
    information(1,1) = cfg_->optim.weight_acc_lim_theta;
    
    // check if an initial velocity should be taken into accound
    if (vel_start_.first)
    {
      EdgeAccelerationStart* acceleration_edge = new EdgeAccelerationStart;
      acceleration_edge->setVertex(0,teb_.PoseVertex(0));
      acceleration_edge->setVertex(1,teb_.PoseVertex(1));
      acceleration_edge->setVertex(2,teb_.TimeDiffVertex(0));
      acceleration_edge->setInitialVelocity(vel_start_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }

    // now add the usual acceleration edge for each tuple of three teb poses
    for (int i=0; i < n - 2; ++i)
    {
      EdgeAcceleration* acceleration_edge = new EdgeAcceleration;
      acceleration_edge->setVertex(0,teb_.PoseVertex(i));
      acceleration_edge->setVertex(1,teb_.PoseVertex(i+1));
      acceleration_edge->setVertex(2,teb_.PoseVertex(i+2));
      acceleration_edge->setVertex(3,teb_.TimeDiffVertex(i));
      acceleration_edge->setVertex(4,teb_.TimeDiffVertex(i+1));
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }
    
    // check if a goal velocity should be taken into accound
    if (vel_goal_.first)
    {
      EdgeAccelerationGoal* acceleration_edge = new EdgeAccelerationGoal;
      acceleration_edge->setVertex(0,teb_.PoseVertex(n-2));
      acceleration_edge->setVertex(1,teb_.PoseVertex(n-1));
      acceleration_edge->setVertex(2,teb_.TimeDiffVertex( teb_.sizeTimeDiffs()-1 ));
      acceleration_edge->setGoalVelocity(vel_goal_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }  
  }
  else // holonomic robot
  {
    Eigen::Matrix<double,3,3> information;
    information.fill(0);
    information(0,0) = cfg_->optim.weight_acc_lim_x;
    information(1,1) = cfg_->optim.weight_acc_lim_y;
    information(2,2) = cfg_->optim.weight_acc_lim_theta;
    
    // check if an initial velocity should be taken into accound
    if (vel_start_.first)
    {
      EdgeAccelerationHolonomicStart* acceleration_edge = new EdgeAccelerationHolonomicStart;
      acceleration_edge->setVertex(0,teb_.PoseVertex(0));
      acceleration_edge->setVertex(1,teb_.PoseVertex(1));
      acceleration_edge->setVertex(2,teb_.TimeDiffVertex(0));
      acceleration_edge->setInitialVelocity(vel_start_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }

    // now add the usual acceleration edge for each tuple of three teb poses
    for (int i=0; i < n - 2; ++i)
    {
      EdgeAccelerationHolonomic* acceleration_edge = new EdgeAccelerationHolonomic;
      acceleration_edge->setVertex(0,teb_.PoseVertex(i));
      acceleration_edge->setVertex(1,teb_.PoseVertex(i+1));
      acceleration_edge->setVertex(2,teb_.PoseVertex(i+2));
      acceleration_edge->setVertex(3,teb_.TimeDiffVertex(i));
      acceleration_edge->setVertex(4,teb_.TimeDiffVertex(i+1));
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }
    
    // check if a goal velocity should be taken into accound
    if (vel_goal_.first)
    {
      EdgeAccelerationHolonomicGoal* acceleration_edge = new EdgeAccelerationHolonomicGoal;
      acceleration_edge->setVertex(0,teb_.PoseVertex(n-2));
      acceleration_edge->setVertex(1,teb_.PoseVertex(n-1));
      acceleration_edge->setVertex(2,teb_.TimeDiffVertex( teb_.sizeTimeDiffs()-1 ));
      acceleration_edge->setGoalVelocity(vel_goal_.second);
      acceleration_edge->setInformation(information);
      acceleration_edge->setTebConfig(*cfg_);
      optimizer_->addEdge(acceleration_edge);
    }  
  }
}



void TebOptimalPlanner::AddEdgesTimeOptimal()
{
  if (cfg_->optim.weight_optimaltime==0)  // default: 1
    return; // if weight equals zero skip adding edges!

  Eigen::Matrix<double,1,1> information;
  information.fill(cfg_->optim.weight_optimaltime);

  for (int i=0; i < teb_.sizeTimeDiffs(); ++i)
  {
    EdgeTimeOptimal* timeoptimal_edge = new EdgeTimeOptimal;
    timeoptimal_edge->setVertex(0,teb_.TimeDiffVertex(i));
    timeoptimal_edge->setInformation(information);
    timeoptimal_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(timeoptimal_edge);
  }
}

void TebOptimalPlanner::AddEdgesShortestPath()
{
  if (cfg_->optim.weight_shortest_path==0)
    return; // if weight equals zero skip adding edges!

  Eigen::Matrix<double,1,1> information;
  information.fill(cfg_->optim.weight_shortest_path);

  for (int i=0; i < teb_.sizePoses()-1; ++i)
  {
    EdgeShortestPath* shortest_path_edge = new EdgeShortestPath;
    shortest_path_edge->setVertex(0,teb_.PoseVertex(i));
    shortest_path_edge->setVertex(1,teb_.PoseVertex(i+1));
    shortest_path_edge->setInformation(information);
    shortest_path_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(shortest_path_edge);
  }
}



void TebOptimalPlanner::AddEdgesKinematicsDiffDrive()
{
  if (cfg_->optim.weight_kinematics_nh==0 && cfg_->optim.weight_kinematics_forward_drive==0)
    return; // if weight equals zero skip adding edges!
  
  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double,2,2> information_kinematics;
  information_kinematics.fill(0.0);
  information_kinematics(0, 0) = cfg_->optim.weight_kinematics_nh;
  information_kinematics(1, 1) = cfg_->optim.weight_kinematics_forward_drive;
  
  for (int i=0; i < teb_.sizePoses()-1; i++) // ignore twiced start only
  {
    EdgeKinematicsDiffDrive* kinematics_edge = new EdgeKinematicsDiffDrive;
    kinematics_edge->setVertex(0,teb_.PoseVertex(i));
    kinematics_edge->setVertex(1,teb_.PoseVertex(i+1));      
    kinematics_edge->setInformation(information_kinematics);
    kinematics_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(kinematics_edge);
  }	 
}

void TebOptimalPlanner::AddEdgesKinematicsCarlike()
{
  if (cfg_->optim.weight_kinematics_nh==0 && cfg_->optim.weight_kinematics_turning_radius==0)
    return; // if weight equals zero skip adding edges!

  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double,2,2> information_kinematics;
  information_kinematics.fill(0.0);
  information_kinematics(0, 0) = cfg_->optim.weight_kinematics_nh;
  information_kinematics(1, 1) = cfg_->optim.weight_kinematics_turning_radius;
  
  for (int i=0; i < teb_.sizePoses()-1; i++) // ignore twiced start only
  {
    EdgeKinematicsCarlike* kinematics_edge = new EdgeKinematicsCarlike;
    kinematics_edge->setVertex(0,teb_.PoseVertex(i));
    kinematics_edge->setVertex(1,teb_.PoseVertex(i+1));      
    kinematics_edge->setInformation(information_kinematics);
    kinematics_edge->setTebConfig(*cfg_);
    optimizer_->addEdge(kinematics_edge);
  }  
}


void TebOptimalPlanner::AddEdgesPreferRotDir()
{
  //TODO(roesmann): Note, these edges can result in odd predictions, in particular
  //                we can observe a substantional mismatch between open- and closed-loop planning
  //                leading to a poor control performance.
  //                At the moment, we keep these functionality for oscillation recovery:
  //                Activating the edge for a short time period might not be crucial and
  //                could move the robot to a new oscillation-free state.
  //                This needs to be analyzed in more detail!
  if (prefer_rotdir_ == RotType::none || cfg_->optim.weight_prefer_rotdir==0)
    return; // if weight equals zero skip adding edges!

  if (prefer_rotdir_ != RotType::right && prefer_rotdir_ != RotType::left)
  {
    ROS_WARN("TebOptimalPlanner::AddEdgesPreferRotDir(): unsupported RotType selected. Skipping edge creation.");
    return;
  }

  // create edge for satisfiying kinematic constraints
  Eigen::Matrix<double,1,1> information_rotdir;
  information_rotdir.fill(cfg_->optim.weight_prefer_rotdir);
  
  for (int i=0; i < teb_.sizePoses()-1 && i < 3; ++i) // currently: apply to first 3 rotations
  {
    EdgePreferRotDir* rotdir_edge = new EdgePreferRotDir;
    rotdir_edge->setVertex(0,teb_.PoseVertex(i));
    rotdir_edge->setVertex(1,teb_.PoseVertex(i+1));      
    rotdir_edge->setInformation(information_rotdir);
    
    if (prefer_rotdir_ == RotType::left)
        rotdir_edge->preferLeft();
    else if (prefer_rotdir_ == RotType::right)
        rotdir_edge->preferRight();
    
    optimizer_->addEdge(rotdir_edge);
  }
}

void TebOptimalPlanner::AddEdgesVelocityObstacleRatio()
{
  Eigen::Matrix<double,2,2> information;
  information(0,0) = cfg_->optim.weight_velocity_obstacle_ratio;
  information(1,1) = cfg_->optim.weight_velocity_obstacle_ratio;
  information(0,1) = information(1,0) = 0;

  auto iter_obstacle = obstacles_per_vertex_.begin();

  for (int index = 0; index < teb_.sizePoses() - 1; ++index)
  {
    for (const ObstaclePtr obstacle : (*iter_obstacle++))
    {
      EdgeVelocityObstacleRatio* edge = new EdgeVelocityObstacleRatio;
      edge->setVertex(0,teb_.PoseVertex(index));
      edge->setVertex(1,teb_.PoseVertex(index + 1));
      edge->setVertex(2,teb_.TimeDiffVertex(index));
      edge->setInformation(information);
      edge->setParameters(*cfg_, robot_model_.get(), obstacle.get());
      optimizer_->addEdge(edge);
    }
  }
}

bool TebOptimalPlanner::hasDiverged() const
{
  // Early returns if divergence detection is not active
  if (!cfg_->recovery.divergence_detection_enable)
    return false;

  auto stats_vector = optimizer_->batchStatistics();

  // No statistics yet
  if (stats_vector.empty())
    return false;

  // Grab the statistics of the final iteration
  const auto last_iter_stats = stats_vector.back();

  return last_iter_stats.chi2 > cfg_->recovery.divergence_detection_max_chi_squared;
}

void TebOptimalPlanner::computeCurrentCost(double obst_cost_scale, double goal_line_cost_scale, bool alternative_time_cost)
{ 
  // check if graph is empty/exist  -> important if function is called between buildGraph and optimizeGraph/clearGraph
  bool graph_exist_flag(false);
  if (optimizer_->edges().empty() && optimizer_->vertices().empty())
  {
    // here the graph is build again, for time efficiency make sure to call this function 
    // between buildGraph and Optimize (deleted), but it depends on the application
    buildGraph(dynamic_obstacle_current_alpha_, dynamic_obstacle_old_alpha_, detour_dynamic_obstacle_method_);	
    optimizer_->initializeOptimization();
  }
  else
  {
    graph_exist_flag = true;
  }
  
  optimizer_->computeInitialGuess();
  
  cost_ = 0;
  cost_ += teb_.getSumOfAllTimeDiffs();

  double time = 0;
  for (int i=1; i < teb_.sizePoses() - 1; ++i)
  {
    if(obstacles_->size()<=1)
      continue;

    double min_dist1 = 99999;
    double min_dist2 = 99999;
    auto min_obs1 = obstacles_->begin();
    auto min_obs2 = obstacles_->begin();

    time += teb_.TimeDiff(i-1);
    // ROS_INFO("current pos: (%f, %f), time: %f", teb_.Pose(i).x(), teb_.Pose(i).y(), time);
    for (ObstContainer::iterator obst = obstacles_->begin(); obst != obstacles_->end(); ++obst)
    {
      if ((*obst)->isDynamic()){
        Eigen::Vector2d obs_position;
        obst->get()->predictCentroidConstantVelocity(time, obs_position);
        CircularObstacle* obst_index = dynamic_cast<CircularObstacle*>(const_cast<Obstacle*>(obst->get()));
        double dist = hypot(teb_.PoseVertex(i)->x()-obs_position.x(), teb_.PoseVertex(i)->y()-obs_position.y()) - (cfg_->robot.robot_radius + obst_index->radius());
        // if (dist < 0)
        //   ROS_INFO("    dist: %f pos: (%f,%f)->(%f,%f), v: (%f,%f)", dist, obst->get()->getCentroid().x(), obst->get()->getCentroid().y(), obs_position.x(), obs_position.y(), obst->get()->getCentroidVelocity().x(), obst->get()->getCentroidVelocity().y());
        if (dist < min_dist1){
          min_dist2 = min_dist1;
          min_obs2 = min_obs1;
          min_dist1 = dist;
          min_obs1 = obst;
        }
        else if (dist < min_dist2){
          min_dist2 = dist;
          min_obs2 = obst;
        }
      }
      else{
        // static obstacle
        Eigen::Vector2d pos = teb_.Pose(i).position();
        double dist = obst->get()->getMinimumDistance(pos) - cfg_->robot.robot_radius;
        if (dist < min_dist1){
          min_dist2 = min_dist1;
          min_obs2 = min_obs1;
          min_dist1 = dist;
          min_obs1 = obst;
        }
        else if (dist < min_dist2){
          min_dist2 = dist;   
          min_obs2 = obst;
        }
      }
    }
    // 首先是min_dist1 + min_dist2很小，接下来再考虑障碍物间的距离。实际上 理想情况下min_dist1 + min_dist2<0就不能走了，但是因为轨迹是离散点，所以这里有误差，需要一个额外阈值dt_ref * v
    double small_epsilon = 0.1; // 理论上这里应该是0，但是我们给一些额外的裕度
    if (min_dist1 + min_dist2 <= cfg_->trajectory.dt_ref * cfg_->robot.max_vel_x){
      // 计算两个 离当前点最近的 障碍物间的距离：
      // 1. 如果两个障碍物都是静态的，那不用算，肯定能走
      if (!min_obs1->get()->isDynamic() && !min_obs2->get()->isDynamic())
        continue;
      // 2. 如果是两个动态，则计算他们的时空距离，看是否大于机器人半径*2+人半径*2
      else if (min_obs1->get()->isDynamic() && min_obs2->get()->isDynamic()){
        Eigen::Vector2d obs_position1;
        min_obs1->get()->predictCentroidConstantVelocity(time, obs_position1);
        CircularObstacle* index_obs1 = dynamic_cast<CircularObstacle*>(const_cast<Obstacle*>(min_obs1->get()));

        Eigen::Vector2d obs_position2;
        min_obs2->get()->predictCentroidConstantVelocity(time, obs_position2);
        CircularObstacle* index_obs2 = dynamic_cast<CircularObstacle*>(const_cast<Obstacle*>(min_obs2->get()));

        double dis = hypot(obs_position1.x()-obs_position2.x(), obs_position1.y()-obs_position2.y()) - (2*cfg_->robot.robot_radius + index_obs1->radius() + index_obs2->radius());      
        if (dis<=small_epsilon)
          cost_ += 999;
      }
      // 3. 如果是一个动态，一个是静态的，则计算静态障碍物到动态障碍物的距离，看是否大于机器人半径*2+人半径
      else if (min_obs1->get()->isDynamic() && !min_obs2->get()->isDynamic()){
        Eigen::Vector2d obs_position1;
        min_obs1->get()->predictCentroidConstantVelocity(time, obs_position1);
        CircularObstacle* index_obs1 = dynamic_cast<CircularObstacle*>(const_cast<Obstacle*>(min_obs1->get()));
        
        double dis = min_obs2->get()->getMinimumDistance(obs_position1) - (2*cfg_->robot.robot_radius+index_obs1->radius());
        if (dis<=small_epsilon)
          cost_ += 999;
      }
      else if (!min_obs1->get()->isDynamic() && min_obs2->get()->isDynamic()){
        Eigen::Vector2d obs_position2;
        min_obs2->get()->predictCentroidConstantVelocity(time, obs_position2);
        CircularObstacle* index_obs2 = dynamic_cast<CircularObstacle*>(const_cast<Obstacle*>(min_obs2->get()));
        
        double dis = min_obs1->get()->getMinimumDistance(obs_position2) - (2*cfg_->robot.robot_radius+index_obs2->radius());
        if (dis<=small_epsilon)
          cost_ += 999;
      }
    }
  }



  // delete temporary created graph
  if (!graph_exist_flag) 
    clearGraph();
}

double TebOptimalPlanner::get2DAsyGaussValue(double wxo, double wyo, double vxo, double vyo, double wx, double wy){
  double rob_radius = 0.2;
  double obs_radius = 0.3;
  double rob_orientation = atan2(vyo,vxo);
  double human_direction = atan2(wy-wyo, wx-wxo);
  double delta_theta = human_direction - rob_orientation;
  if (delta_theta<0)
    delta_theta += 3.1415;

  double sigma_lr = 0.05+0.2*hypot(vxo,vyo), sigma_f = 0.5+1.0*hypot(vxo,vyo), sigma_b = 0.05+0.2*hypot(vxo,vyo);
  double A = 2.0;
  double dis = hypot(wx-wxo, wy-wyo)-rob_radius-obs_radius;

  double fb_part;
  if (delta_theta>=1.67 && delta_theta<=1.67+3.14)
    fb_part = (dis*cos(delta_theta))*(dis*cos(delta_theta))/(2*sigma_b*sigma_b);
  else
    fb_part = (dis*cos(delta_theta))*(dis*cos(delta_theta))/(2*sigma_f*sigma_f);
  double lr_part = (dis*sin(delta_theta))*(dis*sin(delta_theta))/(2*sigma_lr*sigma_lr);
  double res = A*pow(2.718,(-lr_part-fb_part));
  return res>0.05?res:0;
}

void TebOptimalPlanner::extractVelocity(const PoseSE2& pose1, const PoseSE2& pose2, double dt, double& vx, double& vy, double& omega) const
{
  if (dt == 0)
  {
    vx = 0;
    vy = 0;
    omega = 0;
    return;
  }
  
  Eigen::Vector2d deltaS = pose2.position() - pose1.position();
  
  if (cfg_->robot.max_vel_y == 0) // nonholonomic robot
  {
    Eigen::Vector2d conf1dir( cos(pose1.theta()), sin(pose1.theta()) );
    // translational velocity
    double dir = deltaS.dot(conf1dir);
    vx = (double) g2o::sign(dir) * deltaS.norm()/dt;
    vy = 0;
  }
  else // holonomic robot
  {
    // transform pose 2 into the current robot frame (pose1)
    // for velocities only the rotation of the direction vector is necessary.
    // (map->pose1-frame: inverse 2d rotation matrix)
    double cos_theta1 = std::cos(pose1.theta());
    double sin_theta1 = std::sin(pose1.theta());
    double p1_dx =  cos_theta1*deltaS.x() + sin_theta1*deltaS.y();
    double p1_dy = -sin_theta1*deltaS.x() + cos_theta1*deltaS.y();
    vx = p1_dx / dt;
    vy = p1_dy / dt;    
  }
  
  // rotational velocity
  double orientdiff = g2o::normalize_theta(pose2.theta() - pose1.theta());
  omega = orientdiff/dt;
}

bool TebOptimalPlanner::getVelocityCommand(double& vx, double& vy, double& omega, int look_ahead_poses) const
{
  if (teb_.sizePoses()<2)
  {
    ROS_ERROR("TebOptimalPlanner::getVelocityCommand(): The trajectory contains less than 2 poses. Make sure to init and optimize/plan the trajectory fist.");
    vx = 0;
    vy = 0;
    omega = 0;
    return false;
  }
  look_ahead_poses = std::max(1, std::min(look_ahead_poses, teb_.sizePoses() - 1));
  double dt = 0.0;
  for(int counter = 0; counter < look_ahead_poses; ++counter)
  {
    dt += teb_.TimeDiff(counter);
    if(dt >= cfg_->trajectory.dt_ref * look_ahead_poses)  // TODO: change to look-ahead time? Refine trajectory?
    {
        look_ahead_poses = counter + 1;
        break;
    }
  }
  if (dt<=0)
  {	
    ROS_ERROR("TebOptimalPlanner::getVelocityCommand() - timediff<=0 is invalid!");
    vx = 0;
    vy = 0;
    omega = 0;
    return false;
  }
	  
  // Get velocity from the first two configurations
  extractVelocity(teb_.Pose(0), teb_.Pose(look_ahead_poses), dt, vx, vy, omega);
  return true;
}

void TebOptimalPlanner::getVelocityProfile(std::vector<geometry_msgs::Twist>& velocity_profile) const
{
  int n = teb_.sizePoses();
  velocity_profile.resize( n+1 );

  // start velocity 
  velocity_profile.front().linear.z = 0;
  velocity_profile.front().angular.x = velocity_profile.front().angular.y = 0;  
  velocity_profile.front().linear.x = vel_start_.second.linear.x;
  velocity_profile.front().linear.y = vel_start_.second.linear.y;
  velocity_profile.front().angular.z = vel_start_.second.angular.z;
  
  for (int i=1; i<n; ++i)
  {
    velocity_profile[i].linear.z = 0;
    velocity_profile[i].angular.x = velocity_profile[i].angular.y = 0;
    extractVelocity(teb_.Pose(i-1), teb_.Pose(i), teb_.TimeDiff(i-1), velocity_profile[i].linear.x, velocity_profile[i].linear.y, velocity_profile[i].angular.z);
  }
  
  // goal velocity
  velocity_profile.back().linear.z = 0;
  velocity_profile.back().angular.x = velocity_profile.back().angular.y = 0;  
  velocity_profile.back().linear.x = vel_goal_.second.linear.x;
  velocity_profile.back().linear.y = vel_goal_.second.linear.y;
  velocity_profile.back().angular.z = vel_goal_.second.angular.z;
}

void TebOptimalPlanner::getFullTrajectory(std::vector<TrajectoryPointMsg>& trajectory) const
{
  int n = teb_.sizePoses();
  
  trajectory.resize(n);
  
  if (n == 0)
    return;
     
  double curr_time = 0;
  
  // start
  TrajectoryPointMsg& start = trajectory.front();
  teb_.Pose(0).toPoseMsg(start.pose);
  start.velocity.linear.z = 0;
  start.velocity.angular.x = start.velocity.angular.y = 0;
  start.velocity.linear.x = vel_start_.second.linear.x;
  start.velocity.linear.y = vel_start_.second.linear.y;
  start.velocity.angular.z = vel_start_.second.angular.z;
  start.time_from_start.fromSec(curr_time);
  
  curr_time += teb_.TimeDiff(0);
  
  // intermediate points
  for (int i=1; i < n-1; ++i)
  {
    TrajectoryPointMsg& point = trajectory[i];
    teb_.Pose(i).toPoseMsg(point.pose);
    point.velocity.linear.z = 0;
    point.velocity.angular.x = point.velocity.angular.y = 0;
    double vel1_x, vel1_y, vel2_x, vel2_y, omega1, omega2;
    extractVelocity(teb_.Pose(i-1), teb_.Pose(i), teb_.TimeDiff(i-1), vel1_x, vel1_y, omega1);
    extractVelocity(teb_.Pose(i), teb_.Pose(i+1), teb_.TimeDiff(i), vel2_x, vel2_y, omega2);
    point.velocity.linear.x = 0.5*(vel1_x+vel2_x);
    point.velocity.linear.y = 0.5*(vel1_y+vel2_y);
    point.velocity.angular.z = 0.5*(omega1+omega2);    
    point.time_from_start.fromSec(curr_time);
    
    curr_time += teb_.TimeDiff(i);
  }
  
  // goal
  TrajectoryPointMsg& goal = trajectory.back();
  teb_.BackPose().toPoseMsg(goal.pose);
  goal.velocity.linear.z = 0;
  goal.velocity.angular.x = goal.velocity.angular.y = 0;
  goal.velocity.linear.x = vel_goal_.second.linear.x;
  goal.velocity.linear.y = vel_goal_.second.linear.y;
  goal.velocity.angular.z = vel_goal_.second.angular.z;
  goal.time_from_start.fromSec(curr_time);
}


bool TebOptimalPlanner::isTrajectoryFeasible(base_local_planner::CostmapModel* costmap_model, const std::vector<geometry_msgs::Point>& footprint_spec,
                                             double inscribed_radius, double circumscribed_radius, int look_ahead_idx)
{
  if (look_ahead_idx < 0 || look_ahead_idx >= teb().sizePoses())
    look_ahead_idx = teb().sizePoses() - 1;
  
  for (int i=0; i <= look_ahead_idx; ++i)
  {           
    if ( costmap_model->footprintCost(teb().Pose(i).x(), teb().Pose(i).y(), teb().Pose(i).theta(), footprint_spec, inscribed_radius, circumscribed_radius) == -1 )
    {
      if (visualization_)
      {
        visualization_->publishInfeasibleRobotPose(teb().Pose(i), *robot_model_);
      }
      return false;
    }
    // Checks if the distance between two poses is higher than the robot radius or the orientation diff is bigger than the specified threshold
    // and interpolates in that case.
    // (if obstacles are pushing two consecutive poses away, the center between two consecutive poses might coincide with the obstacle ;-)!
    if (i<look_ahead_idx)
    {
      double delta_rot = g2o::normalize_theta(g2o::normalize_theta(teb().Pose(i+1).theta()) -
                                              g2o::normalize_theta(teb().Pose(i).theta()));
      Eigen::Vector2d delta_dist = teb().Pose(i+1).position()-teb().Pose(i).position();
      if(fabs(delta_rot) > cfg_->trajectory.min_resolution_collision_check_angular || delta_dist.norm() > inscribed_radius)
      {
        int n_additional_samples = std::max(std::ceil(fabs(delta_rot) / cfg_->trajectory.min_resolution_collision_check_angular), 
                                            std::ceil(delta_dist.norm() / inscribed_radius)) - 1;
        PoseSE2 intermediate_pose = teb().Pose(i);
        for(int step = 0; step < n_additional_samples; ++step)
        {
          intermediate_pose.position() = intermediate_pose.position() + delta_dist / (n_additional_samples + 1.0);
          intermediate_pose.theta() = g2o::normalize_theta(intermediate_pose.theta() + 
                                                           delta_rot / (n_additional_samples + 1.0));
          if ( costmap_model->footprintCost(intermediate_pose.x(), intermediate_pose.y(), intermediate_pose.theta(),
            footprint_spec, inscribed_radius, circumscribed_radius) == -1 )
          {
            if (visualization_) 
            {
              visualization_->publishInfeasibleRobotPose(intermediate_pose, *robot_model_);
            }
            return false;
          }
        }
      }
    }
  }
  return true;
}

} // namespace teb_local_planner
