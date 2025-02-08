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

#ifndef GRAPH_SEARCH_INTERFACE_H
#define GRAPH_SEARCH_INTERFACE_H

#ifdef BOOST_NO_CXX11_DEFAULTED_FUNCTIONS
  #include <boost/graph/adjacency_list.hpp>
#else
  // Workaround for a bug in boost graph library (concerning directed graphs), boost version 1.48:
  // boost::add_vertex requires a move constructor/assignment operator in one of the underlying boost objects if C++11 is activated,
  // but they are missing. The compiler fails due to an implicit deletion. We just deactivate C++11 default functions for now.
  #define BOOST_NO_CXX11_DEFAULTED_FUNCTIONS
  #include <boost/graph/adjacency_list.hpp>
  #undef BOOST_NO_CXX11_DEFAULTED_FUNCTIONS
#endif

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/utility.hpp>
#include <boost/random.hpp>

#include <Eigen/Core>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include <teb_local_planner/h_signature.h>
#include <teb_local_planner/pose_se2.h>
#include <teb_local_planner/teb_config.h>
#include <costmap_2d/costmap_2d.h>
#include <teb_local_planner/map_process.h>
#include <time.h>

namespace teb_local_planner
{

class HomotopyClassPlanner; // Forward declaration

//! Vertex in the graph that is used to find homotopy classes (only stores 2D positions)
struct HcGraphVertex
{
public:
  Eigen::Vector2d pos; // position of vertices in the map
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//! Abbrev. for the homotopy class search-graph type @see HcGraphVertex
typedef boost::adjacency_list < boost::listS, boost::vecS, boost::directedS, HcGraphVertex, boost::no_property > HcGraph;
//! Abbrev. for vertex type descriptors in the homotopy class search-graph
typedef boost::graph_traits<HcGraph>::vertex_descriptor HcGraphVertexType;
//! Abbrev. for edge type descriptors in the homotopy class search-graph
typedef boost::graph_traits<HcGraph>::edge_descriptor HcGraphEdgeType;
//! Abbrev. for the vertices iterator of the homotopy class search-graph
typedef boost::graph_traits<HcGraph>::vertex_iterator HcGraphVertexIterator;
//! Abbrev. for the edges iterator of the homotopy class search-graph
typedef boost::graph_traits<HcGraph>::edge_iterator HcGraphEdgeIterator;
//! Abbrev. for the adjacency iterator that iterates vertices that are adjecent to the specified one
typedef boost::graph_traits<HcGraph>::adjacency_iterator HcGraphAdjecencyIterator;

//!< Inline function used for calculateHSignature() in combination with HCP graph vertex descriptors
inline std::complex<long double> getCplxFromHcGraph(HcGraphVertexType vert_descriptor, const HcGraph& graph)
{
  return std::complex<long double>(graph[vert_descriptor].pos.x(), graph[vert_descriptor].pos.y());
}

//!< Inline function used for initializing the TEB in combination with HCP graph vertex descriptors
inline const Eigen::Vector2d& getVector2dFromHcGraph(HcGraphVertexType vert_descriptor, const HcGraph& graph)
{
  return graph[vert_descriptor].pos;
}

inline const Eigen::Vector2d& getSelf(const Eigen::Vector2d& value)
{
  return value;
}


class graphicProcess
{
public:
  graphicProcess(costmap_2d::Costmap2D *costmap2d, const TebConfig& cfg, HomotopyClassPlanner* hcp) : cfg_(&cfg), hcp_(hcp), costmap2d_(costmap2d), static_safety_margin_(0), dynamic_safety_margin_(0){};
  ~graphicProcess(){}
  void clearGraph() {graph_.clear();}
  void updateDynamicObstacle(std::vector<std::vector<double>> dynamic_obstacle);
  /**
   * @brief Create a graph containing points in the global frame that can be used to explore new possible paths between start and goal.
   *
   * This version of the graph creation places a keypoint on the left and right side of each obstacle w.r.t to the goal heading. \n
   * All feasible paths between start and goal point are extracted using a Depth First Search afterwards. \n
   * This version works very well for small point obstacles. For more complex obstacles call the createProbRoadmapGraph()
   * method that samples keypoints in a predefined area and hopefully finds all relevant alternative paths.
   *
   * @see createProbRoadmapGraph
   * @param start Start pose from wich to start on (e.g. the current robot pose).
   * @param goal Goal pose to find paths to (e.g. the robot's goal).
   * @param dist_to_obst Allowed distance to obstacles: if not satisfying, the path will be rejected (note, this is not the distance used for optimization).
   * @param obstacle_heading_threshold Value of the normalized scalar product between obstacle heading and goal heading in order to take them (obstacles) into account [0,1]
   * @param start_velocity start velocity (optional)
   * @param free_goal_vel if \c true, a nonzero final velocity at the goal pose is allowed, otherwise the final velocity will be zero (default: false)
   */
  void createGraph(const PoseSE2& start, const PoseSE2& goal, const std::vector<geometry_msgs::PoseStamped>& local_plan, costmap_2d::Costmap2D* costmap2d, double dist_to_obst, double obstacle_heading_threshold, const geometry_msgs::Twist* start_velocity, bool free_goal_vel = false, std::pair<double,double> global_goal={0,0});
  void addPaths(const PoseSE2& start, const PoseSE2& goal, const std::vector<geometry_msgs::PoseStamped>& local_plan, costmap_2d::Costmap2D* costmap2d, double dist_to_obst, double obstacle_heading_threshold, const geometry_msgs::Twist* start_velocity, bool free_goal_vel);
  mapProcess* getMapProcess() {return &map_cv_obj_;};
  HcGraph graph_; //!< Store the graph that is utilized to find alternative homotopy classes.

protected:
  const TebConfig* cfg_; //!< Config class that stores and manages all related parameters
  HomotopyClassPlanner* const hcp_; //!< Raw pointer to the HomotopyClassPlanner. The HomotopyClassPlanner itself is guaranteed to outlive the graph search class it is holding.
  mapProcess map_cv_obj_;
  std::vector<std::vector<double>> dynamic_obstacle_;
  costmap_2d::Costmap2D* costmap2d_;
  double static_safety_margin_;
  double dynamic_safety_margin_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


} // end namespace

#endif // GRAPH_SEARCH_INTERFACE_H
