#ifndef _MAP_PROCESS_H_
#define _MAP_PROCESS_H_
#include <math.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include <teb_local_planner/utilizes.h>
#include <teb_local_planner/pose_se2.h>
#include <teb_local_planner/obstacles.h>
#include <random>
#include <ctime>
#include <cassert>
#include <costmap_2d/costmap_2d.h>
#include <teb_local_planner/dynamicvoronoi.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

class mapProcess{
  public:
    /**  main functions  **/
    mapProcess(){};
    // 0. form goal lines
    std::vector<std::vector<Point2D>> findGoalLineGroups(costmap_2d::Costmap2D* costmap, const std::vector<geometry_msgs::PoseStamped>& local_plan, std::pair<double,double> global_goal);
    // 1. initialize maps derived from costmap
    void initialize(costmap_2d::Costmap2D* costmap, const teb_local_planner::PoseSE2& start, const teb_local_planner::PoseSE2& goal, const std::vector<geometry_msgs::PoseStamped>& local_plan, const std::vector<std::vector<double>>& dynamic_obstacle, std::pair<double,double> global_goal, std::pair<double,double> velocity_in_map);
    // 2. cluster all points into several group of obstacles by iterating the whole map to and calling addObsConnect()
    std::map<int, std::vector<Point2D>> clusterObs();
    std::map<int, std::vector<Point2D>> dilateObs();
    // 2.1. merge the point (x,y) to the nearby obs group, or create a new obs group
    void addObsConnect(int x, int y);
    void addObsConnect(int x, int y, std::vector<std::vector<int>>& map_obs_labeled, std::map<int, std::vector<Point2D>>& obs_list, int& label_index, const int& width, const int& height);
    // 3. outline the border ponits of obs group by comparing the original obs group and the dilated one
    std::map<int, std::vector<Point2D>> borderIdentify();
    bool deepSearchForFindingBorders(Point2D current, std::vector<std::vector<bool>>& map_is_visited_temp, const std::vector<Point2D>& kernel, const Point2D& start, const int& label);
    // 4. extract corner points which are the key points among border points 
    std::map<int, std::vector<Point2D>> cornerIdentify();
    // 5. establish connections between obstacle group, start point, and goal lines
    void connectObstacleGroups();
    void connectStartAndGoalToObstacleGroups();
    // 6. find all non-homo paths, first finding general paths, then deriving several normal paths
    std::vector<std::vector<Eigen::Vector2d>> findHomoPaths(std::vector<std::pair<std::pair<double, double>, std::pair<double, double>>>& goal_endpoints_list);
    // 6.1. find general paths, each general path consists of several obs-group-IDs
    bool depthFirst(std::vector<int>& visited, std::vector<std::vector<int>>& res);
    // 6.2. find the normal path around the obs group obsID, starting at P_start and ending at P_target
    std::vector<Point2D> getPathAlongsideObs(int obsID, std::string traj_type, Point2D P_start, Point2D P_target);
    // 6.3. shorten the normal path 
    std::vector<Point2D_float> shortenPath (std::vector<Point2D> path, bool& is_delete);
    // 7. do the same things like borderIdentify(), generate obstacle boundaries for TEB
    std::map<int, std::vector<std::pair<double,double>>> getStaticObsIdentificationInWorld();
    std::map<int, std::vector<double>> getDynamicObsIdentificationInWorld();
    bool deepSearchForFindingBorders(std::map<int, std::vector<Point2D>>& border_list_temp, std::vector<std::vector<int>>& map_border_labeled_temp, Point2D current, std::vector<std::vector<bool>>& map_is_visited_temp, const std::vector<Point2D>& kernel, const Point2D& start, const int& label);
    void add_to_homo_paths_pruned(std::vector<Eigen::Vector2d> path);
    std::pair<std::pair<double,double>, std::pair<double,double>> getGoalLineEnds(Point2D goal);

    /**  utilities  defined in map_process_utilizes.cpp **/
    void setParams(const int& max_path_explore_number_for_GraphicTEB, const int& max_path_remained_for_GraphicTEB, const bool& is_hallway, const bool& is_cos_limitation, const bool& is_father_visit_limitation, const double& epsilon_for_early_stop);
    cv::Mat convertVector2Mat(const std::vector<uint8_t>& v, int channels, int rows);
    void map2world(int mx, int my, double& wx, double& wy);
    void map2world(float mx, float my, double& wx, double& wy);
    void map2worldLarge(int mx, int my, double& wx, double& wy);
    void world2map(int& mx, int& my, double wx, double wy);
    std::vector<Eigen::Vector2d> transPoint2DinMapToVector2dinWorld(std::vector<Point2D> points);
    std::vector<Eigen::Vector2d> transPoint2DinMapToVector2dinWorld(std::vector<Point2D_float> points);
    std::vector<Point2D> Bresenham (const Point2D& p1, const Point2D& p2);
    std::vector<Point2D_float> Bresenham (const Point2D_float& p1, const Point2D_float& p2);
    std::vector<Point2D_float> BresenhamDirect (const Point2D_float& p1, const Point2D_float& p2);
    bool checkInMap(int x, int y);
    bool checkInMap(int x, int y, int width, int height);
    bool checkCollision (const Point2D& p1, const Point2D& p2);
    bool checkCollision (const std::vector<Point2D>& vec_line, const int& ID1, const int& ID2);
    bool checkCollision (const std::vector<Point2D>& vec_line, const int& ID);
    bool checkCollision (const std::vector<Point2D>& vec_line);
    bool checkCollisionOnlyObs (const std::vector<Point2D>& vec_line);
    bool checkCollisionOnlyObs (const std::vector<Point2D_float>& vec_line);
    bool checkCollisionOnlyObsAlignedWithBresenham(const Point2D& p1, const Point2D& p2);
    bool checkCollisionOnlyObsAlignedWithBresenhamDirect(const Point2D_float& p1, const Point2D_float& p2);
    bool checkCollisionObsAndBorderAlignedWithBresenham(const Point2D& p1, const Point2D& p2, int ID);
    bool checkCollisionObsAndBorderAlignedWithBresenham(const Point2D& p1, const Point2D& p2, int ID1, int ID2);
    bool checkCollisionNotSelfBorder (const std::vector<Point2D>& vec_line, const int& ID1, const int& ID2);
    void printVector(std::vector<Point2D> obj);
    void printVectorSegmented(std::vector<Point2D> obj);
    int getMod(const int& input, const int& length);

    /**  access to outside  **/
    std::map<int, std::vector<Point2D>> getTempList() {return temp_list_;};
    std::map<std::pair<int,int>, std::map<std::string, std::pair<Point2D, Point2D>>> getConnectGraph() {return connection_graph_;};
    std::vector<std::vector<std::pair<int,int>>> getMapBoundaries(){return map_boundaries_;};
    class std::vector<std::vector<Point2D>> getGoalLineList(){return goal_line_lists_;};
    Point2D getFartherGoal(){return farther_goal_;};
    Point2D getNearGoal() {return near_goal_;};
    int getStartID() {return startID_;};
    std::map<int, std::vector<Point2D>> getObsList(){return obs_list_;};
    std::map<int, std::vector<Point2D>> getBorderList(){return border_list_;};
    std::map<int, std::vector<Point2D>> getCornerList(){return corner_list_;};
    std::map<int, std::vector<Point2D>> getDilateObsList(){return dilated_obs_list_;};
    std::map<std::pair<int,int>, std::vector<Point2D>> getVoronoiList(){return voronoi_list_;};
    std::vector<std::vector<Eigen::Vector2d>> getHomoPaths() {return homo_paths_;};
    std::vector<std::vector<Eigen::Vector2d>> getHomoPathsOrigin() {return homo_paths_origin_;};
    std::vector<std::vector<Eigen::Vector2d>> getHomoPathsPruned() {return homo_paths_add_to_TEB_;};
    std::vector<std::vector<int>> getMapObsLabeled() {return map_obs_shrinked_labeled_;};
    std::vector<std::vector<int>> getMapBorderLabeled() {return map_border_labeled_;};
    std::pair<Point2D, Point2D> getShortestPathBetweenObs(int ID1, int ID2);
    std::pair<Point2D, Point2D> getClockwiseConsistentPathBetweenObs(int ID1, int ID2);
    std::pair<Point2D, Point2D> getCounterClockwiseConsistentPathBetweenObs(int ID1, int ID2);
    std::pair<Point2D, Point2D> getClockwise2CounterClockwisePathBetweenObs(int ID1, int ID2);
    std::pair<Point2D, Point2D> getCounterClockwise2ClockwisePathBetweenObs(int ID1, int ID2);

  public:
    int width_large_;
    int height_large_;
    double resolution_large_;
    double origin_x_large_;
    double origin_y_large_;
    int shrink_dis_;
    int ignore_dis_;

    std::map<std::pair<int,int>, std::vector<Point2D>> voronoi_list_;
    std::map<std::pair<int,int>, std::map<std::string, std::pair<Point2D, Point2D>>> connection_graph_;   // {ID1 -> ID2}: connectionType:{pointFromID1, pointFromID2}
    std::map<int, std::vector<Point2D>> temp_list_;
    Point2D farther_goal_;
    Point2D near_goal_;
    std::vector<std::vector<std::pair<int,int>>> map_boundaries_;


    // parameters related to costmap
    costmap_2d::Costmap2D* costmap_;
    uint32_t width_, height_;
    double origin_x_, origin_y_;
    float resolution_;
    int start_in_map_x_, start_in_map_y_, startID_, startIndex_;
    int goal_in_map_x_, goal_in_map_y_, goalID_start_, goalIndex_start_;    
    double obs_radius_, rob_radius_, length_in_map_;
    std::pair<double,double> velocity_in_map_;
    std::vector<std::vector<double>> dynamic_obstacle_;   // save the state of pedestrians {x,y,vx,vy,r}
    std::vector<std::vector<Point2D>> goal_line_lists_;

    // maps derived from costmap
    cv::Mat map_cv_;                      // 2D Mat map with static obs and dynamic obs
    cv::Mat map_cv_without_dynamic_obs_;  // 2D Mat map with only static obs
    std::vector<std::vector<int>> map_obs_shrinked_labeled_;    // 2D pure map without dilation by the robot radius

    // map generated in clusterObs() and addObsConnect()
    std::vector<std::vector<int>> map_obs_labeled_;   // a 2D map labeling obs points by groupID
    std::vector<std::vector<bool>> map_is_dynamic_;   // where the point (mx, my) is a point from dynamic pedestrian
    std::vector<std::vector<float>> map_vel_x_;
    std::vector<std::vector<float>> map_vel_y_;
    std::vector<std::vector<bool>> map_obs_occupied_without_ignore_;
    std::map<int, std::vector<Point2D>> obs_list_;    // group ID: obs points
    std::vector<int> label_list_;                     // projection from index to group ID
    std::vector<int> label_static_list_;
    std::vector<int> label_dynamic_list_;
    int label_index_;                                 // to-be assigned label ID
    std::map<int, std::vector<Point2D>> dilated_obs_list_;
    std::vector<std::vector<int>> map_dilated_obs_labeled_;

    // map saving border points which are outlined from map_obs_labeled_
    std::vector<std::vector<int>> map_border_labeled_;  // a 2D map labeling border points by groupID
    std::map<int, std::vector<Point2D>> border_list_;   // group ID: border points
    // map saving corner points which are outlined from map_border_labeled_
    std::vector<std::vector<int>> map_corner_labeled_;  // a 2D map labeling corner points by groupID
    std::map<int, std::vector<Point2D>> corner_list_;   // group ID: corner points
    
    // 2D data map to save the connections between two obs groups
    std::vector<std::vector<Eigen::Vector2d>> homo_paths_origin_;
    std::vector<std::vector<Eigen::Vector2d>> homo_paths_;                // non-homo paths before H-signature examination
    std::vector<double> length_approximate_;                              // a vector with each data indicating the length of the related path
    std::vector<std::vector<Eigen::Vector2d>> homo_paths_add_to_TEB_;     // non-homo paths after H-signature examination
    int max_path_explore_number_for_GraphicTEB_;                          // the preset threshold of the  DepthFirst Search
    int max_path_remained_for_GraphicTEB_;
    bool is_hallway_;
    bool is_cos_limitation_;
    bool is_father_visit_limitation_;
    bool is_limitation_;                                                  // a preset parameter, if true, only obs groups visible to each other will be connected during homo DepthFirst Search
    double epsilon_for_early_stop_;
};

#endif