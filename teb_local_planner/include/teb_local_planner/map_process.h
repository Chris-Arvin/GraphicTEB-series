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

class mapProcess{
  public:
    /**  main functions  **/
    // 1. initialize maps derived from costmap
    mapProcess(){};
    mapProcess(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    mapProcess(costmap_2d::Costmap2D* costmap, const teb_local_planner::PoseSE2& start, const teb_local_planner::PoseSE2& goal, const std::vector<std::vector<double>>& dynamic_obstacle);
    // 2. cluster all points into several group of obstacles by iterating the whole map to and calling addObsConnect()
    std::map<int, std::vector<Point2D>> clusterObs();
    // 2.1. merge the point (x,y) to the nearby obs group, or create a new obs group
    void addObsConnect(int x, int y);
    // 3. outline the border ponits of obs group by comparing the original obs group and the dilated one
    std::map<int, std::vector<Point2D>> borderIdentify();
    bool deepSearchForFindingBorders(Point2D current, std::vector<std::vector<bool>>& map_is_visited_temp, const std::vector<Point2D>& kernel, const Point2D& start, const int& label);
    // 4. extract corner points which are the key points among border points 
    std::map<int, std::vector<Point2D>> cornerIdentify();
    // 5. find paths between obs group by iterating all the two obs groups and calling findPathBetweenObs()
    std::vector<std::vector<std::pair<Point2D, Point2D>>> createPathBetweenObs();
    bool isVisited(int ID1, int ID2);
    void setVisited(int ID1, int ID2);
    // 5.1. find all the path between two given obs groups
    std::vector<int> findPathBetweenObs(int obsID1, int obsID2);
    void setCoverObs(int ID1, int ID2, std::vector<int> covers);
    std::vector<int> getCoverObs(int ID1, int ID2);
    void setPath(std::pair<Point2D, Point2D> path);
    // 6. find all non-homo paths, first finding general paths, then deriving several normal paths
    std::vector<std::vector<Eigen::Vector2d>> findHomoPaths(const teb_local_planner::PoseSE2 &start, const teb_local_planner::PoseSE2 &goal, const int& max_path_explore_number_for_GraphicTEB, const int& max_path_remained_for_GraphicTEB, const bool& is_limitation, const bool& is_hallway);
    // 6.1. find general paths, each general path consists of several obs-group-IDs
    bool depthFirst(std::vector<int>& visited, int goal_index, std::vector<std::vector<int>>& res);
    // 6.2. find the line-shaped path between obs group ID1 and group ID2, this path is the normal path consisting of several points (x,y)
    std::pair<Point2D, Point2D> getPathBetweenObs(int ID1, int ID2);
    // 6.3. find the normal path around the obs group obsID, starting at P_start and ending at P_target
    std::pair<std::vector<Point2D>,std::vector<Point2D>> getPathAlongsideObs(int obsID, Point2D P_start, Point2D P_target);
    // 6.4. shorten the normal path 
    std::vector<Point2D> shortenPath (std::vector<Point2D> path);
    // 7. do the same things like borderIdentify()
    std::map<int, std::vector<std::pair<double,double>>> getStaticObsIdentificationInWorld();
    bool deepSearchForFindingBorders(std::map<int, std::vector<Point2D>>& border_list_temp, std::vector<std::vector<int>>& map_border_labeled_temp, Point2D current, std::vector<std::vector<bool>>& map_is_visited_temp, const std::vector<Point2D>& kernel, const Point2D& start, const int& label);

    /**  utilities  **/
    cv::Mat convertVector2Mat(const std::vector<uint8_t>& v, int channels, int rows);
    void map2world(int mx, int my, double& wx, double& wy);
    void world2map(int& mx, int& my, double wx, double wy);
    std::vector<Eigen::Vector2d> transPoint2DinMapToVector2dinWorld(std::vector<Point2D> points);
    std::vector<Point2D> Bresenham (const Point2D& p1, const Point2D& p2);
    bool checkInMap(int x, int y);
    std::pair<Point2D, Point2D> checkCoverBorder(const Point2D& p1, const Point2D& p2);
    bool checkCollision (const Point2D& p1, const Point2D& p2);
    int findNearestCornerFromBorder (const Point2D& p);
    void printVector(std::vector<Point2D> obj);
    void printVectorSegmented(std::vector<Point2D> obj);
    void clearSelf();

    /**  access to outside  **/
    std::vector<int> getLabelList(){return label_list_;};
    std::map<int, std::vector<Point2D>> getObsList(){return obs_list_;};
    std::map<int, std::vector<Point2D>> getBorderList(){return border_list_;};
    std::map<int, std::vector<Point2D>> getCornerList(){return corner_list_;};
    std::vector<std::vector<std::pair<Point2D,Point2D>>> getEdgesGraph(){return edges_graph_;};
    std::vector<std::vector<std::vector<int>>> getCoversGraph() {return covers_graph_;};
    std::vector<std::vector<Eigen::Vector2d>> getHomoPaths() {return homo_paths_;};
    std::vector<std::vector<Eigen::Vector2d>> getHomoPathsPruned() {return homo_paths_pruned_;};
    std::vector<std::vector<int>> getMapObsLabeled();
    std::vector<std::vector<double>> getDynamicObsIdentificationInWorld(){return dynamic_obstacle_;};
    void add_to_homo_paths_pruned(std::vector<Eigen::Vector2d> path);


  private:
    // parameters related to costmap
    costmap_2d::Costmap2D* costmap_;
    uint32_t width_, height_;
    double origin_x_, origin_y_;
    float resolution_;
    int start_in_map_x_, start_in_map_y_, startID_, startIndex_;
    int goal_in_map_x_, goal_in_map_y_, goalID_, goalIndex_;    
    double obs_radius_, rob_radius_, length_in_map_;
    std::vector<std::vector<double>> dynamic_obstacle_;   // save the state of pedestrians {x,y,vx,vy,r}
    // maps derived from costmap
    std::vector<int8_t> map_vector_;      // 1D char map
    std::vector<uint8_t> map_int_;        // 1D int map
    cv::Mat map_cv_;                      // 2D Mat map with static obs and dynamic obs
    cv::Mat map_cv_without_dynamic_obs_;  // 2D Mat map with only static obs
    std::vector<std::vector<bool>> map_obs_labeled_outline_;    // 2D map indicating whether the borderline point is covered by obstacle
    std::vector<int> id_list_of_borders_on_the_outline_;        // 1D vector saving the IDs of obs groups intersecting the map borderline
    std::vector<std::vector<int>> map_obs_shrinked_labeled_;    // 2D pure map without dilation by the robot radius
    // map generated in clusterObs() and addObsConnect()
    std::vector<std::vector<int>> map_obs_labeled_;   // a 2D map labeling obs points by groupID
    std::map<int, std::vector<Point2D>> obs_list_;    // group ID: obs points
    std::vector<int> label_list_;                     // projection from index to group ID
    int label_index_;                                 // to-be assigned label ID
    // map saving border points which are outlined from map_obs_labeled_
    std::vector<std::vector<int>> map_border_labeled_;  // a 2D map labeling border points by groupID
    std::map<int, std::vector<Point2D>> border_list_;   // group ID: border points
    // map saving corner points which are outlined from map_border_labeled_
    std::vector<std::vector<int>> map_corner_labeled_;  // a 2D map labeling corner points by groupID
    std::map<int, std::vector<Point2D>> corner_list_;   // group ID: corner points
    // 2D data map to save the connections between two obs groups
    std::vector<std::vector<bool>> visited_;                              // whether the connection between two IDs has been visited (assigned)
    std::vector<std::vector<std::pair<Point2D,Point2D>>>  edges_graph_;   // save the specific points through which two groups can be connected directly
    std::vector<std::vector<std::vector<int>>>  covers_graph_;            // save the specific paths when connecting two obs groups
    std::vector<std::vector<Eigen::Vector2d>> homo_paths_;                // non-homo paths before H-signature examination
    std::vector<double> length_approximate_;                              // a vector with each data indicating the length of the related path
    std::vector<std::vector<Eigen::Vector2d>> homo_paths_pruned_;         // non-homo paths after H-signature examination
    int max_path_explore_number_for_GraphicTEB_;                          // the preset threshold of the  DepthFirst Search
    bool is_limitation_;                                                  // a preset parameter, if true, only obs groups visible to each other will be connected during homo DepthFirst Search
};

#endif