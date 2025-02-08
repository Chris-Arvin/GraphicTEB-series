/**
 *  \author Arvin <1120210190@mail.nankai.edu.cn>
*/
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include <nav_msgs/Odometry.h>
#include <pedsim_msgs/TrackedPerson.h>
#include <pedsim_msgs/TrackedPersons.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

class ObstacleProcess{
  public:
    ObstacleProcess(ros::NodeHandle nh);
    void YamlReader();
    void MatchMap();
    // callbacks
    void ObstacleCallback(const visualization_msgs::Marker::ConstPtr& walls);
    void PersonCallback(const pedsim_msgs::TrackedPersons::ConstPtr& persons);
    

  private:
    /// publishers
    ros::Publisher pub_map;
    ros::Publisher pub_map_with_people;
    ros::Publisher pub_person;
    /// subscribers
    ros::Subscriber sub_obstacles;
    ros::Subscriber sub_person;
  
  private:
    /// message to be published
    nav_msgs::OccupancyGrid map;
    nav_msgs::OccupancyGrid map_with_people;        
    ros::NodeHandle nh_;
    std::string frame_id;
    double resolution;
    double person_diameter;
    bool is_map_initialized;
    bool is_have_pedestrian;
    double xMin, xMax, yMin, yMax;
};