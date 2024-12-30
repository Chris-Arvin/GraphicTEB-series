#include <pedsim_map/sim_map.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fake_map_server");
  ros::NodeHandle n;
  ros::Rate loop_rate(25);
  ObstacleProcess mapProcess(n);
  mapProcess.YamlReader();
  while (ros::ok()){
    mapProcess.MatchMap();
    ros::spinOnce();
    loop_rate.sleep();
  }
}