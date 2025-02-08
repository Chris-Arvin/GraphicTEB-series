#include <pedsim_map/sim_map.h>

ObstacleProcess::ObstacleProcess(ros::NodeHandle nh){
  nh_ = nh;
  is_map_initialized = false;
  is_have_pedestrian = false;
  xMin=10000;
  xMax=-10000;
  yMin=10000;
  yMax=-10000;
  pub_map = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
  pub_map_with_people = nh_.advertise<nav_msgs::OccupancyGrid>("/map_with_people", 1, true);
  pub_person = nh_.advertise<pedsim_msgs::TrackedPersons>("/persons", 1, true);
  sub_obstacles = nh_.subscribe<visualization_msgs::Marker>("/pedsim_visualizer/walls", 1, boost::bind(&ObstacleProcess::ObstacleCallback, this ,_1));
  sub_person = nh_.subscribe<pedsim_msgs::TrackedPersons>("/pedsim_visualizer/tracked_persons", 1, boost::bind(&ObstacleProcess::PersonCallback, this ,_1));
}

void ObstacleProcess::YamlReader(){
  nh_.param<std::string>("pedsim_fakemap/frame_id", frame_id, "odom");
  nh_.param<double>("pedsim_fakemap/person_diameter", person_diameter, 0.4);
  nh_.param<double>("pedsim_fakemap/resolution", resolution, 0.1);
}

void ObstacleProcess::MatchMap(){
  tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
  tf::Quaternion q;
  q.setRPY(0, 0, 0.0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "odom"));
}


void ObstacleProcess::PersonCallback(const pedsim_msgs::TrackedPersons::ConstPtr& persons) {
  is_have_pedestrian = true;
  // remap the topic into /persons
  pub_person.publish(persons);
  // publish the map with obstacles as well as persons
  if (is_map_initialized){
    map_with_people = map;
    double temp_x, temp_y;
    for (auto person:persons->tracks){
      temp_x = person.pose.pose.position.x;
      temp_y = person.pose.pose.position.y;
      for (int i = floor(temp_x/resolution-xMin/resolution-person_diameter/resolution/2); i< ceil(temp_x/resolution-xMin/resolution+person_diameter/resolution/2); i++){  
        for (int j = floor(temp_y/resolution-yMin/resolution-person_diameter/resolution/2); j< ceil(temp_y/resolution-yMin/resolution+person_diameter/resolution/2); j++){  
          map_with_people.data[MAP_IDX(map_with_people.info.width, i, j)] = 100; 
        }
      }
    }
    pub_map_with_people.publish(map_with_people);
  }
}

void ObstacleProcess::ObstacleCallback(const visualization_msgs::Marker::ConstPtr& walls) {
  if (!is_map_initialized){
    // find the origin and size of the map
    double px,py;  
    for (auto p:walls->points){
      px = p.x;
      py = p.y;
      if (px < xMin) xMin = px;
      if (px > xMax) xMax = px;
      if (py < yMin) yMin = py;
      if (py > yMax) yMax = py;
    }
    xMin -= 2;
    xMax += 2;
    yMin -= 2;
    yMax += 2;

    int width = ceil((xMax-xMin)/resolution);
    int height = ceil((yMax-yMin)/resolution);

    map.info.resolution = resolution;
    map.info.width = width;
    map.info.height = height;
    map.info.map_load_time = ros::Time::now();

    map.info.origin.position.x = xMin;
    map.info.origin.position.y = yMin;
    map.info.origin.position.z = 0;
    map.info.origin.orientation.x = 0;
    map.info.origin.orientation.y = 0;
    map.info.origin.orientation.z = 0;
    map.info.origin.orientation.w = 1;

    map.header.frame_id = frame_id;
    map.header.stamp = ros::Time::now();
    ROS_INFO("Read a %d X %d map @ %.3lf m/cell",
              map.info.width,
              map.info.height,
              map.info.resolution);

    map.data.resize(map.info.width * map.info.height);
    for (unsigned int i=0; i<map.info.width * map.info.height; i++){
      map.data[i] = 1;
    }

    // project the obstacles into the map
    for (auto p:walls->points){
      px = p.x;
      py = p.y;
      // 0.1是障碍物自身的宽度
      for (int i=floor((p.x-0.05)/resolution); i<=floor((p.x+0.05)/resolution); i++){
        for (int j=floor((p.y-0.05)/resolution); j<=floor((p.y+0.05)/resolution); j++){
          if (MAP_IDX(map.info.width, i-xMin/resolution, j-yMin/resolution) < map.data.size()){
            map.data[MAP_IDX(map.info.width, i-xMin/resolution, j-yMin/resolution)] = 100;
          }
        }
      }
    }
    is_map_initialized = true;
    pub_map.publish(map);
  }
  else{
    map.info.map_load_time = ros::Time::now();
    map.header.stamp = ros::Time::now();
    pub_map.publish(map);
  }
  if (!is_have_pedestrian){
    pub_map_with_people.publish(map);
  }
}