#include <teb_local_planner/map_process.h>


std::vector<std::vector<Point2D>> mapProcess::findGoalLineGroups(costmap_2d::Costmap2D* costmap, const std::vector<geometry_msgs::PoseStamped>& local_plan, std::pair<double,double> global_goal){
  // 求一个res，顺时针排列所有goal line group
  std::vector<std::vector<Point2D>> res;
  // 被赋值的private value：下列 以及 near_goal_, farther_goal_, 
  shrink_dis_ = 40;   // 大小地图间的距离差值
  width_large_ = costmap->getSizeInCellsX();
  height_large_ = costmap->getSizeInCellsY();
  resolution_large_ = costmap->getResolution();
  origin_x_large_ = costmap->getOriginX();
  origin_y_large_ = costmap->getOriginY();
  // 大costmap的goal
  int mx = (local_plan.back().pose.position.x-origin_x_large_)/resolution_large_;
  int my = (local_plan.back().pose.position.y-origin_y_large_)/resolution_large_;
  farther_goal_ = Point2D(mx,my);

  //如果farther_goal在小local_map内，直接返回。考虑更大，但返回更早
  if (mx>=shrink_dis_-5 && mx<width_large_-shrink_dis_+5 && my>=shrink_dis_-5 && my<height_large_-shrink_dis_+5){
    farther_goal_.x = farther_goal_.x - shrink_dis_;
    farther_goal_.y = farther_goal_.y - shrink_dis_;
    for (auto p:local_plan){
      int mx = (p.pose.position.x-origin_x_large_)/resolution_large_;
      int my = (p.pose.position.y-origin_y_large_)/resolution_large_;
      if (mx>=shrink_dis_ && mx<width_large_-shrink_dis_ && my>=shrink_dis_ && my<height_large_-shrink_dis_){
          near_goal_ = Point2D(mx-shrink_dis_,my-shrink_dis_);
      }
    }
    res.push_back({near_goal_});
    goal_line_lists_ = res;
    return res;
  }

  // goal line是在小local map的边界处
  std::vector<std::vector<bool>> map_potential_goal = std::vector<std::vector<bool>> (width_large_, std::vector<bool>(height_large_, false));
  for (int i=shrink_dis_; i<width_large_-shrink_dis_; i++){
    map_potential_goal[i][shrink_dis_] = true;
    map_potential_goal[i][height_large_-1-shrink_dis_] = true;
  }
  for (int j=shrink_dis_; j<height_large_-shrink_dis_; j++){
    map_potential_goal[shrink_dis_][j] = true;
    map_potential_goal[width_large_-1-shrink_dis_][j] = true;
  }
  // 遍历local_plan，在map_potential_goal中求得小local map边界与goal的交点，得到near_goal
  Point2D_float near_goal_float;
  for (auto p:local_plan){
    int mx = (p.pose.position.x-origin_x_large_)/resolution_large_;
    int my = (p.pose.position.y-origin_y_large_)/resolution_large_;
    if (checkInMap(mx,my,width_large_,height_large_) && map_potential_goal[mx][my]){
        near_goal_ = Point2D(mx,my);
        near_goal_float = Point2D_float(p.pose.position.x, p.pose.position.y);
        break; 
    }
  }
  // 在map_potential_goal中用曼哈顿距离求near_goal附近点，仅仅near_goal的goal_lin_half_length距离以内的点会被视作候选goal line点 （若离得太远则改成false，扔掉）
  // ROS_INFO("================== %d vs %d", int(1.1*hypot(global_goal.first-near_goal_float.x, global_goal.second-near_goal_float.y)/resolution_large_), int(0.3*(width_large_-2*shrink_dis_)));

  int goal_line_half_length = std::min(int(1.1*hypot(global_goal.first-near_goal_float.x, global_goal.second-near_goal_float.y)/resolution_large_), int(0.3*(width_large_-2*shrink_dis_))) - 2 ;
  goal_line_half_length = std::max(1, goal_line_half_length );

  // int goal_line_half_length = std::max(1, int(0.3 * (width_large_-2*shrink_dis_)-2) );
  for (int x=0; x<width_large_; x++){
    for (int y=0; y<height_large_; y++){
      if (!map_potential_goal[x][y]) continue;
      // 计算曼哈顿距离
      int dis = std::abs(x-near_goal_.x) + std::abs(y-near_goal_.y);
      // 判断点(x,y)是否在near_goal附近
      if (dis >= goal_line_half_length)
        map_potential_goal[x][y] = false;
    }
  }
  // 遍历剩余的map_potential_goal，对值为true的所有点，求他们的x_min, x_max, y_min, y_max
  int x_min = width_large_-1, y_min = height_large_-1;
  int x_max = 0, y_max = 0;
  for (int i=0; i<width_large_; i++){
    for (int j=0; j<height_large_; j++){
      if (!map_potential_goal[i][j]) continue;
      x_min = std::min(x_min, i);
      y_min = std::min(y_min, j);
      x_max = std::max(x_max, i);
      y_max = std::max(y_max, j);
    }
  }
  // 用于记录goal_line的极限
  int goal_line_x_min=x_min, goal_line_x_max=x_max, goal_line_y_min=y_min, goal_line_y_max=y_max;
  // 以(width_*0.5, height_*0.5)为中心，判断father_goal所处的象限，把象限的角作用到min和max中，扩大后续从father_goal到goal_line的搜索范围
  if (farther_goal_.x<width_large_*0.5 && farther_goal_.y<height_large_*0.5){
    x_min = 0;
    y_min = 0;
  }
  else if (farther_goal_.x>width_large_*0.5 && farther_goal_.y<height_large_*0.5){
    x_max = width_large_-1;
    y_min = 0;
  }
  else if (farther_goal_.x<width_large_*0.5 && farther_goal_.y>height_large_*0.5){
    x_min = 0;
    y_max = height_large_-1;
  }
  else if (farther_goal_.x>width_large_*0.5 && farther_goal_.y>height_large_*0.5){
    x_max = width_large_-1;
    y_max = height_large_-1;
  }
  // 赋值三个map {大map，小map，goal map}，用于可视化
  map_boundaries_.push_back({{0,0}, {0,height_large_-1}, {width_large_-1,height_large_-1}, {width_large_-1,0}});
  map_boundaries_.push_back({{shrink_dis_,shrink_dis_}, {shrink_dis_,height_large_-1-shrink_dis_}, {width_large_-1-shrink_dis_,height_large_-1-shrink_dis_}, {width_large_-1-shrink_dis_,shrink_dis_}});
  map_boundaries_.push_back({{x_min,y_min}, {x_min,y_max}, {x_max,y_max}, {x_max,y_min}});

  // 提取局部小地图，由于(x_min,y_min), (y_min, y_max)
  // assign map
  unsigned char* data = costmap->getCharMap();
  std::vector<int8_t> map_vector;
  // assign static obstacles
  for (unsigned int i = 0; i < width_large_*height_large_; i++)
    map_vector.push_back(data[ i ]);

  // true则为obstacle
  int small_width = x_max-x_min+1;
  int small_height = y_max-y_min+1;
  std::vector<std::vector<bool>> map_small = std::vector<std::vector<bool>> (small_width, std::vector<bool>(small_height, false));
  for (int i=x_min; i<=x_max; i++){
    for (int j=y_min; j<=y_max; j++){
      if (map_vector[i+j*width_large_]==(char)0){
        map_small[i-x_min][j-y_min] = true;
      }
    }
  }

  // 创建小地图small_map，在后续中赋值map_obstacle_labeled和obs_list
  int label_index = 1;
  std::vector<std::vector<int>> map_obstacle_labeled;
  std::map<int, std::vector<Point2D>> obs_list;
  map_obstacle_labeled = std::vector<std::vector<int>> (small_width, std::vector<int>(small_height));

  // 斜对角遍历2D矩阵
  for (int x_iter=0; x_iter<small_height+small_width; x_iter++){
    // x+y的和 为 x_iter
    int x = std::min(x_iter, (int)small_width-1);
    int y = std::max((int)x_iter-x, 0);
    // 和不变时，x逐渐减小，y逐渐增大
    while (x>=0 && y<small_height){
      // 左上的四邻接 检测是否可以connect
      if (map_small[x][y])
        addObsConnect(x,y,map_obstacle_labeled,obs_list, label_index,small_width,small_height);
      x--;
      y++;
    }
  }

  // 顺时针赋值all_goal_line_points，先找到四个可行边界 ..._maintain
  int father_x_in_map = farther_goal_.x-x_min;
  father_x_in_map = std::max(0,father_x_in_map);
  father_x_in_map = std::min(father_x_in_map, small_width-1);
  int father_y_in_map = farther_goal_.y-y_min;
  father_y_in_map = std::max(0,father_y_in_map);
  father_y_in_map = std::min(father_y_in_map, small_height-1);
  // ROS_INFO("(%d,%d): (%d,%d) - (%d,%d)", map_obstacle_labeled.size(), map_obstacle_labeled.front().size(), x_min,y_min, farther_goal_.x,farther_goal_.y);
  int farther_goal_list_index = map_obstacle_labeled[father_x_in_map][father_y_in_map];
  std::vector<Point2D> all_goal_line_points;
  std::vector<Point2D> y_max_maintain;
  std::vector<Point2D> x_max_maintain;
  std::vector<Point2D> y_min_maintain;
  std::vector<Point2D> x_min_maintain;
  // 注意costmap的坐标系，是以机器人当前朝向为正x的右手系
  for (int x = goal_line_x_min+1; x<=goal_line_x_max; x++){
    int y = goal_line_y_max;
    if (map_potential_goal[x][y]){
      y_max_maintain.push_back(Point2D(x,y));
    }
  }
  for (int y = goal_line_y_max-1; y>=goal_line_y_min; y--){
    int x = goal_line_x_max;
    if (map_potential_goal[x][y]){
      x_max_maintain.push_back(Point2D(x,y));
    }
  }
  for (int x = goal_line_x_max-1; x>=goal_line_x_min; x--){
    int y = goal_line_y_min;
    if (map_potential_goal[x][y]){
      y_min_maintain.push_back(Point2D(x,y));
    }
  }  
  for (int y = goal_line_y_min+1; y<=goal_line_y_max; y++){
    int x = goal_line_x_min;
    if (map_potential_goal[x][y]){
      x_min_maintain.push_back(Point2D(x,y));
    }
  }
  // 确定起点、顺序、叠加问题，共 四 * 2 种情况：
  // CASE1: father_goal的引入并未带来x_min和y_min的改变： 如果goal_line_x_min==goal_lin_x_max或goal_line_y_min==goal_lin_y_max，则只添加对应的就好，否则从y_min_maintain开始
  if (x_min==goal_line_x_min && y_min==goal_line_y_min){
    if (goal_line_x_min==goal_line_x_max){
      all_goal_line_points=x_max_maintain;
    }
    else if (goal_line_y_min==goal_line_y_max){
      all_goal_line_points=y_max_maintain;
    }
    else{
      all_goal_line_points.insert(all_goal_line_points.end(), x_min_maintain.begin(), x_min_maintain.end());
      all_goal_line_points.insert(all_goal_line_points.end(), y_max_maintain.begin(), y_max_maintain.end());
      all_goal_line_points.insert(all_goal_line_points.end(), x_max_maintain.begin(), x_max_maintain.end());
      all_goal_line_points.insert(all_goal_line_points.end(), y_min_maintain.begin(), y_min_maintain.end());
    }
  }
  // CASE2: father_goal的引入并未带来x_min和y_max变： 如果goal_line_x_min==goal_line_x_max或goal_line_y_min==goal_line_y_max，则只添加对应的就好，否则从x_min_maintain开始
  else if (x_min==goal_line_x_min && y_max==goal_line_y_max){
    if (goal_line_x_min==goal_line_x_max){
      all_goal_line_points=x_max_maintain;
    }
    else if (goal_line_y_min==goal_line_y_max){
      all_goal_line_points=y_min_maintain;
    }
    else{
      all_goal_line_points.insert(all_goal_line_points.end(), y_max_maintain.begin(), y_max_maintain.end());
      all_goal_line_points.insert(all_goal_line_points.end(), x_max_maintain.begin(), x_max_maintain.end());
      all_goal_line_points.insert(all_goal_line_points.end(), y_min_maintain.begin(), y_min_maintain.end());
      all_goal_line_points.insert(all_goal_line_points.end(), x_min_maintain.begin(), x_min_maintain.end());
    }
  }
  // CASE3: father_goal的引入并未带来x_max和y_min变： 如果goal_line_x_min==goal_line_x_max或goal_line_y_min==goal_line_y_max，则只添加对应的就好，否则从x_max_maintain开始
  else if(x_max==goal_line_x_max && y_min==goal_line_y_min){
    if (goal_line_x_min==goal_line_x_max)
      all_goal_line_points=x_min_maintain;
    else if (goal_line_y_min==goal_line_y_max)
      all_goal_line_points=y_max_maintain;
    else{
      all_goal_line_points.insert(all_goal_line_points.end(), y_min_maintain.begin(), y_min_maintain.end());
      all_goal_line_points.insert(all_goal_line_points.end(), x_min_maintain.begin(), x_min_maintain.end());
      all_goal_line_points.insert(all_goal_line_points.end(), y_max_maintain.begin(), y_max_maintain.end());
      all_goal_line_points.insert(all_goal_line_points.end(), x_max_maintain.begin(), x_max_maintain.end());
    }
  }
  // CASE4: father_goal的引入并未带来x_max和y_max变： 如果goal_line_x_min==goal_line_x_max或goal_line_y_min==goal_line_y_max，则只添加对应的就好，否则从y_max_maintain开始
  else if (x_max==goal_line_x_max && y_max==goal_line_y_max){
    if (goal_line_x_min==goal_line_x_max){
      all_goal_line_points=x_min_maintain;
    }
    else if (goal_line_y_min==goal_line_y_max){
      all_goal_line_points=y_min_maintain;
    }
    else{
      all_goal_line_points.insert(all_goal_line_points.end(), x_max_maintain.begin(), x_max_maintain.end());
      all_goal_line_points.insert(all_goal_line_points.end(), y_min_maintain.begin(), y_min_maintain.end());
      all_goal_line_points.insert(all_goal_line_points.end(), x_min_maintain.begin(), x_min_maintain.end());
      all_goal_line_points.insert(all_goal_line_points.end(), y_max_maintain.begin(), y_max_maintain.end());
    }
  }

  // 如果goal_line被障碍物截断，则把all_goal_line_points给拆成多份
  std::vector<Point2D> one_goal_line;
  // 找到四个角里在map_obstacle_labeled中的那个，设置为0，便于后续的截断  
  int x1=shrink_dis_-x_min, y1=shrink_dis_-y_min;
  int x2=width_large_-1-shrink_dis_-x_min, y2=height_large_-1-shrink_dis_-y_min;
  int temp_x,temp_y;
  if (0<=x1 && x1<small_width)
    temp_x = x1;
  else
    temp_x = x2;
  if (0<=y1 && y1<small_height)
    temp_y = y1;
  else
    temp_y = y2;
  // goal lines被障碍物或角截断
  for (auto iter=all_goal_line_points.begin(); iter!=all_goal_line_points.end(); iter++){
    if (map_obstacle_labeled[iter->x-x_min][iter->y-y_min] == farther_goal_list_index && !(iter->x-x_min==temp_x && iter->y-y_min==temp_y)){
      one_goal_line.push_back(*iter);
    }
    else{
      if(!one_goal_line.empty()){
        // 长度大于等于3的goal line才会被考虑，因为它会被收缩边界
        if (one_goal_line.size()>=3){
          one_goal_line.erase(one_goal_line.begin());
          one_goal_line.pop_back();
          res.push_back(one_goal_line);
        }
        one_goal_line.clear();
      }
    }
  }
  // 加入最后一个one_goal_line
  if(!one_goal_line.empty()){
    // 长度大于等于3的goal line才会被考虑，因为它会被收缩边界
    if (one_goal_line.size()>=3){
      one_goal_line.erase(one_goal_line.begin());
      one_goal_line.pop_back();
      res.push_back(one_goal_line);
    }
    // 到全局目标附近啦
    else{
      res.push_back(one_goal_line);
    }
    one_goal_line.clear();
  }


  // 将res转换到小地图下
  for (auto iter_o=res.begin(); iter_o!=res.end(); iter_o++){
    for (auto iter_i=iter_o->begin(); iter_i!=iter_o->end(); iter_i++){
      iter_i->x -= shrink_dis_;
      iter_i->y -= shrink_dis_;
    }
  }
  near_goal_.x = near_goal_.x - shrink_dis_;
  near_goal_.y = near_goal_.y - shrink_dis_;
  farther_goal_.x = farther_goal_.x - shrink_dis_;
  farther_goal_.y = farther_goal_.y - shrink_dis_;
  goal_line_lists_ = res;
  return res;
}

// initialize maps derived from costmap
void mapProcess::initialize(costmap_2d::Costmap2D* costmap, const teb_local_planner::PoseSE2& start, const teb_local_planner::PoseSE2& goal, const std::vector<geometry_msgs::PoseStamped>& local_plan, const std::vector<std::vector<double>>& dynamic_obstacle, std::pair<double,double> global_goal, std::pair<double,double> velocity_in_map){
  // 获得下costmap下的goal lines
  dynamic_obstacle_ = dynamic_obstacle;
  costmap_ = costmap;
  width_ = costmap_->getSizeInCellsX() - shrink_dis_*2;
  height_ = costmap_->getSizeInCellsY() - shrink_dis_*2;
  resolution_ = costmap_->getResolution();
  origin_x_ = costmap_->getOriginX() + shrink_dis_*resolution_;
  origin_y_ = costmap_->getOriginY() + shrink_dis_*resolution_;
  velocity_in_map_ = velocity_in_map;
  // initialize vectors
  map_obs_labeled_ = std::vector<std::vector<int>> (width_, std::vector<int>(height_));
  map_obs_occupied_without_ignore_ = std::vector<std::vector<bool>> (width_, std::vector<bool>(height_, false));
  map_border_labeled_ = std::vector<std::vector<int>> (width_, std::vector<int>(height_));
  map_corner_labeled_ = std::vector<std::vector<int>> (width_, std::vector<int>(height_));
  map_is_dynamic_ = std::vector<std::vector<bool>> (width_, std::vector<bool>(height_, false));
  map_vel_x_ = std::vector<std::vector<float>> (width_, std::vector<float>(height_, 0));
  map_vel_y_ = std::vector<std::vector<float>> (width_, std::vector<float>(height_, 0));
  // 1. 求整个map_process过程中用到的small map: 
  //     忽略四周的ignore_dis距离的地图 map_char [ 障碍物点='d'，非障碍物点=(char)0 ]
  //     不忽略四周的ignore_dis距离的地图 map_obs_occupied_without_ignore_ [ 障碍物点=true，非障碍物点=false ]
  unsigned char* data = costmap_->getCharMap();
  std::vector<uint8_t> map_char;
  ignore_dis_ = 4;   // 忽略的长度
  for (int y=shrink_dis_; y<=height_large_-shrink_dis_-1; y++){
    for (int x=shrink_dis_; x<=width_large_-shrink_dis_-1; x++){
      if (x<shrink_dis_+ignore_dis_ || x>width_large_-shrink_dis_-ignore_dis_-1 || y<shrink_dis_+ignore_dis_ || y>height_large_-shrink_dis_-ignore_dis_-1)
        map_char.push_back((char)0);
      else{
        if(data[ x + y*width_large_ ]!=(char)0)
          map_char.push_back('d');
        else
          map_char.push_back((char)0);
      }
      if (data[x+y*width_large_]!=(char)0)
        map_obs_occupied_without_ignore_[x-shrink_dis_][y-shrink_dis_] = true;
    }
  }
  // 2. project dynamic pedestrians onto the map_char and map_is_dynamic_
  length_in_map_ = 0;
  if (!dynamic_obstacle_.empty()){
    length_in_map_ = dynamic_obstacle_.front().back()/resolution_;
    // ROS_INFO("===-=-= %f",length_in_map_);
    for (auto p:dynamic_obstacle_){
      int mx, my;
      std::pair<double,double> robot2goal = {global_goal.first-start.position().x(), global_goal.second-start.position().y()};
      std::pair<double,double> robot2human = {p[0]-start.position().x(), p[1]-start.position().y()};
      if (hypot(robot2human.first, robot2human.second)>=8)
        continue;
      auto robot2human_project_on_robot2goal = (robot2goal.first*robot2human.first + robot2goal.second*robot2human.second)/(std::hypot(robot2goal.first, robot2goal.second));
      // ROS_INFO("===-=-= %f: (%f,%f) (%f,%f)",robot2human_project_on_robot2goal, robot2goal.first, robot2goal.second, robot2human.first, robot2human.second);
      if (robot2human_project_on_robot2goal<-0.5) continue;   // 如果人落后于机器人0.5米以上，则忽略
      // ROS_INFO("    retained");
      world2map(mx,my,p[0],p[1]);
      // 并清空动态障碍物周围的两个单位resolution的障碍物，以防止临近的动态障碍物和静态障碍物被合并到一起
      int peddle_dis = 2;
      for (int dx=-length_in_map_-peddle_dis; dx<=length_in_map_+peddle_dis; dx++){
        for (int dy=-length_in_map_-peddle_dis; dy<=length_in_map_+peddle_dis; dy++){
          // 在地图内
          if (mx+dx>=ignore_dis_ && mx+dx<=width_-ignore_dis_ && my+dy>=ignore_dis_ && my+dy<=height_-ignore_dis_){
            // 将动态障碍物project到map_vector_和map_is_dynamic_上
            if (dx>=-length_in_map_ && dx<=length_in_map_ && dy>=-length_in_map_ && dy<=length_in_map_){
              map_char[mx+dx+(my+dy)*width_] = 'd';
              map_is_dynamic_[mx+dx][my+dy] = true;
              map_vel_x_[mx+dx][my+dy] = p[2];
              map_vel_y_[mx+dx][my+dy] = p[3];
            }
            // clear动态障碍物周围peddle_dis个像素
            else{
              map_char[mx+dx+(my+dy)*width_] = (char)0;
              map_is_dynamic_[mx+dx][my+dy] = false;              
            }
          }
        }
      }
    }
  }

  // 1. 把map_int转化成二维的map_cv_，进行腐蚀运算，使得后续得到的boundary的 也是obstacle occupied的
  map_cv_ = convertVector2Mat(map_char, 1, height_);
  cv::Mat kernel = (cv::Mat_<uchar>(3,3) << 0,1,0,1,1,1,0,1,0);
  cv::erode(map_cv_, map_cv_, kernel);

  // 2. 同样的 计算一个不包含行人的、包括大large map中所有静态障碍物的map，用于给teb提供static obstacle gourps的polygons
  std::vector<uint8_t> map_int_without_pedestrians;
  for (int y=0; y<height_large_; y++){
    for (int x=0; x<width_large_; x++){
      if (x<shrink_dis_+ignore_dis_ || x>width_large_-shrink_dis_-ignore_dis_-1 || y<shrink_dis_+ignore_dis_ || y>height_large_-shrink_dis_-ignore_dis_-1)
        map_int_without_pedestrians.push_back((char)0);
      else{
        if(data[ x + y*width_large_ ]!=(char)0)
          map_int_without_pedestrians.push_back('d');
        else
          map_int_without_pedestrians.push_back((char)0);
      }
    }
  }
  map_cv_without_dynamic_obs_ = convertVector2Mat(map_int_without_pedestrians, 1, height_large_);
    
  // translate start and goal points into map frame (resolution)
  world2map(start_in_map_x_, start_in_map_y_, start.position().x(), start.position().y());
  goal_in_map_x_ = near_goal_.x;
  goal_in_map_y_ = near_goal_.y;
}


std::map<int, std::vector<Point2D>> mapProcess::clusterObs(){
  // 斜对角遍历2D矩阵
  label_index_ = 1;
  for (int x_iter=0; x_iter<width_+height_; x_iter++){
    // x+y的和 为 x_iter
    int x = std::min(x_iter, (int)width_-1);
    int y = std::max((int)x_iter-x, 0);
    // 和不变时，x逐渐减小，y逐渐增大
    while (x>=0 && y<height_){
      // 左上的四邻接 检测是否可以connect
      if (map_cv_.at<uchar>(y,x)=='d'){
        addObsConnect(x,y);
      }
      x--;
      y++;
    }
  }
  // 记录所有label
  for (const auto& i:obs_list_)
    label_list_.push_back(i.first);
  return obs_list_;
}

void mapProcess::addObsConnect(int x, int y){
  bool is_connected = false;
  // 右下检测 看是否可以直接connect
  if (checkInMap(x+1,y-1) && map_obs_labeled_[x+1][y-1]!=0){
    map_obs_labeled_[x][y] = map_obs_labeled_[x+1][y-1];
    is_connected = true;
  }
  int old_label, new_label;
  // 正下检测（归并检测）
  if(checkInMap(x,y-1) && map_obs_labeled_[x][y-1]!=0){
    // 归并
    old_label = map_obs_labeled_[x][y];
    new_label = map_obs_labeled_[x][y-1];
    if (is_connected && old_label!=new_label){
      auto temp = obs_list_[old_label];
      // 转存obs_list_中 map_obs_labeled_[x][y]中的元素 到labeled_[x][y-1]中
      obs_list_.erase(old_label);
      obs_list_[new_label].insert(obs_list_[new_label].end(),temp.begin(),temp.end());
      // 改变地图
      for (auto p:temp){
        map_obs_labeled_[p.x][p.y]=new_label;
      }
    }
    map_obs_labeled_[x][y] = map_obs_labeled_[x][y-1];
    is_connected = true;
  }  
  // 左下检测（归并检测）
  if (checkInMap(x-1,y-1) && map_obs_labeled_[x-1][y-1]!=0){
    old_label = map_obs_labeled_[x][y];
    new_label = map_obs_labeled_[x-1][y-1];
    if (is_connected && old_label!=new_label){
      auto temp = obs_list_[old_label];
      obs_list_.erase(old_label);
      obs_list_[new_label].insert(obs_list_[new_label].end(),temp.begin(),temp.end());
      for (auto p:temp){
        map_obs_labeled_[p.x][p.y]=new_label;
      }
    }
    map_obs_labeled_[x][y] = map_obs_labeled_[x-1][y-1];
    is_connected = true;
  }
  // 左侧检测（归并检测）
  if (checkInMap(x-1,y) && map_obs_labeled_[x-1][y]!=0){
    old_label = map_obs_labeled_[x][y];
    new_label = map_obs_labeled_[x-1][y];
    if (is_connected && old_label!=new_label){
      auto temp = obs_list_[old_label];
      obs_list_.erase(old_label);
      obs_list_[new_label].insert(obs_list_[new_label].end(),temp.begin(),temp.end());
      for (auto p:temp){
        map_obs_labeled_[p.x][p.y]=new_label;
      }
    }
    map_obs_labeled_[x][y] = map_obs_labeled_[x-1][y];
    is_connected = true;
  }
  // 如果不能connect，则直接开辟一个新的障碍物群
  if (!is_connected){
    map_obs_labeled_[x][y] = label_index_++;
  }
  // 将障碍物点添加到对应的obs_list中
  obs_list_[map_obs_labeled_[x][y]].push_back(Point2D(x,y,0,map_obs_labeled_[x][y]));
}

void mapProcess::addObsConnect(int x, int y, std::vector<std::vector<int>>& map_obs_labeled, std::map<int, std::vector<Point2D>>& obs_list, int& label_index, const int& width, const int& height){
  bool is_connected = false;
  // 右下检测 看是否可以直接connect
  if (checkInMap(x+1,y-1,width,height) && map_obs_labeled[x+1][y-1]!=0){
    map_obs_labeled[x][y] = map_obs_labeled[x+1][y-1];
    is_connected = true;
  }
  int old_label, new_label;
  // 正下检测（归并检测）
  if(checkInMap(x,y-1,width,height) && map_obs_labeled[x][y-1]!=0){
    // 归并
    old_label = map_obs_labeled[x][y];
    new_label = map_obs_labeled[x][y-1];
    if (is_connected && old_label!=new_label){
      auto temp = obs_list[old_label];
      // 转存obs_list_中 map_obs_labeled[x][y]中的元素 到labeled_[x][y-1]中
      obs_list.erase(old_label);
      obs_list[new_label].insert(obs_list[new_label].end(),temp.begin(),temp.end());
      // 改变地图
      for (auto p:temp){
        map_obs_labeled[p.x][p.y]=new_label;
      }
    }
    map_obs_labeled[x][y] = map_obs_labeled[x][y-1];
    is_connected = true;
  }  
  // 左下检测（归并检测）
  if (checkInMap(x-1,y-1,width,height) && map_obs_labeled[x-1][y-1]!=0){
    old_label = map_obs_labeled[x][y];
    new_label = map_obs_labeled[x-1][y-1];
    if (is_connected && old_label!=new_label){
      auto temp = obs_list[old_label];
      obs_list.erase(old_label);
      obs_list[new_label].insert(obs_list[new_label].end(),temp.begin(),temp.end());
      for (auto p:temp){
        map_obs_labeled[p.x][p.y]=new_label;
      }
    }
    map_obs_labeled[x][y] = map_obs_labeled[x-1][y-1];
    is_connected = true;
  }
  // 左侧检测（归并检测）
  if (checkInMap(x-1,y,width,height) && map_obs_labeled[x-1][y]!=0){
    old_label = map_obs_labeled[x][y];
    new_label = map_obs_labeled[x-1][y];
    if (is_connected && old_label!=new_label){
      auto temp = obs_list[old_label];
      obs_list.erase(old_label);
      obs_list[new_label].insert(obs_list[new_label].end(),temp.begin(),temp.end());
      for (auto p:temp){
        map_obs_labeled[p.x][p.y]=new_label;
      }
    }
    map_obs_labeled[x][y] = map_obs_labeled[x-1][y];
    is_connected = true;
  }
  // 如果不能connect，则直接开辟一个新的障碍物群
  if (!is_connected){
    map_obs_labeled[x][y] = label_index++;
  }
  // 将障碍物点添加到对应的obs_list中
  obs_list[map_obs_labeled[x][y]].push_back(Point2D(x,y,0,map_obs_labeled[x][y]));
}

std::map<int, std::vector<Point2D>> mapProcess::dilateObs(){
  // 1. 建立一个map_obs_outlined，derived from map_obs_labled, and outline it by 999
  auto map_obs_outlined = map_obs_labeled_;
  for (int x=0; x<width_; x++){
    map_obs_outlined[x][0] = 999;
    map_obs_outlined[x][height_-1] = 999;
  }
  for (int y=0; y<height_; y++){
    map_obs_outlined[0][y] = 999;
    map_obs_outlined[width_-1][y] = 999;
  }
  map_obs_outlined[start_in_map_x_][start_in_map_y_] = 999;
  // 2. 求map_nonvoronoi_labeled: 除了voronoi point的value=0，其他都是由map_obs_labeled_扩展的ID
  DynamicVoronoi voronoi;
  voronoi.initializeMap(map_obs_outlined);
  voronoi.update(); 
  voronoi.prune();
  std::vector<std::vector<int>> map_nonvoronoi_labeled = std::vector<std::vector<int>> (width_, std::vector<int>(height_));
  for (int x=0; x<voronoi.getSizeX(); x++){
    for (int y=0; y<voronoi.getSizeY(); y++){
      if (!voronoi.isVoronoi(x,y)){
        map_nonvoronoi_labeled[x][y] = map_obs_labeled_[voronoi.getDataCell(x,y).obstX][voronoi.getDataCell(x,y).obstY];
      }
    }
  }
  // 3. 为每个obstacle group，利用闭运算 求得其闭运算后的local map: resultMat
  // 对resultMat和map_nonvoronoi_labeled取交集，得到max-convex的obstacle group map
  map_dilated_obs_labeled_ = std::vector<std::vector<int>> (width_, std::vector<int>(height_));
  for (auto ID:label_list_){
    // 对每个obstacle group求它的矩形边界
    int min_x=width_, max_x=0, min_y=height_, max_y=0;
    for (auto p:obs_list_[ID]){
      if (p.x<min_x)
        min_x = p.x;
      if (p.y<min_y)
        min_y = p.y;
      if (p.x>max_x)
        max_x = p.x;
      if (p.y>max_y)
        max_y = p.y;
    }
    int length_x = max_x - min_x + 1;
    int length_y = max_y - min_y + 1;
    // 构建局部小地图matBool，包含obs和border点。 对matBool进行闭运算
    cv::Mat matBool(3*length_x, 3*length_y, CV_8U, cv::Scalar(0));
    for (auto p:obs_list_[ID]){
      matBool.at<uchar>(p.x-min_x+length_x, p.y-min_y+length_y) = 1;
    }
    // 卷积核需为奇数
    int knl_x = length_x;
    int knl_y = length_y;
    if (knl_x%2==0)
      knl_x++;
    if (knl_y%2==0)
      knl_y++;
    // 为啥kernel中要用knl_y, knl_x 而不是knl_x, knl_y? 因为opencv的kernel是行列颠倒的，所以行列颠倒一下就好。
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(knl_y, knl_x));
    cv::Mat resultMat;
    cv::morphologyEx(matBool, resultMat, cv::MORPH_CLOSE, kernel);

    // 4. 求map_dilated_obs_labled_和dilated_obs_list_。点需要满足：1. 周围一圈没有voronoi point，2. resultMat为1，3. map_nonvoronoi_labeled也为ID.
    std::vector<Point2D> delta_set = {Point2D(-1,-1),Point2D(-1,0),Point2D(-1,1),Point2D(0,-1),Point2D(0,1),Point2D(1,-1),Point2D(1,0),Point2D(1,1)};
    for (int i=length_x; i<length_x*2; i++){
      for (int j=length_y; j<length_y*2;j++){
        int local_map_x = i+min_x-length_x;
        int local_map_y = j+min_y-length_y;
        // 保存
        bool is_voronoi_point_around = false;
        for (auto del:delta_set){
          if (map_nonvoronoi_labeled[local_map_x+del.x][local_map_y+del.y]==0){
            is_voronoi_point_around = true;
            break;
          }
        }
        if (!is_voronoi_point_around && resultMat.at<uchar>(i,j)==1 && map_nonvoronoi_labeled[local_map_x][local_map_y]==ID){
          dilated_obs_list_[ID].push_back(Point2D(local_map_x, local_map_y));
          map_dilated_obs_labeled_[local_map_x][local_map_y] = ID;
        }
      }
    }
  }
  return dilated_obs_list_;
}

std::map<int, std::vector<Point2D>>mapProcess::borderIdentify(){
  std::vector<uint8_t> map_char;
  for (int j=0; j<height_; j++){
    for (int i=0; i<width_; i++){
      if (map_dilated_obs_labeled_[i][j]!=0)
        map_char.push_back('d');
      else
        map_char.push_back((char)0);
    }
  }
  map_cv_ = convertVector2Mat(map_char, 1, height_);

  cv::Mat edge_map = map_cv_;
  cv::Mat dst, comp;
  cv::dilate(map_cv_, dst, cv::Mat());
  cv::compare(edge_map, dst, comp, cv::CMP_NE);
  // cv::imshow("Canny edge detection", comp);
  // cv::waitKey(0);

  // 这个循环用于保存map_border_labeled_这个地图
  std::vector<Point2D> kernel = {Point2D(-1,-1),Point2D(-1,0),Point2D(-1,1),Point2D(0,-1),Point2D(0,1),Point2D(1,-1),Point2D(1,0),Point2D(1,1)};
  for(int j=0; j < comp.rows; j++){
    for(int i=0; i<comp.cols; i++){
      // 只要comp中的元素是非空的，那么它就是边缘点
      if(comp.at<uint8_t>(j,i)){
        // 八临接找label
        int label = 0;
        for (auto delta:kernel){
          if (checkInMap(i+delta.x, j+delta.y) && map_dilated_obs_labeled_[i+delta.x][j+delta.y]!=0){
            label = map_dilated_obs_labeled_[i+delta.x][j+delta.y];
            break;
          }
        }
        // 添加border点到border map中
        map_border_labeled_[i][j] = label;
      }
    }
  }

  // 扩展归并，拟合border_list
  auto map_is_visited_temp = std::vector<std::vector<bool>> (width_, std::vector<bool>(height_, false));
  // 遍历时的顺序为：左、上、左上、下、左下、右、右上、右下。即：上与右优先遍历、非斜线优先遍历，保证borderlist是顺时针的
  std::vector<Point2D> kernel2 = {Point2D(-1,0),Point2D(0,1),Point2D(-1,1),Point2D(0,-1),Point2D(-1,-1),Point2D(1,0),Point2D(1,1),Point2D(1,-1)};
  for (int y=0; y<dst.rows; y++){
    for (int x=0; x<dst.cols; x++){
      auto label = map_border_labeled_[x][y];
      // 如果是非border点或者被visited过的点，则continue
      if (label == 0 || map_is_visited_temp[x][y]==true)
        continue;
      // 标记当前点为已读
      Point2D p = Point2D(x,y);
      map_is_visited_temp[p.x][p.y] = true;
      border_list_[label].push_back(Point2D(x,y,0,label));
      // 深搜，进行归并
      deepSearchForFindingBorders(p, map_is_visited_temp, kernel2, p, label);
    }
  }

  // 剔除剩余的冗余border点，即剔除那些距离上一个点超过4个像素点的点
  for (auto border=border_list_.begin(); border!=border_list_.end();border++){
    auto iter = border->second.begin();
    while (iter!=border->second.end()){
      auto iter_next = std::next(iter);
      if (iter_next == border->second.end())
        break;
      if (abs(iter->x-iter_next->x)+abs(iter->y-iter_next->y)>4){
        border->second.erase(iter_next, border->second.end());
        break;
      }
      iter++;
    }
  }

  return border_list_;
}

// 对一个点的八邻街进行深搜，找到整个label形成的border list。 
bool mapProcess::deepSearchForFindingBorders(Point2D current, std::vector<std::vector<bool>>& map_is_visited_temp, const std::vector<Point2D>& kernel, const Point2D& start, const int& label){
  for (auto k:kernel){
    if (current.x+k.x==start.x &&current.y+k.y==start.y){
      return true;
    }
    if (checkInMap(current.x+k.x, current.y+k.y) &&map_border_labeled_[current.x+k.x][current.y+k.y]==label && map_is_visited_temp[current.x+k.x][current.y+k.y] == false){
      Point2D current_new(current.x, current.y);
      current_new.x = current.x+k.x;
      current_new.y = current.y+k.y;
      map_is_visited_temp[current_new.x][current_new.y] = true;
      border_list_[label].push_back(Point2D(current_new.x,current_new.y,0,label));
      if (deepSearchForFindingBorders(current_new, map_is_visited_temp, kernel, start, label))
        return true;
    }
  }
  return false;
}


bool mapProcess::deepSearchForFindingBorders(std::map<int, std::vector<Point2D>>& border_list_temp, std::vector<std::vector<int>>& map_border_labeled_temp, Point2D current, std::vector<std::vector<bool>>& map_is_visited_temp, const std::vector<Point2D>& kernel, const Point2D& start, const int& label){
  for (auto k:kernel){
    if (current.x+k.x==start.x &&current.y+k.y==start.y){
      return true;
    }
    if (checkInMap(current.x+k.x, current.y+k.y) &&map_border_labeled_temp[current.x+k.x][current.y+k.y]==label && map_is_visited_temp[current.x+k.x][current.y+k.y] == false){
      Point2D current_new(current.x, current.y);
      current_new.x = current.x+k.x;
      current_new.y = current.y+k.y;
      map_is_visited_temp[current_new.x][current_new.y] = true;
      border_list_temp[label].push_back(Point2D(current_new.x,current_new.y,0,label));
      if (deepSearchForFindingBorders(border_list_temp, map_border_labeled_temp, current_new, map_is_visited_temp, kernel, start, label))
        return true;
    }
  }
  return false;
}


std::map<int, std::vector<Point2D>> mapProcess::cornerIdentify(){
  std::vector<Point2D> delta_set = {Point2D(-1,-1),Point2D(-1,0),Point2D(-1,1),Point2D(0,1),Point2D(1,1),Point2D(1,0),Point2D(1,-1),Point2D(0,-1)};
  for (auto iter_o = border_list_.begin(); iter_o!=border_list_.end();iter_o++){
    int ID = iter_o->first;
    auto border_points = iter_o->second;
    // 如果borner点太少的话 就直接全要了
    if (border_points.size()<=4){
      corner_list_[ID] = border_points;
      for (auto p:border_points)
        map_corner_labeled_[p.x][p.y] = ID;
    }
    // 如果点比较多，就进行筛选，选出corner点
    else{
      std::vector<Point2D> corner_points = {border_points[0]};
      float k_larger = 999;
      float k_smaller = -999;
      bool is_initialized = false;
      bool keep_positive = false;   // 如果夹角在pi和-pi时分界时，需要保证k_tan是正的
      for (int j=1; j<border_points.size()+1; j++){
        // 求当前point下的斜率角k
        int i = j%border_points.size();
        float k_tan = atan2(border_points[i].y-corner_points.back().y, border_points[i].x-corner_points.back().x);
        if (keep_positive && k_tan<0)
          k_tan += 2*3.14;
        // 如果已经初始化过了，那么就判断当前点是否为corner点，标准是：当前斜率超出了历史累计的斜率角限制
        if ( is_initialized ){          
          if (k_tan>k_larger || k_tan<k_smaller){
            corner_points.push_back(border_points[(i-1)%border_points.size()]);
            k_larger = 999;
            k_smaller = -999;
            is_initialized = false;
            keep_positive = false;
          }
        }
        // 求当前点的前一个点的方向向量，然后求出与之相邻的两个非选择点，并计算这两个非选择点的斜率角k_tan1和k_tan2
        auto last_point = border_points[(i-1)%border_points.size()];
        auto direction = Point2D(border_points[i].x-last_point.x, border_points[i].y-last_point.y);
        int idx = -1;
        for (int k =0; k<delta_set.size(); k++){
          if (direction.x==delta_set[k].x && direction.y==delta_set[k].y){
            idx = k;
            break;
          }
        }
        Point2D not_selected_point1;
        not_selected_point1.x = last_point.x + delta_set[(idx+1)%delta_set.size()].x;
        not_selected_point1.y = last_point.y + delta_set[(idx+1)%delta_set.size()].y;
        float k_tan1 = atan2(not_selected_point1.y-corner_points.back().y, not_selected_point1.x-corner_points.back().x);
        Point2D not_selected_point2;
        not_selected_point2.x = last_point.x + delta_set[(idx-1)%delta_set.size()].x;
        not_selected_point2.y = last_point.y + delta_set[(idx-1)%delta_set.size()].y;
        float k_tan2 = atan2(not_selected_point2.y-corner_points.back().y, not_selected_point2.x-corner_points.back().x);
        // atan2会在[-pi,pi]，当k_tan1和k_tan2在pi和-pi处分界时，会出bug：
        //    在第一次扩展时，如果k_tan1和k_tan2的差值大于pi/2，则当前整个迭代都会在pi附近，此时设keep_positive=true，后续每次求k_tan时 如果k_tan是负的 那么它需要加上2pi，以保证正值。
        if (is_initialized==false && abs(k_tan1 - k_tan2)>3.0){
          keep_positive = true;
        }
        if (keep_positive){
          if (k_tan1<0)
            k_tan1 += 2*3.14;
          if (k_tan2<0)
            k_tan2 += 2*3.14;
        }
        // 根据k_tan1和k_tan2的大小关系，增量式地更新k_larger和k_smaller
        if (k_tan1 > k_tan2){
          k_larger = std::min(k_larger, k_tan1);
          k_smaller = std::max(k_smaller, k_tan2);
        }
        else{
          k_larger = std::min(k_larger, k_tan2);
          k_smaller = std::max(k_smaller, k_tan1);
        }
        is_initialized = true;
      }
      // 筛选出来的corner点，加入到最终的列表和地图中
      corner_list_[ID] = corner_points;
      for (auto p:corner_points)
        map_corner_labeled_[p.x][p.y] = ID;
    }
  }
  return corner_list_;
}

std::vector<Point2D> mapProcess::getPathAlongsideObs(int obsID, std::string traj_type, Point2D P_start, Point2D P_target){
  // 特殊情况 如果相等的话 直接返回
  if (P_start.x==P_target.x && P_start.y==P_target.y)
    return {P_start};

  // 正常查找
  assert(traj_type=="clockwise" || traj_type=="counterclockwise");
  int index_start=-1, index_target=-1;
  const auto& vector_list = border_list_[obsID];
  for (int i=0; i<vector_list.size(); i++){
    if (vector_list[i].x==P_start.x && vector_list[i].y==P_start.y){
      index_start = i;
    }
    if (vector_list[i].x==P_target.x && vector_list[i].y==P_target.y){
      index_target = i;
    }
  }
  assert(index_start!=-1);
  assert(index_target!=-1);

  if (traj_type=="clockwise"){
    std::vector<Point2D> clockwise;
    if (index_target >= index_start){
      for (int j=index_start; j<=index_target; j++)
        clockwise.push_back(vector_list[j]);
    }
    else{
      for (int j=index_start; j<=vector_list.size()-1; j++)
        clockwise.push_back(vector_list[j]);
      for (int j=0; j<=index_target; j++)
        clockwise.push_back(vector_list[j]);
    }
    // 清除corner点间的border点
    int last_corner_index = -1;
    int i=0;
    while (i<clockwise.size()){
      if (map_corner_labeled_[clockwise[i].x][clockwise[i].y] == obsID){
        if (last_corner_index==-1 || last_corner_index+1==i){
          last_corner_index = i;
          // 如果起点可以直接连过来 就先清除一步分
          if (i>1){
            auto vec_line = Bresenham(clockwise[0], clockwise[i]);
            bool is_erase = true;
            for (auto p:vec_line){
              if(map_obs_labeled_[p.x][p.y]!=0){
                is_erase = false;
                break;
              }
            }
            if (is_erase){
              clockwise.erase(clockwise.begin()+1, clockwise.begin()+i);
              last_corner_index = 1 ;
              i = 1;
            }
          }
        }
        else{
          clockwise.erase(clockwise.begin()+last_corner_index+1, clockwise.begin()+i);
          last_corner_index ++ ;
          i = last_corner_index;
        }
      }
      i++;
    }
    return clockwise;
  }

  else if (traj_type=="counterclockwise"){
    std::vector<Point2D> counter_clockwise;
    if (index_target >= index_start){
      for (int j=index_start; j>=0; j--)
        counter_clockwise.push_back(vector_list[j]);
      for (int j =vector_list.size()-1; j>=index_target; j--)
        counter_clockwise.push_back(vector_list[j]);
    }
    else{
      for (int j=index_start; j>=index_target; j--)
        counter_clockwise.push_back(vector_list[j]);
    }
    // 清除corner点间的border点
    int last_corner_index = -1;
    int i=0;
    while (i<counter_clockwise.size()){
      if (map_corner_labeled_[counter_clockwise[i].x][counter_clockwise[i].y] == obsID){
        if (last_corner_index==-1 || last_corner_index+1==i){
          last_corner_index = i;
          // 如果起点可以直接连过来 就先清除一步分
          if (i>1){
            auto vec_line = Bresenham(counter_clockwise[0], counter_clockwise[i]);
            bool is_erase = true;
            for (auto p:vec_line){
              if(map_obs_labeled_[p.x][p.y]!=0){
                is_erase = false;
                break;
              }
            }
            if (is_erase){
              counter_clockwise.erase(counter_clockwise.begin()+1, counter_clockwise.begin()+i);
              last_corner_index = 1 ;
              i = 1;
            }
          }
        }
        else{
          counter_clockwise.erase(counter_clockwise.begin()+last_corner_index+1, counter_clockwise.begin()+i);
          last_corner_index ++ ;
          i = last_corner_index;
        }
      }
      i++;
    }
    return counter_clockwise;
  }
  assert("error"=="error");
  return {};
}


// 求两两障碍物之间的最短连接 by 深度优先搜索
void mapProcess::connectObstacleGroups(){
  // 1. 针对map_border_labeled_进行voronoi划分，并把结果保存到voronoi_list_中
  std::vector<std::vector<int>> map_for_voronoi = map_border_labeled_;
  // 求voronoi points
  DynamicVoronoi voronoi;
  // initialize voronoi object it with the map
  voronoi.initializeMap(map_for_voronoi);
  // update distance map and Voronoi diagram
  voronoi.update(); 
  // prune the Voronoi
  voronoi.prune();
  // voronoi_list_仅仅是为了可视化
  for (int x=0; x<voronoi.getSizeX(); x++){
    for (int y=0; y<voronoi.getSizeY(); y++){
      if (voronoi.isVoronoi(x,y)){
        auto cell = voronoi.getDataCell(x,y);
        int ID1 = map_for_voronoi[cell.fatherPoints.first.x][cell.fatherPoints.first.y];
        int ID2 = map_for_voronoi[cell.fatherPoints.second.x][cell.fatherPoints.second.y];
        voronoi_list_[std::pair<int, int>(ID1,ID2)].push_back(Point2D(x,y));
        voronoi_list_[std::pair<int, int>(ID2,ID1)].push_back(Point2D(x,y));
      }
    }
  }

  // 2. 对每个voronoi point，找寻其父节点p1和p2，建立双相连接（把p2添加到p1的candidate_connection中，p1添加到p2的candidate_connection中）
  for (int x=0; x<voronoi.getSizeX(); x++){
    for (int y=0; y<voronoi.getSizeY(); y++){
      if (voronoi.isVoronoi(x,y)){
        auto cell = voronoi.getDataCell(x,y);
        // 找到两个voronoi cell的父点p1 with ID1 and p2 with ID2
        Point2D p1 = Point2D(cell.fatherPoints.first.x, cell.fatherPoints.first.y);
        int ID1 = map_for_voronoi[p1.x][p1.y];
        Point2D p2 = Point2D(cell.fatherPoints.second.x, cell.fatherPoints.second.y);
        int ID2 = map_for_voronoi[p2.x][p2.y];
        
        // 在ID1的border_list_中找到p1，在p1.candidate_connection中添加p2
        for (auto iter = border_list_[ID1].begin(); iter!=border_list_[ID1].end(); iter++){
          if (iter->x==p1.x && iter->y==p1.y){
            // 防止重复添加点
            bool add_flag = true;
            for (auto temp:iter->candidate_connection){
              if (temp.first==p2.x && temp.second==p2.y){
                add_flag = false;
                break;
              }
            }
            if (add_flag)
              iter->candidate_connection.push_back({p2.x, p2.y});
            break;
          }
        }
        // 在ID2的border_list_中找到p2，在p2.candidate_connection中添加p1
        for (auto iter = border_list_[ID2].begin(); iter!=border_list_[ID2].end(); iter++){
          if (iter->x==p2.x && iter->y==p2.y){
            // 防止重复添加点
            bool add_flag = true;
            for (auto temp:iter->candidate_connection){
              if (temp.first==p1.x && temp.second==p1.y){
                add_flag = false;
                break;
              }
            }
            if (add_flag)
              iter->candidate_connection.push_back({p1.x, p1.y});
            break;
          }
        }
      }
    }
  }

  // 3. 先求从ID1连到ID2的过程中，ID1上可连点的index：connections_indexs；再求ID连ID2中 顺时针的两个端点index：connection_ends
  // {ID1 -> ID2} -> {ID1中的indexes}
  std::map<std::pair<int,int>, std::vector<int>> connection_indexs;
  for(auto temp:border_list_){
    int ID1 = temp.first;
    for (int index = 0; index < temp.second.size(); index++){
      for (auto p:temp.second[index].candidate_connection){
        int ID2 = map_for_voronoi[p.first][p.second];
        // 空的 或者 最后一个元素不是index
        if (connection_indexs[{ID1,ID2}].empty() || connection_indexs[{ID1,ID2}].back()!=index){
          connection_indexs[{ID1,ID2}].push_back(index);
        }
      }
    }
  }
  // connection_ends: {ID1 -> ID2} -> {ID1的start_index, ID1的end_index} (顺时针)
  std::map<std::pair<int,int>, std::pair<int, int >> connection_ends;
  for(auto temp:connection_indexs){
    int ID1 = temp.first.first;
    int ID2 = temp.first.second;
    std::vector<int> indexs = temp.second;
    // ID1的border上就一个点可以连接到ID2，start就是该点本身
    if (indexs.size()==1)
      connection_ends[{ID1,ID2}] = {indexs.front(), indexs.front()};
    // 计算indexs中相邻量元素的的数值差，差值最大的两个元素会被设定为{end, start}
    else{
      int max_diff = -1;
      int start = 0;
      int end = 0;
      for (int i=0; i<indexs.size()-1; i++){
        int diff = indexs[i+1]-indexs[i];
        if (diff>max_diff){
          max_diff = diff;
          end = indexs[i];
          start = indexs[i+1];
        }
      }
      int diff = border_list_[ID1].size()+indexs.front()-indexs.back();
      if (diff>max_diff){
        max_diff = diff;
        end = indexs.back();
        start = indexs.front();
      }
      connection_ends[{ID1,ID2}] = {start, end};
    }
  }

  // 4. 两个border list的连接方式共有5种："clockwise_consistent", "counterclockwise_consistent", "shortest", "clockwise2counterclockwise", "counterclockwise2clockwise"，接下来分别求取他们。
  // 4.1 在connection_graph_标记clockwise和counterwise的点对 connection_graph_: <ID1->ID2>[connection type] = ID1_border_point -> ID2_border_point
  for (auto temp:connection_ends){
    int ID1 = temp.first.first;
    int ID1_start = temp.second.first;
    int ID1_end = temp.second.second;

    int ID2 = temp.first.second;
    int ID2_start = connection_ends[{ID2, ID1}].first;
    int ID2_end = connection_ends[{ID2, ID1}].second;
    connection_graph_[{ID1, ID2}]["clockwise_consistent"] = std::make_pair(border_list_[ID1][ID1_start], border_list_[ID2][ID2_end]);
    connection_graph_[{ID1, ID2}]["counterclockwise_consistent"] = std::make_pair(border_list_[ID1][ID1_end], border_list_[ID2][ID2_start]);
  }

  // 4.2 对任意两个border list，遍历所有border的所有candidate_connection，找到dis最小的两个点，作为shortest连接
  for (auto temp:border_list_){
    int ID1 = temp.first;
    for (auto border_poi_in_ID1:temp.second){
      // 如果无candidate_connection 跳过
      if (border_poi_in_ID1.candidate_connection.empty()) continue;
      int x_ID1 = border_poi_in_ID1.x;
      int y_ID1 = border_poi_in_ID1.y;
      // 遍历所有candidate_connection
      for (auto border_poi_in_ID2 : border_poi_in_ID1.candidate_connection){
        int x_ID2 = border_poi_in_ID2.first;
        int y_ID2 = border_poi_in_ID2.second;
        int ID2 = map_for_voronoi[x_ID2][y_ID2];
        // clockwise 只保存第一个见到的点 （原因：border_list_本身就是顺时针运转的）
        if (connection_graph_[{ID1,ID2}].find("shortest") == connection_graph_[{ID1,ID2}].end()){
          connection_graph_[{ID1,ID2}]["shortest"] = std::make_pair(Point2D(x_ID1, y_ID1, 0, ID1), Point2D(x_ID2, y_ID2, 0, ID2));
        }
        else{
          if (hypot(x_ID1-x_ID2, y_ID1-y_ID2) < hypot(connection_graph_[{ID1, ID2}]["shortest"].first.x-connection_graph_[{ID1, ID2}]["shortest"].second.x, connection_graph_[{ID1, ID2}]["shortest"].first.y-connection_graph_[{ID1, ID2}]["shortest"].second.y)){
            connection_graph_[{ID1,ID2}]["shortest"] = std::make_pair(Point2D(x_ID1, y_ID1, 0, ID1), Point2D(x_ID2, y_ID2, 0, ID2));
          }
        }
      }
    }
  }

  // 4.3 求clockwise2counter 和 counterclockwise2clockwise
  for (auto temp:connection_ends){
    // ROS_INFO("ID1: %d, ID2: %d", temp.first.first, temp.first.second);
    // ID1对应的param
    int ID1 = temp.first.first;
    int ID1_start_index = temp.second.first;
    int ID1_end_index = temp.second.second;
    int ID1_border_length = border_list_[ID1].size();
    int ID1_index_diff = getMod(ID1_end_index+1 - ID1_start_index, ID1_border_length);
    // ID2对应的param
    int ID2 = temp.first.second;
    int ID2_start_index = connection_ends[{ID2,ID1}].first;
    int ID2_end_index = connection_ends[{ID2,ID1}].second;
    int ID2_border_length = int(border_list_[ID2].size());
    int ID2_index_diff = getMod(ID2_end_index+1 - ID2_start_index, ID2_border_length);

    // 4.3.1 考虑clockwise2counterclockwise，最理想的情况是两个start能无collision地连接
    int ID1_index_res=-1, ID2_index_res=-1;
    int ID1_index_temp = ID1_start_index;
    int ID2_index_temp = ID2_start_index;
    while(true){
      // ROS_INFO("+++++ %d, %d", ID1_index_temp, ID2_index_temp);
      // 成功连接
      if (!checkCollisionObsAndBorderAlignedWithBresenham(border_list_[ID1][ID1_index_temp], border_list_[ID2][ID2_index_temp], ID1, ID2)){
        ID1_index_res = ID1_index_temp;
        ID2_index_res = ID2_index_temp;
        break;
      }
      // 确定滑动谁
      else{
        if( getMod(ID1_index_temp+1 - ID1_start_index, ID1_border_length)*1.0 / ID1_index_diff < getMod(ID2_index_temp+1 - ID2_start_index, ID2_border_length)*1.0 / ID2_index_diff )
          ID1_index_temp = getMod(ID1_index_temp+1, ID1_border_length);
        else
          ID2_index_temp = getMod(ID2_index_temp+1, ID2_border_length);
      }
      if (ID1_index_temp==ID1_end_index && ID2_index_temp==ID2_end_index)
        break;
    }
    if (ID1_index_res!=-1 && ID2_index_res!=-1)
      connection_graph_[{ID1, ID2}]["clockwise2counterclockwise"] = std::make_pair(border_list_[ID1][ID1_index_res], border_list_[ID2][ID2_index_res]);

    // 4.3.2 考虑counterclockwise2clockwise, 最理想的情况是两个end能无collision地连接
    ID1_index_res=-1; ID2_index_res=-1;
    ID1_index_temp = ID1_end_index;
    ID2_index_temp = ID2_end_index;
    while(true){
      auto vec_line = Bresenham(border_list_[ID1][ID1_index_temp], border_list_[ID2][ID2_index_temp]);
      bool is_collision = checkCollision(vec_line, ID1, ID2);
      // 成功连接
      if (!is_collision){
        ID1_index_res = ID1_index_temp;
        ID2_index_res = ID2_index_temp;
        break;
      }
      // 确定滑动谁
      else{
        if( getMod(ID1_index_temp+ID1_border_length - ID1_start_index, ID1_border_length)*1.0 / ID1_index_diff > getMod(ID2_index_temp+ID2_border_length - ID2_start_index, ID2_border_length)*1.0 / ID2_index_diff )
          ID1_index_temp = getMod(ID1_index_temp-1+ID1_border_length, ID1_border_length);
        else
          ID2_index_temp = getMod(ID2_index_temp-1+ID2_border_length, ID2_border_length);
      }
      if (ID1_index_temp==ID1_start_index && ID2_index_temp==ID2_start_index)
        break;
    }
    if (ID1_index_res!=-1 && ID2_index_res!=-1)
      connection_graph_[{ID1, ID2}]["counterclockwise2clockwise"] = std::make_pair(border_list_[ID1][ID1_index_res], border_list_[ID2][ID2_index_res]);    
  }
}

void mapProcess::connectStartAndGoalToObstacleGroups(){
  // 1. 把start添加到多个list中
  startID_ = label_index_++;
  startIndex_ = label_list_.size();
  label_list_.push_back(startID_);
  obs_list_[startID_] = {Point2D(start_in_map_x_, start_in_map_y_, 0, startID_)};
  border_list_[startID_] = {Point2D(start_in_map_x_, start_in_map_y_, 0, startID_)};
  corner_list_[startID_] = {Point2D(start_in_map_x_, start_in_map_y_, 0, startID_)};
  // 把goal_line添加到多个list中
  goalID_start_ = label_index_;
  goalIndex_start_ = label_list_.size();
  for (auto goal_line:goal_line_lists_){
    int ID = label_index_++;
    label_list_.push_back(ID);
    obs_list_[ID] = goal_line;
    border_list_[ID] = goal_line;
    corner_list_[ID] = goal_line;
  }

  // 为每个border group / goal line都寻找一个离start最近的border point作为连接点
  for (auto temp:border_list_){
    int ID = temp.first;
    if (ID==startID_) continue;    //跳过起点
    double min_dis = 99999;
    double target_x, target_y;
    int target_index;
    for (int i=0; i<temp.second.size(); i++){
      auto p =temp.second[i];
      if (!checkCollisionObsAndBorderAlignedWithBresenham(Point2D(start_in_map_x_, start_in_map_y_), Point2D(p.x, p.y), ID)){
        border_list_[ID][i].setStartConnection();
        if (hypot(p.x-start_in_map_x_, p.y-start_in_map_y_) < min_dis){
          min_dis = hypot(p.x-start_in_map_x_, p.y-start_in_map_y_);
          target_x = p.x;
          target_y = p.y;
          target_index = i;
        }
      }
    }

    if (min_dis != 99999){
      connection_graph_[{startID_, ID}]["shortest"] = std::make_pair(Point2D(start_in_map_x_, start_in_map_y_, 0, startID_), Point2D(target_x, target_y, 0, ID));
    }
  }

  // 删除错误的绕行group。如果某个group A连接向另一个group B的local boundary中，包含了shortest连接向目标点的点，则断开group A和group B的连接
  std::vector<std::pair<int,int>> erase_ID_list;
  for (auto temp:connection_graph_){
    // 考虑start to groupA的连接
    if (temp.first.first==startID_){
      int ID_groupA = temp.first.second;
      auto p2start = connection_graph_[{startID_,ID_groupA}]["shortest"].second;
      for (auto connect:connection_graph_){
        // 起点是ID_groupA
        if(connect.first.first==ID_groupA){
          int ID_groupB = connect.first.second;
          if (ID_groupB>=startID_) continue;  // 忽略goal lines
          auto p_clockwise2groupB=connect.second["clockwise_consistent"].first;
          auto p_counterclockwise2groupB=connect.second["counterclockwise_consistent"].first;
          int p2start_index = -1;
          int p_clockwise2GroupB_index = -1;
          int p_counter_clockwise2groupB_index = -1;
          auto border_groupA = border_list_[ID_groupA];
          for (int i=0; i<border_groupA.size(); i++){
            if (border_groupA[i].x==p2start.x && border_groupA[i].y==p2start.y)
              p2start_index = i;
            if (border_groupA[i].x==p_clockwise2groupB.x && border_groupA[i].y==p_clockwise2groupB.y)
              p_clockwise2GroupB_index = i;
            if (border_groupA[i].x==p_counterclockwise2groupB.x && border_groupA[i].y==p_counterclockwise2groupB.y)
              p_counter_clockwise2groupB_index = i;
          }
          // 检查三个index的关系
          bool is_in = false;
          for (int i=p_clockwise2GroupB_index; i<=p_clockwise2GroupB_index+border_groupA.size(); i++){
            if(i%border_groupA.size() == p2start_index){
              is_in = true;
              break;
            }
            if(i%border_groupA.size() == p_counter_clockwise2groupB_index)
              break;
          }
          if (is_in){
            erase_ID_list.push_back({ID_groupA, ID_groupB});
          }
        }
      }
    }
  }
  for(auto IDs:erase_ID_list)
    connection_graph_.erase({IDs.first, IDs.second});


  // 2. 寻找goal line的连接，利用每个obs只留一个最近的goal_line的shortest连接
  // 把goal line添加到地图中
  auto goal_auged_map_border_labeled = map_border_labeled_;
  for (int goalIndex=goalIndex_start_; goalIndex<label_list_.size(); goalIndex++){
    int goalID = label_list_[goalIndex];
    for (auto poi:border_list_[goalID])
      goal_auged_map_border_labeled[poi.x][poi.y] = goalID;
  }
  // 暴力求解
  for (auto temp:border_list_){    
    int ID_obs = temp.first;
    if (ID_obs==startID_ || ID_obs>=goalID_start_) continue;    // 跳过起点和其他终点
    double min_dis = 99999;
    double min_p_goal_x, min_p_goal_y;
    int min_goalID;
    double min_p_obs_x, min_p_obs_y;
    // 遍历IDobs下的border point，寻找最近的goal line
    for(int i=0; i<temp.second.size(); i++){
      auto p_obs = temp.second[i];
      double min_temp_dis_from_p_obs_to_p_goal = 99999;
      Point2D min_temp_goal;
      int min_temp_goal_ID;
      // 遍历所有goal lines
      for (int goalIndex=goalIndex_start_; goalIndex<label_list_.size(); goalIndex++){
        int goalID = label_list_[goalIndex];
        // 按照距current_dis从低到高，对index排个序
        std::vector<int> index_list_ordered;
        std::vector<double> dis_list_ordered;
        // 遍历index为goalID的goal line的每个点，按照距离排序，得到dis_list_ordered和index_list_ordered
        for (int j=0; j<border_list_[goalID].size(); j++){
          auto p_goal = border_list_[goalID][j];
          double current_dis = hypot(p_obs.x-p_goal.x, p_obs.y-p_goal.y);
          bool is_insert = false;
          for (int k=0; k<index_list_ordered.size(); k++){
            if (current_dis < dis_list_ordered[k]){
              index_list_ordered.insert(index_list_ordered.begin()+k, j);
              dis_list_ordered.insert(dis_list_ordered.begin()+k, current_dis);
              is_insert = true;
              break;
            }
          }
          if (!is_insert){
            index_list_ordered.push_back(j);
            dis_list_ordered.push_back(current_dis);
          }
        }
        assert(index_list_ordered.size()==border_list_[goalID].size());
        // 按顺序检查所有的连线，只要有collisoin-free的，就保存全局数据和局部数据，然后直接break
        for (auto index_temp:index_list_ordered){
          auto p_goal = border_list_[goalID][index_temp];
          double current_dis = hypot(p_obs.x-p_goal.x, p_obs.y-p_goal.y);
          bool is_collision = checkCollisionOnlyObsAlignedWithBresenham(p_obs, p_goal);
          if (!is_collision){
            // 全局数据
            if (current_dis < min_dis){
              min_dis = current_dis;
              min_p_goal_x = p_goal.x;
              min_p_goal_y = p_goal.y;
              min_goalID = goalID;
              min_p_obs_x = p_obs.x;
              min_p_obs_y = p_obs.y;
            }
            // 局部数据
            if (current_dis<min_temp_dis_from_p_obs_to_p_goal){
              min_temp_dis_from_p_obs_to_p_goal = current_dis;
              min_temp_goal = p_goal;
              min_temp_goal_ID = goalID;
            }
            // 由于dis是升序的，所以只要已经找到了一个collision-free点，就可以直接break，后续的没有意义啦
            break;
          }
        }
      }
      // 利用局部数据，确定当前border point 与 goal line所有点的shortest连接
      if(min_temp_dis_from_p_obs_to_p_goal != 99999)
        border_list_[ID_obs][i].setGoalConnection({min_temp_goal.x, min_temp_goal.y}, min_temp_goal_ID);
    }
    // 利用全局数据，确定当前boder list 与 goal line所有点的shorest连接
    if (min_dis != 99999)
      connection_graph_[{ID_obs, min_goalID}]["shortest"] = std::make_pair(Point2D(min_p_obs_x, min_p_obs_y, 0, ID_obs), Point2D(min_p_goal_x, min_p_goal_y, 0, min_goalID));
  }


  // 删除错误的绕行group。如果某个group A连接向另一个group B的local boundary中，包含了shortest连接向目标点的点，则断开group A和group B的连接
  erase_ID_list.clear();
  for (auto temp:connection_graph_){
    // 考虑start to groupA的连接
    if (temp.first.second>=goalID_start_){
      int ID_groupA = temp.first.first;
      int goalID = temp.first.second;
      auto p2goal = connection_graph_[{ID_groupA, goalID}]["shortest"].first;
      for (auto connect:connection_graph_){
        // 起点是ID_groupA
        if(connect.first.second==ID_groupA){
          int ID_groupB = connect.first.first;
          if (ID_groupB>=startID_) continue;  // 忽略start和其他goal lines
          auto p_clockwise2groupB=connect.second["counterclockwise_consistent"].second;
          auto p_counterclockwise2groupB=connect.second["clockwise_consistent"].second;
          int p2start_index = -1;
          int p_clockwise2GroupB_index = -1;
          int p_counter_clockwise2groupB_index = -1;
          auto border_groupA = border_list_[ID_groupA];
          for (int i=0; i<border_groupA.size(); i++){
            if (border_groupA[i].x==p2goal.x && border_groupA[i].y==p2goal.y)
              p2start_index = i;
            if (border_groupA[i].x==p_clockwise2groupB.x && border_groupA[i].y==p_clockwise2groupB.y)
              p_clockwise2GroupB_index = i;
            if (border_groupA[i].x==p_counterclockwise2groupB.x && border_groupA[i].y==p_counterclockwise2groupB.y)
              p_counter_clockwise2groupB_index = i;
          }
          // 检查三个index的关系
          bool is_in = false;
          for (int i=p_clockwise2GroupB_index; i<=p_clockwise2GroupB_index+border_groupA.size(); i++){
            if(i%border_groupA.size() == p2start_index){
              is_in = true;
              break;
            }
            if(i%border_groupA.size() == p_counter_clockwise2groupB_index)
              break;
          }
          if (is_in){
            erase_ID_list.push_back({ID_groupB, ID_groupA});
          }
        }
      }
    }
  }
  for(auto IDs:erase_ID_list)
    connection_graph_.erase({IDs.first, IDs.second});
    

}


std::pair<Point2D, Point2D> mapProcess::getShortestPathBetweenObs(int ID1, int ID2){
  return connection_graph_[{ID1,ID2}]["shortest"];
}

std::pair<Point2D, Point2D> mapProcess::getClockwiseConsistentPathBetweenObs(int ID1, int ID2){
  return connection_graph_[{ID1,ID2}]["clockwise_consistent"];
}

std::pair<Point2D, Point2D> mapProcess::getCounterClockwiseConsistentPathBetweenObs(int ID1, int ID2){
  return connection_graph_[{ID1,ID2}]["counterclockwise_consistent"];
}

std::pair<Point2D, Point2D> mapProcess::getClockwise2CounterClockwisePathBetweenObs(int ID1, int ID2){
  return connection_graph_[{ID1,ID2}]["clockwise2counterclockwise"];
}

std::pair<Point2D, Point2D> mapProcess::getCounterClockwise2ClockwisePathBetweenObs(int ID1, int ID2){
  return connection_graph_[{ID1,ID2}]["counterclockwise2clockwise"];
}


// 启动深搜，找同伦轨迹
std::vector<std::vector<Eigen::Vector2d>> mapProcess::findHomoPaths(std::vector<std::pair<std::pair<double, double>, std::pair<double, double>>>& goal_endpoints_list){
  // for (auto temp:connection_graph_){
  //   auto ID1 = temp.first.first;
  //   auto ID2 = temp.first.second;
  //   ROS_INFO("==== %d --> %d", ID1, ID2);
  //   for (auto traj:temp.second){
  //     ROS_INFO("  %s: (%d,%d), (%d,%d)", traj.first.c_str(), traj.second.first.x, traj.second.first.y, traj.second.second.x, traj.second.second.y);
  //     // std::cout<<traj.first<<": "<<traj.second.first.x<<","<<traj.second.first.y<<"; "<<traj.second.second.x<<","<<traj.second.second.y<<std::endl;
  //   }
  // }
  auto t = ros::Time::now();
  // 找起点和终点的ID
  float min_dis_start = 99999.9, min_dis_goal = 99999.9;
  // 启动深搜，找到res这个二维的vector，保存多条由obs组成的轨迹
  std::vector<int> visited;
  std::vector<std::vector<int>> res;
  visited.push_back(startID_);
  depthFirst(visited, res);
  ROS_INFO("    !!! Finding [%d]/[%d] general trajectories during depthFirst", res.size(), max_path_explore_number_for_GraphicTEB_);
  // for (int i=0; i<res.size(); i++){
  //   auto path = res[i];
  //   std::cout<<i<<" path: ";
  //   for (auto id:path)
  //     std::cout<<id<<" ";
  //   std::cout<<std::endl;
  // }
  std::vector<std::vector<Point2D>> homo_paths_in_map;

  // 起点的vec形式
  auto start_vec = Point2D(start_in_map_x_, start_in_map_y_, 0, startID_);
  for (auto pathIDs:res){
    // homo path 超出最大限制
    if (max_path_remained_for_GraphicTEB_!=-1 && homo_paths_in_map.size()>max_path_remained_for_GraphicTEB_)
      break;
    // 从当前paths的长度index_start开始，接下来index_max_increment个轨迹，都和r相关
    int index_start = homo_paths_in_map.size();
    int index_max_increment = pow(2, pathIDs.size()-2); //去除起点和终点，接下来index_max_increment都是和当前pathIDs相关的
    // 找到map像素下的所有轨迹homo_paths_in_map
    for (int i=0; i<index_max_increment; i++){
      homo_paths_in_map.push_back({start_vec});
    }
    // 一共两个点 / 一步直接到目标点
    if (pathIDs.size()==2){
      for (int i=0; i<index_max_increment; i++)
        homo_paths_in_map[index_start+i].push_back(getShortestPathBetweenObs(pathIDs.front(), pathIDs.back()).second);
    }
    // 一共三个点
    else if (pathIDs.size()==3){
      int lastID = pathIDs[0];
      int currentID = pathIDs[1];
      int nextID = pathIDs[2];
      // 第二个obs group上做绕行时 只有shortest的起点、只有shortest的终点
      auto robot_point = getShortestPathBetweenObs(lastID, currentID).first;
      auto start_point_on_currentID_shortest = getShortestPathBetweenObs(lastID, currentID).second;
      auto end_point_on_currentID_shortest = getShortestPathBetweenObs(currentID, nextID).first;
      auto goal_point = getShortestPathBetweenObs(currentID, nextID).second;
      // 找start和goal的index
      int start_index = -1;
      int end_index = -1;
      auto border_currentID = border_list_[currentID];
      int border_length = int(border_currentID.size());
      for (int j=0; j<border_currentID.size(); j++){
        if (border_currentID[j].x==start_point_on_currentID_shortest.x && border_currentID[j].y==start_point_on_currentID_shortest.y)
          start_index = j;
        if (border_currentID[j].x==end_point_on_currentID_shortest.x && border_currentID[j].y==end_point_on_currentID_shortest.y)
          end_index = j;
      }
      // ROS_INFO("goal_point: %d,%d", goal_point.x, goal_point.y);
      // clockwise
      Point2D start_point_on_currentID_clockwise = start_point_on_currentID_shortest;
      Point2D end_point_on_currentID_clockwise = end_point_on_currentID_shortest;
      Point2D goal_point_clockwise = goal_point;
      int d_start_clockwise_iter = 0;
      bool is_start_clockwise_continue = true;
      int d_end_clockwise_iter = 0;
      bool is_end_clockwise_continue = true;
      double start_last_nearest_dis_clockwise = -1;
      double end_last_nearest_dis_clockwise = -1;
      // ROS_INFO("+++  %d, %d: %d", start_index, end_index, currentID);
      while( getMod(start_index+d_start_clockwise_iter, border_length) != getMod(end_index+d_end_clockwise_iter, border_length) ){
        // 如果都能走 那就谁走的少谁先走
        if (is_start_clockwise_continue && is_end_clockwise_continue){
          if (abs(d_start_clockwise_iter) < abs(d_end_clockwise_iter))
            d_start_clockwise_iter++;
          else
            d_end_clockwise_iter--;
        }
        // 如果只有start能走 那只能是start走
        else if (is_start_clockwise_continue && !is_end_clockwise_continue)
          d_start_clockwise_iter++;
        // 如果只有end能走 那只能是end走
        else if (!is_start_clockwise_continue && is_end_clockwise_continue)
          d_end_clockwise_iter--;
        // 都是false
        else
          break;
        // ROS_INFO("(%d->%d), (%d->%d)  ", d_start_clockwise_iter, getMod(start_index+d_start_clockwise_iter, border_length), d_end_clockwise_iter, getMod(end_index+d_end_clockwise_iter, border_length));
        // check start->poi;
        if (is_start_clockwise_continue){
          auto poi1 = border_currentID[getMod(start_index+d_start_clockwise_iter,border_length)];
          auto last_p = border_currentID[getMod(start_index+d_start_clockwise_iter-1,border_length)];
          // 不撞
          if (poi1.can_connect_start){
            // 距离没有显著变大 则更新
            double dis_current_to_start = hypot(poi1.x-robot_point.x, poi1.y-robot_point.y);
            double dis_last_to_start = hypot(start_point_on_currentID_clockwise.x-robot_point.x, start_point_on_currentID_clockwise.y-robot_point.y);
            double dis_last_to_current = abs(start_point_on_currentID_clockwise.x-poi1.x)+abs(start_point_on_currentID_clockwise.y-poi1.y);
            if(dis_last_to_start+dis_last_to_current >= dis_current_to_start){
              start_point_on_currentID_clockwise = poi1;
            }
            // 距离显著变大，失败
            else
              is_start_clockwise_continue = false;  
          }
          else
            is_start_clockwise_continue = false;
        }
        // check poi->goal_line
        if (is_end_clockwise_continue){
          auto poi2 = border_currentID[getMod(end_index+d_end_clockwise_iter,border_length)];
          if (poi2.can_connect_goal){
            Point2D temp_res = Point2D(poi2.goal_point.first, poi2.goal_point.second);
            double dis_current_to_goal = hypot(poi2.x-temp_res.x, poi2.y-temp_res.y);
            double dis_last_to_goal = hypot(end_point_on_currentID_clockwise.x-goal_point_clockwise.x, end_point_on_currentID_clockwise.y-goal_point_clockwise.y);
            double dis_last_to_current = abs(end_point_on_currentID_clockwise.x-poi2.x)+abs(end_point_on_currentID_clockwise.y-poi2.y);
            if(dis_last_to_goal+dis_last_to_current >= dis_current_to_goal){
              end_point_on_currentID_clockwise = poi2;
              goal_point_clockwise = temp_res;
            }
            else
              is_end_clockwise_continue = false;  
          }
          else
            is_end_clockwise_continue = false;
        }
      }
      // std::cout<<"====== done1 "<<end_point_on_currentID_clockwise.x<<","<<end_point_on_currentID_clockwise.y<<"; "<<goal_point_clockwise.x<<","<<goal_point_clockwise.y<<std::endl;

      // counterclockwise
      Point2D start_point_on_currentID_counterclockwise = start_point_on_currentID_shortest;
      Point2D end_point_on_currentID_counterclockwise = end_point_on_currentID_shortest;
      Point2D goal_point_counterclockwise = goal_point;
      int d_start_counterclockwise_iter = 0;
      bool is_start_counterclockwise_continue = true;
      int d_end_counterclockwise_iter = 0;
      bool is_end_counterclockwise_continue = true;
      double start_last_nearest_dis_counterclockwise = -1;
      double end_last_nearest_dis_counterclockwise = -1;
      // ROS_INFO("+++  %d, %d", start_index, end_index);
      while( getMod(start_index+d_start_counterclockwise_iter, border_length) != getMod(end_index+d_end_counterclockwise_iter, border_length) ){
        // 如果都能走 那就谁走的少谁先走
        if (is_start_counterclockwise_continue && is_end_counterclockwise_continue){
          if (abs(d_start_counterclockwise_iter) < abs(d_end_counterclockwise_iter))
            d_start_counterclockwise_iter--;
          else
            d_end_counterclockwise_iter++;
        }
        // 如果只有start能走 那只能是start走
        else if (is_start_counterclockwise_continue && !is_end_counterclockwise_continue)
          d_start_counterclockwise_iter--;
        // 如果只有end能走 那只能是end走
        else if (!is_start_counterclockwise_continue && is_end_counterclockwise_continue)
          d_end_counterclockwise_iter++;
        // 都是false
        else
          break;
        // ROS_INFO("(%d->%d), (%d->%d)  ", d_start_clockwise_iter, getMod(start_index+d_start_clockwise_iter, border_length), d_end_clockwise_iter, getMod(end_index+d_end_clockwise_iter, border_length));
        // check start->poi;
        if (is_start_counterclockwise_continue){
          auto poi1 = border_currentID[getMod(start_index+d_start_counterclockwise_iter,border_length)];
          auto last_p = border_currentID[getMod(start_index+d_start_counterclockwise_iter+1,border_length)];
          if (poi1.can_connect_start){
            double dis_current_to_start = hypot(poi1.x-robot_point.x, poi1.y-robot_point.y);
            double dis_last_to_start = hypot(start_point_on_currentID_counterclockwise.x-robot_point.x, start_point_on_currentID_counterclockwise.y-robot_point.y);
            double dis_last_to_current = abs(start_point_on_currentID_counterclockwise.x-poi1.x)+abs(start_point_on_currentID_counterclockwise.y-poi1.y);
            if(dis_last_to_start+dis_last_to_current >= dis_current_to_start){
              start_point_on_currentID_counterclockwise = poi1;
            }
            // 距离显著变大，失败
            else
              is_start_counterclockwise_continue = false;  
          }
          else
            is_start_counterclockwise_continue = false;
        }
        // check poi->goal_line
        if (is_end_counterclockwise_continue){
          auto poi2 = border_currentID[getMod(end_index+d_end_counterclockwise_iter,border_length)];
          auto last_p = border_currentID[getMod(end_index+d_end_counterclockwise_iter-1,border_length)];
          if (poi2.can_connect_goal){
            Point2D temp_res = Point2D(poi2.goal_point.first, poi2.goal_point.second);
            double dis_current_to_goal = hypot(poi2.x-temp_res.x, poi2.y-temp_res.y);
            double dis_last_to_goal = hypot(end_point_on_currentID_counterclockwise.x-goal_point_counterclockwise.x, end_point_on_currentID_counterclockwise.y-goal_point_counterclockwise.y);
            double dis_last_to_current = abs(end_point_on_currentID_counterclockwise.x-poi2.x)+abs(end_point_on_currentID_counterclockwise.y-poi2.y);
            if(dis_last_to_goal+dis_last_to_current >= dis_current_to_goal){
              end_point_on_currentID_counterclockwise = poi2;
              goal_point_counterclockwise = temp_res;
            }
            else
              is_end_counterclockwise_continue = false;  
          }
          else
            is_end_counterclockwise_continue = false;
        }
      }
      // std::cout<<"====== done2"<<std::endl;
      // ROS_INFO("(%d,%d),  (%d,%d)", goal_point_clockwise.x, goal_point_clockwise.y, goal_point_counterclockwise.x, goal_point_counterclockwise.y);

      for (int j=0; j<index_max_increment; j++){
        if (j%2==0){
          auto traj_to_clockwise = getPathAlongsideObs(currentID, "clockwise", start_point_on_currentID_clockwise, end_point_on_currentID_clockwise);
          homo_paths_in_map[index_start + j].insert(homo_paths_in_map[index_start + j].end(), traj_to_clockwise.begin(), traj_to_clockwise.end());
          homo_paths_in_map[index_start + j].push_back(goal_point_clockwise);
        }
        else{
          auto traj_to_counterclockwise = getPathAlongsideObs(currentID, "counterclockwise", start_point_on_currentID_counterclockwise, end_point_on_currentID_counterclockwise);
          homo_paths_in_map[index_start + j].insert(homo_paths_in_map[index_start + j].end(), traj_to_counterclockwise.begin(), traj_to_counterclockwise.end());
          homo_paths_in_map[index_start + j].push_back(goal_point_counterclockwise);
        }
      }
    }
    // 总点数大于3
    else{
      // 处理1->n-1
      for (int i=1; i<=pathIDs.size()-1-2; i++){
        // std::cout<<"current i: "<<i<<std::endl;
        int lastID = pathIDs[i-1];
        int currentID = pathIDs[i];
        int nextID = pathIDs[i+1];
        // 第二个点(start是第一个点) 前序稳定shortest 当前有顺逆、下步有顺逆 共4种情况
        if (i==1){
          auto end_point_for_clockwise_to_clockwise =  getClockwiseConsistentPathBetweenObs(currentID, nextID).first;
          auto end_point_for_clockwise_to_counterclockwise =  getClockwise2CounterClockwisePathBetweenObs(currentID, nextID).first;
          auto end_point_for_counterclockwise_to_clockwise =  getCounterClockwise2ClockwisePathBetweenObs(currentID, nextID).first;
          auto end_point_for_counterclockwise_to_counterclockwise =  getCounterClockwiseConsistentPathBetweenObs(currentID, nextID).first;

          //在start_point_on_currentID_shortest的基础上，找start_point_on_currentID_clockwise和start_point_on_currentID_counterclockwise
          auto start_point_on_currentID_shortest = getShortestPathBetweenObs(lastID, currentID).second;
          auto border_currentID = border_list_[currentID];
          int start_point_index=-1;   // shortest point的index
          for (int j=0; j<border_currentID.size(); j++){
            if (border_currentID[j].x==start_point_on_currentID_shortest.x && border_currentID[j].y==start_point_on_currentID_shortest.y){
              start_point_index = j;
              break;
            }
          }
          // ROS_INFO("start_point_index: %d", start_point_index);

          int clockwise_stop_index=-1;  // clockwise绕行中最早遇见的点
          for (int j=start_point_index; j<start_point_index+border_currentID.size(); j++){
            j = getMod(j, border_currentID.size());
            if ((border_currentID[j].x==end_point_for_clockwise_to_clockwise.x && border_currentID[j].y==end_point_for_clockwise_to_clockwise.y)||(border_currentID[j].x==end_point_for_clockwise_to_clockwise.x && border_currentID[j].y==end_point_for_clockwise_to_clockwise.y)){
              clockwise_stop_index = j;
              break;
            }
          }
          // ROS_INFO("clockwise_stop_index: %d", clockwise_stop_index);

          int counterclockwise_stop_index=-1;   // counterclockwise绕行中最早遇见的点
          for (int j=start_point_index; j>start_point_index-int(border_currentID.size()); j--){
            j = getMod(j, border_currentID.size());
            if (border_currentID[j].x==end_point_for_counterclockwise_to_clockwise.x && border_currentID[j].y==end_point_for_counterclockwise_to_clockwise.y || border_currentID[j].x==end_point_for_counterclockwise_to_counterclockwise.x && border_currentID[j].y==end_point_for_counterclockwise_to_counterclockwise.y){
              counterclockwise_stop_index = j;
              break;
            }
          }
          // ROS_INFO("counterclockwise_stop_index: %d", counterclockwise_stop_index);

          // 找clockwise的点
          Point2D start_point_on_currentID_clockwise = start_point_on_currentID_shortest;
          // std::cout<<"start: "<<start_point_index<<  "target: "<<clockwise_stop_index<<"  length: "<<border_currentID.size()<<std::endl;
          for (int j=start_point_index; getMod(j,border_currentID.size())!=clockwise_stop_index; j++){
            // std::cout<<j<<"->";
            j = getMod(j,border_currentID.size());
            // std::cout<<j<<", ";
            auto vec_line = Bresenham(Point2D(start_in_map_x_, start_in_map_y_), border_currentID[j]);
            // 直接归还付
            if (vec_line.size()<=3){
              start_point_on_currentID_clockwise = start_point_on_currentID_shortest;
              break;
            }
            // else
            vec_line.erase(vec_line.begin());
            vec_line.pop_back();
            if(checkCollision(vec_line, currentID))
              break;
            start_point_on_currentID_clockwise = border_currentID[j];
          }
          // std::cout<<std::endl;
          // ROS_INFO("1. clockwise_search_done");

          // 找counterclockwise的点
          // std::cout<<"start: "<<start_point_index<<  "target: "<<counterclockwise_stop_index<<"  length: "<<border_currentID.size()<<std::endl;
          Point2D start_point_on_currentID_counterclockwise = start_point_on_currentID_shortest;          
          for (int j=start_point_index; getMod(j+border_currentID.size(),border_currentID.size())!=counterclockwise_stop_index; j--){
            // std::cout<<j<<"->";
            j = getMod(j+border_currentID.size(), border_currentID.size());
            // std::cout<<j<<", ";
            auto vec_line = Bresenham(Point2D(start_in_map_x_, start_in_map_y_), border_currentID[j]);
            // 直接归还付
            if (vec_line.size()<=3){
              start_point_on_currentID_counterclockwise = start_point_on_currentID_shortest;
              break;
            }
            // else
            vec_line.erase(vec_line.begin());
            vec_line.pop_back();
            if(checkCollision(vec_line, currentID))
              break;
            start_point_on_currentID_counterclockwise = border_currentID[j];
          }
          // ROS_INFO("2. counterclockwise_search_done");
                                         
          for (int j=0; j<index_max_increment; j++){
            // non 顺 顺 （last、current、next）
            if (j%4==0){
              auto traj_by_clockwise_to_clockwise = getPathAlongsideObs(currentID, "clockwise", start_point_on_currentID_clockwise, end_point_for_clockwise_to_clockwise);
              homo_paths_in_map[index_start + j].insert(homo_paths_in_map[index_start + j].end(), traj_by_clockwise_to_clockwise.begin(), traj_by_clockwise_to_clockwise.end());
            }
            // non 顺 逆
            else if (j%4==1){
              auto traj_by_clockwise_to_counterclockwise = getPathAlongsideObs(currentID, "clockwise", start_point_on_currentID_clockwise, end_point_for_clockwise_to_counterclockwise);
              homo_paths_in_map[index_start + j].insert(homo_paths_in_map[index_start + j].end(), traj_by_clockwise_to_counterclockwise.begin(), traj_by_clockwise_to_counterclockwise.end());
            }
            // non 逆 顺
            else if (j%4==2){
              auto traj_by_counterclockwise_to_clockwise = getPathAlongsideObs(currentID, "counterclockwise", start_point_on_currentID_counterclockwise, end_point_for_counterclockwise_to_clockwise);
              homo_paths_in_map[index_start + j].insert(homo_paths_in_map[index_start + j].end(), traj_by_counterclockwise_to_clockwise.begin(), traj_by_counterclockwise_to_clockwise.end());
            }
            // non 逆 逆
            else if (j%4==3){
              auto traj_by_counterclockwise_to_counterclockwise = getPathAlongsideObs(currentID, "counterclockwise", start_point_on_currentID_counterclockwise, end_point_for_counterclockwise_to_counterclockwise);
              homo_paths_in_map[index_start + j].insert(homo_paths_in_map[index_start + j].end(), traj_by_counterclockwise_to_counterclockwise.begin(), traj_by_counterclockwise_to_counterclockwise.end());
            }
            else
              assert("error");
          }
        }
        // 第三个点 i==2 -> i==n-3  注意：共n个点，0是起点 i最大到n-3；n-2要特定处理（因为n-1是goal）；n-1也要特定处理
        else{
          // 此时共8种情况
          auto start_point_for_clockwise_to_clockwise =  getClockwiseConsistentPathBetweenObs(lastID, currentID).second;
          auto start_point_for_clockwise_to_counterclockwise =  getClockwise2CounterClockwisePathBetweenObs(lastID, currentID).second;
          auto start_point_for_counterclockwise_to_clockwise =  getCounterClockwise2ClockwisePathBetweenObs(lastID, currentID).second;
          auto start_point_for_counterclockwise_to_counterclockwise =  getCounterClockwiseConsistentPathBetweenObs(lastID, currentID).second;

          auto end_point_for_clockwise_to_clockwise =  getClockwiseConsistentPathBetweenObs(currentID, nextID).first;
          auto end_point_for_clockwise_to_counterclockwise =  getClockwise2CounterClockwisePathBetweenObs(currentID, nextID).first;
          auto end_point_for_counterclockwise_to_clockwise =  getCounterClockwise2ClockwisePathBetweenObs(currentID, nextID).first;
          auto end_point_for_counterclockwise_to_counterclockwise =  getCounterClockwiseConsistentPathBetweenObs(currentID, nextID).first;

          // tips: 下列三个字中 第二位是啥，函数的参数一就是啥；一二位相同则参数二为对应clockwise或counterclockwise，不同则shortest；二三位同样的原则决定参数三
          for (int j=0; j<index_max_increment; j++){
            // 顺 顺 顺
            if (j%8==0){
              auto traj_from_clockwise_by_clockwise_to_clockwise = getPathAlongsideObs(currentID, "clockwise", start_point_for_clockwise_to_clockwise, end_point_for_clockwise_to_clockwise);
              homo_paths_in_map[index_start + j].insert(homo_paths_in_map[index_start + j].end(), traj_from_clockwise_by_clockwise_to_clockwise.begin(), traj_from_clockwise_by_clockwise_to_clockwise.end());
            }
            // 顺 顺 逆
            else if (j%8==1){
              auto traj_from_clockwise_by_clockwise_to_counterclockwise = getPathAlongsideObs(currentID, "clockwise", start_point_for_clockwise_to_clockwise, end_point_for_clockwise_to_counterclockwise);
              homo_paths_in_map[index_start + j].insert(homo_paths_in_map[index_start + j].end(), traj_from_clockwise_by_clockwise_to_counterclockwise.begin(), traj_from_clockwise_by_clockwise_to_counterclockwise.end());
            }
            // 顺 逆 顺
            else if (j%8==2){
              auto traj_from_clockwise_by_counterclockwise_to_clockwise = getPathAlongsideObs(currentID, "counterclockwise", start_point_for_clockwise_to_counterclockwise, end_point_for_counterclockwise_to_clockwise);
              homo_paths_in_map[index_start + j].insert(homo_paths_in_map[index_start + j].end(), traj_from_clockwise_by_counterclockwise_to_clockwise.begin(), traj_from_clockwise_by_counterclockwise_to_clockwise.end());
            }
            // 顺 逆 逆
            else if (j%8==3){
            auto traj_from_clockwise_by_counterclockwise_to_counterclockwise = getPathAlongsideObs(currentID, "counterclockwise", start_point_for_clockwise_to_counterclockwise, end_point_for_counterclockwise_to_counterclockwise);
              homo_paths_in_map[index_start + j].insert(homo_paths_in_map[index_start + j].end(), traj_from_clockwise_by_counterclockwise_to_counterclockwise.begin(), traj_from_clockwise_by_counterclockwise_to_counterclockwise.end());
            }
            // 逆 顺 顺
            else if (j%8==4){
            auto traj_from_counterclockwise_by_clockwise_to_clockwise = getPathAlongsideObs(currentID, "clockwise", start_point_for_counterclockwise_to_clockwise, end_point_for_clockwise_to_clockwise);
              homo_paths_in_map[index_start + j].insert(homo_paths_in_map[index_start + j].end(), traj_from_counterclockwise_by_clockwise_to_clockwise.begin(), traj_from_counterclockwise_by_clockwise_to_clockwise.end());
            }
            // 逆 顺 逆
            else if (j%8==5){
            auto traj_from_counterclockwise_by_clockwise_to_counterclockwise = getPathAlongsideObs(currentID, "clockwise", start_point_for_counterclockwise_to_clockwise, end_point_for_clockwise_to_counterclockwise);
              homo_paths_in_map[index_start + j].insert(homo_paths_in_map[index_start + j].end(), traj_from_counterclockwise_by_clockwise_to_counterclockwise.begin(), traj_from_counterclockwise_by_clockwise_to_counterclockwise.end());
            }
            // 逆 逆 顺
            else if (j%8==6){
            auto traj_from_counterclockwise_by_counterclockwise_to_clockwise = getPathAlongsideObs(currentID, "counterclockwise", start_point_for_counterclockwise_to_counterclockwise, end_point_for_counterclockwise_to_clockwise);
              homo_paths_in_map[index_start + j].insert(homo_paths_in_map[index_start + j].end(), traj_from_counterclockwise_by_counterclockwise_to_clockwise.begin(), traj_from_counterclockwise_by_counterclockwise_to_clockwise.end());
            }
            // 逆 逆 逆
            else if (j%8==7){
            auto traj_from_counterclockwise_by_counterclockwise_to_counterclockwise = getPathAlongsideObs(currentID, "counterclockwise", start_point_for_counterclockwise_to_counterclockwise, end_point_for_counterclockwise_to_counterclockwise);
              homo_paths_in_map[index_start + j].insert(homo_paths_in_map[index_start + j].end(), traj_from_counterclockwise_by_counterclockwise_to_counterclockwise.begin(), traj_from_counterclockwise_by_counterclockwise_to_counterclockwise.end());
            }
            else
              assert("error");
          }
        }
      }

      // 处理倒数第二个detour 和 goal点
      int lastID = pathIDs[pathIDs.size()-3];
      int currentID = pathIDs[pathIDs.size()-2];
      int nextID = pathIDs[pathIDs.size()-1];
      auto goal_point = getShortestPathBetweenObs(currentID, nextID).second;

      // 倒数第二个detour 共四种情况
      auto start_point_for_clockwise_to_clockwise =  getClockwiseConsistentPathBetweenObs(lastID, currentID).second;
      auto start_point_for_clockwise_to_counterclockwise =  getClockwise2CounterClockwisePathBetweenObs(lastID, currentID).second;
      auto start_point_for_counterclockwise_to_clockwise =  getCounterClockwise2ClockwisePathBetweenObs(lastID, currentID).second;
      auto start_point_for_counterclockwise_to_counterclockwise =  getCounterClockwiseConsistentPathBetweenObs(lastID, currentID).second;

      auto end_point_on_currentID_shortest = getShortestPathBetweenObs(currentID, nextID).first;

      // 以shortest为起点，找clockwise/counterclockwise连接到goal时，currentID上的border点
      auto border_currentID = border_list_[currentID];
      auto border_goalID = border_list_[nextID];
      int start_index=-1;   // shortest point的index
      for (int j=0; j<border_currentID.size(); j++){
        if (border_currentID[j].x==end_point_on_currentID_shortest.x && border_currentID[j].y==end_point_on_currentID_shortest.y){
          start_index = j;
          break;
        }
      }
      // ROS_INFO("start_index: %d", start_index);
      assert(start_index!=-1);

      int clockwise_stop_index=-1;  // clockwise绕行中最早遇见的点
      // ROS_INFO("======-=-=+= (%d,%d), (%d,%d)",start_point_for_clockwise_to_clockwise.x, start_point_for_clockwise_to_clockwise.y, start_point_for_counterclockwise_to_clockwise.x, start_point_for_counterclockwise_to_clockwise.y);
      // 为啥啊 为啥25>-10==false啊????
      // for (int j=start_index; j>start_index-10; j--){
      for (int j=start_index; j>start_index-int(border_currentID.size()); j--){
        // ROS_INFO("%d: (%d,%d)", j, border_currentID[j].x, border_currentID[j].y);
        j = getMod(j+border_currentID.size(), border_currentID.size());
        if ((border_currentID[j].x==start_point_for_clockwise_to_clockwise.x && border_currentID[j].y==start_point_for_clockwise_to_clockwise.y)||(border_currentID[j].x==start_point_for_counterclockwise_to_clockwise.x && border_currentID[j].y==start_point_for_counterclockwise_to_clockwise.y)){
          clockwise_stop_index = j;
          break;
        }
      }
      // ROS_INFO("clockwise_stop_index: %d", clockwise_stop_index);
      assert(clockwise_stop_index!=-1);

      int counterclockwise_stop_index=-1;   // counterclockwise绕行中最早遇见的点
      // todo 这里有bug 下面的两个数一样大
      for (int j=start_index; j<start_index+border_currentID.size(); j++){
        j = getMod(j, border_currentID.size());
        if (border_currentID[j].x==start_point_for_clockwise_to_counterclockwise.x && border_currentID[j].y==start_point_for_clockwise_to_counterclockwise.y || border_currentID[j].x==start_point_for_counterclockwise_to_counterclockwise.x && border_currentID[j].y==start_point_for_counterclockwise_to_counterclockwise.y){
          counterclockwise_stop_index = j;
          break;
        }
      }
      // ROS_INFO("counterclockwise_stop_index: %d", counterclockwise_stop_index);
      assert(counterclockwise_stop_index!=-1);
      
      // 找clockwise的点
      Point2D end_point_on_currentID_clockwise = end_point_on_currentID_shortest;
      Point2D goal_point_on_goal_line_clockwise = goal_point;
      for (int j=start_index; getMod(j+border_currentID.size(), border_currentID.size())!=clockwise_stop_index; j--){
        // ROS_INFO("===== %d , %d / %d", j,(j+border_currentID.size())%border_currentID.size(),clockwise_stop_index);
        j = getMod(j+border_currentID.size(), border_currentID.size());
        // 比较两个距离 AC 与 AB+BC
        if (border_currentID[j].can_connect_goal ){
          double dis_current_to_goal = hypot(border_currentID[j].goal_point.first-border_currentID[j].x, border_currentID[j].goal_point.second-border_currentID[j].y);
          double dis_last_to_last_goal = hypot(end_point_on_currentID_clockwise.x-goal_point_on_goal_line_clockwise.x, end_point_on_currentID_clockwise.y-goal_point_on_goal_line_clockwise.y);
          double dis_current_to_last = fabs(border_currentID[j].x-end_point_on_currentID_clockwise.x) + fabs(border_currentID[j].y-end_point_on_currentID_clockwise.y);
          if (dis_current_to_goal < dis_last_to_last_goal + dis_current_to_last){
            end_point_on_currentID_clockwise = border_currentID[j];
            goal_point_on_goal_line_clockwise = Point2D(border_currentID[j].goal_point.first, border_currentID[j].goal_point.second);
          }
        }
      }

      // 找counterclockwise的点
      Point2D end_point_on_currentID_counterclockwise = end_point_on_currentID_shortest;
      Point2D goal_point_on_goal_line_counterclockwise = goal_point;
      // ROS_INFO("==========++_ %d", counterclockwise_stop_index);
      for (int j=start_index; getMod(j,border_currentID.size())!=counterclockwise_stop_index; j++){
        j = getMod(j, border_currentID.size());
        if (border_currentID[j].can_connect_goal ){
          double dis_current_to_goal = hypot(border_currentID[j].goal_point.first-border_currentID[j].x, border_currentID[j].goal_point.second-border_currentID[j].y);
          double dis_last_to_last_goal = hypot(end_point_on_currentID_counterclockwise.x-goal_point_on_goal_line_counterclockwise.x, end_point_on_currentID_counterclockwise.y-goal_point_on_goal_line_counterclockwise.y);
          double dis_current_to_last = fabs(border_currentID[j].x-end_point_on_currentID_counterclockwise.x) + fabs(border_currentID[j].y-end_point_on_currentID_counterclockwise.y);
          if (dis_current_to_goal < dis_last_to_last_goal + dis_current_to_last){
            end_point_on_currentID_counterclockwise = border_currentID[j];
            goal_point_on_goal_line_counterclockwise = Point2D(border_currentID[j].goal_point.first, border_currentID[j].goal_point.second);
          }
        }
      }
      // std::cout<<std::endl;


      for (int j=0; j<index_max_increment; j++){
        // 顺 顺 non
        if (j%4==0){
          auto traj_from_clockwise_by_clockwise_to_non = getPathAlongsideObs(currentID, "clockwise", start_point_for_clockwise_to_clockwise, end_point_on_currentID_clockwise);
          homo_paths_in_map[index_start + j].insert(homo_paths_in_map[index_start + j].end(), traj_from_clockwise_by_clockwise_to_non.begin(), traj_from_clockwise_by_clockwise_to_non.end());
          homo_paths_in_map[index_start + j].push_back(goal_point_on_goal_line_clockwise);
        }
        // 顺 逆 non
        else if (j%4==1){
          auto traj_from_clockwise_by_counterclockwise_to_non = getPathAlongsideObs(currentID, "counterclockwise", start_point_for_clockwise_to_counterclockwise, end_point_on_currentID_counterclockwise);
          homo_paths_in_map[index_start + j].insert(homo_paths_in_map[index_start + j].end(), traj_from_clockwise_by_counterclockwise_to_non.begin(), traj_from_clockwise_by_counterclockwise_to_non.end());
          homo_paths_in_map[index_start + j].push_back(goal_point_on_goal_line_counterclockwise);
        }
        // 逆 顺 non
        else if (j%4==2){
          auto traj_from_counterclockwise_by_clockwise_to_non = getPathAlongsideObs(currentID, "clockwise", start_point_for_counterclockwise_to_clockwise, end_point_on_currentID_clockwise);
          homo_paths_in_map[index_start + j].insert(homo_paths_in_map[index_start + j].end(), traj_from_counterclockwise_by_clockwise_to_non.begin(), traj_from_counterclockwise_by_clockwise_to_non.end());
          homo_paths_in_map[index_start + j].push_back(goal_point_on_goal_line_clockwise);
        }
        // 逆 逆 non
        else if (j%4==3){
          auto traj_from_counterclockwise_by_counterclockwise_to_non = getPathAlongsideObs(currentID, "counterclockwise", start_point_for_counterclockwise_to_counterclockwise, end_point_on_currentID_counterclockwise);
          homo_paths_in_map[index_start + j].insert(homo_paths_in_map[index_start + j].end(), traj_from_counterclockwise_by_counterclockwise_to_non.begin(), traj_from_counterclockwise_by_counterclockwise_to_non.end());
          homo_paths_in_map[index_start + j].push_back(goal_point_on_goal_line_counterclockwise);
        }
        else
          assert("error");        
      }
    }
  }

  // 删除压边界的轨迹；把轨迹缩短（剔除中间的无碰撞点）；把轨迹从map转到world像素下
  // 按照从小到大的顺序排列homo_paths_
  std::vector<double> dis_list;
  auto temp_map = map_border_labeled_;
  for (int i=startIndex_; i<label_list_.size(); i++){
    for (auto p:border_list_[label_list_[i]])
      temp_map[p.x][p.y] = label_list_[i];
  }

  double dis_appened_threshold = 1000.0;
  for (auto path:homo_paths_in_map){
    // cos_limitation的第二轮检测
    auto edge_robot2goal = Point2D(goal_in_map_x_-start_in_map_x_, goal_in_map_y_-start_in_map_y_);
    // auto edge_robot2goal = Point2D(path.back().x-path.front().x, path.back().y-path.front().y);
    bool is_delete = false;
    for (int i=1; i<path.size()-1; i++){
      // 只考虑connection的连接
      if (temp_map[path[i].x][path[i].y] == temp_map[path[i-1].x][path[i-1].y])
        continue;
      auto edge_self2potential = Point2D(path[i].x-path[i-1].x, path[i].y-path[i-1].y);
      auto cos_theta = (edge_self2potential.x * edge_robot2goal.x + edge_self2potential.y*edge_robot2goal.y)/hypot(edge_self2potential.x, edge_self2potential.y) / hypot(edge_robot2goal.x, edge_robot2goal.y);
      if (cos_theta<=0.2){
        is_delete = true;
        break;
      }
    }
    double dis_append = 0;
    if (is_delete)
      dis_append = dis_appened_threshold;

    bool is_cover_obs = false;
    auto path_shortened = shortenPath(path, is_cover_obs);
    auto goal_endpoints = getGoalLineEnds(path.back());

    // 如果path_shortened压到了原始障碍物，则删掉
    if (is_cover_obs)
      continue;

    double dis = 0;
    for (int i=0; i<path_shortened.size()-1; i++){
      dis += hypot(path_shortened[i].x-path_shortened[i+1].x, path_shortened[i].y-path_shortened[i+1].y);
    }
    dis += dis_append;

    // 按照dis升序插入homo_paths_
    for(int i=0; i<dis_list.size(); i++){
      if (dis<dis_list[i]){
        dis_list.insert(dis_list.begin()+i, dis);
        homo_paths_.insert(homo_paths_.begin()+i, transPoint2DinMapToVector2dinWorld(path_shortened));
        // 保存original path
        homo_paths_origin_.insert(homo_paths_origin_.begin()+i, transPoint2DinMapToVector2dinWorld(path));
        goal_endpoints_list.insert(goal_endpoints_list.begin()+i, goal_endpoints);
        break;
      }
    }
    if (dis_list.size()==0)
    {
      dis_list.push_back(dis);
      homo_paths_.push_back(transPoint2DinMapToVector2dinWorld(path_shortened));
      // 保存original path
      homo_paths_origin_.push_back(transPoint2DinMapToVector2dinWorld(path));
      goal_endpoints_list.push_back(goal_endpoints);
    }
    else if (dis>dis_list.back()){
      dis_list.push_back(dis);
      homo_paths_.push_back(transPoint2DinMapToVector2dinWorld(path_shortened));
      // 保存original path
      homo_paths_origin_.push_back(transPoint2DinMapToVector2dinWorld(path));
      goal_endpoints_list.push_back(goal_endpoints);
    }
  }
  // 如果存在满足cos_limitation的轨迹，则仅保留他们；若都不满足，否则保留最短的那个
  int index_path_satisfy_cos_limitation = 0;
  for(auto dis:dis_list){
    if (dis<dis_appened_threshold)
      index_path_satisfy_cos_limitation++;
  }
  int num_delete_path = homo_paths_.size()-index_path_satisfy_cos_limitation;
  // 开始删除
  if (index_path_satisfy_cos_limitation>0){
    homo_paths_.erase(homo_paths_.begin()+index_path_satisfy_cos_limitation, homo_paths_.end());
    goal_endpoints_list.erase(goal_endpoints_list.begin()+index_path_satisfy_cos_limitation, goal_endpoints_list.end());
  }
  else{
    homo_paths_.erase(homo_paths_.end()-num_delete_path, homo_paths_.end());
    goal_endpoints_list.erase(goal_endpoints_list.end()-num_delete_path, goal_endpoints_list.end());
  }
  ROS_INFO("    Finding [%d + %d]/[%d] normal trajectories during findHomoPaths", homo_paths_.size(), num_delete_path, max_path_remained_for_GraphicTEB_);
  return homo_paths_;
}

std::pair<std::pair<double,double>, std::pair<double,double>> mapProcess::getGoalLineEnds(Point2D goal){
  std::pair<std::pair<double,double>, std::pair<double,double>> res = { {0,0}, {0,0}  };
  double min_dis = 99999;
  int closest_index = -1;
  for (int index=goalIndex_start_; index<label_list_.size(); index++){
    for (auto p:border_list_[label_list_[index]]){
      if (hypot(p.x-goal.x, p.y-goal.y)<min_dis){
        min_dis = hypot(p.x-goal.x, p.y-goal.y);
        closest_index = index;
      }
    }
  }
  double w_x1, w_y1, w_x2, w_y2;
  map2world(border_list_[label_list_[closest_index]].front().x, border_list_[label_list_[closest_index]].front().y, w_x1, w_y1);
  map2world(border_list_[label_list_[closest_index]].back().x, border_list_[label_list_[closest_index]].back().y, w_x2, w_y2);

  res = { {w_x1, w_y1}, {w_x2, w_y2} };
  // ROS_INFO("    getGoalLineEnds: %f, %f; %f, %f", res.first.first, res.first.second, res.second.first, res.second.second);
  return res;
}


bool mapProcess::depthFirst(std::vector<int>& visited, std::vector<std::vector<int>>& res){
  // 超出轨迹最大数量
  if (max_path_explore_number_for_GraphicTEB_!=-1 && res.size() >= max_path_explore_number_for_GraphicTEB_)
    return false;
  // 拿出来最后一个点，如果该点是goal点，则形成新的teb轨迹，并检测是否形成新同伦类
  int backID = visited.back();
  if (backID >= goalID_start_ ){
    std::vector<int> path_temp = visited;
    //计算path_temp的长度
    int length_temp = 0;        
    std::vector<Point2D> points_temp;
    for (int i=0; i<path_temp.size()-1; i++){
      auto pointsBet = getShortestPathBetweenObs(path_temp[i], path_temp[i+1]);
      points_temp.push_back(pointsBet.first);
      points_temp.push_back(pointsBet.second);
    }
    for (int i=0; i<points_temp.size()-1; i++){
      length_temp += std::hypot(points_temp[i].x-points_temp[i+1].x, points_temp[i].y-points_temp[i+1].y);
    }    
    //按照长度 升序插入路径
    bool is_insert = false;
    for (int i=0; i<length_approximate_.size();i++){
      if (length_temp<length_approximate_[i]){
        auto pos1 = length_approximate_.begin();
        auto pos2 = res.begin();
        std::advance(pos1,i);
        std::advance(pos2,i);
        length_approximate_.insert(pos1, length_temp);
        res.insert(pos2,path_temp);
        is_insert = true;
        break;
      }
    }

    if (!is_insert){
      length_approximate_.push_back(length_temp);
      res.push_back(path_temp);
    }
    return true;
  }

  // 遍历所有adjacent，得到一些列的候选扩展ID potential_IDs
  // 对adjacent的group排序
  std::vector<std::pair<int,float>> potential_IDs;
  for (auto iter = connection_graph_.begin(); iter!=connection_graph_.end(); iter++){
    if (iter->first.first != backID) continue;    // 需以back这个ID为起点
    int ID2 = iter->first.second;
    auto pois = connection_graph_[{backID, ID2}]["shortest"];

    // goal排在最前面
    if (ID2>=goalID_start_)
      potential_IDs.push_back({ID2, 0});
    else{
      float dis1 = hypot(pois.first.x-pois.second.x, pois.first.y-pois.second.y);
      float dis2 = 99999;
      for (int j=goalIndex_start_; j<label_list_.size(); j++){
        dis2 = std::min(dis2, float(hypot(pois.second.x-border_list_[label_list_[j]].front().x, pois.second.y-border_list_[label_list_[j]].front().y)));
        dis2 = std::min(dis2, float(hypot(pois.second.x-border_list_[label_list_[j]].back().x, pois.second.y-border_list_[label_list_[j]].back().y)));
      }
      float dis = dis1+dis2;
      bool is_insert = false;
      for (int j=0; j<potential_IDs.size(); j++){
        if(dis<potential_IDs[j].second){
          potential_IDs.insert(potential_IDs.begin()+j, {ID2, dis});
          is_insert =true;
          break; 
        }
      }
      if(!is_insert)
        potential_IDs.push_back({ID2,dis});
    }
  }


  // prune策略1: 父亲节点是否层走过某个可扩展层
  std::map<int, bool> is_father_can_visit;
  for (auto iter = potential_IDs.begin(); iter!=potential_IDs.end(); iter++)
    is_father_can_visit[iter->first] = false;
  // bool is_goal_can_directly_reached = false;
  // // start是否可以直接连接到全局目标点
  // for(int i=goalIndex_start_; i<label_list_.size(); i++){
  //   if (connection_graph_.find({startID_, label_list_[i]})!=connection_graph_.end()){
  //     if (goal_line_lists_.size()==1 && goal_line_lists_.front().size()==1){
  //       is_goal_can_directly_reached = true;
  //       break;    
  //     }    
  //   }
  // }  
  // 当开启fathre_visit的limitation 且 起点和终点不可直接连接时（这时他们往往离的很近了），开启此限制：
  // if (is_father_visit_limitation_ && !is_goal_can_directly_reached){
  if (is_father_visit_limitation_){
    for (auto iter = potential_IDs.begin(); iter!=potential_IDs.end(); iter++){
      int potentialID = iter->first;
      for (auto fatherID:visited){
        if (fatherID == backID) continue;
        if (potentialID<goalID_start_ && connection_graph_.find({fatherID, potentialID})!=connection_graph_.end()){
          is_father_can_visit[potentialID] = true;
        }
      }
    }
  }

  // prune策略2 该potentialID是否已经在visited中
  std::map<int, bool> is_potentialID_in_visit;
  for (auto iter = potential_IDs.begin(); iter!=potential_IDs.end(); iter++)
    is_potentialID_in_visit[iter->first] = false;
  for (auto iter=visited.begin(); iter!=visited.end(); iter++){
    int potentialID = *iter;
    is_potentialID_in_visit[potentialID] = true;
  }

  // 确定间之后留下来的IDs
  std::vector<int> remainedIDs;
  // ROS_INFO("%d: ", backID);
  for (auto iter = potential_IDs.begin(); iter!=potential_IDs.end(); iter++){
    int potentialID = iter->first;
    // ROS_INFO("%    d: ", potentialID);
    // prune策略1: 存在（祖）父节点可以走过该点 且该节点不是动态障碍物
    if (is_father_can_visit[potentialID]){
    // if (is_father_can_visit[potentialID] && !map_is_dynamic_[potential_obs_center.x][potential_obs_center.y]){
      // std::cout<<"skip 2: "<<is_father_can_visit[potentialID]<<", "<<map_is_dynamic_[potential_obs_center.x][potential_obs_center.y]<<std::endl;
      // ROS_INFO("skip 1: ");
      continue;      
    }    
    // prune策略2: 已经从该obs走过
    if (is_potentialID_in_visit[potentialID]){
      // std::cout<<"skip 2"<<std::endl;
      continue;
    }
    // prune策略3: 反向行走(cos limitation)
    bool is_delete = false;
    // 起点开始的连接
    if (backID==startID_){
      assert((connection_graph_[{backID, potentialID}].size() == 1));
      auto pois = connection_graph_[{backID, potentialID}]["shortest"];
      auto edge_self2potential = Point2D(pois.second.x-pois.first.x, pois.second.y-pois.first.y);
      auto edge_robot2goal = Point2D(goal_in_map_x_-start_in_map_x_, goal_in_map_y_-start_in_map_y_);
      auto cos_theta = (edge_self2potential.x * edge_robot2goal.x + edge_self2potential.y*edge_robot2goal.y)/hypot(edge_self2potential.x, edge_self2potential.y) / hypot(edge_robot2goal.x, edge_robot2goal.y);
      if (cos_theta<=0.2 && abs(edge_self2potential.x)+abs(edge_self2potential.y)>2)
        is_delete =true;
    }
    // group间的连接
    else{
      int vilation_count = 0;
      assert( (connection_graph_[{backID, potentialID}].size()==5)  );
      auto edge_robot2goal = Point2D(goal_in_map_x_-start_in_map_x_, goal_in_map_y_-start_in_map_y_);
      for(auto connection:connection_graph_[{backID, potentialID}]){
        if(connection.first == "shortest")
          continue;
        auto pois = connection.second;
        auto edge_self2potential = Point2D(pois.second.x-pois.first.x, pois.second.y-pois.first.y);
        auto cos_theta = (edge_self2potential.x * edge_robot2goal.x + edge_self2potential.y*edge_robot2goal.y)/hypot(edge_self2potential.x, edge_self2potential.y) / hypot(edge_robot2goal.x, edge_robot2goal.y);    
        if (cos_theta<=0.2)
          vilation_count++;
        if (vilation_count>=3)
          is_delete = true;
      }
    }
    // if (is_cos_limitation_ && cos_theta <= 0.0)
    if (is_cos_limitation_ && is_delete){
      // std::cout<<"skip 3: "<<cos_theta<<", ("<<edge_self2potential.x<<","<<edge_self2potential.y<<"), ("<<edge_robot2goal.x<<","<<edge_robot2goal.y<<")"<<std::endl;
      // std::cout<<"skip 3"<<std::endl;
      continue;
    }
    // ROS_INFO("cos_theta: %f, %f",cos_theta, std::acos(cos_theta));
    remainedIDs.push_back(potentialID);
  }

  // 扩展adjacent obs
  for (auto potentialID:remainedIDs){
    visited.push_back(potentialID);
    depthFirst(visited, res);
    visited.pop_back();
  }
  return false;
}



// 选择每个障碍物的边界点作为该障碍物的identification，outline一下
// 通过每个逆时针排序的外檐有序点 在 内檐点集中进行顺时针旋转，来获取内檐边界的逆时针序列
std::map<int, std::vector<std::pair<double,double>>> mapProcess::getStaticObsIdentificationInWorld(){
  // 1. 对每个障碍物群 收缩0.2m，得到map_obs_shrinked_labeled_
  map_obs_shrinked_labeled_ = std::vector<std::vector<int>> (width_large_, std::vector<int>(height_large_));
  cv::Mat dst1, dst2, comp;
  cv::erode(map_cv_without_dynamic_obs_, dst1, cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3)),cv::Point(-1,-1),2); // 这里用 2*0.1=0.2m更合适。。 少缩0.1m
  cv::erode(dst1, dst2, cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3)));
  cv::compare(dst1, dst2, comp, cv::CMP_NE);
  // std::cout<<std::endl<<cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(6, 6))<<std::endl;
  // cv::imshow("Canny edge detection", comp);
  // cv::waitKey(0);
  // 赋值腐蚀后的障碍物地图
  for(int j=shrink_dis_; j < dst1.rows-shrink_dis_; j++)
    for(int i=shrink_dis_; i<dst1.cols-shrink_dis_;i++)
      if(dst1.at<uint8_t>(j,i) && (map_obs_labeled_[i-shrink_dis_][j-shrink_dis_]<startID_))
        map_obs_shrinked_labeled_[i][j] = map_obs_labeled_[i-shrink_dis_][j-shrink_dis_];

  // 2. 对收缩0.2m后的障碍物群的边界点，得到border_labeled_temp
  auto border_labeled_temp = std::vector<std::vector<int>> (width_, std::vector<int>(height_,0));
  std::vector<Point2D> kernel = {Point2D(-1,-1),Point2D(-1,0),Point2D(-1,1),Point2D(0,-1),Point2D(0,1),Point2D(1,-1),Point2D(1,0),Point2D(1,1)};
  for(int j=shrink_dis_; j < comp.rows-shrink_dis_; j++){
    for(int i=shrink_dis_; i<comp.cols-shrink_dis_;i++){
      // 只要comp中的元素是非空的，那么它就是边缘点
      if(comp.at<uint8_t>(j,i) && map_obs_labeled_[i-shrink_dis_][j-shrink_dis_]<startID_){
        // 如果这个点本身就有ID，那是最好的
        if (map_obs_labeled_[i-shrink_dis_][j-shrink_dis_]!=0)
          border_labeled_temp[i-shrink_dis_][j-shrink_dis_] = map_obs_labeled_[i-shrink_dis_][j-shrink_dis_];
        // 在附近找这个border点的ID
        else{
          double min_dis = 999;
          int ID=0;
          for (int dx=-ignore_dis_; dx<ignore_dis_ && ID==0; dx++){
            for (int dy=-ignore_dis_; dy<ignore_dis_ && ID==0; dy++){
              if (checkInMap(i-shrink_dis_+dx,j-shrink_dis_+dy) && map_obs_labeled_[i-shrink_dis_+dx][j-shrink_dis_+dy]!=0 && map_obs_labeled_[i-shrink_dis_+dx][j-shrink_dis_+dy]<startID_){
                if (hypot(dx,dy)<min_dis){
                  min_dis  = hypot(dx,dy);
                  ID = map_obs_labeled_[i-shrink_dis_+dx][j-shrink_dis_+dy];
                }
              }
            }
          }
          border_labeled_temp[i-shrink_dis_][j-shrink_dis_] = ID;
        }
      }
    }
  }

  // 3. 对border_labeled_temp中的border点进行顺时针排序，得到border_list_temp
  std::map<int, std::vector<Point2D>> border_list_temp;
  auto map_is_visited_temp = std::vector<std::vector<bool>> (width_, std::vector<bool>(height_, false));
  // 遍历时的顺序为：左、上、左上、下、左下、右、右上、右下。即：上与右优先遍历、非斜线优先遍历，保证borderlist是顺时针的
  std::vector<Point2D> kernel2 = {Point2D(-1,0),Point2D(0,1),Point2D(-1,1),Point2D(0,-1),Point2D(-1,-1),Point2D(1,0),Point2D(1,1),Point2D(1,-1)};
  for (int y=0; y<border_labeled_temp.front().size(); y++){
    for (int x=0; x<border_labeled_temp.size(); x++){
      auto label = border_labeled_temp[x][y];
      // 如果是非border点或者被visited过的点，则continue
      if (label == 0 || map_is_visited_temp[x][y]==true)
        continue;
      // 标记当前点为已读
      Point2D p = Point2D(x,y);
      map_is_visited_temp[p.x][p.y] = true;
      border_list_temp[label].push_back(Point2D(x,y,0,label));
      // 深搜，进行归并
      deepSearchForFindingBorders(border_list_temp, border_labeled_temp, p, map_is_visited_temp, kernel2, p, label);
    }
  }

  // 4. 遍历border_list_temp，如果一个border list内部的某两个的欧式距离>2，则他会被切分为两个border list，并被赋予新的label_index_
  std::map<int,std::vector<Point2D>> remained_border;
  label_index_++;
  for (auto border=border_list_temp.begin(); border!=border_list_temp.end();border++){
    // 起点不要
    if (border->second.size()<=1)
      continue;
    auto iter = border->second.begin();
    while (iter!=border->second.end()){
      auto iter_next = std::next(iter);
      if (iter_next == border->second.end())
        break;
      if (abs(iter->x-iter_next->x)+abs(iter->y-iter_next->y)>4){
        // 分段保存多余点
        std::vector<Point2D> temp(iter_next, border->second.end());
        // 如果长度只有1，直接扔掉就好啦
        if (temp.size()!=1){
          int start_index = 0, end_index = 1;
          // start_index和end_index双指针前进，距离小于4且连续的一段 会被保存到remained_border中并赋予新的label_index_
          while (end_index<temp.size()){
            if (abs(temp[end_index].x-temp[end_index-1].x)+abs(temp[end_index].y-temp[end_index-1].y)>4){
              if (end_index-start_index>1){
                for (int i=start_index; i<=end_index-1; i++)
                  remained_border[label_index_].push_back(temp[i]);
                label_index_++;
              }
              start_index = end_index;
            }
            end_index++;
          }
          // 针对最后一段的添加
          if (end_index-start_index>1){
            for (int i=start_index; i<=end_index-1; i++)
              remained_border[label_index_].push_back(temp[i]);
            label_index_++;
          }
        }
        // 在border中去除掉多余点
        border->second.erase(iter_next, border->second.end());
        break;
      }
      iter++;
    }
  }
  for (auto p:remained_border)
    border_list_temp[p.first] = p.second;

  // 将border_list_temp中的点转为世界坐标系下的点，并存入res中。实际上这里是把每个border points转化成连续的polygon的形式
  std::map<int, std::vector<std::pair<double,double>>> res;
  double wx,wy;
  for (auto border:border_list_temp){
    for (auto p:border.second){
      map2world(p.x, p.y, wx, wy);
      res[border.first].push_back(std::pair<double, double>(wx,wy));
    }
  }

  return res;
}


std::map<int, std::vector<double>> mapProcess::getDynamicObsIdentificationInWorld(){
  // 对应dynamic obstacle的ID和位置
  std::map<int, std::vector<double>> res;
  if (!dynamic_obstacle_.empty()){
    for (auto p:dynamic_obstacle_){
      int mx, my;
      world2map(mx,my,p[0],p[1]);
      if (mx>=ignore_dis_ && mx<=width_-ignore_dis_ && my>=ignore_dis_ && my<=height_-ignore_dis_){
        int index = map_obs_labeled_[mx][my];
        assert(index!=0);
        res[index] = p;
      }
    }
  }
  // 找static和dynamic obs的IDs
  for (int i=0; i<startIndex_; i++){
    if (res.find(i)==res.end())
      label_static_list_.push_back(i);
    else
      label_dynamic_list_.push_back(i);
  }

  return res;
}

void mapProcess::add_to_homo_paths_pruned(std::vector<Eigen::Vector2d> path){
  homo_paths_add_to_TEB_.push_back(path);
}



std::vector<Point2D_float> mapProcess::shortenPath (std::vector<Point2D> path, bool& is_delete){
  // 构造float类型的轨迹
  std::vector<Point2D_float> path_float;
  for (auto p:path){
    path_float.push_back(Point2D_float(p.x, p.y));
  }
  // 把start和goal lines也投影到map_border_labeled_中，得到temp_map
  auto temp_map = map_border_labeled_;
  for (int i=startIndex_; i<label_list_.size(); i++){
    for (auto p:border_list_[label_list_[i]])
      temp_map[p.x][p.y] = label_list_[i];
  }

  // 填补空白
  int index = 1;
  while (index<path_float.size()){
    auto poi1 = path_float[index-1];
    auto poi2 = path_float[index];
    if (abs(poi1.x-poi2.x)+ abs(poi1.y-poi2.y)>2){
      if (temp_map[int(poi1.x)][int(poi1.y)]==temp_map[int(poi2.x)][int(poi2.y)]){
        auto vec_line = Bresenham(poi1, poi2);
        vec_line.pop_back();
        vec_line.erase(vec_line.begin());
        if (!vec_line.empty()){
          path_float.insert(path_float.begin()+index, vec_line.begin(), vec_line.end());
          index += vec_line.size();
        }
      }
      else{
        path_float[index-1].setShorten();
        path_float[index].setShorten();
        auto vec_line = BresenhamDirect(poi1, poi2);
        vec_line.pop_back();
        vec_line.erase(vec_line.begin());
        if (!vec_line.empty()){
          path_float.insert(path_float.begin()+index, vec_line.begin(), vec_line.end());
          index += vec_line.size();
        }        
      }
    }
    else
      index++;
  }
  // group间的shorten
  index = 0;
  while (index<path_float.size()){
    if (path_float[index].is_shorten_key_point){
      int d_start = 0;
      int d_end = 0;
      int threhold = 30;
      threhold = std::min(threhold, index);
      threhold = std::min(threhold, int(path_float.size()-index-1));
      while (d_start<threhold && d_end<threhold){
        // if(!checkCollisionOnlyObs(BresenhamDirect(path_float[index- (d_start+1)], path_float[index+ (d_end+1)]))){
        if(!checkCollisionOnlyObsAlignedWithBresenhamDirect(path_float[index- (d_start+1)], path_float[index+ (d_end+1)])){
          d_start++;
          d_end++;
        }
        // else if (!checkCollisionOnlyObs(BresenhamDirect(path_float[index- (d_start)], path_float[index+ (d_end+1)])))
        else if (!checkCollisionOnlyObsAlignedWithBresenhamDirect(path_float[index- (d_start)], path_float[index+ (d_end+1)]))
          d_end++;
        // else if (!checkCollisionOnlyObs(BresenhamDirect(path_float[index- (d_start+1)], path_float[index+ (d_end)])))
        else if (!checkCollisionOnlyObsAlignedWithBresenhamDirect(path_float[index- (d_start+1)], path_float[index+ (d_end)]))
          d_start++;
        else
          break;
      }
      int erase_start = index-d_start + 1;
      int erase_end = index+d_end;
      if (erase_start<erase_end){
        path_float.erase(path_float.begin()+erase_start, path_float.begin()+erase_end);
        index -= d_start;
      }
    }
    index++;
  }

  // 填充中间点
  index = 1;
  while(index<path_float.size()){
    if ( abs(path_float[index].x-path_float[index-1].x) + abs(path_float[index].y-path_float[index-1].y) >2 ){
      auto vec_line = BresenhamDirect(path_float[index-1], path_float[index]);
      vec_line.pop_back();
      vec_line.erase(vec_line.begin());
      path_float.insert(path_float.begin()+index, vec_line.begin(),vec_line.end());
      index += vec_line.size();
    }
    index++;
  }
  
  // 二次，针对group内的点的shorten
  index = 0;
  while (index<path_float.size()){
    int d_start = 0;
    int d_end = 0;
    int threhold = 10;
    threhold = std::min(threhold, index);
    threhold = std::min(threhold, int(path_float.size()-index-1));
    while (d_start<threhold && d_end<threhold){
      // if(temp_map[path_float[index- (d_start+1)].x][path_float[index- (d_start+1)].y]==temp_map[path_float[index+ (d_end+1)].x][path_float[index+ (d_end+1)].y] && !checkCollisionOnlyObs(BresenhamDirect(path_float[index- (d_start+1)], path_float[index+ (d_end+1)]))){
      if(temp_map[path_float[index- (d_start+1)].x][path_float[index- (d_start+1)].y]==temp_map[path_float[index+ (d_end+1)].x][path_float[index+ (d_end+1)].y] && !checkCollisionOnlyObsAlignedWithBresenhamDirect(path_float[index- (d_start+1)], path_float[index+ (d_end+1)])){
        d_start++;
        d_end++;
      }
      // else if (temp_map[path_float[index- d_start].x][path_float[index- d_start].y]==temp_map[path_float[index+ (d_end+1)].x][path_float[index+ (d_end+1)].y]  &&  !checkCollisionOnlyObs(BresenhamDirect(path_float[index- (d_start)], path_float[index+ (d_end+1)])))
      else if (temp_map[path_float[index- d_start].x][path_float[index- d_start].y]==temp_map[path_float[index+ (d_end+1)].x][path_float[index+ (d_end+1)].y]  &&  !checkCollisionOnlyObsAlignedWithBresenhamDirect(path_float[index- (d_start)], path_float[index+ (d_end+1)]))
        d_end++;
      // else if (temp_map[path_float[index- (d_start+1)].x][path_float[index- (d_start+1)].y]==temp_map[path_float[index+ d_end].x][path_float[index+ d_end].y]  &&  !checkCollisionOnlyObs(BresenhamDirect(path_float[index- (d_start+1)], path_float[index+ (d_end)])))
      else if (temp_map[path_float[index- (d_start+1)].x][path_float[index- (d_start+1)].y]==temp_map[path_float[index+ d_end].x][path_float[index+ d_end].y]  &&  !checkCollisionOnlyObsAlignedWithBresenhamDirect(path_float[index- (d_start+1)], path_float[index+ (d_end)]))
        d_start++;
      else
        break;
    }
    int erase_start = index-d_start + 1;
    int erase_end = index+d_end;
    if (erase_start<erase_end){
      path_float.erase(path_float.begin()+erase_start, path_float.begin()+erase_end);
      index -= d_start;
    }
    index++;
  }
  
  // 填充中间点
  index = 1;
  while(index<path_float.size()){
    if ( abs(path_float[index].x-path_float[index-1].x) + abs(path_float[index].y-path_float[index-1].y) >2 ){
      auto vec_line = BresenhamDirect(path_float[index-1], path_float[index]);
      vec_line.pop_back();
      vec_line.erase(vec_line.begin());
      path_float.insert(path_float.begin()+index, vec_line.begin(),vec_line.end());
      index += vec_line.size();
    }
    index++;
  }

  int count = 0;
  for (auto p:path_float){
    if (p.x<=ignore_dis_ || p.x>=width_-ignore_dis_ || p.y<=ignore_dis_ || p.y>=height_-ignore_dis_){
      std::vector<std::pair<float,float>> around_points = {{floor(p.x), floor(p.y)}, {floor(p.x), ceil(p.y)}, {ceil(p.x), floor(p.y)}, {ceil(p.x), ceil(p.y)} };
      int temp_count = 0;
      for (auto poi:around_points){
        int x = int(poi.first);
        int y = int(poi.second);
        if (!(x>0 && x<width_ && y>0 && y<height_))
          temp_count++;
        else if (map_obs_occupied_without_ignore_[x][y])
          temp_count++;
      }
      if (temp_count>=3)        
        count++;
    }
  }
  if (count>=3)
    is_delete = true;

  // 轨迹的稀疏化处理
  index = 0;
  int step = 7;
  double max_dis = 0.4 * 10;
  while (index<path_float.size()){
    int end_index = std::min(index+step, int(path_float.size())-1);
    while(end_index>index){
      // if(!checkCollisionOnlyObs(BresenhamDirect(path_float[index], path_float[end_index])) && hypot(path_float[index].x-path_float[end_index].x, path_float[index].y-path_float[end_index].y)<=max_dis ){
      if(!checkCollisionOnlyObsAlignedWithBresenhamDirect(path_float[index], path_float[end_index]) && hypot(path_float[index].x-path_float[end_index].x, path_float[index].y-path_float[end_index].y)<=max_dis ){
        path_float.erase(path_float.begin()+index+1, path_float.begin()+end_index);
        break;
      }
      end_index--;
    }
    index++;
  }


  return path_float;
}