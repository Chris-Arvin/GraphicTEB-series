#include <teb_local_planner/map_process.h>

// initialize maps derived from costmap
mapProcess::mapProcess(const nav_msgs::OccupancyGrid::ConstPtr& msg){
    map_vector_ = msg->data;
    width_ = msg->info.width;
    height_ = msg->info.height;
    resolution_ = msg->info.resolution;
    origin_x_ = msg->info.origin.position.x;
    origin_y_ = msg->info.origin.position.y;
    map_obs_labeled_ = std::vector<std::vector<int>> (width_, std::vector<int>(height_));
    map_border_labeled_ = std::vector<std::vector<int>> (width_, std::vector<int>(height_));
    map_corner_labeled_ = std::vector<std::vector<int>> (width_, std::vector<int>(height_));
    for (auto point:map_vector_){
      if (point != (char)0){
        map_int_.push_back('d');
      }
      else{
        map_int_.push_back((uint8_t)point);
      }
    }
    map_cv_ = convertVector2Mat(map_int_, 1, height_);
}

// initialize maps derived from costmap
mapProcess::mapProcess(costmap_2d::Costmap2D* costmap, const teb_local_planner::PoseSE2& start, const teb_local_planner::PoseSE2& goal, const std::vector<std::vector<double>>& dynamic_obstacle){
  dynamic_obstacle_ = dynamic_obstacle;
  costmap_ = costmap;
  width_ = costmap_->getSizeInCellsX();
  height_ = costmap_->getSizeInCellsY();
  resolution_ = costmap_->getResolution();
  origin_x_ = costmap_->getOriginX();
  origin_y_ = costmap_->getOriginY();
  // project map
  char cost_translation_table[256];
  cost_translation_table[0] = 0;  // NO obstacle
  cost_translation_table[253] = 99;  // INSCRIBED obstacle
  cost_translation_table[254] = 100;  // LETHAL obstacle
  cost_translation_table[255] = -1;  // UNKNOWN
  for (int i = 1; i < 253; i++)
    cost_translation_table[ i ] = char(1 + (97 * (i - 1)) / 251);
  // initialize vectors
  map_obs_labeled_ = std::vector<std::vector<int>> (width_, std::vector<int>(height_));
  map_obs_labeled_outline_ = std::vector<std::vector<bool>> (width_, std::vector<bool>(height_, false));
  map_obs_shrinked_labeled_ = std::vector<std::vector<int>> (width_, std::vector<int>(height_));
  map_border_labeled_ = std::vector<std::vector<int>> (width_, std::vector<int>(height_));
  map_corner_labeled_ = std::vector<std::vector<int>> (width_, std::vector<int>(height_));
  // assign map
  unsigned char* data = costmap_->getCharMap();
  // assign static obstacles
  for (unsigned int i = 0; i < width_*height_; i++)
    map_vector_.push_back(cost_translation_table[ data[ i ]]);
  auto map_vector_copied = map_vector_;
  // assign dynamic pedestrians
  rob_radius_ = 0.2;
  obs_radius_ = 0.3;
  length_in_map_ = (rob_radius_+obs_radius_)/resolution_;
  for (auto p:dynamic_obstacle_){
    int mx, my;
    world2map(mx,my,p[0],p[1]);
    for (int dx = -length_in_map_; dx<=length_in_map_; dx++){
      for (int dy=-length_in_map_; dy<=length_in_map_; dy++){
        if (checkInMap(mx+dx,my+dy) && std::hypot(dx,dy)<=length_in_map_){
          map_vector_[mx+dx+(my+dy)*width_] = 100;
        }
      }
    }
  }
  // set the borderlines of map_obs_labeled_outline_ as true
  for (int i=0; i<width_; i++){
    if (map_vector_[i] != (char)0)
      map_obs_labeled_outline_[i][0] = true;
    if (map_vector_[map_vector_.size()-1-i] != (char)0)
      map_obs_labeled_outline_[width_-1-i][height_-1] = true;
  }
  for (int j=0; j<height_; j++){
    if (map_vector_[j*width_] != (char)0)
      map_obs_labeled_outline_[0][j] = true;
    if (map_vector_[j*width_+width_-1] != (char)0)
      map_obs_labeled_outline_[width_-1][j] = true;
  }
  // set the borderlines of [map_vector_ and map_vector_copied] as 0
  for (int i=0; i<width_; i++){
    map_vector_[i] = 0;
    map_vector_[map_vector_.size()-1-i] = 0;
    map_vector_copied[i] = 0;
    map_vector_copied[map_vector_copied.size()-1-i] = 0;
  }
  for (int j=0; j<height_; j++){
    map_vector_[j*width_] = 0;
    map_vector_[j*width_+width_-1] = 0;
    map_vector_copied[j*width_] = 0;
    map_vector_copied[j*width_+width_-1] = 0;
  }
  // translate start and goal points into map frame (resolution)
  world2map(start_in_map_x_, start_in_map_y_, start.position().x(), start.position().y());
  world2map(goal_in_map_x_, goal_in_map_y_, goal.position().x(), goal.position().y());
  // convert [map_cv_ and map_cv_without_dynamic_obs_] with the unit being 0 or 1
  std::vector<uint8_t> map_int_copied;
  for (int i=0; i<map_vector_.size(); i++){
    // map_vector
    auto p1 = map_vector_[i];
    if (p1 != (char)0)
      map_int_.push_back('d');
    else
      map_int_.push_back((uint8_t)p1);
    // map_vector_copied
    auto p2 = map_vector_copied[i];
    if (p2 != (char)0)
      map_int_copied.push_back('d');
    else
      map_int_copied.push_back((uint8_t)p2);
  }
  map_cv_ = convertVector2Mat(map_int_, 1, height_);
  map_cv_without_dynamic_obs_ = convertVector2Mat(map_int_copied, 1, height_);
}

// convert 1D vector into 2D mat
cv::Mat mapProcess::convertVector2Mat(const std::vector<uint8_t>& v, int channels, int rows)
{
	cv::Mat mat = cv::Mat(v);//将vector变成单列的mat
	cv::Mat dest = mat.reshape(channels, rows).clone();//PS：必须clone()一份，否则返回出错
  return dest;
}

// translate a point between map frame and world frame
void mapProcess::map2world(int mx, int my, double& wx, double& wy){
  wx = origin_x_ + (mx + 0.5) * resolution_;
  wy = origin_y_ + (my + 0.5) * resolution_;
}
void mapProcess::world2map(int& mx, int& my, double wx, double wy){
  mx = (wx-origin_x_)/resolution_;
  my = (wy-origin_y_)/resolution_;
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
      char value = map_int_[x+y*width_];
      // 左上的四邻接 检测是否可以connect
      if (value=='d'){
        addObsConnect(x,y);
      }
      x--;
      y++;
    }
  }

  startID_ = label_index_++;
  goalID_ = label_index_++;
  obs_list_[startID_] = {Point2D(start_in_map_x_,start_in_map_y_,0,startID_)};
  obs_list_[goalID_] = {Point2D(goal_in_map_x_,goal_in_map_y_,0,goalID_)};


  // 记录所有label
  for (const auto& i:obs_list_){
    label_list_.push_back(i.first);
  }
  // initialize visited_
  for (int i=0; i<obs_list_.size();i++){
    visited_.push_back({});
    for (int j=0; j<obs_list_.size();j++){
      visited_[i].push_back(false);
    }
  }
  // initialize edges_graph_
  std::vector<std::pair<Point2D, Point2D>> temp1 = {};
  std::pair<Point2D, Point2D> temp2 = {};
  for (int i=0; i<obs_list_.size();i++){
    edges_graph_.push_back(temp1);
    for (int j=0; j<obs_list_.size();j++){
      edges_graph_[i].push_back(temp2);
    }
  }
  // initialize covers_graph_
  for (int i=0; i<obs_list_.size();i++){
    covers_graph_.push_back({});
    for (int j=0; j<obs_list_.size();j++){
      covers_graph_[i].push_back({});
    }
  }
  // assign startID and goalID
  for(int i=0; i<label_list_.size(); i++){
    if (label_list_[i] == startID_){
      startIndex_ = i;
    }
    if(label_list_[i]==goalID_){
      goalIndex_ = i;
    }
  }
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

bool mapProcess::checkInMap(int x, int y){
  if (x>=0 && x<width_ && y>=0 && y<height_){
    return true;
  }
  return false;
}

std::map<int, std::vector<Point2D>>mapProcess::borderIdentify(){
  cv::Mat edge_map = map_cv_;
  cv::Mat dst, comp;
  cv::dilate(map_cv_, dst, cv::Mat());
  cv::compare(edge_map, dst, comp, cv::CMP_NE);
  // cv::imshow("Canny edge detection", comp);
  // cv::waitKey(0);

  // 这个循环用于保存map_border_labeled_这个地图
  std::vector<Point2D> kernel = {Point2D(-1,-1),Point2D(-1,0),Point2D(-1,1),Point2D(0,-1),Point2D(0,1),Point2D(1,-1),Point2D(1,0),Point2D(1,1)};
  for(int j=0; j < comp.rows; j++)
  {
      for(int i=0; i<comp.cols;i++)
      {
          // 只要comp中的元素是非空的，那么它就是边缘点
          if(comp.at<uint8_t>(j,i))
          {
              // 八临接找label
              int label = 0;
              for (auto delta:kernel){
                if (checkInMap(i+delta.x, j+delta.y) && map_obs_labeled_[i+delta.x][j+delta.y]!=0){
                  label = map_obs_labeled_[i+delta.x][j+delta.y];
                  break;
                }
              }
              // assert(label!=0); //除了0 还有可能是start或gal
              // 添加border点到border map中
              map_border_labeled_[i][j] = label;
          }
      }
  }

  // 扩展归并，拟合border_list
  auto map_is_visited_temp = std::vector<std::vector<bool>> (width_, std::vector<bool>(height_, false));
  // 遍历时的顺序为：右、上、右上、下、右下、左、左上、左下。即：上与右优先遍历、非斜线优先遍历，保证borderlist是顺时针的
  std::vector<Point2D> kernel2 = {Point2D(1,0),Point2D(0,1),Point2D(1,1),Point2D(0,-1),Point2D(1,-1),Point2D(-1,0),Point2D(-1,1),Point2D(-1,-1)};
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

  // auto border = border_list_.begin();
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
  border_list_[startID_] = {Point2D(start_in_map_x_,start_in_map_y_,0,startID_)};
  border_list_[goalID_] = {Point2D(goal_in_map_x_,goal_in_map_y_,0,goalID_)};
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
    cv::Mat cornered_map;
    cornered_map = map_cv_;
    // cvtColor(map_cv_, cornered_map, cv::COLOR_BGR2GRAY);
    cv::Mat dst, dst_norm;
    // Harris corner detect
    // https://blog.csdn.net/fengweichangzi/article/details/119001661
    // int kThresh = 118;
    int kThresh = 80;
    int kBlockSize = 2;    
    int kApertureSize = 3;
    double k = 0.04;
    cv::cornerHarris(cornered_map, dst, kBlockSize, kApertureSize, k);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1);

    // draw detected corners，这个循环只修改地图map_coner_labeled
    std::vector<Point2D> kernel = {Point2D(-1,-1),Point2D(-1,0),Point2D(-1,1),Point2D(0,-1),Point2D(0,-1),Point2D(1,-1),Point2D(1,0),Point2D(1,1)};
    for(int j=0; j < dst_norm.rows; j++)
    {
        for(int i=0; i<dst_norm.cols;i++)
        {
            if((int)dst_norm.at<float>(j,i) > kThresh)
            {
              // 八临接找label，归并到border上
              int label = 0;
              int x,y;
              for (auto delta:kernel){
                if (checkInMap(i+delta.x,j+delta.y) && map_border_labeled_[i+delta.x][j+delta.y]!=0){
                  label = map_border_labeled_[i+delta.x][j+delta.y];
                  x = i+delta.x;
                  y = j+delta.y;
                  break;
                }
              }
              if (label !=0 && map_corner_labeled_[x][y]==0){
                map_corner_labeled_[x][y] = label;
              }

            }
        }
    }

    // 从border_list中拿出corner_list,并且将border_list中 是corder的值的value改成1
    for (auto iter_o=border_list_.begin(); iter_o!=border_list_.end();iter_o++){
      int label = iter_o->first;
      for(auto iter_i=iter_o->second.begin();iter_i!=iter_o->second.end();iter_i++){
        if (map_corner_labeled_[iter_i->x][iter_i->y] != 0){
          // corner_list_[map_corner_labeled_[iter_i->x][iter_i->y]].push_back(Point2D(iter_i->x,iter_i->y));
          iter_i->value = 1;
        }
      }
    }
    
    // 遍历，用动态窗口（掩模）找到最大的corner_list组合
    for (auto iter_o=border_list_.begin(); iter_o!=border_list_.end();iter_o++){
      int label = iter_o->first;
      int label_len = iter_o->second.size();
      int step = (floor)(float(label_len)/8.0);   // 将整个border均等分成8份（对应8临界矩阵），每份之间的步长是step
      // 将step约束到0~10
      while (step>=20){
        step /= 2;
      }
      int nums; // 总步长共nums步
      if (step==0)
        nums = 1;
      else
        nums = (ceil)(float(label_len)/step);   
      int kernel[nums];  // 按照step，确定kernel的点位（差值）
      for (int i=0; i<nums; i++)
        kernel[i] = i*step;
      int max_inner = 0;
      int max_start_index = -1;
      for(int inner=0; inner< label_len; inner++){
        int temp = 0;
        for (auto delta:kernel){
          temp += iter_o->second[(inner + delta + label_len)%label_len].value;
        }
        if (temp > max_inner){
          max_inner = temp;
          max_start_index = inner;
        }
      }

      std::vector<int> temp_vec;
      for (auto delta:kernel){
        // value值保存的是该点在border_list中的index
        temp_vec.push_back((max_start_index + delta+label_len)%label_len);
      }
      std::sort(temp_vec.begin(), temp_vec.end());
      for (auto index:temp_vec){
        corner_list_[label].push_back(Point2D(iter_o->second[index].x, iter_o->second[index].y, 0, label));
      }

      corner_list_[startID_] = {Point2D(start_in_map_x_,start_in_map_y_,0,startID_)};
      corner_list_[goalID_] = {Point2D(goal_in_map_x_,goal_in_map_y_,0,goalID_)};
    }        
    return corner_list_;
}

/** 
 * @return .first: clockwise, .second counter-clockwise
 * 
 * **/
std::pair<std::vector<Point2D>,std::vector<Point2D>> mapProcess::getPathAlongsideObs(int obsID, Point2D P_start, Point2D P_target){

  int index_start=-1, index_target=-1;
  int i=0;
  const auto& vector_list = border_list_[obsID];
  // const auto& vector_list = corner_list_[obsID];
  for(auto it = vector_list.begin(); it!=vector_list.end(); it++){
    if (it->x==P_start.x && it->y==P_start.y){
      index_start = i;
    }
    if (it->x==P_target.x && it->y==P_target.y){
      index_target = i;
    }
    ++i;
  }
  assert(index_start!=-1);
  assert(index_target!=-1);
  std::vector<Point2D> counter_clockwise;
  std::vector<Point2D> clockwise;
  if (index_target >= index_start){
    for (int j=index_start; j<=index_target; j++)
      counter_clockwise.push_back(vector_list[j]);
    for (int j=index_start; j>=0; j--)
      clockwise.push_back(vector_list[j]);
    for (int j =vector_list.size()-1; j>=index_target; j--)
      clockwise.push_back(vector_list[j]);
  }
  else{
    for (int j=index_start; j<=vector_list.size()-1; j++)
      counter_clockwise.push_back(vector_list[j]);
    for (int j=0; j<=index_target; j++)
      counter_clockwise.push_back(vector_list[j]);
    for (int j=index_start; j>=index_target; j--)
      clockwise.push_back(vector_list[j]);
  }
  // std::cout<<("---")<<std::endl;
  // std::cout<<index_target<<", "<<index_start<<std::endl;;
  // std::cout<<obsID<<","<<P_start.x<<","<<P_start.y<<"; "<<P_target.x<<"."<<P_target.y<<std::endl;
  // printVector(vector_list);
  // printVector(clockwise);
  // printVector(counter_clockwise);
  // std::cout<<("---")<<std::endl;
  return std::pair<std::vector<Point2D>,std::vector<Point2D>> (clockwise, counter_clockwise);
}


// 深度优先搜索
std::vector<std::vector<std::pair<Point2D, Point2D>>> mapProcess::createPathBetweenObs(){
  for (int i=0; i<label_list_.size();i++){
    for (int j=i+1; j<label_list_.size();j++){
      if (!isVisited(label_list_[i],label_list_[j])){
        setVisited(label_list_[i],label_list_[j]);
        findPathBetweenObs(label_list_[i],label_list_[j]);
      }
    }
  }

  return edges_graph_;
}

// 检测corner1和corner2之间的连线，如果cover到了自身障碍物，那么说明该index1和index2不够好，进行内部的跳转;
// 之后检测这条最短的线是否cover到了其他障碍物，如果cover了，进行深搜
std::vector<int> mapProcess::findPathBetweenObs(int obsID1, int obsID2){
  std::vector<Point2D>& sub_corner_list_ID1 = corner_list_[obsID1];
  std::vector<Point2D>& sub_corner_list_ID2 = corner_list_[obsID2];
  int index1=0, last_index1=0, last_last_index1=0, index2=0, last_index2=0, last_last_index2=0;
  Point2D corner1 = sub_corner_list_ID1[index1];
  Point2D corner2 = sub_corner_list_ID2[index2];
  Point2D corner1_nearest_border, corner2_nearest_border;
  double dis11, dis12, dis13, min_dis1;
  int ID1_size=sub_corner_list_ID1.size(), dir1=0;
  double dis21, dis22, dis23, min_dis2;
  int ID2_size=sub_corner_list_ID2.size(), dir2=0;
  double current_min_dis, his_min_dis = 9999;
  bool is_jump = false;   // 保存是否经过了障碍物内部的跳点，这影响了while循环的终止条件
  std::vector<int> obs_in_ID1_and_ID2;
  std::vector<int> res1, res2;
  // 在这个while中，检测corner1和corner2之间的连线，如果cover到了自身障碍物，那么说明该index1和index2不够好，进行内部的跳转;
  while (true){
    is_jump = false;
    auto cover_check = checkCoverBorder(corner1, corner2);
    corner1_nearest_border = cover_check.first;
    corner2_nearest_border = cover_check.second;

    // 障碍物内部 进行跳跃式地更新
    // 确保最近点不是自己
    if (!(corner1_nearest_border.x==corner1.x && corner1_nearest_border.y==corner1.y)){
      last_last_index1 = last_index1;
      last_index1 = index1;
      index1 = findNearestCornerFromBorder (corner1_nearest_border);
      corner1 = sub_corner_list_ID1[index1];
      is_jump = true;
    }

    if (corner2_nearest_border.id == obsID2){
      if (!(corner2_nearest_border.x==corner2.x && corner2_nearest_border.y==corner2.y)){
        last_last_index2 = last_index2;
        last_index2 = index2;
        index2 = findNearestCornerFromBorder(corner2_nearest_border);
        corner2 = sub_corner_list_ID2[index2];
        is_jump = true;
      }
    }

    // obsID1应该往哪个方向走 -1,0,1
    dis11 = std::hypot(sub_corner_list_ID1[(index1-1+ID1_size)%ID1_size].x-corner2.x, sub_corner_list_ID1[(index1-1+ID1_size)%ID1_size].y-corner2.y);
    dis12 = std::hypot(corner1.x-corner2.x,corner1.y-corner2.y);
    dis13 = std::hypot(sub_corner_list_ID1[(index1+1+ID1_size)%ID1_size].x-corner2.x, sub_corner_list_ID1[(index1+1+ID1_size)%ID1_size].y-corner2.y);
    min_dis1 = std::min(dis11,std::min(dis12,dis13));
    if (ID1_size==1)
      dir1 = 0;
    else if (min_dis1 == dis11)
      dir1 = -1;
    else if (min_dis1 == dis12)
      dir1 = 0;
    else
      dir1 = 1;

    // obsID2应该往哪个方向走 -1,0,1
    dis21 = std::hypot(sub_corner_list_ID2[(index2-1+ID2_size)%ID2_size].x-corner1.x, sub_corner_list_ID2[(index2-1+ID2_size)%ID2_size].y-corner1.y);
    dis22 = std::hypot(corner1.x-corner2.x,corner1.y-corner2.y);
    dis23 = std::hypot(sub_corner_list_ID2[(index2+1+ID2_size)%ID2_size].x-corner1.x, sub_corner_list_ID2[(index2+1+ID2_size)%ID2_size].y-corner1.y);
    min_dis2 = std::min(dis21,std::min(dis22,dis23));
    if (ID2_size==1)
      dir2 = 0;
    else if (min_dis2 == dis21)
      dir2 = -1;
    else if (min_dis2 == dis22)
      dir2 = 0;
    else
      dir2 = 1;
    
    last_last_index1 = last_index1;
    last_last_index2 = last_index2;
    last_index1 = index1;
    last_index2 = index2;
    if (min_dis1<min_dis2)
      index1 = (index1+dir1+ID1_size)%ID1_size;
    else
      index2 = (index2+dir2+ID2_size)%ID2_size;

    corner1 = sub_corner_list_ID1[index1];
    corner2 = sub_corner_list_ID2[index2];

    current_min_dis = std::min(min_dis1, min_dis2);
    his_min_dis = std::min(current_min_dis, his_min_dis);
    /** 结束条件 二选一
     * 1. 无法再通过单步移动来减小distance 且 上一步不是跳点（防止连很长的线）
     * 2. 在两点中摆动，当摆到最近距离时
     * **/
    if (dir1==0 && dir2==0 && (!is_jump) || last_last_index1==index1 && last_last_index2==index2 && current_min_dis == his_min_dis)
      break;
  }

  // 在这里检测这条最短的线是否cover到了其他障碍物，如果cover了，进行深搜。
  // 从nearest_border.id2，将线段一份为二，进行递归
  if (corner2_nearest_border.id!=obsID2 && corner2_nearest_border.id!=0){
    if (!isVisited(obsID1,corner2_nearest_border.id)){
      setVisited(obsID1, corner2_nearest_border.id);
      res1 = findPathBetweenObs(obsID1, corner2_nearest_border.id);
    }
    else{
      res1 = getCoverObs(obsID1, corner2_nearest_border.id);
    }
    if (!isVisited(corner2_nearest_border.id, obsID2)){
      setVisited(corner2_nearest_border.id, obsID2);
      res2 = findPathBetweenObs(corner2_nearest_border.id, obsID2);
    }
    else{
      res2 = getCoverObs(corner2_nearest_border.id, obsID2);
    }

    for (auto p:res1)
      obs_in_ID1_and_ID2.push_back(p);
    obs_in_ID1_and_ID2.push_back(corner2_nearest_border.id);
    for (auto p:res2)
      obs_in_ID1_and_ID2.push_back(p);
    
    setCoverObs(obsID1, obsID2, obs_in_ID1_and_ID2);
    return obs_in_ID1_and_ID2;
  }

  setPath(std::make_pair(corner1,corner2));
  return obs_in_ID1_and_ID2;  // 如果执行到了setPath，那说明障碍物是可以直接连接的，此return必然为空
}

/**
 * 给定两个点p1和p2，返回p1所属的障碍物中离p2最近的点 以及，p1指向p2时，遇到的第一个不为p1的id的障碍物的点
 * 注意 p1和p2需要带id值
 * **/
std::pair<Point2D, Point2D> mapProcess::checkCoverBorder(const Point2D& p1, const Point2D& p2){
  const std::vector<Point2D> line_vec= Bresenham(p1,p2);
  Point2D front_encounter_p = p1;
  Point2D back_encounter_p = p2;
  int label;
  for (auto it=line_vec.begin(); it!=line_vec.end(); it++){
    label = map_border_labeled_[it->x][it->y];
    // 如果压倒了p2自己，则直接break
    if (it->x==p2.x && it->y==p2.y){
      back_encounter_p.id = label;
      back_encounter_p.x = it->x;
      back_encounter_p.y = it->y;
      break;
    }    
    // 遇见障碍物点，就标记上
    if ( label!=0 ){
      // 注意，从p1出发后有两种情况，第一种压到了p1自己所在的障碍物的border，这样的话，会持续更新到本障碍物的最远border点；第二种是走着走着压到了别的障碍物，这时只给出第一个遇见的点即可。
      if (label==p1.id || front_encounter_p.id == 0){
        front_encounter_p.id = label;
        front_encounter_p.x = it->x;
        front_encounter_p.y = it->y;
      }
      // 同样p2也是有两种情况
      if (label==p2.id && label==back_encounter_p.id)
        break;
      back_encounter_p.id = label;
      back_encounter_p.x = it->x;
      back_encounter_p.y = it->y;
      if (label!=p2.id && label!=p1.id)
        break;
    }
  }
  if (p1.id==startID_ || p1.id==goalID_)
    front_encounter_p = p1;
  if ((p2.id==startID_ || p2.id==goalID_) && (back_encounter_p.id==0 || back_encounter_p.x==goal_in_map_x_ && back_encounter_p.y==goal_in_map_y_))
    back_encounter_p = p2;

  return std::make_pair(front_encounter_p, back_encounter_p);  
}



// generate a vector pointed from p1 to p2 by Bresenham algorithm
// note: 4-adjacent points, line_vec contains p1 and p2
std::vector<Point2D> mapProcess::Bresenham (const Point2D& p1, const Point2D& p2)
{
  std::vector<Point2D> line_vec = {p1};
  int dx = std::abs(p2.x-p1.x);
  int dy = std::abs(p2.y-p1.y);
  int sx = p2.x>p1.x?1:-1;
  int sy = p2.y>p1.y?1:-1;
  int x = p1.x;
  int y = p1.y;
  if (dx>dy){
    int e = -dx;
    for (int i=0; i<dx; i++){
      x += sx;
      e += 2*dy;
      line_vec.push_back(Point2D(x,y));
      if (e>=0){
        y += sy;
        e -= 2*dx;
        line_vec.push_back(Point2D(x,y));
      }
      
    }
  }

  else if (dx<dy){
    int e = -dy;
    for (int i=0; i<dy; i++){
      y += sy;
      e += 2*dx;
      line_vec.push_back(Point2D(x,y));
      if (e>=0){
        x += sx;
        e -= 2*dy;
        line_vec.push_back(Point2D(x,y));
      }
    }
  }

  else{ //dx == dy
    for (int i=0; i<dx; i++){
      x += sx;
      y += sy;
      line_vec.push_back(Point2D(x,y));
    }
  }

  return line_vec;
}

bool mapProcess::checkCollision (const Point2D& p1, const Point2D& p2){
  auto line_vec = Bresenham(p1, p2);
  for (auto p:line_vec)
  {
    int label = map_obs_labeled_[p.x][p.y];
    // if (label!=0)
    if (label!=0 && p.x!=start_in_map_x_ && p.y!=start_in_map_y_ && p.x!=goal_in_map_x_ && p.y!=goal_in_map_y_)
      return true;
  }
  return false;
}

// 返回的是corner_list_[p.id]中，最近corner对应的index
int mapProcess::findNearestCornerFromBorder (const Point2D& p){
  if (p.id==startID_ || p.id==goalID_)
    return 0;
  const std::vector<Point2D>& candidate_corner = corner_list_[p.id];
  float min_dis = 9999;
  float dis;
  int min_index = 0, i=0;
  for (auto it=candidate_corner.begin(); it!=candidate_corner.end();it++){
    dis = hypot(it->x-p.x, it->y-p.y);
    if (dis < min_dis){
      min_dis = dis;
      min_index = i;
    }
    i++;
  }
  return min_index;
}

void mapProcess::printVector(std::vector<Point2D> obj){
  for(auto it=obj.begin();it!=obj.end();it++){
    std::cout<<"("<<it->x<<","<<it->y<<","<<it->value<<","<<it->id<<") ";
  }
  std::cout<<std::endl;
}

void mapProcess::printVectorSegmented(std::vector<Point2D> obj){
  int old_id = -1;
  for(auto it=obj.begin();it!=obj.end();it++){
    if (old_id!=it->id && old_id!=-1)
      std::cout<<std::endl;
    std::cout<<"("<<it->x<<","<<it->y<<","<<it->value<<","<<it->id<<") ";
    old_id = it->id;
  }
  std::cout<<std::endl;
}



bool mapProcess::isVisited(int ID1, int ID2){
  int index1,index2;
  for(int i=0; i<label_list_.size(); i++){
    if (label_list_[i] == ID1){
      index1 = i;
    }
    if(label_list_[i]==ID2){
      index2 = i;
    }
  }
  return visited_[index1][index2];
}

void mapProcess::setVisited(int ID1, int ID2){
  int index1,index2;
  for(int i=0; i<label_list_.size(); i++){
    if (label_list_[i] == ID1){
      index1 = i;
    }
    if(label_list_[i]==ID2){
      index2 = i;
    }
  }
  visited_[index1][index2] = true;
  visited_[index2][index1] = true;
}


void mapProcess::setPath(std::pair<Point2D, Point2D> path){
  int ID1=path.first.id, ID2=path.second.id;
  int index1,index2;
  for(int i=0; i<label_list_.size(); i++){
    if (label_list_[i] == ID1){
      index1 = i;
    }
    if(label_list_[i]==ID2){
      index2 = i;
    }
  }
  edges_graph_[index1][index2] = path;
  edges_graph_[index2][index1] = std::make_pair(path.second,path.first);
};

std::pair<Point2D, Point2D> mapProcess::getPathBetweenObs(int ID1, int ID2){
  int index1,index2;
  for(int i=0; i<label_list_.size(); i++){
    if (label_list_[i] == ID1){
      index1 = i;
    }
    if(label_list_[i]==ID2){
      index2 = i;
    }
  }
  return edges_graph_[index1][index2];
}

void mapProcess::setCoverObs(int ID1, int ID2, std::vector<int> covers){
  // 这里的covers是初始的ID值
  int index1,index2;
  for(int i=0; i<label_list_.size(); i++){
    if (label_list_[i] == ID1){
      index1 = i;
    }
    if(label_list_[i]==ID2){
      index2 = i;
    }
  }
  covers_graph_[index1][index2] = covers;
  std::vector<int> covers_rev = covers;
  std::reverse(covers_rev.begin(),covers_rev.end());
  covers_graph_[index2][index1] = covers_rev;
}

std::vector<int> mapProcess::getCoverObs(int ID1, int ID2){
  int index1,index2;
  for(int i=0; i<label_list_.size(); i++){
    if (label_list_[i] == ID1){
      index1 = i;
    }
    if(label_list_[i]==ID2){
      index2 = i;
    }
  }
  return covers_graph_[index1][index2];
}

// 启动深搜，找同伦轨迹
std::vector<std::vector<Eigen::Vector2d>> mapProcess::findHomoPaths(const teb_local_planner::PoseSE2 &start, const teb_local_planner::PoseSE2 &goal, const int& max_path_explore_number_for_GraphicTEB, const int& max_path_remained_for_GraphicTEB, const bool& is_limitation, const bool& is_hallway){
  auto t = ros::Time::now();
  //找起点和终点的ID
  float min_dis_start = 99999999.9, min_dis_goal = 99999999.9;
  max_path_explore_number_for_GraphicTEB_ = max_path_explore_number_for_GraphicTEB;
  is_limitation_ = is_limitation;
  // 找到起点和终点的obs的id对应的index
  int start_index, goal_index;
  for(int i=0; i<label_list_.size(); i++){
    if (label_list_[i] == startID_){
      start_index = i;
    }
    if(label_list_[i]==goalID_){
      goal_index = i;
    }
  }

  // 启动深搜，找到res这个二维的vector，保存多条由obs组成的轨迹
  std::vector<int> visited;
  std::vector<std::vector<int>> res;
  visited.push_back(start_index);
  depthFirst(visited, goal_index, res);
  if (max_path_remained_for_GraphicTEB!=-1 && res.size()>max_path_remained_for_GraphicTEB){
    auto iter = res.begin();
    std::advance(iter, max_path_remained_for_GraphicTEB);
    res.erase(iter, res.end());
  }
  // std::cout<<"   1"<<std::endl;
  std::vector<std::vector<Point2D>> homo_paths_in_map;
  // std::cout<<"         ininer1:"<<(ros::Time::now() - t).toSec()<<std::endl;

  // 对res进行解析，找到对应的路径簇paths。
  // 找到id_list_of_borders_on_the_outline_
  if (is_hallway){
    int max_x = map_border_labeled_.size();
    int max_y = map_border_labeled_[0].size();
    for (int i=0; i<max_x; i++){
      int id1 = map_border_labeled_[i][0];
      int id2 = map_border_labeled_[i][max_y-1];
      if ((id1==id2) && (id1!=0) && (find(id_list_of_borders_on_the_outline_.begin(),id_list_of_borders_on_the_outline_.end(),id1)==id_list_of_borders_on_the_outline_.end()))
        id_list_of_borders_on_the_outline_.push_back(id1);
    }
    for (int i=0; i<max_y; i++){
      int id1 = map_border_labeled_[0][i];
      int id2 = map_border_labeled_[max_x-1][i];
      if ((id1==id2) && (id1!=0) && (find(id_list_of_borders_on_the_outline_.begin(),id_list_of_borders_on_the_outline_.end(),id1)==id_list_of_borders_on_the_outline_.end()))
        id_list_of_borders_on_the_outline_.push_back(id1);
    }
  }

  // 起点的vec形式
  auto start_vec = Point2D(start_in_map_x_, start_in_map_y_, 0, startID_);
  auto goal_vec = Point2D(goal_in_map_x_, goal_in_map_y_, 0, goalID_);
  for (auto r:res){
    // 从当前paths的长度index_start开始，接下来index_max_increment个轨迹，都和r相关
    int index_start = homo_paths_in_map.size();
    int index_max_increment = pow(2, r.size()-2); //去除起点和终点
    // 找到map像素下的所有轨迹homo_paths_in_map
    for (int i=0; i<index_max_increment; i++){
      homo_paths_in_map.push_back({start_vec});
    }
    // 保存当前障碍物的进入时对应的corner点
    Point2D last_corner;
    for (int i=0; i<r.size()-1;i++){  //去除起点和终点
      auto pointsBet = getPathBetweenObs(label_list_[r[i]], label_list_[r[i+1]]); // fix bug 0909
      // 起点的话 仅用于赋值后面障碍物的入corner点
      if (i==0){
        last_corner = pointsBet.second;
        last_corner.id = label_list_[r[i+1]];      
        continue;
      }
      else{
        auto pointsAlong = getPathAlongsideObs(last_corner.id, last_corner, pointsBet.first);
        auto clockwise = pointsAlong.first;
        auto counterclockwise = pointsAlong.second;
        int separated_num = pow(2,i);
        int step = index_max_increment/separated_num;
        for (int j=0; j<separated_num; j++){
          for (int k=0; k<step; k++){
            // std::cout<<index_start<<", "<<index_max_increment<<"; "<<j<<", "<<step<<", "<<k<<std::endl;
            if (j%2==0)
              homo_paths_in_map[index_start+j*step+k].insert(homo_paths_in_map[index_start+j*step+k].end(), clockwise.begin(), clockwise.end());
            else
              homo_paths_in_map[index_start+j*step+k].insert(homo_paths_in_map[index_start+j*step+k].end(), counterclockwise.begin(), counterclockwise.end());
          }
        }

        last_corner = pointsBet.second;
        last_corner.id = label_list_[r[i+1]];
        // std::cout<<"-"<<std::endl;
      }
    }

    //赋值goal_vec
    for (int i=index_start; i<index_start+index_max_increment; i++){
      homo_paths_in_map[i].push_back(goal_vec);
    }
  }

  // std::cout<<"         ininer2:"<<(ros::Time::now() - t).toSec()<<std::endl;  
  // std::cout<<"   2"<<std::endl;

  // 删除压边界的轨迹；把轨迹缩短（剔除中间的无碰撞点）；把轨迹从map转到world像素下
  for (auto path:homo_paths_in_map){
    bool flag = false;
    for (auto p:path){
      if (map_obs_labeled_outline_[p.x][p.y]==true){
        flag = true;
        break;
      }
    }
    if (flag)
      continue;
    // std::cout<<"       0"<<std::endl;
    auto path_shortened = shortenPath(path);    
    // std::cout<<"-"<<std::endl;
    // printVector(path_shortened);
    // std::cout<<"       1"<<std::endl;
    homo_paths_.push_back(transPoint2DinMapToVector2dinWorld(path_shortened));
    // std::cout<<"       2"<<std::endl;
  }
  // std::cout<<"   3"<<std::endl;
  // std::cout<<"obstacle combination size: "<<res.size()<<std::endl;
  // for (auto path:res)
    // std::cout<<path.size()-2<<std::endl;  
  // std::cout<<"path size: "<<homo_paths_.size()<<std::endl;
  // for (auto path:homo_paths_){
  //   for (auto p:path){
  //     std::cout<<p.x()<<","<<p.y()<<"; ";
  //   }
  //   std::cout<<std::endl;
  // }
  // std::cout<<"         ininer3:"<<(ros::Time::now() - t).toSec()<<std::endl;  
  return homo_paths_;
}


std::vector<Eigen::Vector2d> mapProcess::transPoint2DinMapToVector2dinWorld(std::vector<Point2D> points){
  std::vector<Eigen::Vector2d> temp;
  for (auto p:points){
    double w_x, w_y;
    map2world(p.x, p.y, w_x, w_y);
    temp.push_back(Eigen::Vector2d(w_x,w_y));
  }
  return temp;
}

bool mapProcess::depthFirst(std::vector<int>& visited, int goal_index, std::vector<std::vector<int>>& res){
  // see http://www.technical-recipes.com/2011/a-recursive-algorithm-to-find-all-paths-between-two-given-nodes/ for details on finding all simple paths  
  // 拿出来最后一个点，如果该点是goal点，则形成新的teb轨迹，并检测是否形成新同伦类
  int back = visited.back();
  // 该obs可以直接连接目标obs
  if (edges_graph_[back][goal_index].first.id!=0){
    std::vector<int> path_temp = visited;
    path_temp.push_back(goal_index);
    //计算path_temp的长度
    int length_temp = 0;
    std::vector<Point2D> points_temp;
    for (int i=0; i<path_temp.size()-1; i++){
      auto pointsBet = getPathBetweenObs(label_list_[path_temp[i]], label_list_[path_temp[i+1]]);
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

  if (max_path_explore_number_for_GraphicTEB_!=-1 && res.size()>=max_path_explore_number_for_GraphicTEB_){
    return false;
  }
  
  // edges_graph是一个方阵
  auto is_father_can_visit = std::vector<bool>(edges_graph_.size(),false);
  if (is_limitation_){
    for (int i=0; i<visited.size()-1; i++){
      for (int j=0; j<edges_graph_.size();j++){
        if (edges_graph_[visited[i]][j].first.id!= 0)
          is_father_can_visit[j] = true;
      }
    }
  }

  // 当back的adjacent中没有goal时，遍历所有adjacent来继续深搜
  for (int i=0; i<edges_graph_[back].size(); i++){
    // 不可直连
    if (edges_graph_[back][i].first.id ==0){
      // std::cout<<1<<std::endl;
      continue;
    }
    // // 该obs直接连接起点
    // if (edges_graph_[i][startIndex_].second.id ==startID_ && back!=startIndex_){
    //   continue;
    // }
    // 已经从该obs走过
    bool is_in = false;
    for (auto iter=visited.begin(); iter!=visited.end(); iter++){
      if (*iter == i)
        is_in = true;
    }
    if (is_in)
      continue;
    // 存在（祖）父节点可以走过该点
    if (is_father_can_visit[i] == true)
      continue;
    
    // 扩展adjacent obs
    visited.push_back(i);
    if (!depthFirst(visited, goal_index, res))
      return false;
    visited.pop_back();

  }
  return true;
}



// 选择每个障碍物的边界点作为该障碍物的identification，outline一下
// 通过每个逆时针排序的外檐有序点 在 内檐点集中进行顺时针旋转，来获取内檐边界的逆时针序列
std::map<int, std::vector<std::pair<double,double>>> mapProcess::getStaticObsIdentificationInWorld(){
  std::map<int, std::vector<Point2D>> border_list_temp;

  cv::Mat dst1, dst2, comp;
  cv::erode(map_cv_without_dynamic_obs_, dst1, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5))); // 这里用 2*0.1=0.2m更合适。。 少缩0.1m
  cv::erode(dst1, dst2, cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3)));
  cv::compare(dst1, dst2, comp, cv::CMP_NE);
  // std::cout<<std::endl<<cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(6, 6))<<std::endl;
  // cv::imshow("Canny edge detection", comp);
  // cv::waitKey(0);
  // 赋值腐蚀后的障碍物地图
  for(int j=0; j < dst1.rows; j++)
    for(int i=0; i<dst1.cols;i++)
      if(dst1.at<uint8_t>(j,i) && (map_obs_labeled_[i][j]!=startID_) && (map_obs_labeled_[i][j]!=goalID_))
        map_obs_shrinked_labeled_[i][j] = map_obs_labeled_[i][j];
  // 以下为找border，和borderIdentify相同
  // 这个循环用于保存border_labeled_temp这个地图
  auto border_labeled_temp = std::vector<std::vector<int>> (width_, std::vector<int>(height_,0));
  std::vector<Point2D> kernel = {Point2D(-1,-1),Point2D(-1,0),Point2D(-1,1),Point2D(0,-1),Point2D(0,1),Point2D(1,-1),Point2D(1,0),Point2D(1,1)};
  for(int j=0; j < comp.rows; j++)
      for(int i=0; i<comp.cols;i++)
          // 只要comp中的元素是非空的，那么它就是边缘点
          if(comp.at<uint8_t>(j,i) && map_obs_labeled_[i][j]!=startID_ && map_obs_labeled_[i][j]!=goalID_)
            border_labeled_temp[i][j] = map_obs_labeled_[i][j];
  // 扩展归并，拟合border_list
  auto map_is_visited_temp = std::vector<std::vector<bool>> (width_, std::vector<bool>(height_, false));
  // 遍历时的顺序为：右、上、右上、下、右下、左、左上、左下。即：上与右优先遍历、非斜线优先遍历，保证borderlist是顺时针的
  std::vector<Point2D> kernel2 = {Point2D(1,0),Point2D(0,1),Point2D(1,1),Point2D(0,-1),Point2D(1,-1),Point2D(-1,0),Point2D(-1,1),Point2D(-1,-1)};
  for (int y=0; y<dst1.rows; y++){
    for (int x=0; x<dst1.cols; x++){
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

  // auto border = border_list_.begin();
  std::map<int,std::vector<Point2D>> remained_border;
  label_index_++;
  for (auto border=border_list_temp.begin(); border!=border_list_temp.end();border++){
    // 起点和终点
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

  for (auto p:remained_border){
    border_list_temp[p.first] = p.second;
  }

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


std::vector<std::vector<int>> mapProcess::getMapObsLabeled(){ 
  map_obs_shrinked_labeled_[start_in_map_x_][start_in_map_y_] = 0;
  map_obs_shrinked_labeled_[goal_in_map_x_][goal_in_map_y_] = 0;
  return map_obs_shrinked_labeled_;
};


void mapProcess::clearSelf(){
  map_vector_.clear();
  map_int_.clear();
  map_cv_.release();

  obs_list_.clear();
  map_obs_labeled_.clear();
  map_obs_shrinked_labeled_.clear();

  border_list_.clear();
  map_border_labeled_.clear();

  corner_list_.clear();
  map_corner_labeled_.clear();


  homo_paths_.clear();
  homo_paths_pruned_.clear();

  label_list_.clear();
  visited_.clear();
  edges_graph_.clear();
  covers_graph_.clear();
  label_index_ = 1;

  id_list_of_borders_on_the_outline_.clear();

}

void mapProcess::add_to_homo_paths_pruned(std::vector<Eigen::Vector2d> path){
  homo_paths_pruned_.push_back(path);
}



std::vector<Point2D> mapProcess::shortenPath (std::vector<Point2D> path){
  // 确定每个obs簇的起点在path中的index
  std::vector<int> id_index;
  for (int i=0; i<path.size();i++)
    if (id_index.size()==0 || id_index.size()!=0 && path[i].id!=path[id_index.back()].id)
      id_index.push_back(i);
  // 仅包含起点和终点，break
  if (id_index.size()<=2)
    return path;
  // std::cout<<"---------"<<std::endl;
  // printVector(path);
  // for (auto p:id_index)
    // std::cout<<p<<", ";
  // std::cout<<std::endl;
  // 对相邻的两个obs：last和next。last的end往前缩，next的start往后退，直到collision
  int j=0;
  int last_start_index;
  int last_end_index;
  int next_start_index;
  int next_end_index;
  bool check1, check2, check3;
  //1. obs簇间的shorten
  while (j+1<id_index.size()){
    last_start_index = std::max(id_index[j],next_start_index);    // index的start其实是个动态的，不然有可能 有些障碍物会经过它前面和后面的squeeze后，直接被全部压缩掉
    last_end_index = id_index[j+1]-1;
    next_start_index = id_index[j+1];
    if (j+1==id_index.size()-1)
      next_end_index = id_index[j+1];
    else
      next_end_index = id_index[j+2]-1;
    // 如果任意一个方向可滑，则can_continue=true
    bool can_continue = true;
    bool can1, can2;
    // 如果障碍物是边界墙，则不会被滑动收缩
    bool flag1=true, flag2=true;
    if (find(id_list_of_borders_on_the_outline_.begin(),id_list_of_borders_on_the_outline_.end(),path[last_end_index].id)!=id_list_of_borders_on_the_outline_.end())
      flag1 = false;
    if (find(id_list_of_borders_on_the_outline_.begin(),id_list_of_borders_on_the_outline_.end(),path[next_start_index].id)!=id_list_of_borders_on_the_outline_.end())
      flag2 = false;

    while(can_continue){
      can_continue = false;
      check1 = true;
      check2 = true;
      check3 = true;
      bool can1 = last_start_index<last_end_index;
      bool can2 = next_start_index<next_end_index;
      // 可左滑
      if (can1 && flag1)   
        check1 = checkCollision(path[last_end_index-1], path[next_start_index]);
      // 可右滑
      if (can2 && flag2)
        check2 = checkCollision(path[last_end_index], path[next_start_index+1]);
      // 可双向同时滑
      if (can1 && can2 && flag1 && flag2)
        check3 = checkCollision(path[last_end_index-1], path[next_start_index+1]);
      if (!check3){
        last_end_index--;
        next_start_index++;
        can_continue = true;
      }
      else if (!check2){
        next_start_index++;
        can_continue = true;
      }
      else if (!check1){
        last_end_index--;
        can_continue = true;
      }
    }
    // 标记可去除的障碍物
    for (int i=last_end_index+1; i<=next_start_index-1; i++){
      path[i].value = -1;
    }

    //2. 障碍物簇内部的去除
    //正向去除
    int k=last_start_index;
    while (k<last_end_index){
      int l = k+1;
      while (l<=last_end_index){
        if (checkCollision(path[k], path[l])){
          break;
        }
        else{
          l++;
        }
      }
      for (int m=k+1; m<=std::min(l-1,last_end_index-1); m++)
        path[m].value = -1;
      k=l;
    }
    //逆向去除
    k= last_end_index;
    while (k>last_start_index){
      int l = k-1;
      while (l>=last_start_index){
        if (path[l].value == -1){
          l--;
          continue;
        }
        else if (checkCollision(path[k], path[l])){
          break;
        }
        else{
          l--;
        }
      }
      for (int m=k-1; m>=std::max(l+1,last_start_index+1); m--)
        path[m].value = -1;
      k=l;
    }

    j++;
  }
  // printVectorSegmented(path);
  auto iter = path.begin();
  while(iter!=path.end()){
    if (iter->value == -1)
      iter = path.erase(iter);
    else
      iter++;
  }

  return path;
}