#include <teb_local_planner/map_process.h>

void mapProcess::setParams(const int& max_path_explore_number_for_GraphicTEB, const int& max_path_remained_for_GraphicTEB, const bool& is_hallway, const bool& is_cos_limitation, const bool& is_father_visit_limitation, const double& epsilon_for_early_stop){
  max_path_explore_number_for_GraphicTEB_ = max_path_explore_number_for_GraphicTEB;
  max_path_remained_for_GraphicTEB_ = max_path_remained_for_GraphicTEB;
  is_hallway_ = is_hallway;
  is_cos_limitation_ = is_cos_limitation;
  is_father_visit_limitation_ = is_father_visit_limitation;
  // epsilon_for_early_stop_ = epsilon_for_early_stop;
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

void mapProcess::map2world(float mx, float my, double& wx, double& wy){
  wx = origin_x_ + (mx + 0.5) * resolution_;
  wy = origin_y_ + (my + 0.5) * resolution_;
}

void mapProcess::map2worldLarge(int mx, int my, double& wx, double& wy){
  wx = origin_x_large_ + (mx + 0.5) * resolution_large_;
  wy = origin_y_large_ + (my + 0.5) * resolution_large_;
}

void mapProcess::world2map(int& mx, int& my, double wx, double wy){
  mx = (wx-origin_x_)/resolution_;
  my = (wy-origin_y_)/resolution_;
}


std::vector<Eigen::Vector2d> mapProcess::transPoint2DinMapToVector2dinWorld(std::vector<Point2D_float> points){
  std::vector<Eigen::Vector2d> temp;
  for (auto p:points){
    double w_x, w_y;
    map2world(p.x, p.y, w_x, w_y);
    temp.push_back(Eigen::Vector2d(w_x,w_y));
  }
  return temp;
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


// generate a vector pointed from p1 to p2 by Bresenham algorithm
// note: 4-adjacent points, line_vec contains p1 and p2
std::vector<Point2D> mapProcess::Bresenham (const Point2D& p1, const Point2D& p2)
{
  std::vector<Point2D> line_vec = {p1};
  int ID = p1.id;
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
      line_vec.push_back(Point2D(x,y,0,ID));
      if (e>=0){
        y += sy;
        e -= 2*dx;
        line_vec.push_back(Point2D(x,y,0,ID));
      }
      
    }
  }

  else if (dx<dy){
    int e = -dy;
    for (int i=0; i<dy; i++){
      y += sy;
      e += 2*dx;
      line_vec.push_back(Point2D(x,y,0,ID));
      if (e>=0){
        x += sx;
        e -= 2*dy;
        line_vec.push_back(Point2D(x,y,0,ID));
      }
    }
  }

  else{ //dx == dy
    for (int i=0; i<dx; i++){
      x += sx;
      y += sy;
      line_vec.push_back(Point2D(x,y,0,ID));
    }
  }

  return line_vec;
}

std::vector<Point2D_float> mapProcess::Bresenham (const Point2D_float& p1, const Point2D_float& p2){
  std::vector<Point2D_float> line_vec = {p1};
  float dx = std::abs(p2.x-p1.x);
  float dy = std::abs(p2.y-p1.y);
  float sx = p2.x>p1.x?1:-1;
  float sy = p2.y>p1.y?1:-1;
  float x = p1.x;
  float y = p1.y;
  if (dx>dy){
    float e = -dx;
    for (int i=0; i<dx; i++){
      x += sx;
      e += 2*dy;
      line_vec.push_back(Point2D_float(x,y));
      if (e>=0){
        y += sy;
        e -= 2*dx;
        line_vec.push_back(Point2D_float(x,y));
      }
      
    }
  }

  else if (dx<dy){
    float e = -dy;
    for (int i=0; i<dy; i++){
      y += sy;
      e += 2*dx;
      line_vec.push_back(Point2D_float(x,y));
      if (e>=0){
        x += sx;
        e -= 2*dy;
        line_vec.push_back(Point2D_float(x,y));
      }
    }
  }

  else{ //dx == dy
    for (int i=0; i<dx; i++){
      x += sx;
      y += sy;
      line_vec.push_back(Point2D_float(x,y));
    }
  }

  return line_vec;
}

std::vector<Point2D_float> mapProcess::BresenhamDirect(const Point2D_float& p1, const Point2D_float& p2){
  std::vector<Point2D_float> line_vec;
  float dis = hypot(p2.x-p1.x, p2.y-p1.y);
  float dx = (p2.x-p1.x)/dis;
  float dy = (p2.y-p1.y)/dis;
  for(int i=0;i<dis;i++){
    line_vec.push_back(Point2D_float(p1.x+dx*i, p1.y+dy*i));
  }
  line_vec.push_back(p2);
  return line_vec;
}


bool mapProcess::checkInMap(int x, int y){
  if (x>=0 && x<width_ && y>=0 && y<height_){
    return true;
  }
  return false;
}

bool mapProcess::checkInMap(int x, int y, int width, int height){
  if (x>=0 && x<width && y>=0 && y<height){
    return true;
  }
  return false;
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


bool mapProcess::checkCollision (const std::vector<Point2D>& vec_line, const int& ID1, const int& ID2){
  for (auto p:vec_line)
  {
    if (!(map_border_labeled_[p.x][p.y]==0 || map_border_labeled_[p.x][p.y]==ID1 || map_border_labeled_[p.x][p.y]==ID2) || map_obs_labeled_[p.x][p.y]!=0)
      return true;
  }
  return false;
}

bool mapProcess::checkCollision (const std::vector<Point2D>& vec_line, const int& ID){
  for (auto p:vec_line)
  {
    if ( map_border_labeled_[p.x][p.y]!=0 && map_border_labeled_[p.x][p.y]!=ID || map_obs_labeled_[p.x][p.y]!=0)
      return true;
  }
  return false;
}

bool mapProcess::checkCollision (const std::vector<Point2D>& vec_line){
  for (auto p:vec_line)
  {
    if (map_border_labeled_[p.x][p.y]!=0 || map_obs_labeled_[p.x][p.y]!=0)
      return true;
  }
  return false;
}

bool mapProcess::checkCollisionOnlyObs (const std::vector<Point2D>& vec_line){
  for (auto p:vec_line)
  {
    if (map_obs_labeled_[p.x][p.y]!=0){
      // std::cout<<"[["<<p.x<<","<<p.y<<"]]; ";
      return true;
    }
  }
  return false;
}


bool mapProcess::checkCollisionOnlyObsAlignedWithBresenham(const Point2D& p1, const Point2D& p2){

  if (map_obs_labeled_[p1.x][p1.y]!=0)
    return true;

  int ID = p1.id;
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
      if (map_obs_labeled_[x][y]!=0)
        return true;
      if (e>=0){
        y += sy;
        e -= 2*dx;
        if (map_obs_labeled_[x][y]!=0)
          return true;
      }
      
    }
  }

  else if (dx<dy){
    int e = -dy;
    for (int i=0; i<dy; i++){
      y += sy;
      e += 2*dx;
      if (map_obs_labeled_[x][y]!=0)
        return true;
      if (e>=0){
        x += sx;
        e -= 2*dy;
        if (map_obs_labeled_[x][y]!=0)
          return true;
      }
    }
  }

  else{ //dx == dy
    for (int i=0; i<dx; i++){
      x += sx;
      y += sy;
      if (map_obs_labeled_[x][y]!=0)
        return true;
    }
  }


  return false;

}

bool mapProcess::checkCollisionOnlyObsAlignedWithBresenhamDirect(const Point2D_float& p1, const Point2D_float& p2){
  if ((map_obs_labeled_[int(p1.x)][int(p1.y)]!=0))
    return true;
  float dis = hypot(p2.x-p1.x, p2.y-p1.y);
  float dx = (p2.x-p1.x)/dis;
  float dy = (p2.y-p1.y)/dis;
  float i =0;
  while(i<dis){
    if ((map_obs_labeled_[int(p1.x+dx*i)][int(p1.y+dy*i)]!=0))
      return true;   
    i ++; 
  }
  if ((map_obs_labeled_[int(p2.x)][int(p2.y)]!=0))
    return true;
  return false;
}

bool mapProcess::checkCollisionObsAndBorderAlignedWithBresenham(const Point2D& p1, const Point2D& p2, int ID){
  if ( map_border_labeled_[p1.x][p1.y]!=0 && map_border_labeled_[p1.x][p1.y]!=ID || map_obs_labeled_[p1.x][p1.y]!=0)
    return true;

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
      if ( map_border_labeled_[x][y]!=0 && map_border_labeled_[x][y]!=ID || map_obs_labeled_[x][y]!=0)
        return true;
      if (e>=0){
        y += sy;
        e -= 2*dx;
        if ( map_border_labeled_[x][y]!=0 && map_border_labeled_[x][y]!=ID || map_obs_labeled_[x][y]!=0)
          return true;
      }
      
    }
  }
  else if (dx<dy){
    int e = -dy;
    for (int i=0; i<dy; i++){
      y += sy;
      e += 2*dx;
      if ( map_border_labeled_[x][y]!=0 && map_border_labeled_[x][y]!=ID || map_obs_labeled_[x][y]!=0)
        return true;
      if (e>=0){
        x += sx;
        e -= 2*dy;
        if ( map_border_labeled_[x][y]!=0 && map_border_labeled_[x][y]!=ID || map_obs_labeled_[x][y]!=0)
          return true;
      }
    }
  }
  else{ //dx == dy
    for (int i=0; i<dx; i++){
      x += sx;
      y += sy;
      if ( map_border_labeled_[x][y]!=0 && map_border_labeled_[x][y]!=ID || map_obs_labeled_[x][y]!=0)
        return true;
    }
  }
  return false;
}


bool mapProcess::checkCollisionObsAndBorderAlignedWithBresenham(const Point2D& p1, const Point2D& p2, int ID1, int ID2){
  if ( map_border_labeled_[p1.x][p1.y]!=0 && map_border_labeled_[p1.x][p1.y]!=ID1 && map_border_labeled_[p1.x][p1.y]!=ID2 || map_obs_labeled_[p1.x][p1.y]!=0)
    return true;

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
      if ( map_border_labeled_[x][y]!=0 && map_border_labeled_[x][y]!=ID1 && map_border_labeled_[x][y]!=ID2 || map_obs_labeled_[x][y]!=0)
        return true;
      if (e>=0){
        y += sy;
        e -= 2*dx;
        if ( map_border_labeled_[x][y]!=0 && map_border_labeled_[x][y]!=ID1 && map_border_labeled_[x][y]!=ID2 || map_obs_labeled_[x][y]!=0)
          return true;
      }
      
    }
  }
  else if (dx<dy){
    int e = -dy;
    for (int i=0; i<dy; i++){
      y += sy;
      e += 2*dx;
      if ( map_border_labeled_[x][y]!=0 && map_border_labeled_[x][y]!=ID1 && map_border_labeled_[x][y]!=ID2 || map_obs_labeled_[x][y]!=0)
        return true;
      if (e>=0){
        x += sx;
        e -= 2*dy;
        if ( map_border_labeled_[x][y]!=0 && map_border_labeled_[x][y]!=ID1 && map_border_labeled_[x][y]!=ID2 || map_obs_labeled_[x][y]!=0)
          return true;
      }
    }
  }
  else{ //dx == dy
    for (int i=0; i<dx; i++){
      x += sx;
      y += sy;
      if ( map_border_labeled_[x][y]!=0 && map_border_labeled_[x][y]!=ID1 && map_border_labeled_[x][y]!=ID2 || map_obs_labeled_[x][y]!=0)
        return true;
    }
  }
  return false;  
}


bool mapProcess::checkCollisionOnlyObs (const std::vector<Point2D_float>& vec_line){
  for (auto p:vec_line)
  {
    if (map_obs_labeled_[int(p.x)][int(p.y)]!=0){
    // if (map_obs_labeled_[int(round(p.x))][int(round(p.y))]!=0){
      // std::cout<<"[["<<p.x<<","<<p.y<<"]]; ";
      return true;
    }
  }
  return false;
}

bool mapProcess::checkCollisionNotSelfBorder (const std::vector<Point2D>& vec_line, const int& ID1, const int& ID2){
  for (auto p:vec_line)
  {
    if (map_border_labeled_[p.x][p.y]==ID1 || map_border_labeled_[p.x][p.y]==ID2 || map_obs_labeled_[p.x][p.y]!=0)
      return true;
  }
  return false;
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

int mapProcess::getMod(const int& input, const int& length){
  int res = input;
  while (res<0)
    res += length;
  return res%length;
}