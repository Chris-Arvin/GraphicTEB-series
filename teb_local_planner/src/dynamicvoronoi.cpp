#include <teb_local_planner/dynamicvoronoi.h>

#include <math.h>
#include <iostream>

DynamicVoronoi::DynamicVoronoi() {
}

DynamicVoronoi::~DynamicVoronoi() {
}

void DynamicVoronoi::initializeEmpty(int _sizeX, int _sizeY) {
  sizeX = _sizeX;
  sizeY = _sizeY;

  for (int x=0; x<sizeX; x++){
    std::vector<dataCell> temp;
    for (int y=0; y<sizeY; y++){
      dataCell c;
      c.dist = INFINITY;
      c.sqdist = INT_MAX;
      c.obstX = invalidObstData;
      c.obstY = invalidObstData;
      c.voronoi = free;
      c.queueing = fwNotQueued;
      c.fatherPoints = {INTPOINT(), INTPOINT()};
      temp.push_back(c);
    }
    data.push_back(temp);
  }

}

void DynamicVoronoi::initializeMap(std::vector<std::vector<int>> map_obs_labeled) {
  map_obs_labeled_ = map_obs_labeled;
  // 初始化data，大小为sizeX*sizeY，每个点都是dataCell格式
  initializeEmpty(map_obs_labeled.size(), map_obs_labeled.front().size());

  for (int x=0; x<sizeX; x++) {
    for (int y=0; y<sizeY; y++) {
      // 对于一个障碍物点(x,y)
      if (map_obs_labeled_[x][y]!=0) {
        dataCell c = data[x][y];
        if (!isOccupied(x,y,c)) {
          // 当(x,y)及其周围八临阶都是障碍物点时，该点的最近障碍物被设置为自己，sqdist=0，voronoi=occupied, queueing=fwProcessed
          bool isSurrounded = true;
          for (int dx=-1; dx<=1; dx++) {
            int nx = x+dx;
            if (nx<=0 || nx>=sizeX-1) continue;
            for (int dy=-1; dy<=1; dy++) {
              if (dx==0 && dy==0) continue;
              int ny = y+dy;
              if (ny<=0 || ny>=sizeY-1) continue;
              if (map_obs_labeled_[nx][ny]==0) {
                isSurrounded = false;
                break;
              }
            }
          }
          if (isSurrounded) {
            c.obstX = x;
            c.obstY = y;
            c.sqdist = 0;
            c.dist=0;
            c.voronoi=occupied;
            c.queueing = fwProcessed;
            data[x][y] = c;
          }
          // 如果(x,y)周围不全是障碍物点，则data[x][y]仅会设置obstX和obstY，不改变其他（voronoi默认free，queueing默认fwNotQueued）。但(x,y)会被addList中
          else{
            // setObstacle
            c.dist = 0;
            c.sqdist = 0;
            c.obstX = x;
            c.obstY = y;
            c.queueing = fwQueued;
            c.voronoi = occupied;
            data[x][y] = c;
            open.push(0, INTPOINT(x,y));
          }
        }
      }
    }
  }
}

void DynamicVoronoi::update(bool is_only_for_distance_map) {
  //！！！ 注意open里的点，不一定是障碍物点，也可以是obstacle-free点。它的排序是递增顺序，按照当前点到其最近障碍物点的距离的平方排序
  while (!open.empty()) {
    INTPOINT p = open.pop();
    int x = p.x;
    int y = p.y;
    dataCell c = data[x][y];
    // 被包裹的障碍物不需要被考虑
    if(c.queueing==fwProcessed) continue; 
    // lower过程，c的最近障碍物点是它自己
    if (c.obstX != invalidObstData && isOccupied(c.obstX,c.obstY,data[c.obstX][c.obstY])) {
      c.queueing = fwProcessed;
      c.voronoi = occupied;
      // 遍历(x,y)的八临接所有点(nx, ny)
      for (int dx=-1; dx<=1; dx++) {
        int nx = x+dx;
        if (nx<=0 || nx>=sizeX-1) continue;
        for (int dy=-1; dy<=1; dy++) {
          if (dx==0 && dy==0) continue;
          int ny = y+dy;
          if (ny<=0 || ny>=sizeY-1) continue;
          dataCell nc = data[nx][ny];
          //  计算(nx,ny)到点(x,y)的最近障碍物点(obstX, obstY)的距离，如果小于(nx, ny)自己的到最近障碍物点的距离/等于但(nx,ny)指向的是一个已经删除的障碍物点：
          //  则把(nx,ny)加入open中，它的最近障碍物点同样指向(obstX, obstY)，sqdist修改为更小的距离 （这个是更新父亲节点呀）
          //  注意！仅在这里更新了节点的sqdist！！！
          int distx = nx-c.obstX;
          int disty = ny-c.obstY;
          int newSqDistance = distx*distx + disty*disty;		
          bool overwrite =  (newSqDistance < nc.sqdist);
          if (overwrite) {
            open.push(newSqDistance, INTPOINT(nx,ny));
            nc.queueing = fwQueued;
            nc.dist = sqrt((double) newSqDistance);
            nc.sqdist = newSqDistance;
            nc.obstX = c.obstX;
            nc.obstY = c.obstY;
          }
          // generate voronoi point
          else { 
            if (!is_only_for_distance_map)
              checkVoro(x,y,nx,ny,c,nc);
          }
          data[nx][ny] = nc;
        }
      }
    }
    data[x][y] = c;
  }
}

float DynamicVoronoi::getDistance( int x, int y ) {
  if( (x>0) && (x<sizeX) && (y>0) && (y<sizeY)) return data[x][y].dist; 
  else return -INFINITY;
}

bool DynamicVoronoi::isVoronoi( int x, int y ) {
  dataCell c = data[x][y];
  // return (c.voronoi==free || c.voronoi==voronoiKeep);
  return c.voronoi==voronoiKeep;
}


void DynamicVoronoi::checkVoro(int x, int y, int nx, int ny, dataCell& c, dataCell& nc) {
  // 距离大于1，保证voronoi点不是adjacent to osbtacle的，nc必须有父亲节点
  if ((c.sqdist>0 || nc.sqdist>0) && nc.obstX!=invalidObstData) {
    // c和nc的父亲节点 不能belong to the same obstacle group
    int ID1 = map_obs_labeled_[c.obstX][c.obstY];
    int ID2 = map_obs_labeled_[nc.obstX][nc.obstY];
    assert(ID1!=0 && ID2!=0);
    // 两个点不是同一个ID
    if (ID1 != ID2){
      // save two father nodes
      c.fatherPoints = {INTPOINT(c.obstX, c.obstY), INTPOINT(nc.obstX, nc.obstY)};
      c.voronoi = free;
      pruneQueue.push(INTPOINT(x,y));

      nc.fatherPoints = {INTPOINT(c.obstX, c.obstY), INTPOINT(nc.obstX, nc.obstY)};
      nc.voronoi = free;
      pruneQueue.push(INTPOINT(nx,ny));
    }
  }
}


bool DynamicVoronoi::isOccupied(int x, int y) {
  dataCell c = data[x][y];
  return (c.obstX==x && c.obstY==y);
}

bool DynamicVoronoi::isOccupied(int &x, int &y, dataCell &c) { 
  return (c.obstX==x && c.obstY==y);
}

void DynamicVoronoi::visualize(const char *filename) {
  // write pgm files

  FILE* F = fopen(filename, "w");
  if (!F) {
    std::cerr << "could not open 'result.pgm' for writing!\n";
    return;
  }
  fprintf(F, "P6\n");
  fprintf(F, "%d %d 255\n", sizeX, sizeY);

  for(int y = sizeY-1; y >=0; y--){      
    for(int x = 0; x<sizeX; x++){	
      unsigned char c = 0;
      if (isVoronoi(x,y)) {
        fputc( 255, F );
        fputc( 0, F );
        fputc( 0, F );
      } else if (data[x][y].sqdist==0) {
        fputc( 0, F );
        fputc( 0, F );
        fputc( 0, F );
      } else {
        float f = 80+(data[x][y].dist*5);
        if (f>255) f=255;
        if (f<0) f=0;
        c = (unsigned char)f;
        fputc( c, F );
        fputc( c, F );
        fputc( c, F );
      }
    }
  }
  fclose(F);
}


void DynamicVoronoi::prune() {
  // filler
  while(!pruneQueue.empty()) {
    INTPOINT p = pruneQueue.front();
    pruneQueue.pop();
    int x = p.x;
    int y = p.y;

    if (data[x][y].voronoi==occupied) continue;
    if (data[x][y].voronoi==freeQueued) continue;

    data[x][y].voronoi = freeQueued;
    open.push(data[x][y].sqdist, p);

    /* tl t tr
       l c r
       bl b br */

    dataCell tr,tl,br,bl;
    tr = data[x+1][y+1];
    tl = data[x-1][y+1];
    br = data[x+1][y-1];
    bl = data[x-1][y-1];

    dataCell r,b,t,l;
    r = data[x+1][y];
    l = data[x-1][y];
    t = data[x][y+1];
    b = data[x][y-1];

    if (x+2<sizeX && r.voronoi==occupied) { 
      // fill to the right
      if (tr.voronoi!=occupied && br.voronoi!=occupied && data[x+2][y].voronoi!=occupied) {
        r.voronoi = freeQueued;
        open.push(r.sqdist, INTPOINT(x+1,y));
        data[x+1][y] = r;
      }
    } 
    if (x-2>=0 && l.voronoi==occupied) { 
      // fill to the left
      if (tl.voronoi!=occupied && bl.voronoi!=occupied && data[x-2][y].voronoi!=occupied) {
        l.voronoi = freeQueued;
        open.push(l.sqdist, INTPOINT(x-1,y));
        data[x-1][y] = l;
      }
    } 
    if (y+2<sizeY && t.voronoi==occupied) { 
      // fill to the top
      if (tr.voronoi!=occupied && tl.voronoi!=occupied && data[x][y+2].voronoi!=occupied) {
        t.voronoi = freeQueued;
        open.push(t.sqdist, INTPOINT(x,y+1));
        data[x][y+1] = t;
      }
    } 
    if (y-2>=0 && b.voronoi==occupied) { 
      // fill to the bottom
      if (br.voronoi!=occupied && bl.voronoi!=occupied && data[x][y-2].voronoi!=occupied) {
        b.voronoi = freeQueued;
        open.push(b.sqdist, INTPOINT(x,y-1));
        data[x][y-1] = b;
      }
    } 
  }


  while(!open.empty()) {
    INTPOINT p = open.pop();
    dataCell c = data[p.x][p.y];
    int v = c.voronoi;
    if (v!=freeQueued && v!=voronoiRetry) { // || v>free || v==voronoiPrune || v==voronoiKeep) {
      //      assert(v!=retry);
      continue;
    }

    markerMatchResult r = markerMatch(p.x,p.y);
    if (r==pruned) c.voronoi = voronoiPrune;
    else if (r==keep) c.voronoi = voronoiKeep;
    else { // r==retry
      c.voronoi = voronoiRetry;
      //      printf("RETRY %d %d\n", x, sizeY-1-y);
      pruneQueue.push(p);
    }
    data[p.x][p.y] = c;

    if (open.empty()) {
      while (!pruneQueue.empty()) {
        INTPOINT p = pruneQueue.front();
        pruneQueue.pop();
        open.push(data[p.x][p.y].sqdist, p);
      }
    }
  }
  //  printf("match: %d\nnomat: %d\n", matchCount, noMatchCount);
}


DynamicVoronoi::markerMatchResult DynamicVoronoi::markerMatch(int x, int y) {
  // implementation of connectivity patterns
  bool f[8];

  int nx, ny;
  int dx, dy;

  int i=0;
  int count=0;
  //  int obstacleCount=0;
  int voroCount=0;
  int voroCountFour=0;

  for (dy=1; dy>=-1; dy--) {
    ny = y+dy;
    for (dx=-1; dx<=1; dx++) {
      if (dx || dy) {
        nx = x+dx;
        dataCell nc = data[nx][ny];
        int v = nc.voronoi;
        bool b = (v<=free && v!=voronoiPrune); 
        //	if (v==occupied) obstacleCount++;
        f[i] = b;
        if (b) {
          voroCount++;
          if (!(dx && dy)) voroCountFour++;
        }
        if (b && !(dx && dy) ) count++;
        //	if (v<=free && !(dx && dy)) voroCount++;
        i++;
      }
    }
  }
  if (voroCount<3 && voroCountFour==1 && (f[1] || f[3] || f[4] || f[6])) {
    //    assert(voroCount<2);
    //    if (voroCount>=2) printf("voro>2 %d %d\n", x, y);
    return keep;
  }

  // 4-connected
  if ((!f[0] && f[1] && f[3]) || (!f[2] && f[1] && f[4]) || (!f[5] && f[3] && f[6]) || (!f[7] && f[6] && f[4])) return keep;
  if ((f[3] && f[4] && !f[1] && !f[6]) || (f[1] && f[6] && !f[3] && !f[4])) return keep;
  


  // keep voro cells inside of blocks and retry later
  if (voroCount>=5 && voroCountFour>=3 && data[x][y].voronoi!=voronoiRetry) {
    return retry;
  }

  return pruned;
}
