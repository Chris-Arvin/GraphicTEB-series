#ifndef _DYNAMICVORONOI_H_
#define _DYNAMICVORONOI_H_


#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include <queue>
#include <map>
#include <teb_local_planner/bucketedqueue.h>

struct dataCell {
  float dist;
  char voronoi;
  char queueing;
  int obstX;
  int obstY;
  bool needsRaise;
  int sqdist;
  std::pair<INTPOINT, INTPOINT> fatherPoints;
};

//! A DynamicVoronoi object computes and updates a distance map and Voronoi diagram.
class DynamicVoronoi {
  
public:
  
  DynamicVoronoi();
  ~DynamicVoronoi();

  //! Initialization with an empty map
  void initializeEmpty(int _sizeX, int _sizeY);
  //! Initialization with a given binary map (false==free, true==occupied)
  void initializeMap(std::vector<std::vector<int>> map_obs_labeled);
  //! update distance map and Voronoi diagram to reflect the changes
  void update(bool is_only_for_distance_map=false);
  //! prune the Voronoi diagram
  void prune();
  //! returns the obstacle distance at the specified location
  float getDistance( int x, int y );
  //! voronoi==free或者voronoiKeep if ture
  bool isVoronoi( int x, int y );
  //! 离点(x,y)最近的障碍物点是它自己 if true
  bool isOccupied(int x, int y);
  //! write the current distance map and voronoi diagram as ppm file
  void visualize(const char* filename="result.ppm");
  //! returns the horizontal size of the workspace/map
  unsigned int getSizeX() {return sizeX;}
  //! returns the vertical size of the workspace/map
  unsigned int getSizeY() {return sizeY;}

private:  
  typedef enum {voronoiKeep=-4, freeQueued = -3, voronoiRetry=-2, voronoiPrune=-1, free=0, occupied=1} State;
  typedef enum {fwNotQueued=1, fwQueued=2, fwProcessed=3, bwQueued=4, bwProcessed=1} QueueingState;
  typedef enum {invalidObstData = SHRT_MAX/2} ObstDataState;
  typedef enum {pruned, keep, retry} markerMatchResult;

  
  // methods
  // 核心操作：当本应该递减的海浪只能开始递增了，那么说明在此处分界了，产生一个voronoi point
  inline void checkVoro(int x, int y, int nx, int ny, dataCell& c, dataCell& nc);
  //! 离点(x,y)最近的障碍物点是它自己 if true
  inline bool isOccupied(int &x, int &y, dataCell &c);
  inline markerMatchResult markerMatch(int x, int y);

  // queues
  BucketPrioQueue open;
  std::queue<INTPOINT> pruneQueue;

  // maps
  int sizeY;
  int sizeX;
  std::vector<std::vector<dataCell>> data;
  std::vector<std::vector<int>> map_obs_labeled_;

public:
  dataCell getDataCell(int x, int y){ return data[x][y];};
  //  dataCell** getData(){ return data; }

};


#endif

