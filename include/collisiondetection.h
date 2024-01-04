#ifndef COLLISIONDETECTION_H
#define COLLISIONDETECTION_H

#include <nav_msgs/OccupancyGrid.h>

#include "constants.h"
#include "lookup.h"
#include "node2d.h"
#include "node3d.h"

namespace HybridAStar {
namespace {
inline void getConfiguration(const Node2D* node, float& x, float& y, float& t) {
  if(abs(node->getX()-node->getFloatX())<1e-6){
    x = node->getFloatX();
    y = node->getFloatY();
  }else{
    x = node->getX() + 0.5;//2D搜索的时候表征这个应该是用整数点的中心点去判断是不是configuration
    y = node->getY() + 0.5;
  }

  // avoid 2D collision checking
  t = 99;
}

inline void getConfiguration(const Node3D* node, float& x, float& y, float& t) {
  x = node->getX();
  y = node->getY();
  t = node->getT();
}
}
/*!
   \brief The CollisionDetection class determines whether a given configuration q of the robot will result in a collision with the environment.

   It is supposed to return a boolean value that returns true for collisions and false in the case of a safe node.
*/
class CollisionDetection {
 public:
  /// Constructor
  CollisionDetection();


  /*!
     \brief evaluates whether the configuration is safe
     \return true if it is traversable, else false
  */
  template<typename T> bool isTraversable(const T* node) const {
    /* Depending on the used collision checking mechanism this needs to be adjusted
       standard: collision checking using the spatial occupancy enumeration
       other: collision checking using the 2d costmap and the navigation stack
    */
    float cost = 1;
    float x;
    float y;
    float t;
    // assign values to the configuration
    getConfiguration(node, x, y, t);

    // // 2D collision test Zhang iyu: we should let the 2D point to check
    //如果2D搜索的时候有33.3(1/(8+1))%的方向认为可以过 ，就认为这个点可以过,as long as 1 one pass
    if(t == 99){
      for(int j = 0 ; j <Constants::headings; j ++){
        if(configurationTest(x+0.5, y+0.5, j*Constants::deltaHeadingRad+Constants::deltaHeadingRad/2)){
          return true;
        }
        //cost += configurationTest(x+0.5, y+0.5, j*Constants::deltaHeadingRad+Constants::deltaHeadingRad/2) ? -100 : 1;
      }
    }else{
      cost = configurationTest(x, y, t) ? -1 : 1;
    }

    // if (true) {
    //   cost = configurationTest(x, y, t) ? 0 : 1;
    // } else {
    //   cost = configurationCost(x, y, t);
    // }

    return cost <= 0;
  }
  /*'''更加精确的2D点的判断，用于是否需要反向'''*/
  bool isTraversablePreciseFor2D(const Node2D* node) const {
    float x = node->getFloatX();
    float y = node->getFloatY();
    for(int j = 0 ; j <Constants::headings; j ++){
      if(configurationTest(x, y, j*Constants::deltaHeadingRad+Constants::deltaHeadingRad/2)){
        return true;
      }
    }
    return false;
  }

  template<typename T> bool isObstacleThisPoint(const T* node) const {
    // std::cout<<" idx: "<<node->getIdx()<<" grid:"<<!grid->data[node->getIdx()]<<std::endl;
      return !grid->data[node->getIdx()];
  }

  /*!
     \brief Calculates the cost of the robot taking a specific configuration q int the World W  这个函数显然还没有完成
     \param x the x position
     \param y the y position
     \param t the theta angle
     \return the cost of the configuration q of W(q)
     \todo needs to be implemented correctly
  */
  float configurationCost(float x, float y, float t) const {return 0;}

  /*!
     \brief Tests whether the configuration q of the robot is in C_free
     \param x the x position
     \param y the y position
     \param t the theta angle
     \return true if it is in C_free, else false
  */
  bool configurationTest(float x, float y, float t) const;
  void visualizeGrid(nav_msgs::OccupancyGrid::Ptr grid, int gridSize, int gridWidth, int gridHeight) const;

  /*!
     \brief updates the grid with the world map
  */
  void updateGrid(nav_msgs::OccupancyGrid::Ptr map) {grid = map;} 

 private:
  /// The occupancy grid
  nav_msgs::OccupancyGrid::Ptr grid;
  /// The collision lookup table
  Constants::config collisionLookup[Constants::headings * Constants::positions];
};
}
#endif // COLLISIONDETECTION_H
