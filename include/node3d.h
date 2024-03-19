#ifndef NODE3D_H
#define NODE3D_H

#include <cmath>
#include <random>
#include "constants.h"
#include "helper.h"
#include <iostream>
namespace HybridAStar {
  class Tolerance {
  public:
    float distanceTolerance;
    float angelTolerance;
    Tolerance(float distanceTolerance, float angelTolerance) : distanceTolerance(distanceTolerance), angelTolerance(angelTolerance) {}
    Tolerance() : distanceTolerance(Constants::tolerance), angelTolerance(Constants::deltaHeadingRad) {}
  };
/*!
   \brief A three dimensional node class that is at the heart of the algorithm.

   Each node has a unique configuration (x, y, theta) in the configuration space C.
*/
class Node3D {
 public:

  /// The default constructor for 3D array initialization
  Node3D(): Node3D(0, 0, 0, 0, 0, nullptr) {}
  Node3D(float x,float y, float t): Node3D(x, y, t, 0, 0, nullptr) {}
  /// Constructor for a node with the given arguments
  Node3D(float x, float y, float t, float g, float h, const Node3D* pred, int prim = 0) {
    this->x = x;
    this->y = y;
    this->t = t;
    this->g = g;
    this->h = h;
    this->pred = pred;
    this->o = false;
    this->c = false;
    this->idx = std::min((int)(t / Constants::deltaHeadingRad),Constants::headings-1) + (int)(y) * Constants::headings + (int)(x)* Constants::headings * Node3D::heightForMap;
    this->prim = prim;
  }

  // GETTER METHODS
  /// get the x position
  float getX() const { return x; }
  /// get the y position
  float getY() const { return y; }
  /// get the heading theta
  float getT() const { return t; }
  /// get the cost-so-far (real value)
  float getG() const { return g; }
  /// get the cost-to-come (heuristic value)
  float getH() const { return h; }
  /// get the total estimated cost
  float getC() const { return g + Constants::heuristicDecayCoefficient * h; }
  /// get the index of the node in the 3D array
  int getIdx() const { return idx; }
  /// get the number associated with the motion primitive of the node
  int getPrim() const { return prim; }
  /// determine whether the node is open
  bool isOpen() const { return o; }
  /// determine whether the node is closed
  bool isClosed() const { return c; }
  /// determine whether the node is open
  const Node3D* getPred() const { return pred; }

  // SETTER METHODS
  /// set the x position
  void setX(const float& x) { this->x = x; }
  /// set the y position
  void setY(const float& y) { this->y = y; }
  /// set the heading theta
  void setT(const float& t) { 
    this->t = t; 
  }
  /// set the cost-so-far (real value)
  void setG(const float& g) { this->g = g; }
  /// set the cost-to-come (heuristic value)
  void setH(const float& h) { this->h = h; }
  /// set and get the index of the node in the 3D grid
  // int setIdx(int width, int height) { //极端情况下有可能越界和headings正好相等
  //   // this->idx = std::min((int)(t / Constants::deltaHeadingRad),Constants::headings-1)* width * height + (int)(y) * width + (int)(x); return idx;
  //   this->idx = std::min((int)(t / Constants::deltaHeadingRad),Constants::headings-1) + (int)(y) * Constants::headings + (int)(x)* Constants::headings * height; return idx;
  // }
  void setPrim(int prim) { this->prim = prim; }
  /// open the node
  void open() { o = true; c = false;}
  /// close the node
  void close() { c = true; o = false; }
  /// set a pointer to the predecessor of the node
  void setPred(const Node3D* pred) { this->pred = pred; }

  // UPDATE METHODS
  /// Updates the cost-so-far for the node x' coming from its predecessor. It also discovers the node.
  void updateG();

  // CUSTOM OPERATORS
  /// Custom operator to compare nodes. Nodes are equal if their x and y position as well as heading is similar.
  bool operator == (const Node3D& rhs) const;
  bool isEqualWithTolerance (const Node3D& rhs,Tolerance tol) const;
  // RANGE CHECKING
  /// Determines whether it is appropriate to find a analytical solution.
  bool isInRange(const Node3D& goal) const;

  //触发是否调用arcShot的条件
  bool isInArcRange(const Node3D& goal) const;
  // GRID CHECKING
  /// Validity check to test, whether the node is in the 3D array.
  bool isOnGrid(const int width, const int height) const;
  bool isOnGrid() const;

  // SUCCESSOR CREATION
  /// Creates a successor in the continous space.
  Node3D* createSuccessor(const int i);
  float get2DDistance(Node3D rhs){
    return sqrt((x-rhs.getX())*(x-rhs.getX())+(y-rhs.getY())*(y-rhs.getY()));
  }

  // CONSTANT VALUES
  /// Number of possible directions
  static const int dir;
  /// @brief 直线弧长，应该是角度乘以pi
  static const float arcLength;
  /// @brief 转弯angel
  static const float steeringAngle;
  /// Possible movements in the x direction
  static std::vector<float> dx;
  /// Possible movements in the y direction
  static std::vector<float> dy;
  /// Possible movements regarding heading theta
  static std::vector<float> dt;
  static const float resolution_mutiplier;
  static std::mt19937 gen; // 随机数生成器
  static std::uniform_real_distribution<> dis; // 分布
  static void initializeVectorsForForward(); // 初始化向量
  static std::vector<Node3D> interpolateDirect(const Node3D& start, const Node3D& end, float interval);
  static int widthForMap;
  static int heightForMap;
 private:
  /// the x position
  float x;
  /// the y position
  float y;
  /// the heading theta
  float t;
  /// the cost-so-far
  float g;
  /// the cost-to-go
  float h;
  /// the index of the node in the 3D array
  int idx;
  /// the open value
  bool o;
  /// the closed value
  bool c;
  /// the motion primitive of the node
  int prim;
  /// the predecessor pointer
  const Node3D* pred;
};
}
#endif // NODE3D_H
