#include "node3d.h"

using namespace HybridAStar;

// CONSTANT VALUES
// possible directions
const int Node3D::dir = 3;

const float Node3D::resolution_mutiplier = Constants::each_meter_to_how_many_pixel;
const float Node3D::arcLength = Constants::arcLengthForAstarSuccessor  * Constants::each_meter_to_how_many_pixel;
const float Node3D::steeringAngle = M_PI * 6.75 / 180.0;



std::mt19937 Node3D::gen(std::random_device{}());
std::uniform_real_distribution<> Node3D::dis(0.8, 1.25);
//之前声明但是没有初始化
std::vector<float> Node3D::dx;
std::vector<float> Node3D::dy;
std::vector<float> Node3D::dt;

void Node3D::initializeVectorsForForward() {
  if (dir == 3) {
      dy =  { 0, -Node3D::arcLength*sin(Node3D::steeringAngle), Node3D::arcLength*sin(Node3D::steeringAngle)};
      dx =  { Node3D::arcLength,   Node3D::arcLength*cos(Node3D::steeringAngle),  Node3D::arcLength*cos(Node3D::steeringAngle)};
      dt = { 0,         Node3D::steeringAngle,   -Node3D::steeringAngle};
  } else if (dir == 5) {
      dy = { 0, -Node3D::arcLength*sin(Node3D::steeringAngle),-Node3D::arcLength*sin(Node3D::steeringAngle/2), 
        Node3D::Node3D::arcLength*sin(Node3D::steeringAngle/2),Node3D::arcLength*sin(Node3D::steeringAngle)};
      dx = { Node3D::arcLength,   Node3D::arcLength*cos(Node3D::steeringAngle), Node3D::arcLength*cos(Node3D::steeringAngle/2),
        Node3D::arcLength*cos(Node3D::steeringAngle/2),Node3D::arcLength*cos(Node3D::steeringAngle)};
      dt = { 0,         Node3D::steeringAngle,   Node3D::steeringAngle/2, -Node3D::steeringAngle/2, -Node3D::steeringAngle};
  }
}

//###################################################
//                                         IS ON GRID
//###################################################
bool Node3D::isOnGrid(const int width, const int height) const {
  return x >= 0 && x < width && y >= 0 && y < height && (int)(t / Constants::deltaHeadingRad) >= 0 && (int)(t / Constants::deltaHeadingRad) < Constants::headings;
}


//###################################################
//                                        IS IN RANGE
//###################################################
bool Node3D::isInRange(const Node3D& goal) const {
  int random = rand() % 10 + 1;
  float dx = std::abs(x - goal.x) / random;
  float dy = std::abs(y - goal.y) / random;
  return (dx * dx) + (dy * dy) < Constants::dubinsShotDistance;
}

bool Node3D::isInArcRange(const Node3D& goal) const {
  float dx = std::abs(x - goal.x) ;
  float dy = std::abs(y - goal.y) ;
  return (dx * dx) + (dy * dy) < Constants::arcShotDistance;
}

//###################################################
//                                   CREATE SUCCESSOR
//###################################################
Node3D* Node3D::createSuccessor(const int i) {
  float xSucc;
  float ySucc;
  float tSucc; 
  float tempDy[Node3D::dir];
  float tempDx[Node3D::dir];
  float randomValue = Node3D::dis(Node3D::gen);
  // 对静态数组进行操作并除以随机数
  for (int j = 0; j < Node3D::dir; ++j) {
      tempDy[j] = dy[j] / randomValue;
      tempDx[j] = dx[j] / randomValue;
  }

  // calculate successor positions forward
  if (i < Node3D::dir) {
    xSucc = x + tempDx[i] * cos(t) - tempDy[i] * sin(t);
    ySucc = y + tempDx[i] * sin(t) + tempDy[i] * cos(t);
    tSucc = Helper::normalizeHeadingRad(t + dt[i]);
  }
  // backwards
  else {
    xSucc = x - tempDx[i - Node3D::dir] * cos(t) - tempDy[i - Node3D::dir] * sin(t);
    ySucc = y - tempDx[i - Node3D::dir] * sin(t) + tempDy[i - Node3D::dir] * cos(t);
    tSucc = Helper::normalizeHeadingRad(t - dt[i - Node3D::dir]);
  }

  return new Node3D(xSucc, ySucc, tSucc, g, 0, this, i);
}


//###################################################
//                                      MOVEMENT COST
//###################################################
void Node3D::updateG() {
  // forward driving
  if (prim < Node3D::dir) {
    // penalize turning
    if (pred->prim != prim) {
      // penalize change of direction
      if (pred->prim > Node3D::dir-1) {
        g += dx[0] * Constants::penaltyTurning * Constants::penaltyCOD;
      } else {
        g += dx[0] * Constants::penaltyTurning;
      }
    } else {
      g += dx[0];
    }
  }
  // reverse driving
  else {
    // penalize turning and reversing
    if (pred->prim != prim) {
      // penalize change of direction
      if (pred->prim < Node3D::dir) {
        g += dx[0] * Constants::penaltyTurning * Constants::penaltyReversing * Constants::penaltyCOD;
      } else {
        g += dx[0] * Constants::penaltyTurning * Constants::penaltyReversing;
      }
    } else {
      g += dx[0] * Constants::penaltyReversing;
    }
  }
}


//###################################################
//                                 3D NODE COMPARISON
//###################################################
bool Node3D::operator == (const Node3D& rhs) const {
  return (int)x == (int)rhs.x &&
         (int)y == (int)rhs.y &&
         (std::abs(t - rhs.t) <= Constants::deltaHeadingRad ||
          std::abs(t - rhs.t) >= Constants::deltaHeadingNegRad);
}
