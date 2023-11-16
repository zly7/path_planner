#ifndef ALGORITHM_SPLIT_H
#define ALGORITHM_SPLIT_H


#include "node3d.h"
#include "node2d.h"
#include "visualize.h"
#include "collisiondetection.h"
#include "constants.h"
#include <cmath>

namespace HybridAStar {
  class Node3D;
  class Node2D;
  class Visualize;
  class AlgorithmSplit {
  public:
    /// The deault constructor
    AlgorithmSplit() {}

  static void node2DToBox(std::vector<Node2D> &path2D,
                              int width,
                              int height,
                              CollisionDetection& configurationSpace,
                              float deltaL);

  static std::vector<Node3D> findBou(Node3D& start,
                              const Node3D& goal,
                              std::vector<Node2D> &path2D,
                              float threshold);

  };
}
#endif // ALGORITHM_SPLIT_H
