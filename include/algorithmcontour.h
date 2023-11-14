#ifndef ALGORITHM_CONTOUR_H
#define ALGORITHM_CONTOUR_H


#include "node3d.h"
#include "node2d.h"
#include "visualize.h"
#include "collisiondetection.h"
#include "Constants.h"
#include <cmath>

namespace HybridAStar {
  class Node3D;
  class Node2D;
  class Visualize;
  class AlgorithmContour {
  public:
    /// The deault constructor
    AlgorithmContour() {}

    const static bool WhetherDebug = true;
    std::vector<std::vector<Node2D*>> contoursFromGrid;
    std::vector<std::pair<Node2D*, Node2D*>> narrowPairs;

    std::vector<std::vector<Node2D*>> findContour(nav_msgs::OccupancyGrid::Ptr grid);
    std::vector<std::pair<Node2D*,Node2D*>> findNarrowContourPair();
    std::vector<std::pair<Node2D*,Node2D*>> findThroughNarrowContourPair();
    bool determineWhetherThrough2DPath(std::vector<Node2D> path, std::vector<std::vector<Node2D*>> contours);
    

  };
}
#endif // ALGORITHM_CONTOUR_H
