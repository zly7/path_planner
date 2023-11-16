#ifndef ALGORITHM_CONTOUR_H
#define ALGORITHM_CONTOUR_H


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
  class directionVector{
  public:
    float x;
    float y;
    directionVector(float x,float y){
      this->x = x;
      this->y = y;
    }
    directionVector(){
      this->x = 0;
      this->y = 0;
    }
    void normalize() {
        float length = sqrt(x * x + y * y);
        if (length != 0) {
            x /= length;
            y /= length;
        }
    }
  };
  class keyInfoForThrouthNarrowPair{
   public:
   directionVector wireUnitVector;
   Node2D* centerPoint;
   directionVector centerVerticalUnitVector;
   Node2D* firstBoundPoint;
   Node2D* secondBoundPoint;
  };
  class AlgorithmContour {
  public:
    /// The deault constructor
    AlgorithmContour() {}
    const static bool WhetherDebug = true;
    cv::Mat gridMap;
    std::vector<std::vector<Node2D*>> contoursFromGrid;
    std::vector<std::pair<Node2D*, Node2D*>> narrowPairs;
    std::vector<std::pair<Node2D*, Node2D*>> throughNarrowPairs;
    std::vector<std::vector<Node2D>> throughNarrowPairsWaypoints; //用来记录狭窄点周围的路径点
    std::vector<keyInfoForThrouthNarrowPair*> keyInfoForThrouthNarrowPairs;

    std::vector<std::vector<Node2D*>> findContour(nav_msgs::OccupancyGrid::Ptr grid);
    void findNarrowContourPair();
    void findThroughNarrowContourPair(const std::vector<Node2D> & path);

    bool determineWhetherThrough2DPath(const std::vector<Node2D> & path, std::pair<Node2D*, Node2D*> narrowPair,std::vector<Node2D> & containingWaypointsTorecord);
    void findKeyInformationForthrouthNarrowPairs(); //找一些关键的信息比如中垂方向向量，中点，
    static bool samplePathAndjudgeAcuteAngel(std::vector<Node2D> & path,directionVector midperpendicular);
    static void visualizeNarrowPairs(std::vector<std::pair<Node2D*,Node2D*>> narrowPairs, const cv::Mat & gridMap);
    static void visualizePathAndItNarrowPair(std::vector<Node2D> & path,std::pair<Node2D*,Node2D*> narrowPair,const cv::Mat & gridMap);
    static void visualizekeyInfoForThrouthNarrowPair(std::pair<Node2D*,Node2D*> narrowPair,keyInfoForThrouthNarrowPair* keyInfo,const cv::Mat & gridMap);
  };
}
#endif // ALGORITHM_CONTOUR_H
