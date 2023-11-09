#include "collisiondetection.h"
#include <opencv2/opencv.hpp>
using namespace HybridAStar;

CollisionDetection::CollisionDetection() {
  this->grid = nullptr; 
  Lookup::collisionLookup(collisionLookup);
}

bool CollisionDetection::configurationTest(float x, float y, float t) const {
  int X = (int)x;
  int Y = (int)y;
  int iX = (int)((x - (long)x) * Constants::positionResolution);
  iX = iX > 0 ? iX : 0;
  int iY = (int)((y - (long)y) * Constants::positionResolution);
  iY = iY > 0 ? iY : 0;
  int iT = (int)(t / Constants::deltaHeadingRad);
  int idx = iY * Constants::positionResolution * Constants::headings + iX * Constants::headings + iT;  //但凡是3D的碰撞检测都会具体到3维坐标的某个点的某个朝向的检测
  int cX;
  int cY;
  // CollisionDetection::visualizeGrid(grid, (int)grid->data.size(), (int)grid->info.width, (int)grid->info.height);
  // for(int i =0;i<collisionLookup[idx].length;++i){
  //   std::cout<<"i: "<<i<<" x: "<<collisionLookup[idx].pos[i].x<<" y: "<<collisionLookup[idx].pos[i].y<<std::endl;
  // }
  for (int i = 0; i < collisionLookup[idx].length; ++i) {
    cX = (X + collisionLookup[idx].pos[i].x);
    cY = (Y + collisionLookup[idx].pos[i].y);

    // make sure the configuration coordinates are actually on the grid  // 这里显然是做一个遍历
    if (cX >= 0 && (unsigned int)cX < grid->info.width && cY >= 0 && (unsigned int)cY < grid->info.height) {
      if (grid->data[cY * grid->info.width + cX]) {
        return false;
      }
    }
  }

  return true;
}

void CollisionDetection::visualizeGrid(nav_msgs::OccupancyGrid::Ptr grid, int gridSize, int gridWidth, int gridHeight) const {

  cv::Mat visGrid(gridHeight, gridWidth, CV_8UC1);
  
  for (int i = 0; i < gridHeight; i++) {
    for (int j = 0; j < gridWidth; j++) {
      int idx = j * gridWidth + i;
      if (idx < gridSize) {
        visGrid.at<uchar>(gridHeight - j - 1, i) = grid->data[idx] ? 255 : 0;  
      } else {
        visGrid.at<uchar>(gridHeight - j - 1, i) = 0;
      }
    }
  }
  cv::imshow("Visualized", visGrid);
  cv::waitKey(0);

}