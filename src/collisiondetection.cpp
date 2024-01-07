#include "collisiondetection.h"
#include <opencv2/opencv.hpp>
using namespace HybridAStar;

CollisionDetection::CollisionDetection() {
  this->grid = nullptr; 
  Lookup::collisionLookup(collisionLookup);
}

bool CollisionDetection::configurationTest(float x, float y, float t) const {
  if(this->whetherVisualize){
    visualizeGridAndVehicle(x,y,t);
  }
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
bool CollisionDetection::configurationTestWithTolerace(float x, float y, float t, int tolerance) const {
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
  int collisionNum = 0;
  int toleranceForCollisionCheckMax = tolerance == -1 ? Constants::toleranceForCollisionCheck : tolerance;
  for (int i = 0; i < collisionLookup[idx].length; ++i) {
    cX = (X + collisionLookup[idx].pos[i].x);
    cY = (Y + collisionLookup[idx].pos[i].y);

    // make sure the configuration coordinates are actually on the grid  // 这里显然是做一个遍历
    if (cX >= 0 && (unsigned int)cX < grid->info.width && cY >= 0 && (unsigned int)cY < grid->info.height) {
      if (grid->data[cY * grid->info.width + cX]) {
        collisionNum++;
      }
      if(collisionNum > Constants::toleranceForCollisionCheck){
        return false;
      }
    }
  }

  return true;
}
void CollisionDetection::visualizeGridAndVehicle(float x, float y, float t)const{
  cv::Mat mapCopy;
  cv::Mat img(grid->info.height, grid->info.width, CV_8UC1);
  for (uint i = 0; i < grid->data.size(); i++) {
    img.data[i] = grid->data[i] >=1 ? 255 : 0; // threshold 
  }
  cv::cvtColor(img, mapCopy, cv::COLOR_GRAY2BGR);
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
  for (int i = 0; i < collisionLookup[idx].length; ++i) {
    cX = (X + collisionLookup[idx].pos[i].x);
    cY = (Y + collisionLookup[idx].pos[i].y);
    cv::circle(mapCopy, cv::Point(cX, cY), 1, cv::Scalar(0, 0, 255), -1);
  }
  cv::imshow("Debug For vehicle test", mapCopy);
  cv::waitKey(0);
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