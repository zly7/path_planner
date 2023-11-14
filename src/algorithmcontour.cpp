#include "algorithmcontour.h"

#include <boost/heap/binomial_heap.hpp>

using namespace HybridAStar;

std::vector<std::vector<Node2D*>> AlgorithmContour::findContour(nav_msgs::OccupancyGrid::Ptr grid){
  // Convert the occupancy grid to an OpenCV Mat
  cv::Mat img(grid->info.height, grid->info.width, CV_8UC1);
  for (uint i = 0; i < grid->data.size(); i++) {
    img.data[i] = grid->data[i] >=1 ? 255 : 0; // threshold 
  }
  if(WhetherDebug){
    cv::imshow("img", img);
    cv::waitKey(0);
  }
  // Find contours
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  cv::Mat contourImg = cv::Mat::zeros(img.size(), CV_8UC3);
  for (uint i = 0; i < contours.size(); i++) {
      cv::Scalar color = cv::Scalar(0, 255, 0); // Green color for contours
      cv::Scalar pointColor = cv::Scalar(0, 0, 255);
      cv::drawContours(contourImg, contours, i, color, 2, cv::LINE_8, hierarchy, 0);
      // Draw each contour point
      for (uint j = 0; j < contours[i].size(); j++) {
          cv::circle(contourImg, contours[i][j], 2, pointColor, -1); // -1 fills the circle
      }
  }

  if(WhetherDebug){
      cv::imshow("Contours", contourImg);
      cv::waitKey(0);
  }

  // Convert contours from OpenCV format to Node2D*
  std::vector<std::vector<Node2D*>> result;
  for (auto& c : contours) {
    std::vector<Node2D*> contour;
    for (auto& p : c) {
      Node2D* node = new Node2D(p.x, p.y);
      contour.push_back(node);
    }
    result.push_back(contour);
  }
  this->contoursFromGrid = result;
  return result;
}

std::vector<std::pair<Node2D*,Node2D*>> findNarrowContourPair(){
    float minDistance = Constants::minContourPairDistance;
    float maxDistance = Constants::maxContourPairDistance;
    
    // 将所有轮廓的节点合并到一个单一的 vector 中
    std::vector<Node2D*> allNodes;
    for (const auto& contour : this->contourFromGrid) {
        allNodes.insert(allNodes.end(), contour.begin(), contour.end());
    }

    // 在合并后的 vector 中进行二重遍历
    for (size_t i = 0; i < allNodes.size(); ++i) {
        for (size_t j = i + 1; j < allNodes.size(); ++j) {
            float distance = allNodes[i]->distanceTo(allNodes[j]);
            if (distance >= minDistance && distance <= maxDistance) {
                this->narrowPairs.emplace_back(allNodes[i], allNodes[j]);
            }
        }
    }
    
    return this->narrowPairs;
}
