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
  this->gridMap = img;
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

void AlgorithmContour::findNarrowContourPair(){
    float minDistance = Constants::minContourPairDistance;
    float maxDistance = Constants::maxContourPairDistance;
    
    // 将所有轮廓的节点合并到一个单一的 vector 中
    std::vector<Node2D*> allNodes;
    for (const auto& contour : this->contoursFromGrid) {
        allNodes.insert(allNodes.end(), contour.begin(), contour.end());
    }

    // 在合并后的 vector 中进行二重遍历
    for (size_t i = 0; i < allNodes.size(); ++i) {
        for (size_t j = i + 1; j < allNodes.size(); ++j) {
            float distance = allNodes[i]->distanceTo(allNodes[j]);
            if (distance > minDistance && distance < maxDistance) {
                this->narrowPairs.emplace_back(allNodes[i], allNodes[j]);
            }
        }
    }
    
    return;
}
void AlgorithmContour::findThroughNarrowContourPair(const std::vector<Node2D> & path){

    for (const auto& narrowPair : this->narrowPairs) {
        std::vector<Node2D> containingWaypointsTorecord;
        if (this->determineWhetherThrough2DPath(path, narrowPair,containingWaypointsTorecord)) {
            this->throughNarrowPairs.push_back(narrowPair);
            std::reverse(containingWaypointsTorecord.begin(),containingWaypointsTorecord.end()); //拿到的path是从尾部trace过来的
            this->throughNarrowPairsWaypoints.push_back(containingWaypointsTorecord);
        }
    }
    return;
};

/*决定狭窄的点是否穿过路径*/
bool AlgorithmContour::determineWhetherThrough2DPath(const std::vector<Node2D>& path, 
          std::pair<Node2D*, Node2D*> narrowPair,std::vector<Node2D> & containingWaypointsTorecord){
    int minContinue = Constants::howManyNode2DDeterminesWhetherThroughNarrowContourPair;
    int continueCount = 0;
    int continueCountMax = 0;
    Node2D* pairFirst = narrowPair.first;
    Node2D* pairSecond = narrowPair.second;
    //应该是再互化同心圆的相交里面
    float maxDistance = pairFirst->distanceTo(pairSecond);

    bool isFlag = false;
    for (const auto& node : path) {
        if (node.distanceTo(narrowPair.first) < maxDistance && node.distanceTo(narrowPair.second) < maxDistance) {
            continueCount++;
            containingWaypointsTorecord.push_back(node);
            if(continueCount > continueCountMax){
                continueCountMax = continueCount;
            }
        }
        else {
            continueCount = 0;
            if(isFlag==true){
              break;
            }else{
              containingWaypointsTorecord.clear();
            }
        }
        if (continueCount >= minContinue) {
            isFlag = true;
        }
    }
    //如果只是在2个圆内，但是没有相交，这个还是不能算作我们需要求的狭窄路径点
    if(isFlag==true && !Helper::isIntersect(pairFirst, pairSecond, &containingWaypointsTorecord[0], &containingWaypointsTorecord[containingWaypointsTorecord.size() - 1])){
        isFlag = false;
    }
    // if(WhetherDebug){
    //     std::cout << "continueCountMax: " << continueCountMax << std::endl;
    //     AlgorithmContour::visualizePathAndItNarrowPair(containingWaypointsTorecord,narrowPair,this->gridMap);
    // }
    return isFlag;
}

void AlgorithmContour::findKeyInformationForthrouthNarrowPairs(){
  int index = 0;
  for (const auto& pair : this->throughNarrowPairs) {
      Node2D* firstPoint = pair.first;
      Node2D* secondPoint = pair.second;

      // 计算指向方向
      keyInfoForThrouthNarrowPair* keyInfo = new keyInfoForThrouthNarrowPair();
      this->keyInfoForThrouthNarrowPairs.push_back(keyInfo);
      directionVector unitWireVector{};
      unitWireVector.x = secondPoint->getFloatX() - firstPoint->getFloatX();
      unitWireVector.y = secondPoint->getFloatY() - firstPoint->getFloatY();
      unitWireVector.normalize();
      keyInfo->wireUnitVector = unitWireVector;
      Node2D* centerPoint = new Node2D((firstPoint->getFloatX() + secondPoint->getFloatX()) / 2,
                                        (firstPoint->getFloatY() + secondPoint->getFloatY()) / 2);
      keyInfo->centerPoint = centerPoint;
      
      directionVector centerVerticalUnitVector{unitWireVector.y,-unitWireVector.x};//和中垂线计算夹角确定中垂线的方向
      if(!samplePathAndjudgeAcuteAngel(this->throughNarrowPairsWaypoints[index],centerVerticalUnitVector)){
        centerVerticalUnitVector.x = -1*centerVerticalUnitVector.x;
        centerVerticalUnitVector.y = -1*centerVerticalUnitVector.y;
      }
      keyInfo->centerVerticalUnitVector = centerVerticalUnitVector;
      float halfWidth = Constants::width / 2.0 * 1.2; //稍微留一些余地
      Node2D* firstBoundPoint = new Node2D(firstPoint->getFloatX() + halfWidth * unitWireVector.x, 
                                          firstPoint->getFloatY() + halfWidth * unitWireVector.y);
      Node2D* secondBoundPoint = new Node2D(secondPoint->getFloatX() - halfWidth * unitWireVector.x, 
                                            secondPoint->getFloatY() - halfWidth * unitWireVector.y);

      keyInfo->firstBoundPoint = firstBoundPoint;
      keyInfo->secondBoundPoint = secondBoundPoint;
  }
};

bool AlgorithmContour::samplePathAndjudgeAcuteAngel(std::vector<Node2D> & path,directionVector midperpendicular){
  float allDot = 0;
  for(uint i =0;i<path.size()-3;i++){
    Node2D* nodeStart = &path[i];
    float x0 = nodeStart->getFloatX();
    float y0 = nodeStart->getFloatY();
    Node2D* nodeEnd = &path[i+3];
    float x1 = nodeEnd->getFloatX();
    float y1 = nodeEnd->getFloatY();
    float x = x1 - x0;
    float y = y1 - y0;
    directionVector vector{x,y};
    vector.normalize();
    float dot = vector.x * midperpendicular.x + vector.y * midperpendicular.y;
    allDot += dot;
  }
  if(allDot > 0){
    return true;
  }else{
    return false;
  }

}


void AlgorithmContour::visualizeNarrowPairs(std::vector<std::pair<Node2D*,Node2D*>> narrowPairs,const cv::Mat & gridMap){
    int pairsCount = narrowPairs.size();
    int howManyCols = (int)ceil(sqrt(pairsCount));
    int rows = ceil(pairsCount / ((float)howManyCols));
    int singleWidth = gridMap.cols;
    int singleHeight = gridMap.rows;
    cv::Mat bigImage(rows * singleHeight, howManyCols * singleWidth, CV_8UC3);

    for (int i = 0; i < pairsCount; ++i) {
        // 创建地图副本
        cv::Mat mapCopy;
        cv::cvtColor(gridMap, mapCopy, cv::COLOR_GRAY2BGR);
        // 绘制点
        cv::circle(mapCopy, cv::Point(narrowPairs[i].first->getFloatX(), narrowPairs[i].first->getFloatY()), 3, cv::Scalar(0, 0, 255), -1);
        cv::circle(mapCopy, cv::Point(narrowPairs[i].second->getFloatX(), narrowPairs[i].second->getFloatY()), 3, cv::Scalar(255, 0, 0), -1);

        // 计算当前图像应该放在大图像的位置
        int row = i / howManyCols;
        int col = i % howManyCols;
        mapCopy.copyTo(bigImage(cv::Rect(col * singleWidth, row * singleHeight, singleWidth, singleHeight)));
    }
      cv::Size newSize(600, 600*gridMap.cols/gridMap.rows);
      cv::Mat resizedImage;
      cv::resize(bigImage, resizedImage, newSize);
      cv::imshow("Resized Narrow Pairs Visualization", resizedImage);
      cv::waitKey(0);

}

void AlgorithmContour::visualizePathAndItNarrowPair(std::vector<Node2D> & path, std::pair<Node2D*, Node2D*> narrowPair, const cv::Mat & gridMap){
    cv::Mat mapCopy;
    cv::cvtColor(gridMap, mapCopy, cv::COLOR_GRAY2BGR);
    
    cv::circle(mapCopy, cv::Point(narrowPair.first->getFloatX(), narrowPair.first->getFloatY()), 3, cv::Scalar(0, 0, 255), -1);
    cv::circle(mapCopy, cv::Point(narrowPair.second->getFloatX(), narrowPair.second->getFloatY()), 3, cv::Scalar(255, 0, 0), -1);

    for(uint i = 0; i < path.size(); i++){
        cv::circle(mapCopy, cv::Point(path[i].getFloatX(), path[i].getFloatY()), 2, cv::Scalar(0, 255, 0), -1);
    }

    cv::imshow("Path and Narrow Pairs Visualization", mapCopy);
    cv::waitKey(0);
}

void AlgorithmContour::visualizekeyInfoForThrouthNarrowPair(std::pair<Node2D*, Node2D*> narrowPair, keyInfoForThrouthNarrowPair* keyInfo, const cv::Mat & gridMap) {
    cv::Mat mapCopy;
    cv::cvtColor(gridMap, mapCopy, cv::COLOR_GRAY2BGR);

    // 绘制窄对的点
    cv::circle(mapCopy, cv::Point(narrowPair.first->getFloatX(), narrowPair.first->getFloatY()), 1.5, cv::Scalar(0, 0, 255), -1);
    cv::circle(mapCopy, cv::Point(narrowPair.second->getFloatX(), narrowPair.second->getFloatY()), 1.5, cv::Scalar(255, 0, 0), -1);
    cv::circle(mapCopy, cv::Point(keyInfo->centerPoint->getFloatX(), keyInfo->centerPoint->getFloatY()), 1.5, cv::Scalar(0, 255, 0), -1);
    cv::circle(mapCopy, cv::Point(keyInfo->firstBoundPoint->getFloatX(), keyInfo->firstBoundPoint->getFloatY()), 1.5, cv::Scalar(0, 255, 0), -1);
    cv::circle(mapCopy, cv::Point(keyInfo->secondBoundPoint->getFloatX(), keyInfo->secondBoundPoint->getFloatY()), 1.5, cv::Scalar(0, 255, 0), -1);
    // 绘制中垂线
    cv::Point center(keyInfo->centerPoint->getFloatX(), keyInfo->centerPoint->getFloatY());
    cv::Point centerVerticalEnd(center.x + Constants::each_meter_to_how_many_pixel * keyInfo->centerVerticalUnitVector.x, center.y + Constants::each_meter_to_how_many_pixel * keyInfo->centerVerticalUnitVector.y);
    cv::line(mapCopy, center, centerVerticalEnd, cv::Scalar(255, 255, 0), 1);

    // 显示图像
    cv::imshow("Key Information Visualization", mapCopy);
    cv::waitKey(0);
}

float AlgorithmContour::findNarrowPassSpace(CollisionDetection &configurationSpace, directionVector &unitWireVector, directionVector &centerVerticalUnitVector, Node2D *startPoint,int width,int height)
{
  float radius=Constants::minRadius;
  //如果超过最大半径就尝试直线
  float maxRadiu=std::sqrt(width*width+height*height)/2/sin(Constants::maxAngle/2);
  while (radius<=maxRadiu)
  {
    float centerX = startPoint->getFloatX()+unitWireVector.x*radius;
    float centerY = startPoint->getFloatY()+unitWireVector.y*radius;
    Node2D* circleCenterPoint = new Node2D(centerX,centerY);
    float cross=centerVerticalUnitVector.x*unitWireVector.y-centerVerticalUnitVector.y*unitWireVector.x;
    if(cross>0){
      cross=1;
    }else cross=-1;
    float angle = atan2f(centerVerticalUnitVector.y,centerVerticalUnitVector.x);
    angle=Helper::normalizeHeadingRad(angle-cross*M_PI/2);
    bool flag=true;
    for(int x=0;x<Constants::maxAngle&&flag;x+=(Constants::maxAngle/100)){
      float angle1=Helper::normalizeHeading(angle+cross*x);
      float pointX=circleCenterPoint->getFloatX()+radius*sin(angle1);
      float pointY=circleCenterPoint->getFloatY()+radius*cos(angle1);
      Node2D nNode =  Node2D(pointX, pointY);
      if(!configurationSpace.isObstacleThisPoint(&nNode)){
        flag=false;
        break;
      }
    }
    if(flag){
      return radius;
    }else{
      radius+=Constants::deltaRadius;
    }
  }
  bool flag=true;
  for(int x=0;x<Constants::straightLength&&flag;x+=(Constants::straightLength/100)){
      float pointX=startPoint->getFloatX()+x*centerVerticalUnitVector.x;
      float pointY=startPoint->getFloatY()+x*centerVerticalUnitVector.y;
      Node2D nNode =  Node2D(pointX, pointY);
      if(!configurationSpace.isObstacleThisPoint(&nNode)){
        flag=false;
        break;
      }
  }
  if(flag){
    return 0;
  }
  radius=Constants::minRadius;
  unitWireVector.x=-unitWireVector.x;
  unitWireVector.y=-unitWireVector.y;
  while (radius<=maxRadiu)
  {
    float centerX = startPoint->getFloatX()+unitWireVector.x*radius;
    float centerY = startPoint->getFloatY()+unitWireVector.y*radius;
    Node2D* circleCenterPoint = new Node2D(centerX,centerY);
    float cross=centerVerticalUnitVector.x*unitWireVector.y-centerVerticalUnitVector.y*unitWireVector.x;
    if(cross>0){
      cross=1;
    }else cross=-1;
    float angle = atan2f(centerVerticalUnitVector.y,centerVerticalUnitVector.x);
    angle=Helper::normalizeHeadingRad(angle-cross*M_PI/2);
    bool flag=true;
    for(int x=0;x<Constants::maxAngle&&flag;x+=(Constants::maxAngle/100)){
      float angle1=Helper::normalizeHeading(angle+cross*x);
      float pointX=circleCenterPoint->getFloatX()+radius*sin(angle1);
      float pointY=circleCenterPoint->getFloatY()+radius*cos(angle1);
      Node2D nNode =  Node2D(pointX, pointY);
      if(!configurationSpace.isObstacleThisPoint(&nNode)){
        flag=false;
        break;
      }
    }
    if(flag){
      return -radius;
    }else{
      radius+=Constants::deltaRadius;
    }
  }
  return -Constants::minRadius/2;
}

void AlgorithmContour::findNarrowPassSpaceForAllPairs(CollisionDetection &configurationSpace,int width,int height)
{
  for (keyInfoForThrouthNarrowPair* myPair:keyInfoForThrouthNarrowPairs ) {
    directionVector unitWireVector{};
    unitWireVector.x = myPair->centerPoint->getFloatX() - myPair->firstBoundPoint->getFloatX();
    unitWireVector.y = myPair->centerPoint->getFloatY() - myPair->firstBoundPoint->getFloatY();
    unitWireVector.normalize();
    myPair->firstRadius1=findNarrowPassSpace(configurationSpace,unitWireVector,myPair->centerVerticalUnitVector,myPair->firstBoundPoint,width,height);
    directionVector centerVerticalUnitVector{-myPair->centerVerticalUnitVector.x,-myPair->centerVerticalUnitVector.y};
    myPair->firstRadius1=findNarrowPassSpace(configurationSpace,unitWireVector,centerVerticalUnitVector,myPair->firstBoundPoint,width,height);
    unitWireVector.x = myPair->centerPoint->getFloatX() - myPair->secondBoundPoint->getFloatX();
    unitWireVector.y = myPair->centerPoint->getFloatY() - myPair->secondBoundPoint->getFloatY();
    unitWireVector.normalize();
    myPair->firstRadius1=findNarrowPassSpace(configurationSpace,unitWireVector,myPair->centerVerticalUnitVector,myPair->secondBoundPoint,width,height);
    myPair->firstRadius1=findNarrowPassSpace(configurationSpace,unitWireVector,centerVerticalUnitVector,myPair->firstBoundPoint,width,height);
  }
}
