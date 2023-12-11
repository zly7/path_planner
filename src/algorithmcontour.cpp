#include "algorithmcontour.h"

#include <boost/heap/binomial_heap.hpp>
#include <CGAL/Cartesian.h>
#include <CGAL/Segment_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

using namespace HybridAStar;

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_2 Point_2;
typedef Kernel::Segment_2 Segment_2;
typedef Kernel::Line_2 Line_2;


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
      Node2D* node = new Node2D((float)p.x+0.5, (float)p.y+0.5);//opencv 寻找角点的时候会选择内部的像素点，然后还有问题就是像素点的值本质上是左下角的int值。所以转化成float的时候全部加0.5
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
    std::set<size_t> nodesToRemove;
    std::vector<Node2D*> middleNodes;

    // 在合并后的 vector 中进行二重遍历
    for (size_t i = 0; i < allNodes.size(); ++i) {
        for (size_t j = i + 1; j < allNodes.size(); ++j) {
            float distance = allNodes[i]->distanceTo(allNodes[j]);
            if (distance < Constants::theMindistanceDetermineWhetherTheSameContourPoint) {
                middleNodes.push_back(Node2D::middle(allNodes[i], allNodes[j]));
                nodesToRemove.insert(i);
                nodesToRemove.insert(j);
            }
        }
    }

    // 移除标记的节点
    for (auto it = nodesToRemove.rbegin(); it != nodesToRemove.rend(); ++it) {
        allNodes.erase(allNodes.begin() + *it);
    }
      // 将新生成的中间节点加入到 allNodes
    allNodes.insert(allNodes.end(), middleNodes.begin(), middleNodes.end());
   // 步骤1: 创建映射
    std::unordered_map<Node2D*, std::pair<float, Node2D*>> closestPairs;

    // 初始化映射，设定初始最近距离为最大值
    for (auto& node : allNodes) {
        closestPairs[node] = std::make_pair(std::numeric_limits<float>::max(), nullptr);
    }

    // 步骤2: 遍历所有节点，更新最近点对信息
    for (size_t i = 0; i < allNodes.size(); ++i) {
        for (size_t j = i + 1; j < allNodes.size(); ++j) {
            float distance = allNodes[i]->distanceTo(allNodes[j]);
            if (distance > minDistance && distance < maxDistance) {
                // 更新节点i的最近点对信息
                if (distance < closestPairs[allNodes[i]].first) {
                    closestPairs[allNodes[i]] = std::make_pair(distance, allNodes[j]);
                }
                // 更新节点j的最近点对信息
                if (distance < closestPairs[allNodes[j]].first) {
                    closestPairs[allNodes[j]] = std::make_pair(distance, allNodes[i]);
                }
            }
        }
    }
    // 从 unordered_map 复制到 vector
    std::vector<std::pair<Node2D*, std::pair<float, Node2D*>>> sortedPairs;
    for (const auto& pair : closestPairs) {
      if(pair.second.second != nullptr){
          sortedPairs.push_back(pair);
      }
    }

    // 对 vector 进行排序,保证是从最小的距离开始添加
    std::sort(sortedPairs.begin(), sortedPairs.end(), 
        [](const std::pair<Node2D*, std::pair<float, Node2D*>>& a, 
          const std::pair<Node2D*, std::pair<float, Node2D*>>& b) {
            return a.second.first < b.second.first;
        });

    //防止重复添加的匿名函数
    auto isPairExists = [&narrowPairs = this->narrowPairs]( Node2D* second) {
        for (const auto& p : narrowPairs) {
            if ((p.first == second ) || (p.second == second)) {
                return true;
            }
        }
        return false;
    };

    for (const auto& pair : sortedPairs) {//保证我匹配的对家没有在当前的narrowPairs里面
        if (pair.second.second != nullptr && !isPairExists(pair.second.second)) {
            this->narrowPairs.emplace_back(pair.first, pair.second.second);
        }
    }
    return;
}
//筛选出穿过路径的有效的狭窄点对
void AlgorithmContour::findThroughNarrowContourPair(const std::vector<Node2D> & path){

    for (const auto& narrowPair : this->narrowPairs) {
        std::vector<Node2D> containingWaypointsTorecord;
        int aroundWaypointsIndex = 0;
        if (this->determineWhetherThrough2DPath(path, narrowPair,containingWaypointsTorecord,aroundWaypointsIndex)) {
            this->throughNarrowPairs.push_back(narrowPair);
            std::reverse(containingWaypointsTorecord.begin(),containingWaypointsTorecord.end()); //拿到的path是从尾部trace过来的
            this->throughNarrowPairsWaypoints.push_back(containingWaypointsTorecord);
        }
    }
    return;
};

/*决定狭窄的点是否穿过路径*/
bool AlgorithmContour::determineWhetherThrough2DPath(const std::vector<Node2D>& path, 
          std::pair<Node2D*, Node2D*> narrowPair,std::vector<Node2D> & containingWaypointsTorecord,int & aroundWaypointsIndex){
    int minContinue = Constants::howManyNode2DDeterminesWhetherThroughNarrowContourPair;
    int continueCount = 0;
    int continueCountMax = 0;
    Node2D* pairFirst = narrowPair.first;
    Node2D* pairSecond = narrowPair.second;
    //应该是再互化同心圆的相交里面
    float maxDistance = pairFirst->distanceTo(pairSecond);
    int index = 0;
    int allIndex = 0;
    bool isFlag = false;
    for (const auto& node : path) {
        if (node.distanceTo(narrowPair.first) < maxDistance && node.distanceTo(narrowPair.second) < maxDistance) {
            continueCount++;
            containingWaypointsTorecord.push_back(node);
            allIndex += index;
            if(continueCount > continueCountMax){
                continueCountMax = continueCount;
            }
        }
        else {
            continueCount = 0;
            if(isFlag==true){
              aroundWaypointsIndex = allIndex / containingWaypointsTorecord.size();
              break;
            }else{
              allIndex = 0;
              containingWaypointsTorecord.clear();
            }
        }
        if (continueCount >= minContinue) {
            isFlag = true;
        }
        index++;
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

void AlgorithmContour::sortThroughNarrowPairsWaypoints(){
  // 检查两个向量的大小是否相等
  if (throughNarrowPairsWaypoints.size() != aroundWaypointsIndexOfThroughNarrowPairs.size()) {
      std::cout << "Error: throughNarrowPairsWaypoints.size() != aroundWaypointsIndexOfThroughNarrowPairs.size()" << std::endl;
      return;
  }

  // 创建一个索引数组，用于排序
  std::vector<size_t> indices(throughNarrowPairsWaypoints.size());
  for (size_t i = 0; i < indices.size(); ++i) {
      indices[i] = i;
  }

  // 使用自定义比较函数排序索引
  std::sort(indices.begin(), indices.end(), [this](size_t a, size_t b) {
      return aroundWaypointsIndexOfThroughNarrowPairs[a] < aroundWaypointsIndexOfThroughNarrowPairs[b];
  });

  // 根据排序后的索引创建一个新的 waypoints 向量
  std::vector<std::vector<Node2D>> sortedWaypoints(throughNarrowPairsWaypoints.size());
  for (size_t i = 0; i < indices.size(); ++i) {
      sortedWaypoints[i] = throughNarrowPairsWaypoints[indices[i]];
  }

  // 更新原来的 waypoints 向量
  throughNarrowPairsWaypoints = sortedWaypoints;
}

void AlgorithmContour::findKeyInformationForthrouthNarrowPairs(){
  int index = 0;
  float offsetForHalfVehicleWidth = ((float) (Constants::width+1)) / 2.0 * Constants::offsetPercentForHalfVehicleWidth; //稍微留一些余地,+1也是为了留余地
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
      float tempT = Helper::normalizeHeadingRad(atan2(centerVerticalUnitVector.y,centerVerticalUnitVector.x));
      Node3D centerVerticalPoint(centerPoint->getFloatX() , 
                                              centerPoint->getFloatY() , 
                                              tempT, 0, 0, nullptr);
      keyInfo->centerVerticalPoint3D = centerVerticalPoint;
      Node2D* firstBoundPoint = new Node2D(firstPoint->getFloatX() + offsetForHalfVehicleWidth * unitWireVector.x, 
                                          firstPoint->getFloatY() + offsetForHalfVehicleWidth * unitWireVector.y);
      Node2D* secondBoundPoint = new Node2D(secondPoint->getFloatX() - offsetForHalfVehicleWidth * unitWireVector.x, 
                                            secondPoint->getFloatY() - offsetForHalfVehicleWidth * unitWireVector.y);

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

    cv::resize(mapCopy, mapCopy, cv::Size(600, 600));
    // 显示图像
    cv::imshow("Key Information Visualization", mapCopy);
    cv::waitKey(0);
}

// 绘制向量中的点
void drawPoints(const std::vector<Node3D>& points, cv::Mat& map, const cv::Scalar& color, float positionMul) {
    int arrowLength = 8;
    auto drawArrow = [&](const Node3D &startNode3D) {
      directionVector direction = directionVector::getUnitVectorFromNode3D(&startNode3D);
      cv::Point start(startNode3D.getX() * positionMul, startNode3D.getY() * positionMul);
      cv::Point end(start.x + direction.x * arrowLength, start.y + direction.y * arrowLength);
      cv::arrowedLine(map, start, end, color, 1, 8, 0, 0.3);
    };
    for (const auto& point : points) {
        drawArrow(point);
    }
}

std::vector<Node3D> AlgorithmContour::findNarrowPassSpace(CollisionDetection& configurationSpace,
    const directionVector& radiusVectorToYuanxin,const directionVector& tangentVector,Node2D* startPoint, int whetherReverse)
{
  if(whetherReverse!=1 && whetherReverse!=0){
    std::cerr<<"whetherReverse should be 1 or 0"<<std::endl;
    return std::vector<Node3D>();
  }
  float radius=Constants::minRadius;
  float maxRadius = Constants::maxRadus;
  std::vector<Node3D> finalCirclePath;
  std::vector<Node3D> qulifiedNode3DList20Degree;
  std::vector<Node3D> qulifiedNode3DList30Degree;
  std::vector<Node3D> qulifiedNode3DList45Degree;

  //如果超过最大半径就尝试直线
  while (radius<=maxRadius)
  {
    float centerX = startPoint->getFloatX()+radiusVectorToYuanxin.x*radius;
    float centerY = startPoint->getFloatY()+radiusVectorToYuanxin.y*radius;

     
    Node2D* circleCenterPoint = new Node2D(centerX,centerY);
    //右手定则
    float cross=tangentVector.x*radiusVectorToYuanxin.y-tangentVector.y*radiusVectorToYuanxin.x;
    if(cross>0){//ni时针
      cross=1;
    }else cross=-1;

    float angleVehicleCurrent = atan2f(tangentVector.y,tangentVector.x);
    angleVehicleCurrent=Helper::normalizeHeadingRad(angleVehicleCurrent);
    float angleRelativeToCircleCurrent = 0;
    bool flag=true;
      
    finalCirclePath.clear();
    float currentRotatedAngel = 0;
    for(currentRotatedAngel=0;currentRotatedAngel<Constants::maxAngleRag;currentRotatedAngel+=Constants::deltaHeadingRad){
      
      angleRelativeToCircleCurrent = Helper::normalizeHeadingRad(angleVehicleCurrent +cross*M_PI/2 + M_PI )+ cross*Constants::deltaHeadingRad;//这里会默认切
      //上面需要+pi的原因是你需要找到指出圆心的向量的方向
      angleVehicleCurrent = Helper::normalizeHeadingRad(angleVehicleCurrent + cross*Constants::deltaHeadingRad);
      float pointX=circleCenterPoint->getFloatX()+radius*cos(angleRelativeToCircleCurrent);
      float pointY=circleCenterPoint->getFloatY()+radius*sin(angleRelativeToCircleCurrent);
      Node3D NodeCurrentVehicle =  Node3D(pointX, pointY, Helper::normalizeHeadingRad(angleVehicleCurrent+M_PI*whetherReverse));
      
      if(!configurationSpace.isTraversable(&NodeCurrentVehicle)){
        flag=false;
        break;
      }else{
        finalCirclePath.push_back(NodeCurrentVehicle);
      }
    }
    if(whetherDeepDebug){
      cv::Mat mapCopy;
      float multiplier = AlgorithmContour::visualizeMultiplier;
      cv::resize(gridMap, mapCopy, cv::Size(), multiplier, multiplier, cv::INTER_NEAREST);
      cv::cvtColor(mapCopy, mapCopy, cv::COLOR_GRAY2BGR);
      cv::circle(mapCopy, cv::Point(circleCenterPoint->getFloatX()*multiplier, circleCenterPoint->getFloatY()*multiplier), 3, cv::Scalar(255, 0, 0), -1);//蓝色是圆心
      cv::circle(mapCopy,cv::Point(startPoint->getFloatX()*multiplier,startPoint->getFloatY()*multiplier),3,cv::Scalar(0, 0, 255), -1);
      drawPoints(finalCirclePath, mapCopy, cv::Scalar(0, 255, 0),multiplier); // 绿色
      cv::imshow("Key Information Visualization", mapCopy);
      cv::waitKey(0);
    }
    if(flag){//代表我直接找到90度的圆
      return finalCirclePath;
    }else{
      radius+=Constants::deltaRadius;
      if(currentRotatedAngel > (1/9)*M_PI && qulifiedNode3DList20Degree.size()==0){
        qulifiedNode3DList20Degree = finalCirclePath;
      }
      if(currentRotatedAngel > 0.125*M_PI && qulifiedNode3DList30Degree.size()==0){
        qulifiedNode3DList30Degree = finalCirclePath;
      }
      if(currentRotatedAngel > 0.25*M_PI && qulifiedNode3DList45Degree.size()==0){
        qulifiedNode3DList45Degree = finalCirclePath;
      }
    }
  }
  if(finalCirclePath.size()==0){
    if(qulifiedNode3DList45Degree.size()!=0){
      return qulifiedNode3DList45Degree;
    }else if(qulifiedNode3DList30Degree.size()!=0){
      return qulifiedNode3DList30Degree;
    }else if(qulifiedNode3DList20Degree.size()!=0){
      return qulifiedNode3DList20Degree;
    }else{
      return finalCirclePath;
    }
  }else{//这里应该会是进行直线搜索
    return finalCirclePath;
  }
  // bool flag=true;
  // for(int x=0;x<Constants::straightLength&&flag;x+=(Constants::straightLength/100)){
  //     float pointX=startPoint->getFloatX()+x*centerVerticalUnitVector.x;
  //     float pointY=startPoint->getFloatY()+x*centerVerticalUnitVector.y;
  //     Node2D nNode =  Node2D(pointX, pointY);
  //     if(!configurationSpace.isTraversable(&nNode)){
  //       flag=false;
  //       break;
  //     }
  // }
  // if(flag){
  //   return 0;
  // }
  
  // return -Constants::minRadius/2;
}
//找到左上左下右上右下圆边界路径
void AlgorithmContour::findNarrowPassSpaceForAllPairs(CollisionDetection &configurationSpace)
{
  for (keyInfoForThrouthNarrowPair* KIpair:keyInfoForThrouthNarrowPairs ) {
    KIpair->containingWaypointsFirstBPBackward=findNarrowPassSpace(configurationSpace,
        KIpair->wireUnitVector.getReverseVector(),KIpair->centerVerticalUnitVector.getReverseVector(),KIpair->firstBoundPoint,1);
    // KIpair->containingWaypointsFirstBPForward=findNarrowPassSpace(configurationSpace,
    //     KIpair->wireUnitVector.getReverseVector(),KIpair->centerVerticalUnitVector,KIpair->firstBoundPoint,0);
    KIpair->containingWaypointsSecondBPBackward=findNarrowPassSpace(configurationSpace,
        KIpair->wireUnitVector,KIpair->centerVerticalUnitVector.getReverseVector(),KIpair->secondBoundPoint,1);
    // KIpair->containingWaypointsSecondBPForward=findNarrowPassSpace(configurationSpace,
    //     KIpair->wireUnitVector,KIpair->centerVerticalUnitVector,KIpair->secondBoundPoint,0);
  }
}



// 主要的可视化函数
void AlgorithmContour::visualizePassSpaceBoundaryForThroughNarrowPair(keyInfoForThrouthNarrowPair* keyInfo, const cv::Mat& gridMap) {
    cv::Mat mapCopy;
    float multiplier = AlgorithmContour::visualizeMultiplier;
    cv::resize(gridMap, mapCopy, cv::Size(), multiplier, multiplier, cv::INTER_NEAREST);
    cv::cvtColor(mapCopy, mapCopy, cv::COLOR_GRAY2BGR);

    // 绘制所有路径点
    // drawPoints(keyInfo->containingWaypointsFirstBPForward, mapCopy, cv::Scalar(255, 0, 0),multiplier); // 红色
    // cv::imshow("Pass Space Boundary Visualization", mapCopy); // 显示图像
    // cv::waitKey(0); // 等待按键
    drawPoints(keyInfo->containingWaypointsFirstBPBackward, mapCopy, cv::Scalar(0, 255, 0),multiplier); // 绿色
    cv::imshow("Pass Space Boundary Visualization", mapCopy); // 显示图像
    cv::waitKey(0); // 等待按键
    // drawPoints(keyInfo->containingWaypointsSecondBPForward, mapCopy, cv::Scalar(0, 0, 255),multiplier); // 蓝色
    // cv::imshow("Pass Space Boundary Visualization", mapCopy); // 显示图像
    // cv::waitKey(0); // 等待按键
    drawPoints(keyInfo->containingWaypointsSecondBPBackward, mapCopy, cv::Scalar(255, 255, 0),multiplier); // 黄色
    cv::imshow("Pass Space Boundary Visualization", mapCopy); // 显示图像
    cv::waitKey(0); // 等待按键
}


finalPassSpaceInOutSet AlgorithmContour::findNarrowPassSpaceInputSetOfNode3D(CollisionDetection& configurationSpace,keyInfoForThrouthNarrowPair* inputPair){
  uint minLength = min(inputPair->containingWaypointsFirstBPBackward.size(), inputPair->containingWaypointsSecondBPBackward.size());
  finalPassSpaceInOutSet resultSet;
  uint size1 = inputPair->containingWaypointsFirstBPBackward.size();
  uint size2 = inputPair->containingWaypointsSecondBPBackward.size();
  for(uint i = 0; i < minLength; i++){
    Node3D* nodeFirst = &inputPair->containingWaypointsFirstBPBackward[size1-i-1];
    Node3D* nodeSecond = &inputPair->containingWaypointsSecondBPBackward[size2-i-1];
    std::vector<Node3D> resultVector = findInputSetOfNode3DByTwoVectorAndMiddleVerticalLine(*nodeFirst,*nodeSecond,*inputPair->centerPoint,inputPair->centerVerticalUnitVector);
    bool flag = true;
    for(auto node:resultVector){
      if(!configurationSpace.isTraversable(&node)){
        flag = false;
        break;
      }
    }
    if(flag){
      resultSet.inSet = resultVector;
      break;
    }
  }
  // int minLength2 = min(inputPair->containingWaypointsFirstBPForward.size(), inputPair->containingWaypointsSecondBPForward.size());
  // size1 = inputPair->containingWaypointsFirstBPForward.size();
  // size2 = inputPair->containingWaypointsSecondBPForward.size();
  // for(int i = 0; i < minLength2; i++){
  //   Node3D* nodeFirst = &inputPair->containingWaypointsFirstBPForward[size1-i-1];
  //   Node3D* nodeSecond = &inputPair->containingWaypointsSecondBPForward[size2-i-1];
  //   std::vector<Node3D> resultVector = findInputSetOfNode3DByTwoVectorAndMiddleVerticalLine(*nodeFirst,*nodeSecond,*inputPair->centerPoint,inputPair->centerVerticalUnitVector);
  //   bool flag = true;
  //   for(auto node:resultVector){
  //     if(!configurationSpace.isTraversable(&node)){
  //       flag = false;
  //       break;
  //     }
  //   }
  //   if(flag){
  //     resultSet.outSet = resultVector;
  //     break;
  //   }
  // }
  // if(resultSet.inSet.size()==0 || resultSet.outSet.size()==0){
  //    std::cerr << "error in function " << __func__ << " at line " << __LINE__ << ": can not find input set of node3D" << std::endl;
  // }
  return resultSet;
}

//从起始点到中点的3D插值
static inline std::vector<Node3D> interpolatePath(Node3D start, Node3D end, float gridSize) {
    std::vector<Node3D> path;

    float distance = std::sqrt(std::pow(end.getX() - start.getX(), 2) + std::pow(end.getY() - start.getY(), 2));
    int numberOfPoints = std::max(1, static_cast<int>(distance / gridSize * 1.5));
    float algelGap = end.getT() - start.getT(); 
    if (algelGap > M_PI) {
        algelGap -= 2 * M_PI;
    } else if (algelGap < -M_PI) {
        algelGap += 2 * M_PI;
    }
    for (int i = 0; i <= numberOfPoints; ++i) {
        float lerpFactor = static_cast<float>(i) / numberOfPoints;
        Node3D node;
        node.setX(start.getX() + lerpFactor * (end.getX() - start.getX()));
        node.setY(start.getY() + lerpFactor * (end.getY() - start.getY()));
        node.setT(Helper::normalizeHeadingRad(start.getT() + lerpFactor * algelGap));
        path.push_back(node);
    }

    return path;
}

std::vector<Node3D> AlgorithmContour::findInputSetOfNode3DByTwoVectorAndMiddleVerticalLine(const Node3D & firstPoint,const Node3D & secondPoint,const Node2D & middlePoint,const directionVector middleVerticalLine){
   Node3D intersection;

    // 创建线段
    Segment_2 segment(Point_2(firstPoint.getX(), firstPoint.getY()), Point_2(secondPoint.getX(), secondPoint.getY()));

    // 计算中垂线方程
    Line_2 middleLine = Line_2(Point_2(middlePoint.getX(), middlePoint.getY()), Point_2(middlePoint.getX() + middleVerticalLine.x, middlePoint.getY() + middleVerticalLine.y));

    // 计算交点
    auto result = CGAL::intersection(segment, middleLine);

    if (const Point_2* p = boost::get<Point_2>(&*result)) { //这里的交点可能是线段
      intersection.setX(static_cast<float>(CGAL::to_double(p->x())));
      intersection.setY(static_cast<float>(CGAL::to_double(p->y())));
    }
    float angelOfIntersection = atan2f(middleVerticalLine.y,middleVerticalLine.x);
    intersection.setT(Helper::normalizeHeadingRad(angelOfIntersection));
    if(this->whetherDeepDebug2){
      cv::Mat mapCopy;
      cv::cvtColor(this->gridMap, mapCopy, cv::COLOR_GRAY2BGR);
      float arrowLength = 10;
      auto drawArrow = [&](const Node3D &startNode3D) {
        directionVector direction = directionVector::getUnitVectorFromNode3D(&startNode3D);
        cv::Point start(startNode3D.getX(), startNode3D.getY());
        cv::Point end(start.x + direction.x * arrowLength, start.y + direction.y * arrowLength);
        cv::arrowedLine(mapCopy, start, end, cv::Scalar(0, 255, 0), 1, 8, 0, 0.3);
      };
      drawArrow(firstPoint);
      drawArrow(secondPoint);
      drawArrow(intersection);
      cv::circle(mapCopy, cv::Point(middlePoint.getX(), middlePoint.getY()), 2, cv::Scalar(0, 0, 255), -1);
      cv::imshow("visual the middle point", mapCopy); // 显示图像
      cv::waitKey(0); // 等待按键
    }
    std::vector<Node3D> resultVector = interpolatePath(firstPoint, intersection, 1.0);
    // 计算从 Intersection 到 SecondPoint 的路径
    std::vector<Node3D> secondPath = interpolatePath(intersection,secondPoint, 1.0);
    // 将两个路径合并
    resultVector.insert(resultVector.end(), secondPath.begin(), secondPath.end());
    
    if(this->whetherDeepDebug2){
      cv::Mat mapCopy;
      int multiplier = AlgorithmContour::visualizeMultiplier;
      cv::resize(gridMap, mapCopy, cv::Size(), multiplier, multiplier, cv::INTER_NEAREST);
      cv::cvtColor(mapCopy, mapCopy, cv::COLOR_GRAY2BGR);
      for (const auto& node : resultVector) {
          // 节点的放大位置
          cv::Point2f nodePos(node.getX() * multiplier, node.getY() * multiplier);

          // 计算方向向量的终点
          float length = 10; // 可以根据需要调整长度
          cv::Point2f direction(cos(node.getT()) * length, sin(node.getT()) * length);
          cv::Point2f endPoint = nodePos + direction;

          // 绘制线段
          cv::arrowedLine(mapCopy, nodePos, endPoint, cv::Scalar(0, 0, 255), 1,8,0,0.3);
      }
      cv::namedWindow("Pass Space Boundary Visualization", cv::WINDOW_NORMAL);
      cv::imshow("Pass Space Boundary Visualization", mapCopy); // 显示图像
      cv::waitKey(0); // 等待按键
    }
    return resultVector;

}



void AlgorithmContour::findNarrowPassSpaceInputSetOfNode3DForAllPairs(CollisionDetection& configurationSpace){
  for (keyInfoForThrouthNarrowPair* KIpair:keyInfoForThrouthNarrowPairs ) {
    this->finalPassSpaceInOutSets.push_back(findNarrowPassSpaceInputSetOfNode3D(configurationSpace,KIpair));
  }
}

// std::vector<Node3D> AlgorithmContour::findArcByTwoPoints(const Node3D & firstPoint,const Node2D& middlePoint, const directionVector middleVerticalLine){
//     Point_2 cgFirstPoint(firstPoint.getX(), firstPoint.getY());
//     Point_2 cgMiddlePoint(middlePoint.getX(), middlePoint.getY());
//     Vector_2 firstPointDirection(std::cos(firstPoint.getT() * M_PI / 180.0), std::sin(firstPoint.getT() * M_PI / 180.0));
//     Vector_2 middlePointDirection(middleVerticalLine.x, middleVerticalLine.y);

//     // Create lines
//     Line_2 firstLine(cgFirstPoint, firstPointDirection.perpendicular(CGAL::CLOCKWISE));
//     Line_2 middleLine(cgMiddlePoint, middlePointDirection.perpendicular(CGAL::CLOCKWISE));

//     // Compute intersection (the arc center)
//     CGAL::Object result = CGAL::intersection(firstLine, middleLine);

//     if (const Point_2 *arcCenter = CGAL::object_cast<Point_2>(&result)) {
//         // Convert the intersection point back to Node3D
//         Node3D center;
//     }
// }

// void AlgorithmContour::findContaingfFeasibleToCenterVerticalPoint(const Node3D & startPoint,CollisionDetection& configurationSpace){

// }

