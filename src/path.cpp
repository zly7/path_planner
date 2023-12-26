#include "path.h"

using namespace HybridAStar;


//###################################################
//                                         CLEAR PATH
//###################################################

void Path::clear() {
  Node3D node;
  path.poses.clear();
  pathNodes.markers.clear();
  pathVehicles.markers.clear();
  path2DNodes.markers.clear();
  pathBoxes.markers.clear();
  addNode(node, 0);
  addVehicle(node, 1);
  publishPath();
  publishPathNodes();
  publishPathVehicles();
  publishPath2DNodes();
  publishPathBoxes();
}

////###################################################
////                                         TRACE PATH
////###################################################
//// __________
//// TRACE PATH
//void Path::tracePath(const Node3D* node, int i) {
//  if (i == 0) {
//    path.header.stamp = ros::Time::now();
//  }

//  if (node == nullptr) { return; }

//  addSegment(node);
//  addNode(node, i);
//  i++;
//  addVehicle(node, i);
//  i++;

//  tracePath(node->getPred(), i);
//}

//###################################################
//                                         TRACE PATH
//###################################################
// __________
// TRACE PATH
void Path::updatePathFromK(const std::vector<Node3D>& nodePath,int k) {
  path.header.stamp = ros::Time::now();
  for (size_t i = 0; i < nodePath.size(); ++i) {
    // std::cout << "add path " << i << " " << nodePath[i].getX() << " " << nodePath[i].getY() << std::endl;
    addSegment(nodePath[i]);
    addNode(nodePath[i], k);
    k++;
    addVehicle(nodePath[i], k);
    k++;
  }
  return;
}

void Path::updatePath(const std::vector<Node3D>& nodePath) {
  path.header.stamp = ros::Time::now();
  int k = 0;
  for (size_t i = 0; i < nodePath.size(); ++i) {
    // std::cout << "add path " << i << " " << nodePath[i].getX() << " " << nodePath[i].getY() << std::endl;
    addSegment(nodePath[i]);
    addNode(nodePath[i], k);
    k++;
    addVehicle(nodePath[i], k);
    k++;
  }
  return;
}
void Path::tempUpdatePathNode(const std::vector<Node3D>& nodePath) {
  path.header.stamp = ros::Time::now();
  int k = 0;
  for (size_t i = 0; i < nodePath.size(); ++i) {
    addNode(nodePath[i], k);
    k++;
  }
  return;
}
void Path::update2DPath(const std::vector<Node2D>& nodePath) {
  path.header.stamp = ros::Time::now();
  int k = 0;

  for (size_t i = 0; i < nodePath.size(); ++i) {
    // std::cout << "add 2D path " << i << " " << nodePath[i].getX() << " " << nodePath[i].getY() << std::endl;
    add2DNode(nodePath[i], k);
    k++;
    add2DBox(nodePath[i],k);
    k++;
  }

  return;
}
// ___________
// ADD SEGMENT
void Path::addSegment(const Node3D& node) {
  geometry_msgs::PoseStamped vertex;
  vertex.pose.position.x = node.getX() * Constants::cellSize;
  vertex.pose.position.y = node.getY() * Constants::cellSize;
  vertex.pose.position.z = 0;
  vertex.pose.orientation.x = 0;
  vertex.pose.orientation.y = 0;
  vertex.pose.orientation.z = 0;
  vertex.pose.orientation.w = 0;
  path.poses.push_back(vertex);
}

// ________
// ADD NODE 添加最后的路径点
void Path::addNode(const Node3D& node, int i) {
  visualization_msgs::Marker pathNode;

  // delete all previous markers
  if (i == 0) {
    pathNode.action = 3;
  }

  pathNode.header.frame_id = "path";
  pathNode.header.stamp = ros::Time(0);
  pathNode.id = i;
  pathNode.type = visualization_msgs::Marker::SPHERE;
  pathNode.scale.x = 1;
  pathNode.scale.y = 1;
  pathNode.scale.z = 1;
  pathNode.color.a = 1.0;

  if (smoothed) {
    pathNode.color.r = Constants::purple.red;
    pathNode.color.g = Constants::purple.green;
    pathNode.color.b = Constants::purple.blue;
  } else {
    pathNode.color.r = Constants::pink.red;
    pathNode.color.g = Constants::pink.green;
    pathNode.color.b = Constants::pink.blue;
  }

  pathNode.pose.position.x = node.getX() * Constants::cellSize;
  pathNode.pose.position.y = node.getY() * Constants::cellSize;
  pathNode.pose.position.z = node.getT(); // 这里因为地图是2D的
  pathNodes.markers.push_back(pathNode);
}

void Path::add2DNode(const Node2D& node, int i) {
  visualization_msgs::Marker pathNode;

  // delete all previous markers
  if (i == 0) {
    pathNode.action = 3;
  }

  pathNode.header.frame_id = "path";
  pathNode.header.stamp = ros::Time(0);
  pathNode.id = i;
  pathNode.type = visualization_msgs::Marker::SPHERE;
  pathNode.action = visualization_msgs::Marker::ADD;
  pathNode.scale.x = 0.5;
  pathNode.scale.y = 0.5;
  pathNode.scale.z = 0.5;
  pathNode.color.a = 1.0;

  if (node.getBoundary()) {
    pathNode.color.r = Constants::black.red;
    pathNode.color.g = Constants::black.green;
    pathNode.color.b = Constants::black.blue;
    pathNode.scale.x = 1.5;
    pathNode.scale.y = 1.5;
    pathNode.scale.z = 1.5;
  } else {
    pathNode.color.r = Constants::black.red;
    pathNode.color.g = Constants::black.green;
    pathNode.color.b = Constants::black.blue;
    pathNode.scale.x = 1.5;
    pathNode.scale.y = 1.5;
    pathNode.scale.z = 1.5;
  }

  pathNode.pose.position.x = node.getX() * Constants::cellSize;
  pathNode.pose.position.y = node.getY() * Constants::cellSize;
  pathNode.pose.position.z = 0;
  pathNode.pose.orientation.x = 0;
  pathNode.pose.orientation.y = 0;
  pathNode.pose.orientation.z = 0;
  pathNode.pose.orientation.w = 1;
  path2DNodes.markers.push_back(pathNode);
}

void Path::add2DBox(const Node2D& node, int i){
  visualization_msgs::Marker pathBox;

  // delete all previous markersg
  if (i == 1) {
    pathBox.action = 3;
  }

  // std::cout<<"  Box   " << node.getRadius() << " " << node.getX() << " " << node.getY() <<std::endl;
  pathBox.header.frame_id = "path";
  pathBox.header.stamp = ros::Time(0);
  pathBox.id = i;

  pathBox.type = visualization_msgs::Marker::CYLINDER;
  pathBox.action = visualization_msgs::Marker::ADD;

  pathBox.pose.position.x = node.getX() * Constants::cellSize;
  pathBox.pose.position.y = node.getY() * Constants::cellSize;
  pathBox.pose.position.z = 0;
  pathBox.pose.orientation.x = 0.0;
  pathBox.pose.orientation.y = 0.0;
  pathBox.pose.orientation.z = 0.0;
  pathBox.pose.orientation.w = 1.0;
  pathBox.scale.x = 2*node.getRadius();  // Diameter of the circle
  pathBox.scale.y = 2*node.getRadius();  // Diameter of the circle
  pathBox.scale.z = 0.0001;  // Height of the cylinder (thickness of the circle)

  // std::cout << "scale " << pathBox.scale.x << " " << pathBox.scale.y << std::endl;

  if (node.getWide()) {
    pathBox.color.r = Constants::red.red;
    pathBox.color.g = Constants::red.green;
    pathBox.color.b = Constants::red.blue;
  } else {
    pathBox.color.r = Constants::blue.red;
    pathBox.color.g = Constants::blue.green;
    pathBox.color.b = Constants::blue.blue;
  }

  pathBox.color.a = 0.05;

  pathBoxes.markers.push_back(pathBox);
}


void Path::addVehicle(const Node3D& node, int i) {
  visualization_msgs::Marker pathVehicle;

  // delete all previous markersg
  if (i == 1) {
    pathVehicle.action = 3;
  }

  pathVehicle.header.frame_id = "path";
  pathVehicle.header.stamp = ros::Time(0);
  pathVehicle.id = i;
  pathVehicle.type = visualization_msgs::Marker::CUBE;
  pathVehicle.scale.x = Constants::length - Constants::bloating * 2;
  pathVehicle.scale.y = Constants::width - Constants::bloating * 2;
  pathVehicle.scale.z = 1;
  pathVehicle.color.a = 0.1;

  if (smoothed) {
    pathVehicle.color.r = Constants::orange.red;
    pathVehicle.color.g = Constants::orange.green;
    pathVehicle.color.b = Constants::orange.blue;
  } else {
    pathVehicle.color.r = Constants::teal.red;
    pathVehicle.color.g = Constants::teal.green;
    pathVehicle.color.b = Constants::teal.blue;
  }

  pathVehicle.pose.position.x = node.getX() * Constants::cellSize;
  pathVehicle.pose.position.y = node.getY() * Constants::cellSize;
  pathVehicle.pose.orientation = tf::createQuaternionMsgFromYaw(node.getT());
  pathVehicles.markers.push_back(pathVehicle);

  visualization_msgs::Marker border;
  border.header.frame_id = "path";
  border.header.stamp = ros::Time(0);
  border.ns = "border";
  border.id = i+2000;//to make the id not the same as the vehicle
  border.type = visualization_msgs::Marker::LINE_LIST;
  border.scale.x = 0.2; // 线条的厚度
  border.color.r = static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX);
  border.color.g = static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX);
  border.color.b = static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX);
  border.color.a = 0.5; 

  // 计算矩形边框的四个顶点
  geometry_msgs::Point p1, p2, p3, p4;
  double length = Constants::length;
  double width = Constants::width;
  p1.x = length / 2;   // 前端，中心右侧
  p1.y = -width / 2;   // 中心下方

  p2.x = length / 2;   // 前端，中心右侧
  p2.y = width / 2;    // 中心上方

  p3.x = -length / 2;  // 后端，中心左侧
  p3.y = width / 2;    // 中心上方

  p4.x = -length / 2;  // 后端，中心左侧
  p4.y = -width / 2;   // 中心下方
 // 假设你已经有了车辆的朝向角 theta
  double theta = node.getT(); // 获取车辆的朝向

  // 计算旋转矩阵
  double cosTheta = cos(theta);
  double sinTheta = sin(theta);

  // 对每个顶点应用旋转
  for (auto& p : {&p1, &p2, &p3, &p4}) {
      double x = p->x;
      double y = p->y;

      p->x = cosTheta * x - sinTheta * y + node.getX();
      p->y = sinTheta * x + cosTheta * y + node.getY();
  }

  // 添加顶点到Marker
  border.points.push_back(p1);
  border.points.push_back(p2);
  border.points.push_back(p2);
  border.points.push_back(p3);
  border.points.push_back(p3);
  border.points.push_back(p4);
  border.points.push_back(p4);
  border.points.push_back(p1); // 闭合矩形

  // 将边框添加到MarkerArray
  pathVehicles.markers.push_back(border);

}
