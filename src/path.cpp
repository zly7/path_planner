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
void Path::updatePath(const std::vector<Node3D>& nodePath,int k) {
  path.header.stamp = ros::Time::now();
  // int k = 0;

  for (size_t i = 0; i < nodePath.size(); ++i) {
    std::cout << "add path " << i << " " << nodePath[i].getX() << " " << nodePath[i].getY() << std::endl;
    addSegment(nodePath[i]);
    addNode(nodePath[i], k);
    k++;
    addVehicle(nodePath[i], k);
    k++;
  }

  return;
}

void Path::update2DPath(const std::vector<Node2D>& nodePath) {
  path.header.stamp = ros::Time::now();
  int k = 0;

  for (size_t i = 0; i < nodePath.size(); ++i) {
    std::cout << "add 2D path " << i << " " << nodePath[i].getX() << " " << nodePath[i].getY() << std::endl;
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
// ADD NODE
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
  pathNode.scale.x = 0.1;
  pathNode.scale.y = 0.1;
  pathNode.scale.z = 0.1;
  pathNode.color.a = 1.0;

  if (smoothed) {
    pathNode.color.r = Constants::pink.red;
    pathNode.color.g = Constants::pink.green;
    pathNode.color.b = Constants::pink.blue;
  } else {
    pathNode.color.r = Constants::purple.red;
    pathNode.color.g = Constants::purple.green;
    pathNode.color.b = Constants::purple.blue;
  }

  pathNode.pose.position.x = node.getX() * Constants::cellSize;
  pathNode.pose.position.y = node.getY() * Constants::cellSize;
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

  if (smoothed) {
    pathNode.color.r = Constants::black.red;
    pathNode.color.g = Constants::black.green;
    pathNode.color.b = Constants::black.blue;
  } else {
    pathNode.color.r = Constants::black.red;
    pathNode.color.g = Constants::black.green;
    pathNode.color.b = Constants::black.blue;
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

  std::cout<<"  Box   " << node.getLeft() << " " << node.getRight() << " " << node.getUp() << " " << node.getDown() <<std::endl;
  pathBox.header.frame_id = "path";
  pathBox.header.stamp = ros::Time(0);
  pathBox.id = i;
  pathBox.type = visualization_msgs::Marker::LINE_LIST;
  pathBox.scale.x = 0.1;
  pathBox.color.a = 1.0;
  // Define the points for the rectangle's corners (assuming a 2D rectangle)
  geometry_msgs::Point point1, point2, point3, point4;
  point1.x = node.getIntX()-node.getLeft();  // X-coordinate of the first corner
  point1.y = node.getIntY()+node.getUp();  // Y-coordinate of the first corner
  point1.z = 0.0;  // Z-coordinate of the first corner (set to 0 for 2D)

  point2.x = node.getIntX()-node.getLeft();  // X-coordinate of the second corner
  point2.y = node.getIntY()-node.getDown();  // Y-coordinate of the second corner
  point2.z = 0.0;  // Z-coordinate of the second corner

  point3.x = node.getIntX()+node.getRight();  // X-coordinate of the third corner
  point3.y = node.getIntY()-node.getDown();  // Y-coordinate of the third corner
  point3.z = 0.0;  // Z-coordinate of the third corner

  point4.x = node.getIntX()+node.getRight();  // X-coordinate of the fourth corner
  point4.y = node.getIntY()+node.getUp();  // Y-coordinate of the fourth corner
  point4.z = 0.0;  // Z-coordinate of the fourth corner

  // Add the points to the marker
  pathBox.points.push_back(point1);
  pathBox.points.push_back(point2);
  pathBox.points.push_back(point2);
  pathBox.points.push_back(point3);
  pathBox.points.push_back(point3);
  pathBox.points.push_back(point4);
  pathBox.points.push_back(point4);
  pathBox.points.push_back(point1);

  std::cout << "scale " << pathBox.scale.x << " " << pathBox.scale.y << std::endl;

  if (node.getWide()) {
    pathBox.color.r = Constants::red.red;
    pathBox.color.g = Constants::red.green;
    pathBox.color.b = Constants::red.blue;
  } else {
    pathBox.color.r = Constants::green.red;
    pathBox.color.g = Constants::green.green;
    pathBox.color.b = Constants::green.blue;
  }

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
}
