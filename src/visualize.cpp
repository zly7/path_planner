#include "visualize.h"
using namespace HybridAStar;
//###################################################
//                                CLEAR VISUALIZATION
//###################################################
void Visualize::clear() {
  poses3D.poses.clear();
  poses3Dreverse.poses.clear();
  poses2D.poses.clear();

  // 3D COSTS
  visualization_msgs::MarkerArray costCubes3D;
  visualization_msgs::Marker costCube3D;
  // CLEAR THE COST HEATMAP
  costCube3D.header.frame_id = "path";
  costCube3D.header.stamp = ros::Time::now();
  costCube3D.id = 0;
  costCube3D.action = 3;
  costCubes3D.markers.push_back(costCube3D);
  pubNodes3DCosts.publish(costCubes3D);

  // 2D COSTS
  visualization_msgs::MarkerArray costCubes2D;
  visualization_msgs::Marker costCube2D;
  // CLEAR THE COST HEATMAP
  costCube2D.header.frame_id = "path";
  costCube2D.header.stamp = ros::Time::now();
  costCube2D.id = 0;
  costCube2D.action = 3;
  costCubes2D.markers.push_back(costCube2D);
  pubNodes2DCosts.publish(costCubes2D);
}

//###################################################
//                                    CURRENT 3D NODE  这一块暂时没太看懂2个函数的区别在哪里
//###################################################
void Visualize::publishNode3DPose(Node3D& node) {
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "path";
  pose.header.stamp = ros::Time::now();
  pose.header.seq = 0;
  pose.pose.position.x = node.getX() * Constants::cellSize;
  pose.pose.position.y = node.getY() * Constants::cellSize;

  //FORWARD
  if (node.getPrim() < 3) {
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(node.getT());
  }
  //REVERSE
  else {
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(node.getT() + M_PI);
  }

  // PUBLISH THE POSE
  pubNode3D.publish(pose);
}

//###################################################
//                              ALL EXPANDED 3D NODES
//###################################################
void Visualize::publishNode3DPoses(Node3D& node) {
  geometry_msgs::Pose pose;
  pose.position.x = node.getX() * Constants::cellSize;
  pose.position.y = node.getY() * Constants::cellSize;

  //FORWARD
  if (node.getPrim() < 3) {
    pose.orientation = tf::createQuaternionMsgFromYaw(node.getT());
    poses3D.poses.push_back(pose);
    poses3D.header.stamp = ros::Time::now();
    // PUBLISH THE POSEARRAY
    pubNodes3D.publish(poses3D);
  }
  //REVERSE
  else {
    pose.orientation = tf::createQuaternionMsgFromYaw(node.getT() + M_PI);
    poses3Dreverse.poses.push_back(pose);
    poses3Dreverse.header.stamp = ros::Time::now();
    // PUBLISH THE POSEARRAY
    pubNodes3Dreverse.publish(poses3Dreverse);
  }

}

//###################################################
//                                    CURRENT 2D NODE
//###################################################
void Visualize::publishNode2DPose(Node2D& node) {
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "path";
  pose.header.stamp = ros::Time::now();
  pose.header.seq = 0;
  pose.pose.position.x = (node.getX() + 0.5) * Constants::cellSize;
  pose.pose.position.y = (node.getY() + 0.5) * Constants::cellSize;
  pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

  // PUBLISH THE POSE
  pubNode2D.publish(pose);
}

//###################################################
//                              ALL EXPANDED 2D NODES
//###################################################
void Visualize::publishNode2DPoses(Node2D& node) {
  if (node.isDiscovered() || Constants::algorithm=="rrt") {
    geometry_msgs::Pose pose;
    pose.position.x = (node.getX() + 0.5) * Constants::cellSize;
    pose.position.y = (node.getY() + 0.5) * Constants::cellSize;
    pose.orientation = tf::createQuaternionMsgFromYaw(0);

    poses2D.poses.push_back(pose);
    poses2D.header.stamp = ros::Time::now();
    // PUBLISH THE POSEARRAY
    pubNodes2D.publish(poses2D);

  }
}
void Visualize::publishNode3DStartAndGoal(Node3D& start, Node3D& goal){
  visualization_msgs::Marker marker;
  marker.header.frame_id = "path";
  marker.header.stamp = ros::Time::now();
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;

  // 设置起点箭头
  marker.pose.position.x = start.getX() * Constants::cellSize;
  marker.pose.position.y = start.getY() * Constants::cellSize;
  marker.pose.position.z = 0;
  marker.pose.orientation = tf::createQuaternionMsgFromYaw(start.getT());
  marker.scale.x = 2;  // 箭头长度
  marker.scale.y = 1; // 箭头宽度
  marker.scale.z = 0; // 箭头高度
  marker.color.a = 0.6; 
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0; // 蓝色表示起点
  pubNode3DStartAndGoal.publish(marker);

  // 设置终点箭头
  marker.id = 1; // 改变ID以区分不同的标记
  marker.pose.position.x = goal.getX() * Constants::cellSize;
  marker.pose.position.y = goal.getY() * Constants::cellSize;
  marker.pose.position.z = 0;
  marker.pose.orientation = tf::createQuaternionMsgFromYaw(goal.getT());
  marker.scale.x = 2;  // 箭头长度
  marker.scale.y = 1; // 箭头宽度
  marker.scale.z = 0; // 箭头高度
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0; // 红色表示终点
  pubNode3DStartAndGoal.publish(marker);
}

//###################################################
//                                    COST HEATMAP 3D
//###################################################
void Visualize::publishNode3DCosts(Node3D* nodes, int width, int height, int depth) {
  visualization_msgs::MarkerArray costCubes;
  visualization_msgs::Marker costCube;

  float min = 1000;
  float max = 0;
  int idx;
  bool once = true;
  float red = 0;
  float green = 0;
  float blue = 0;
  int count = 0;

  ColorGradient heatMapGradient;
  heatMapGradient.createDefaultHeatMapGradient();

  float values[width * height];

  // ________________________________
  // DETERMINE THE MAX AND MIN VALUES
  for (int i = 0; i < width * height; ++i) {
    values[i] = 1000;

    // iterate over all headings
    for (int k = 0; k < depth; ++k) {
      idx = k * width * height + i;

      // set the minimum for the cell
      if (nodes[idx].isClosed() || nodes[idx].isOpen()) {
        values[i] = nodes[idx].getC();
      }
    }

    // set a new minimum
    if (values[i] > 0 && values[i] < min) {
      min = values[i];
    }

    // set a new maximum
    if (values[i] > 0 && values[i] > max && values[i] != 1000) {
      max = values[i];
    }
  }

  // _______________
  // PAINT THE CUBES
  for (int i = 0; i < width * height; ++i) {
    // if a value exists continue
    if (values[i] != 1000) {
      count++;

      // delete all previous markers
      if (once) {
        costCube.action = 3;
        once = false;
      } else {
        costCube.action = 0;
      }


      costCube.header.frame_id = "path";
      costCube.header.stamp = ros::Time::now();
      costCube.id = i;
      costCube.type = visualization_msgs::Marker::CUBE;
      values[i] = (values[i] - min) / (max - min);
      costCube.scale.x = Constants::cellSize;
      costCube.scale.y = Constants::cellSize;
      costCube.scale.z = 0.1;
      costCube.color.a = 0.5;
      heatMapGradient.getColorAtValue(values[i], red, green, blue);
      costCube.color.r = red;
      costCube.color.g = green;
      costCube.color.b = blue;
      // center in cell +0.5
      costCube.pose.position.x = (i % width + 0.5) * Constants::cellSize;
      costCube.pose.position.y = ((i / width) % height + 0.5) * Constants::cellSize;
      costCubes.markers.push_back(costCube);
    }
  }

  if (Constants::coutDEBUG) {
    std::cout << "3D min cost: " << min << " | max cost: " << max << std::endl;
    std::cout << count << " 3D nodes expanded " << std::endl;
  }

  // PUBLISH THE COSTCUBES
  pubNodes3DCosts.publish(costCubes);
}

//###################################################
//                                    COST HEATMAP 2D
//###################################################
void Visualize::publishNode2DCosts(Node2D* nodes, int width, int height) {
  visualization_msgs::MarkerArray costCubes;
  visualization_msgs::Marker costCube;

  float min = 1000;
  float max = 0;
  bool once = true;
  float red = 0;
  float green = 0;
  float blue = 0;
  int count = 0;

  ColorGradient heatMapGradient;
  heatMapGradient.createDefaultHeatMapGradient();

  float values[width * height];

  // ________________________________
  // DETERMINE THE MAX AND MIN VALUES
  for (int i = 0; i < width * height; ++i) {
    values[i] = 1000;

    // set the minimum for the cell
    if (nodes[i].isDiscovered()) {
      values[i] = nodes[i].getG();

      // set a new minimum
      if (values[i] > 0 && values[i] < min) { min = values[i]; }

      // set a new maximum
      if (values[i] > 0 && values[i] > max) { max = values[i]; }
    }
  }

  // _______________
  // PAINT THE CUBES
  for (int i = 0; i < width * height; ++i) {
    // if a value exists continue
    if (nodes[i].isDiscovered()) {
      count++;

      // delete all previous markers
      if (once) {
        costCube.action = 3;
        once = false;
      } else {
        costCube.action = 0;
      }


      costCube.header.frame_id = "path";
      costCube.header.stamp = ros::Time::now();
      costCube.id = i;
      costCube.type = visualization_msgs::Marker::CUBE;
      values[i] = (values[i] - min) / (max - min);
      costCube.scale.x = Constants::cellSize;
      costCube.scale.y = Constants::cellSize;
      costCube.scale.z = 0.1;
      costCube.color.a = 0.5;
      heatMapGradient.getColorAtValue(values[i], red, green, blue);
      costCube.color.r = red;
      costCube.color.g = green;
      costCube.color.b = blue;
      // center in cell +0.5
      costCube.pose.position.x = (i % width + 0.5) * Constants::cellSize;
      costCube.pose.position.y = ((i / width) % height + 0.5) * Constants::cellSize;
      costCubes.markers.push_back(costCube);
    }
  }

  if (Constants::coutDEBUG) {
    std::cout << "2D min cost: " << min << " | max cost: " << max << std::endl;
    std::cout << count << " 2D nodes expanded " << std::endl;
  }

  // PUBLISH THE COSTCUBES
  pubNodes2DCosts.publish(costCubes);
}

