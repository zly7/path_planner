#ifndef PATH_H
#define PATH_H

#include <iostream>
#include <cstring>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

#include "node3d.h"
#include "node2d.h"
#include "constants.h"
#include "helper.h"
namespace HybridAStar {
/*!
   \brief A class for tracing and visualizing the path generated by the Planner
*/
class Path {
 public:
  /// The default constructor initializing the path object and setting publishers for the same.
  Path(bool smoothed = false) {
    std::string pathTopic = "/path";
    std::string pathNodesTopic = "/pathNodes";
    std::string pathVehicleTopic = "/pathVehicle";
    std::string path2DNodesTopic = "/path2DNodes";
    std::string pathBoxesTopic = "/pathBoxes";
   
    if (smoothed) {
      pathTopic = "/sPath";
      pathNodesTopic = "/sPathNodes";
      pathVehicleTopic = "/sPathVehicle";
      path2DNodesTopic = "/sPath2DNodes";
      pathBoxesTopic = "/sPathBoxes";
      this->smoothed = smoothed;
    }

    // _________________
    // TOPICS TO PUBLISH
    pubPath = n.advertise<nav_msgs::Path>(pathTopic, 1);
    pubPathNodes = n.advertise<visualization_msgs::MarkerArray>(pathNodesTopic, 1);
    pubPathVehicles = n.advertise<visualization_msgs::MarkerArray>(pathVehicleTopic, 1);
    pubPath2DNodes = n.advertise<visualization_msgs::MarkerArray>(path2DNodesTopic, 1);
    pubPathBoxes = n.advertise<visualization_msgs::MarkerArray>(pathBoxesTopic, 1);
    

    // CONFIGURE THE CONTAINER
    path.header.frame_id = "path";
  }

  //  // __________
  //  // TRACE PATH
  //  /*!
  //     \brief Given a node pointer the path to the root node will be traced recursively
  //     \param node a 3D node, usually the goal node
  //     \param i a parameter for counting the number of nodes
  //  */
  //  void tracePath(const Node3D* node, int i = 0);
  /*!
     \brief Given a node pointer the path to the root node will be traced recursively
     \param node a 3D node, usually the goal node
     \param i a parameter for counting the number of nodes
  */
  void updatePathFromK(const std::vector<Node3D> &nodePath, int k);
  void updatePath(const std::vector<Node3D> &nodePath);
  void update2DPath(const std::vector<Node2D> &nodePath);
  /*!
     \brief Adds a segment to the path
     \param node a 3D node
  */
  void addSegment(const Node3D& node);
  /*!
     \brief Adds a node to the path
     \param node a 3D node
     \param i a parameter for counting the number of nodes
  */
  void addNode(const Node3D& node, int i);
  /*!
     \brief Adds a node to the path
     \param node a 3D node
     \param i a parameter for counting the number of nodes
  */
  void add2DNode(const Node2D& node, int i);
  void add2DBox(const Node2D& node, int i);
  /*!
     \brief Adds a vehicle shape to the path
     \param node a 3D node
     \param i a parameter for counting the number of nodes
  */
  void addVehicle(const Node3D& node, int i);

  void tempUpdatePathNode(const std::vector<Node3D> &nodePath);

  // ______________
  // PUBLISH METHODS

  /// Clears the path
  void clear();
  /// Publishes the path
  void publishPath() { pubPath.publish(path); }
  /// Publishes the nodes of the path
  void publishPathNodes() { pubPathNodes.publish(pathNodes); }
  /// Publishes the vehicle along the path
  void publishPathVehicles() { pubPathVehicles.publish(pathVehicles); }
  /// Publishes the nodes of the 2D path
  void publishPath2DNodes() { pubPath2DNodes.publish(path2DNodes); }
  void publishPathBoxes() { pubPathBoxes.publish(pathBoxes); }

 private:
  /// A handle to the ROS node
  ros::NodeHandle n;
  /// Publisher for the path as a spline
  ros::Publisher pubPath;
  /// Publisher for the nodes on the path
  ros::Publisher pubPathNodes;
  /// Publisher for the vehicle along the path
  ros::Publisher pubPathVehicles;
  /// Publisher for the nodes on the 2D path
  ros::Publisher pubPath2DNodes;
  /// Publisher for the Boxes on the 2D path
  ros::Publisher pubPathBoxes;
  /// Path data structure for visualization
  nav_msgs::Path path;
  /// Nodes data structure for visualization
  visualization_msgs::MarkerArray pathNodes;
  /// Vehicle data structure for visualization
  visualization_msgs::MarkerArray pathVehicles;
  /// Nodes data structure for visualization
  visualization_msgs::MarkerArray path2DNodes;
  /// Nodes data structure for visualization

  visualization_msgs::MarkerArray pathBoxes;
  /// Value that indicates that the path is smoothed/post processed
  bool smoothed = false;
};
}
#endif // PATH_H
