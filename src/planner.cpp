#include "planner.h"

using namespace HybridAStar;
//###################################################
//                                        CONSTRUCTOR
//###################################################
Planner::Planner() {
  // _____
  // TODOS
  //    initializeLookups();
  // Lookup::collisionLookup(collisionLookup);
  // ___________________
  // COLLISION DETECTION 
  //    CollisionDetection configurationSpace;
  // _________________
  // TOPICS TO PUBLISH
  pubStart = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/start", 1);

  // ___________________
  // TOPICS TO SUBSCRIBE
  if (Constants::manual) {
    subMap = n.subscribe("/map", 1, &Planner::setMap, this);
  } else {
    subMap = n.subscribe("/occ_map", 1, &Planner::setMap, this);
  }

  subGoal = n.subscribe("/move_base_simple/goal", 1, &Planner::setGoal, this);
  subStart = n.subscribe("/initialpose", 1, &Planner::setStart, this);
};

//###################################################
//                                       LOOKUPTABLES
//###################################################
void Planner::initializeLookups() {
  if (Constants::dubinsLookup) {
    Lookup::dubinsLookup(dubinsLookup);
  }

  Lookup::collisionLookup(collisionLookup);
}

//###################################################
//                                                MAP
//###################################################
void Planner::setMap(const nav_msgs::OccupancyGrid::Ptr map) {  // 这里显然是地图 
  if (Constants::coutDEBUG) {
    std::cout << "I am seeing the map..." << std::endl;
  }

  grid = map;
  //update the configuration space with the current map
  configurationSpace.updateGrid(map);
  //create array for Voronoi diagram
  //  ros::Time t0 = ros::Time::now(); 这里没有根据resolution,cellSize修改
  int height = map->info.height;
  int width = map->info.width;
  std::cout << "when setting map in Planner, height: " << height << " width: " << width << std::endl;
  bool** binMap;
  binMap = new bool*[width];

  for (int x = 0; x < width; x++) { binMap[x] = new bool[height]; }

  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      binMap[x][y] = map->data[y * width + x] ? true : false;
    }
  }

  voronoiDiagram.initializeMap(width, height, binMap);
  voronoiDiagram.update();
  voronoiDiagram.visualize();
//  ros::Time t1 = ros::Time::now();
//  ros::Duration d(t1 - t0);
//  std::cout << "created Voronoi Diagram in ms: " << d * 1000 << std::endl;

  // plan if the switch is not set to manual and a transform is available
  if (!Constants::manual && listener.canTransform("/map", ros::Time(0), "/base_link", ros::Time(0), "/map", nullptr)) {

    listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);

    // assign the values to start from base_link
    start.pose.pose.position.x = transform.getOrigin().x();
    start.pose.pose.position.y = transform.getOrigin().y();
    tf::quaternionTFToMsg(transform.getRotation(), start.pose.pose.orientation);

    if (grid->info.height >= start.pose.pose.position.y && start.pose.pose.position.y >= 0 &&
        grid->info.width >= start.pose.pose.position.x && start.pose.pose.position.x >= 0) {
      // set the start as valid and plan
      validStart = true;
    } else  {
      validStart = false;
    }

    plan();
  }
}

//###################################################
//                                   INITIALIZE START
//###################################################
void Planner::setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial) {
  float x = initial->pose.pose.position.x / Constants::cellSize;  //在这主打一个转换
  float y = initial->pose.pose.position.y / Constants::cellSize;
  float t = tf::getYaw(initial->pose.pose.orientation);
  // publish the start without covariance for rviz
  geometry_msgs::PoseStamped startN;
  startN.pose.position = initial->pose.pose.position;
  startN.pose.orientation = initial->pose.pose.orientation;
  startN.header.frame_id = "map";
  startN.header.stamp = ros::Time::now();

  std::cout << "I am seeing a new start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;

  if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
    validStart = true;
    start = *initial;

    if (Constants::manual) { plan();}

    // publish start for RViz
    pubStart.publish(startN);
  } else {
    std::cout << "invalid start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  }
}

//###################################################
//                                    INITIALIZE GOAL
//###################################################
void Planner::setGoal(const geometry_msgs::PoseStamped::ConstPtr& end) {
  // retrieving goal position
  float x = end->pose.position.x / Constants::cellSize;
  float y = end->pose.position.y / Constants::cellSize;
  float t = tf::getYaw(end->pose.orientation);

  std::cout << "I am seeing a new goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;

  if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
    validGoal = true;
    goal = *end;

    if (Constants::manual) { plan();}

  } else {
    std::cout << "invalid goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  }
}

//###################################################
//                                      PLAN THE PATH  这个函数是大核心，规划路径
//###################################################
void Planner::plan() {
  // if a start as well as goal are defined go ahead and plan
  if (validStart && validGoal) {

    // ___________________________
    // LISTS ALLOWCATED ROW MAJOR ORDER
    int width = grid->info.width;
    int height = grid->info.height;
    int depth = Constants::headings;
    int length = width * height * depth;
    // define list pointers and initialize lists
    // Node3D* nodes3D = new Node3D[length]();
    // Node2D* nodes2D = new Node2D[width * height]();

    // ________________________
    // retrieving goal position
    float x = goal.pose.position.x / Constants::cellSize;
    float y = goal.pose.position.y / Constants::cellSize;
    float t = tf::getYaw(goal.pose.orientation);
    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t);
    const Node3D nGoal(x, y, t, 0, 0, nullptr);
    const Node2D nGoal2D(x, y, 0, 0, nullptr);
    // __________
    // DEBUG GOAL
      //  const Node3D nGoal(21,8,4.71239, 0, 0, nullptr);


    // _________________________
    // retrieving start position
    x = start.pose.pose.position.x / Constants::cellSize;
    y = start.pose.pose.position.y / Constants::cellSize;
    t = tf::getYaw(start.pose.pose.orientation);
    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t);
    Node3D nStart(x, y, t, 0, 0, nullptr);
    Node2D nStart2D(x, y, 0, 0, nullptr);
    // ___________
    // DEBUG START
      //  Node3D nStart(22,7,5.49779, 0, 0, nullptr);


    // ___________________________
    // START AND TIME THE PLANNING
    ros::Time t0 = ros::Time::now();

    // CLEAR THE VISUALIZATION
    visualization.clear();
    // CLEAR THE PATH
    path.clear();
    smoothedPath.clear();
    if(Constants::algorithm == "split_hybrid_astar"){

      Node2D* nodes2D = new Node2D[width * height]();
      Node2D* nSolution2D = Algorithm::aStar2D(nStart2D, nGoal2D, nodes2D, width, height, configurationSpace, visualization);
      smoother.tracePath2D(nSolution2D);
      
      std::vector<Node2D> path2D=smoother.getPath2D();
      std::cout << "Path 2D Result: " << std::endl;
      for (const auto& node : path2D) {
        std::cout << "Node: X = " << node.getX() << ", Y = " << node.getY() << std::endl;
      }
      float deltaL=0.3;
      AlgorithmSplit::node2DToBox(path2D,width,height,configurationSpace,deltaL);
      float threshold=10;
      std::vector<Node3D> nodeBou=AlgorithmSplit::findBou(nStart,nGoal,path2D,threshold);
      path.update2DPath(path2D);

      path.publishPath2DNodes();
      path.publishPathBoxes();
      smoothedPath.publishPathBoxes();
      smoothedPath.publishPath2DNodes();
      
      std::cout<<"findBou Finished!"<<std::endl;
      int k=0;
      Node3D* nSolution=nullptr;
      delete [] nodes2D;
      for(size_t i = 1; i<nodeBou.size() ; i++){
        Node3D* nodes3D = new Node3D[length]();
        Node2D* nodes2D = new Node2D[width * height]();
        std::cout << "start " << i << " " << nodeBou[i].getX() << " " << nodeBou[i].getY() <<std::endl;
        std::cout << "end   " << i << " " << nodeBou[i-1].getX() << " " << nodeBou[i-1].getY() <<std::endl;
        nSolution = Algorithm::hybridAStar(nodeBou[i], nodeBou[i-1], nodes3D, nodes2D, width, height, configurationSpace, dubinsLookup, visualization);
        // nodeBou[i-1].setPred(nSolution);
        std::cout<<" nSolusion "<<nSolution->getX()<<" "<<nSolution->getY()<<std::endl;
        std::cout<<" nSolusion end "<<nodeBou[i-1].getX()<<" "<<nodeBou[i-1].getY()<<std::endl;
        // TRACE THE PATH
        smoother.tracePath(nSolution,0,smoother.getPath());
        // CREATE THE UPDATED PATH
        std::cout << "3D path number" << smoother.getPath().size() << std::endl;
        // path.updatePathFromK(smoother.getPath(),k);
        // smoother.smoothPath(voronoiDiagram); //you don't know its effect
        // CREATE THE UPDATED PATH
        // smoothedPath.updatePathFromK(smoother.getPath(),k);
        // k++;


        delete [] nodes3D;
        delete [] nodes2D;

        // Node3D* nodes3D = new Node3D[length]();
        // Node2D* nodes2D = new Node2D[width * height]();


      }
      // smoother.tracePath(nSolution);
      //   // CREATE THE UPDATED PATH
      // std::cout << "3D path number" << smoother.getPath().size() << std::endl;
      // path.updatePathFromK(smoother.getPath(),k);
      //   // smoother.smoothPath(voronoiDiagram); //you don't know its effect
      //   // CREATE THE UPDATED PATH
      smoothedPath.updatePathFromK(smoother.getPath(),k);
      k++;
    }else if(Constants::algorithm == "hybrid_astar"){
      Node3D* nodes3D = new Node3D[length]();
      Node2D* nodes2D = new Node2D[width * height]();
      // FIND THE PATH
      Node3D* nSolution = Algorithm::hybridAStar(nStart, nGoal, nodes3D, nodes2D, width, height, configurationSpace, dubinsLookup, visualization);
      // TRACE THE PATH
      smoother.tracePath(nSolution);
      // CREATE THE UPDATED PATH
      path.updatePath(smoother.getPath());
      // SMOOTH THE PATH
      smoother.smoothPath(voronoiDiagram);
      // CREATE THE UPDATED PATH
      smoothedPath.updatePath(smoother.getPath());

        delete [] nodes3D;
        delete [] nodes2D;
    }else if(Constants::algorithm == "contour_hybrid_astar"){
      Node2D* nodes2D = new Node2D[width * height]();
      Node2D* nSolution2D = Algorithm::aStar2D(nStart2D, nGoal2D, nodes2D, width, height, configurationSpace, visualization);

      smoother.tracePath2D(nSolution2D);
      std::vector<Node2D> path2D=smoother.getPath2D();
      path.update2DPath(path2D);//这里是为了画出2D的路径
      path.publishPath2DNodes();

      AlgorithmContour algorithmContour;
      algorithmContour.findContour(grid);     
      algorithmContour.findNarrowContourPair();
      AlgorithmContour::visualizeNarrowPairs(algorithmContour.narrowPairs,algorithmContour.gridMap);
      algorithmContour.findThroughNarrowContourPair(path2D);
      for(int i = 0;i<algorithmContour.throughNarrowPairs.size();i++){
        AlgorithmContour::visualizePathAndItNarrowPair(algorithmContour.throughNarrowPairsWaypoints[i],
              algorithmContour.throughNarrowPairs[i],algorithmContour.gridMap);
      }
      algorithmContour.findKeyInformationForthrouthNarrowPairs();
      for(int i = 0;i<algorithmContour.throughNarrowPairs.size();i++){
        AlgorithmContour::visualizekeyInfoForThrouthNarrowPair(algorithmContour.throughNarrowPairs[i],
              algorithmContour.keyInfoForThrouthNarrowPairs[i],algorithmContour.gridMap);
      }
      algorithmContour.findNarrowPassSpaceForAllPairs(configurationSpace);

      delete [] nodes2D;
    }else{
      std::cout<<"algorithm error"<<std::endl;
    }
 
    ros::Time t1 = ros::Time::now();
    ros::Duration d(t1 - t0);
    std::cout << "TIME in ms: " << d * 1000 << std::endl;

    // _________________________________
    // PUBLISH THE RESULTS OF THE SEARCH
    path.publishPath();
    path.publishPathNodes();
    path.publishPathVehicles();
    smoothedPath.publishPath();
    smoothedPath.publishPathNodes();
    smoothedPath.publishPathVehicles();
    // visualization.publishNode3DCosts(nodes3D, width, height, depth);
    // visualization.publishNode2DCosts(nodes2D, width, height);
    
    // if(Constants::algorithm == "split_hybrid_astar"){
    //   path.publishPath2DNodes();
    //   path.publishPathBoxes();
    //   smoothedPath.publishPathBoxes();
    //   smoothedPath.publishPath2DNodes();
    // }



    // delete [] nodes3D;
    // delete [] nodes2D;

  } else {
    std::cout << "missing goal or start" << std::endl;
  }
}
