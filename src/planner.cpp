// #define DEBUG_VISUAL_ALGORITHMCONTOUR
// #define DEBUG_MANUAL_START_GOAL
// #define DEBUG_SHOW_INSTANT_ALGORITHMCONTOUR
// #define DEBUG_VISUAL_COSTMAP
#define DEBUG_SAVE_PICTURE
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
  pubAlgorithm = n.advertise<std_msgs::String>("/algForHA", 1);
  pubNotification = n.advertise<std_msgs::Int32>("start_notification", 5);
  if(Constants::useAutoTest){
    timerForAutoTest = n.createTimer(ros::Duration(1.0), &Planner::timerForAutoTestCallback, this);
  }
  // ___________________
  // TOPICS TO SUBSCRIBE
  if (Constants::manual) {
    subMap = n.subscribe("/map", 1, &Planner::setMap, this);
  } else {
    subMap = n.subscribe("/occ_map", 1, &Planner::setMap, this);
  }

  subGoal = n.subscribe("/move_base_simple/goal", 1, &Planner::setGoal, this);
  subStart = n.subscribe("/initialpose", 1, &Planner::setStart, this);
  Node3D::initializeVectorsForForward();//在planner里面的构造器调用这个初始化方向的函数
};

void Planner::timerForAutoTestCallback(const ros::TimerEvent& event){
  if(!whetherStartTest){
    std_msgs::Int32 msg;
    msg.data = point_index;  
    pubNotification.publish(msg);
  }
}

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
  Node3D::widthForMap = width;
  Node2D::widthForMap = width;
  Node3D::heightForMap = height;
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
  }else{
    std_msgs::Int32 msg;
    msg.data = point_index;  
    pubNotification.publish(msg);
  }
}

//###################################################
//                                   INITIALIZE START
//###################################################
void Planner::setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial) {
  whetherStartTest = true;
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
  whetherStartTest = true;
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
void Planner::plan() { //plan 这个函数是可能被反复调用的
  std_msgs::String msgAlg;
  msgAlg.data = Constants::algorithm;
  pubAlgorithm.publish(msgAlg);
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
    #ifdef DEBUG_MANUAL_START_GOAL
    //TPCAP22:173,145,0 128.2587064677 80.0000000000 8.0694652727
    //TPCAP5:128.2587064677 80.0000000000 8.0694652727
    //TPCAP8:179.0049751244 80.0000000000 8.1156136567
    x = 128.2587064677;
    y = 80;
    t = Helper::normalizeHeadingRad(8.06946);
    #endif

    // set theta to a value (0,2PI]
    t = Helper::normalizeHeadingRad(t);
    const Node3D nGoal(x, y, t, 0, 0, nullptr);
    const Node2D nGoal2D(x, y, 0, 0, nullptr);
    // retrieving start position
    x = start.pose.pose.position.x / Constants::cellSize;
    y = start.pose.pose.position.y / Constants::cellSize;
    t = tf::getYaw(start.pose.pose.orientation);
    #ifdef DEBUG_MANUAL_START_GOAL
    // TPCAP22:68 184 4.71 5:80.0000000000 134.7263681592 3.6742185844 
    //TPCAP5:80.0000000000 134.7263681592 3.6742185844 
    //TPCAP8:80.0000000000 109.3532338308 6.5222085871 
    x = 80;
    y = 134.7263681592;
    t = Helper::normalizeHeadingRad(3.6742);
    #endif
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
    smoother.clear();

    std_msgs::Int32 msg1;
    msg1.data = -1;  
    pubNotification.publish(msg1);
    if(!configurationSpace.isTraversable(&nStart)){
      std::cout<<"起始点不能通行"<<std::endl;
    }
    if (!configurationSpace.isTraversable(&nGoal)) {
      std::cout<<"目标点不能通行"<<std::endl;
    }

    if(Constants::algorithm == "split_hybrid_astar"){
      auto startTime = std::chrono::high_resolution_clock::now();
      Node2D* nodes2D = new Node2D[width * height]();
      Node2D* nSolution2D = Algorithm::aStar2D(nStart2D, nGoal2D, nodes2D, width, height, configurationSpace, visualization);
      smoother.tracePath2D(nSolution2D);
      
      std::vector<Node2D> path2D=smoother.getPath2D();
      std::reverse(path2D.begin(),path2D.end());
      float deltaL=0.1 * Constants::each_meter_to_how_many_pixel;
      AlgorithmSplit::node2DToBox(path2D,width,height,configurationSpace,deltaL);
      path.update2DPath(path2D);
      path.publishPath2DNodes();
      path.publishPathBoxes();
      float threshold=Constants::width * 1.4;
      std::vector<Node3D> nodeBou=AlgorithmSplit::findBou(nStart,nGoal,path2D,threshold); //当时陈睿瑶这里没有把path反向
      std::vector<multiGoalSet3D> multiGoalsBou;
      for(size_t k = 0; k<nodeBou.size();k++){
        if(k==0 || k==nodeBou.size()-1){
          multiGoalSet3D goals = multiGoalSet3D();
          goals.addGoal(nodeBou[k]);
          multiGoalsBou.push_back(goals);
        }else{
          multiGoalSet3D goals = multiGoalSet3D::fuzzyOneNodeToSetForSplitAstar(configurationSpace,nodeBou[k]);
          if(goals.goals.size()>0){
            multiGoalsBou.push_back(goals);
          }
        }
      }
      // multiGoalSet3D::printMultiGoalsBou(multiGoalsBou);
      std::cout<<"findBou Finished!"<<std::endl;
      Node3D tempSolution=nStart;
      Node3D * nSolution = nullptr;
      delete [] nodes2D;
      
      for(size_t i = 1; i<nodeBou.size() ; i++){
        Node3D* nodes3D = new Node3D[length]();
        Node2D* nodes2D = new Node2D[width * height]();
        nSolution = Algorithm::hybridAStarMultiGoals(tempSolution, multiGoalsBou[i], nodes3D, nodes2D, width, height, 
              configurationSpace, dubinsLookup, visualization);
        // TRACE THE PATH
        smoother.tracePathAndReverse(nSolution);
        if(nSolution!=nullptr){
          tempSolution = *nSolution;
        }else{
          std::cout<<"Split A-star第"<<i<<"个目标点搜索失败,出现重大问题"<<std::endl;
        }
        delete [] nodes3D;
        delete [] nodes2D;
      }
      this->path.updatePath(smoother.getPath());
      auto stop = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - startTime);
      std::cout << "split HA搜索消耗时间: "<< duration.count() << "  ms" << std::endl;
    }else if(Constants::algorithm == "hybrid_astar"){
      auto startTime = std::chrono::high_resolution_clock::now();
      Node3D* nodes3D = new Node3D[length]();
      Node2D* nodes2D = new Node2D[width * height]();
      // FIND THE PATH
      Node3D* nSolution = Algorithm::hybridAStar(nStart, nGoal, nodes3D, nodes2D, width, height, configurationSpace, dubinsLookup, visualization);
      // TRACE THE PATH
      smoother.tracePath(nSolution);
      // CREATE THE UPDATED PATH
      path.updatePath(smoother.getPath());
      // SMOOTH THE PATH
      // smoother.smoothPath(voronoiDiagram);//计时的时候不应该计算这段
      // CREATE THE UPDATED PATH
      smoothedPath.updatePath(smoother.getPath());

      delete [] nodes3D;
      delete [] nodes2D;
      auto stop = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - startTime);
      std::cout << "HA搜索消耗时间: "<< duration.count() << "  ms" << std::endl;
    }else if(Constants::algorithm == "contour_hybrid_astar"){

      Node2D* nodes2D = new Node2D[width * height]();
      auto startTime = std::chrono::high_resolution_clock::now();
      Node2D* nSolution2D = Algorithm::aStar2D(nStart2D, nGoal2D, nodes2D, width, height, configurationSpace, visualization);
      smoother.tracePath2D(nSolution2D);
      std::vector<Node2D> path2D=smoother.getPath2D();
      std::reverse(path2D.begin(),path2D.end());
      path.update2DPath(path2D);//这里是为了画出2D的路径
      path.publishPath2DNodes();
      auto stop = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - startTime);
      std::cout << "2D A* 使用时间: "<< duration.count() << "  ms" << std::endl;
      startTime = std::chrono::high_resolution_clock::now();
      AlgorithmContour algorithmContour;
      algorithmContour.findContour(grid);     
      algorithmContour.findNarrowContourPair();
      algorithmContour.findThroughNarrowContourPair(path2D);
      algorithmContour.sortThroughNarrowPairsWaypoints();//按照路径前后重排序狭窄点
      stop  = std::chrono::high_resolution_clock::now();
      duration += std::chrono::duration_cast<std::chrono::milliseconds>(stop - startTime);
      auto tempDuration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - startTime);
      std::cout << "寻找到狭窄点对消耗时间: "<< tempDuration.count() << "  ms" << std::endl;
      #ifdef DEBUG_VISUAL_ALGORITHMCONTOUR
      // AlgorithmContour::visualizeNarrowPairs(algorithmContour.narrowPairs,algorithmContour.gridMap);
      for(uint i = 0;i<algorithmContour.throughNarrowPairs.size();i++){
        AlgorithmContour::visualizePathAndItNarrowPair(algorithmContour.throughNarrowPairsWaypoints[i],
              algorithmContour.throughNarrowPairs[i],algorithmContour.gridMap);
      }
      #endif
      if(Constants::saveMapCsv){
        algorithmContour.saveMapCsv(nStart,nGoal);
      }
      startTime = std::chrono::high_resolution_clock::now();
      algorithmContour.findKeyInformationForthrouthNarrowPairs();
      algorithmContour.findNarrowPassSpaceForAllPairs(configurationSpace,nGoal);
      algorithmContour.findNarrowPassSpaceInputSetOfNode3DForAllPairs(configurationSpace);
      stop  = std::chrono::high_resolution_clock::now();
      duration += std::chrono::duration_cast<std::chrono::milliseconds>(stop - startTime);
      tempDuration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - startTime);
      std::cout << "寻找到狭窄点对的可行域消耗时间: "<< tempDuration.count() << "  ms" << std::endl;
      #ifdef DEBUG_VISUAL_ALGORITHMCONTOUR
      for(uint i = 0;i<algorithmContour.throughNarrowPairs.size();i++){
        AlgorithmContour::visualizekeyInfoForThrouthNarrowPair(algorithmContour.throughNarrowPairs[i],
              algorithmContour.keyInfoForThrouthNarrowPairs[i],algorithmContour.gridMap);
      }
      for(uint i = 0;i<algorithmContour.throughNarrowPairs.size();i++){
        AlgorithmContour::visualizePassSpaceBoundaryForThroughNarrowPair(algorithmContour.keyInfoForThrouthNarrowPairs[i],algorithmContour.gridMap);
      }
      AlgorithmContour::visualizeInsetForThroughNarrowPairs(algorithmContour.finalPassSpaceInOutSets,algorithmContour.gridMap);
      #endif
      #ifdef  DEBUG_SAVE_PICTURE
      algorithmContour.savePicturePathAndItNarrowPair(path2D);
      algorithmContour.savePictureNarrowSpaceBoundary();
      algorithmContour.savePictureNarrowSpaceInputSet();
      #endif  
      startTime = std::chrono::high_resolution_clock::now();
      Node3D tempStart;

      for(uint i = 0;i<algorithmContour.finalPassSpaceInOutSets.size();i++){
        if(i==0){
          tempStart = nStart;
        }
        Node3D* nSolution1= nullptr;
        Node3D* nodes3D = new Node3D[length]();
        Node2D* nodes2DSplitSearch = new Node2D[width * height]();
        multiGoalSet3D goals = multiGoalSet3D();
        goals.addGoals(algorithmContour.finalPassSpaceInOutSets[i].inSet);
        nSolution1 = Algorithm::hybridAStarMultiGoals(tempStart, goals, nodes3D, nodes2DSplitSearch, width, height, 
            configurationSpace, dubinsLookup, visualization);
        smoother.tracePathAndReverse(nSolution1);//在h函数里面有可以省略后面两个参数的定义
        if(!Constants::whetherSplitSearch){
          tempStart = *nSolution1;
        }
        #ifdef DEBUG_SHOW_INSTANT_ALGORITHMCONTOUR
        smoothedPath.tempUpdatePathNode(smoother.getPath());
        smoothedPath.publishPathNodes();
        #endif
        #ifdef DEBUG_VISUAL_COSTMAP
        visualization.publishNode3DCosts(nodes3D, width, height, depth);
        visualization.publishNode2DCosts(nodes2DSplitSearch, width, height);
        #endif
        if(Constants::whetherSplitSearch){
            Node3D* nSolution2 = nullptr; //缓存目标节点
            Node3D startSecondStage = *nSolution1;
            // auto startTime2 = std::chrono::high_resolution_clock::now();
            delete [] nodes3D;
            delete [] nodes2DSplitSearch;
            Node3D* nodes3Dt = new Node3D[length]();
            Node2D* nodes2DSplitSearcht = new Node2D[width * height]();
            // auto endTime2 = std::chrono::high_resolution_clock::now();
            // auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(endTime2 - startTime2);
            // std::cout << "删除数组内存申请数组内存花费时间: "<< duration2.count() << "  ms" << std::endl;
            nSolution2= Algorithm::hybridAStar(startSecondStage, algorithmContour.keyInfoForThrouthNarrowPairs[i]->getSecondStageMiddleVerticalPoint(), nodes3Dt, nodes2DSplitSearcht, width, height, 
              configurationSpace, dubinsLookup, visualization);
            //tempGoal = Algorithm::ArcShot(startSecondStage, algorithmContour.keyInfoForThrouthNarrowPairs[i]->getSecondStageMiddleVerticalPoint(), configurationSpace);
            if(nSolution2!=nullptr ){
              smoother.tracePathAndReverse(nSolution2->getPred());
            }
            if(nSolution2==nullptr){
              std::cout<<"在Set之间的dubinsShot搜索出现了问题"<<std::endl;
            }
            tempStart = *nSolution2; 
            #ifdef DEBUG_VISUAL_COSTMAP
            visualization.publishNode3DCosts(nodes3Dt, width, height, depth);
            visualization.publishNode2DCosts(nodes2DSplitSearcht, width, height);
            #endif
            delete [] nodes3Dt;
            delete [] nodes2DSplitSearcht;
        }
        #ifdef DEBUG_SHOW_INSTANT_ALGORITHMCONTOUR
        std::cout<<"第"<<i<<"个Set搜索完毕"<<std::endl;
        smoothedPath.tempUpdatePathNode(smoother.getPath());
        smoothedPath.publishPathNodes();
        #endif
      }
      Node3D* nodes3D = new Node3D[length]();
      Node2D* nodes2DSplitSearch = new Node2D[width * height]();
      Node3D* tempGoal;
      #ifdef DEBUG_SHOW_INSTANT_ALGORITHMCONTOUR
        if(!configurationSpace.isTraversable(&nGoal)){
          std::cout<<"目标设置有误，无法搜索"<<std::endl;
        }
      #endif
      if (Constants::each_meter_to_how_many_pixel >= 6) {
          multiGoalSet3D goalsMulFinal = multiGoalSet3D::fuzzyOneNodeToSet(configurationSpace,nGoal);
          auto startGoal = algorithmContour.finalPassSpaceInOutSets.size() > 0 ? smoother.getPath().back(): nStart;//最后这个节点这里是带方向的
          tempGoal = Algorithm::hybridAStarMultiGoals(startGoal, goalsMulFinal, nodes3D, nodes2DSplitSearch, width, height, configurationSpace, dubinsLookup, visualization);
      } else {
          auto startGoal = algorithmContour.finalPassSpaceInOutSets.size() > 0 ? smoother.getPath().back() : nStart;
          tempGoal = Algorithm::hybridAStar(startGoal, nGoal, nodes3D, nodes2DSplitSearch, width, height, configurationSpace, dubinsLookup, visualization);
      }
      if(tempGoal!=nullptr ){//每次都会反向trace一下
        smoother.tracePathAndReverse(tempGoal);
      }else{
        std::cout<<"在进入最后一段搜索的时候tempgoal是nullptr"<<std::endl;
      }
      if(Constants::whetherFuzzyGoal){
        // Node3D nonConstGoal{nGoal.getX(),nGoal.getY(),nGoal.getT(),0,0,nullptr};
        std::vector<Node3D> interpolatedPath = Node3D::interpolateDirect(*tempGoal,nGoal,Constants::arcLengthForAstarSuccessor);
        smoother.getPathNotConst().insert(smoother.getPathNotConst().end(), interpolatedPath.begin(), interpolatedPath.end());
      }
      this->path.updatePath(smoother.getPath());
      delete [] nodes3D;
      delete [] nodes2DSplitSearch;
      delete [] nodes2D;

      stop = std::chrono::high_resolution_clock::now();
      duration += std::chrono::duration_cast<std::chrono::milliseconds>(stop - startTime);
      tempDuration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - startTime);
      std::cout << "多次HA搜索消耗时间: "<< tempDuration.count() << "  ms" << std::endl;
      std::cout << "AlgorithmContour总花时间: "<< duration.count() << "  ms" << std::endl;
      // CREATE THE UPDATED PATH
      smoothedPath.updatePath(smoother.getPath());
    }else{
      std::cout<<"algorithm error"<<std::endl;
    }
 
    ros::Time t1 = ros::Time::now();
    ros::Duration d(t1 - t0);
    std::cout << "整个流程连带可视化和人关闭窗口时间 in ms: " << d * 1000 << std::endl;
    validStart=false;
    validGoal=false;
    point_index++;

    std_msgs::Int32 msg2;
    msg2.data = point_index;  
    pubNotification.publish(msg2);

    std::cout<<"point_index: "<<point_index<<std::endl;

    // _________________________________
    // PUBLISH THE RESULTS OF THE SEARCH
    path.publishPath();
    path.publishPathNodes();
    path.publishPathVehicles();
    smoothedPath.publishPath();
    smoothedPath.publishPathNodes();
    smoothedPath.publishPathVehicles();
    std_msgs::Int32 msg;
    msg.data = -2;  
    pubNotification.publish(msg);

    
  } else {
    std::cout << "missing goal or start" << std::endl;
  }
  
  
}
