// #define DEBUG_TIME_ASTAR3D
// #define DEBUG_TIME_UPDATEH
#include "algorithm.h"

typedef CGAL::Exact_circular_kernel_2 Kernel;
typedef Kernel::Point_2 Point_2;
typedef Kernel::Line_2 Line_2;
typedef Kernel::Circle_2 Circle_2;
typedef Kernel::Circular_arc_point_2   Circular_arc_point_2;
typedef Kernel::Circular_arc_2 Circular_arc_2;
typedef Kernel::Vector_2               Vector_2;

using namespace HybridAStar;

float aStar2DForUpdateHValueOf3DNode(Node2D& start, Node2D& goal, Node2D* nodes2D, int width, int height, CollisionDetection& configurationSpace, Visualize& visualization);
void updateHFor3DNode(Node3D& start, const Node3D& goal, Node2D* nodes2D, float* dubinsLookup, int width, int height, CollisionDetection& configurationSpace, Visualize& visualization);



double measureTime(std::function<void()> func) {
    auto start = std::chrono::high_resolution_clock::now();
    func();
    auto stop = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = stop - start;
    return elapsed.count();
}

//###################################################
//                                    NODE COMPARISON
//###################################################
/*!
   \brief A structure to sort nodes in a heap structure
*/
struct CompareNodes {
  /// Sorting 3D nodes by increasing C value - the total estimated cost
  bool operator()(const Node3D* lhs, const Node3D* rhs) const {
    return lhs->getC() > rhs->getC();
  }
  /// Sorting 2D nodes by increasing C value - the total estimated cost
  bool operator()(const Node2D* lhs, const Node2D* rhs) const {
    return lhs->getC() > rhs->getC();
  }
};

//###################################################
//                                        3D A*
//###################################################
Node3D* Algorithm::hybridAStar(Node3D& start,
                               const Node3D& goal,
                               Node3D* nodes3D,
                               Node2D* nodes2D,
                               int width,
                               int height,
                               CollisionDetection& configurationSpace,
                               float* dubinsLookup,
                               Visualize& visualization)
                              {
  multiGoalSet3D goalSet;
  goalSet.addGoal(goal);
  return hybridAStarMultiGoals(start, goalSet, nodes3D, nodes2D, width, height, configurationSpace, dubinsLookup, visualization);
                               }
Node3D* Algorithm::hybridAStarMultiGoals(Node3D& start,
                               multiGoalSet3D& goalSet,
                               Node3D* nodes3D,
                               Node2D* nodes2D,
                               int width,
                               int height,
                               CollisionDetection& configurationSpace,
                               float* dubinsLookup,
                               Visualize& visualization) {
  if(Constants::visualizationStartAndGoal){
    visualization.publishNode3DStartAndGoal(start, goalSet.virtualCenterNode3D);
  }
  // PREDECESSOR AND SUCCESSOR INDEX
  int iPred, iSucc;
  float newG;
  // Number of possible directions, 3 for forward driving and an additional 3 for reversing
  int dir = Constants::reverse ? 2*Node3D::dir : Node3D::dir;
  // Number of iterations the algorithm has run for stopping based on Constants::iterations
  int iterations = 0;
  // VISUALIZATION DELAY
  ros::Duration d(0.003);
  start.setPred(nullptr);//清空
  // OPEN LIST AS BOOST IMPLEMENTATION
  typedef boost::heap::binomial_heap<Node3D*,
          boost::heap::compare<CompareNodes>
          > priorityQueue;
  priorityQueue O;

  // update h value 下面这句话不清楚为什么作者会写，事实上注释掉好像没关系
  //updateHFor3DNode(start, goal, nodes2D, dubinsLookup, width, height, configurationSpace, visualization);
  // mark start as open
  start.open();
  // push on priority queue aka open list
  O.push(&start);
  iPred = start.setIdx(width, height); //这里的3D指的是带有xyt的节点，最后一个t是yaw角，
  nodes3D[iPred] = start;
  std::cout<<"start "<<start.getX()<<" "<<start.getY()<< " " << start.getT()<<std::endl;
  // NODE POINTER
  Node3D* nPred;
  Node3D* nSucc;

  #ifdef DEBUG_TIME_ASTAR3D
  int currentLoop = 0;
  double allRuningTime = 0;
  double allRuningTimeDubinsShot = 0;
  double allRuningTimeUpdateH = 0;
  double allRuningTimeisTraversal = 0;
  #endif

  // continue until O empty
  while (!O.empty()) {
    #ifdef DEBUG_TIME_ASTAR3D
    auto startLoopTime = std::chrono::high_resolution_clock::now();
    #endif
    // pop node with lowest cost from priority queue pop出堆的第一步
    nPred = O.top();
    // set index
    iPred = nPred->setIdx(width, height);
    iterations++;

    // RViz visualization
    if (Constants::visualization) {
      visualization.publishNode3DPoses(*nPred);
      visualization.publishNode3DPose(*nPred);
      d.sleep();
    }

    // _____________________________
    // LAZY DELETION of rewired node
    // if there exists a pointer this node has already been expanded
    if (nodes3D[iPred].isClosed()) {
      // pop node from the open list and start with a fresh node
      O.pop();
      continue;
    }
    // _________________
    // EXPANSION OF NODE
    else if (nodes3D[iPred].isOpen()) {
      // add node to closed list
      nodes3D[iPred].close();
      // remove node from open list
      O.pop();

      // _________
      // GOAL TEST //既然能走到这，nPred肯定是通的
      for(auto &goal : goalSet.goals){
        if ((*nPred).isEqualWithTolerance(goal)) {
          std::cout<<"总迭代次数: "<< iterations<<std::endl;
          std::cout<<"npred "<<nPred->getX()<<" "<<nPred->getY()<< " " << nPred->getT()<<std::endl;
          std::cout<<"goal "<<goal.getX()<<" "<<goal.getY() << ""<< goal.getT()<<std::endl; 
          std::cout<<"Hybrid3D结束搜索总的搜索次数: "<<iterations<<std::endl;
          return nPred;
        }
      }
      if(iterations > Constants::iterations){
        std::cout<<"到达了最长的搜索迭代次数,未能找到目标点"<<std::endl;
        return nullptr;
      }
      

      // ____________________
      // CONTINUE WITH SEARCH
      
      // _______________________
      // SEARCH WITH DUBINS SHOT
      if (Constants::useArcShot && nPred->isInArcRange(goalSet.virtualCenterNode3D )){
        for (auto &goal : goalSet.goals){
          nSucc = ArcShot(*nPred, goal, configurationSpace);
          if (nSucc != nullptr && *nSucc == goal) {  // 这里的相等就很妙，就是整数相等，整数映射空间
          std::cout << "通过ArcShot 命中结束点" << std::endl;
          std::cout<< "start "<< start.getX()<<" "<<start.getY()<<std::endl;
          std::cout<<"nSucc "<<nSucc->getX()<<" "<<nSucc->getY()<<std::endl;
          std::cout<<"goal "<<goal.getX()<<" "<<goal.getY()<<std::endl; 
          return nSucc;
          }
        }
      }
      if (Constants::dubinsShot && nPred->isInRange(goalSet.virtualCenterNode3D) ) {
        #ifdef DEBUG_TIME_ASTAR3D
        auto startDubinsShotTime = std::chrono::high_resolution_clock::now();
        #endif
        for (auto &goal : goalSet.goals){
          nSucc = dubinsShot(*nPred, goal, configurationSpace);
          if (nSucc != nullptr && *nSucc == goal) {  // 这里的相等就很妙，就是整数相等，整数映射空间
          std::cout << "通过dubinShot 命中结束点" << std::endl;
          std::cout<<"nSucc "<<nSucc->getX()<<" "<<nSucc->getY()<<std::endl;
          std::cout<<"goal "<<goal.getX()<<" "<<goal.getY()<<std::endl; 
          return nSucc;
          }
        }
        #ifdef DEBUG_TIME_ASTAR3D
        auto stopDubinsShotTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = stopDubinsShotTime - startDubinsShotTime;
        allRuningTimeDubinsShot += elapsed.count();
        #endif
      }

      // ______________________________
      // SEARCH WITH FORWARD SIMULATION
      for (int i = 0; i < dir; i++) {  //dir 就是一个int，然后3 or 6
        // create possible successor
        nSucc = nPred->createSuccessor(i);
        // set index of the successor
        iSucc = nSucc->setIdx(width, height);

        // ensure successor is on grid and traversable
        #ifdef DEBUG_TIME_ASTAR3D
        auto startisTraversableTime = std::chrono::high_resolution_clock::now();
        #endif
        bool isTraversable = configurationSpace.isTraversable(nSucc);
        #ifdef DEBUG_TIME_ASTAR3D
        auto stopisTraversableTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsedisTraversable = stopisTraversableTime - startisTraversableTime;
        allRuningTimeisTraversal += elapsedisTraversable.count();
        #endif
        if (nSucc->isOnGrid(width, height) && isTraversable) {

          // ensure successor is not on closed list or it has the same index as the predecessor
          if (!nodes3D[iSucc].isClosed() || iPred == iSucc) {

            // calculate new G value
            nSucc->updateG();
            newG = nSucc->getG();

            // if successor not on open list or found a shorter way to the cell
            if (!nodes3D[iSucc].isOpen() || newG < nodes3D[iSucc].getG() || iPred == iSucc) {

              // calculate H value
              #ifdef DEBUG_TIME_ASTAR3D
              auto startupdateHTime = std::chrono::high_resolution_clock::now();
              #endif
              updateHFor3DNode(*nSucc, goalSet.virtualCenterNode3D, nodes2D, dubinsLookup, width, height, configurationSpace, visualization);  //计算真实的花费
              #ifdef DEBUG_TIME_ASTAR3D
              auto stopupdateHTime = std::chrono::high_resolution_clock::now();
              std::chrono::duration<double> elapsedupdateH = stopupdateHTime - startupdateHTime;
              std::cout << "elapsedupdateH: " << elapsedupdateH.count() << std::endl;
              allRuningTimeUpdateH += elapsedupdateH.count();
              #endif

              // if the successor is in the same cell but the C value is larger
              if (iPred == iSucc && nSucc->getC() > nPred->getC() + Constants::tieBreaker) {
                delete nSucc;
                continue;
              }
              // if successor is in the same cell and the C value is lower, set predecessor to predecessor of predecessor
              else if (iPred == iSucc && nSucc->getC() <= nPred->getC() + Constants::tieBreaker) {
                nSucc->setPred(nPred->getPred());
              }

              if (nSucc->getPred() == nSucc) {
                std::cout << "looping";
              }

              // put successor on open list
              nSucc->open();
              nodes3D[iSucc] = *nSucc;
              O.push(&nodes3D[iSucc]);
              delete nSucc;
            } else { delete nSucc; }
          } else { delete nSucc; }
        } else { delete nSucc; }
      }
      
    }
    #ifdef DEBUG_TIME_ASTAR3D
    auto stopLoopTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = stopLoopTime - startLoopTime;
    allRuningTime += elapsed.count();
    if(currentLoop % 10 == 0) {
      std::cout << "currentLoop: "<< currentLoop<<" all Runing time: " << allRuningTime << std::endl;
      std::cout << "currentLoop: "<< currentLoop<<" all Runing time DubinsShot: " << allRuningTimeDubinsShot << std::endl;
      std::cout << "currentLoop: "<< currentLoop<<" all Runing time updateH: " << allRuningTimeUpdateH << std::endl;
      std::cout << "currentLoop: "<< currentLoop<<" all Runing time isTraversal: " << allRuningTimeisTraversal << std::endl;
    }
    currentLoop++;
    #endif
  }

  if (O.empty()) {
    return nullptr;
  }

  return nullptr;
}

//###################################################
//                                        2D A*这个函数应该是goal作为start传入
//###################################################
float aStar2DForUpdateHValueOf3DNode(Node2D& start,
            Node2D& goal,
            Node2D* nodes2D,
            int width,
            int height,
            CollisionDetection& configurationSpace,
            Visualize& visualization) {

  // PREDECESSOR AND SUCCESSOR INDEX
  int iPred, iSucc;
  float newG;

  // reset the open and closed list//这里果然有清空的代码
  for (int i = 0; i < width * height; ++i) {
    nodes2D[i].reset();
  }

  // VISUALIZATION DELAY
  ros::Duration d(0.001);

  boost::heap::binomial_heap<Node2D*,
        boost::heap::compare<CompareNodes>> O;
  // update h value
  start.updateH(goal);
  // mark start as open
  start.open();
  // push on priority queue
  O.push(&start);
  iPred = start.setIdx(width);
  nodes2D[iPred] = start;

  // NODE POINTER
  Node2D* nPred;
  Node2D* nSucc;

  // continue until O empty
  while (!O.empty()) {
    // pop node with lowest cost from priority queue
    nPred = O.top();
    // set index
    iPred = nPred->setIdx(width);

    // _____________________________
    // LAZY DELETION of rewired node
    // if there exists a pointer this node has already been expanded
    if (nodes2D[iPred].isClosed()) {
      // pop node from the open list and start with a fresh node
      O.pop();
      continue;
    }
    // _________________
    // EXPANSION OF NODE
    else if (nodes2D[iPred].isOpen()) {
      // add node to closed list
      nodes2D[iPred].close();
      nodes2D[iPred].discover();

      // RViz visualization
      if (Constants::visualization2D) {
        // std::cout << "2d visual npred :" << nPred->getX() << " " << nPred->getY() << std::endl;
        visualization.publishNode2DPoses(*nPred);
        visualization.publishNode2DPose(*nPred);

      }

      // remove node from open list
      O.pop();

      // _________
      // GOAL TEST
      if (*nPred == goal) {
        return nPred->getG();
      }
      // ____________________
      // CONTINUE WITH SEARCH
      else {
        // _______________________________
        // CREATE POSSIBLE SUCCESSOR NODES
        for (int i = 0; i < Node2D::dir; i++) {
          // create possible successor
          nSucc = nPred->createSuccessor(i); //创建后继这里会做g的继承
          // set index of the successor
          iSucc = nSucc->setIdx(width);

          // ensure successor is on grid ROW MAJOR
          // ensure successor is not blocked by obstacle
          // ensure successor is not on closed list
          if (nSucc->isOnGrid(width, height) &&  configurationSpace.isObstacleThisPoint(nSucc) && !nodes2D[iSucc].isClosed()) {
            // calculate new G value
            nSucc->updateG(); //在这里的G应该累加,然而事实上确实累加了
            newG = nSucc->getG();

            // if successor not on open list or g value lower than before put it on open list
            if (!nodes2D[iSucc].isOpen() || newG < nodes2D[iSucc].getG()) {
              // calculate the H value
              nSucc->updateH(goal);
              // put successor on open list
              nSucc->open();
              nodes2D[iSucc] = *nSucc;
              O.push(&nodes2D[iSucc]);
              delete nSucc;
            } else { delete nSucc; }
          } else { delete nSucc; }
        }
      }
    }
  }

  // return large number to guide search away
  return 1000;
}

Node2D* Algorithm::aStar2D(Node2D& start,
            const Node2D& goal,
            Node2D* nodes2D,
            int width,
            int height,
            CollisionDetection& configurationSpace,
            Visualize& visualization) {

  // PREDECESSOR AND SUCCESSOR INDEX
  int iPred, iSucc;
  float newG;

  // reset the open and closed list
  for (int i = 0; i < width * height; ++i) {
    nodes2D[i].reset();
  }

  // VISUALIZATION DELAY
  ros::Duration d(0.001);

  boost::heap::binomial_heap<Node2D*,
        boost::heap::compare<CompareNodes>> O;
  // update h value
  start.updateH(goal);
  // mark start as open
  start.open();
  // push on priority queue
  O.push(&start);
  iPred = start.setIdx(width);
  nodes2D[iPred] = start;

  // NODE POINTER
  Node2D* nPred;
  Node2D* nSucc;
  
  // continue until O empty
  while (!O.empty()) {
    // pop node with lowest cost from priority queue
    nPred = O.top();
    // set index
    iPred = nPred->setIdx(width);

    // _____________________________
    // LAZY DELETION of rewired node
    // if there exists a pointer this node has already been expanded
    if (nodes2D[iPred].isClosed()) {
      // pop node from the open list and start with a fresh node
      O.pop();
      continue;
    }
    // _________________
    // EXPANSION OF NODE
    else if (nodes2D[iPred].isOpen()) {
      // add node to closed list
      nodes2D[iPred].close();
      nodes2D[iPred].discover();

      // RViz visualization
      if (Constants::visualization2D) {
        visualization.publishNode2DPoses(*nPred);
        visualization.publishNode2DPose(*nPred);
        d.sleep();
      }

      // remove node from open list
      O.pop();

      // _________
      // GOAL TEST
      if (*nPred == goal) {
        return nPred;
      }
      // ____________________
      // CONTINUE WITH SEARCH
      else {
        // _______________________________
        // CREATE POSSIBLE SUCCESSOR NODES
        for (int i = 0; i < Node2D::dir; i++) {
          // create possible successor
          nSucc = nPred->createSuccessor(i);
          // set index of the successor
          iSucc = nSucc->setIdx(width);

          // ensure successor is on grid ROW MAJOR
          // ensure successor is not blocked by obstacle
          // ensure successor is not on closed list
          if (nSucc->isOnGrid(width, height) &&  configurationSpace.isTraversable(nSucc) && !nodes2D[iSucc].isClosed()) {
            // calculate new G value
            nSucc->updateG(); //在这里的G应该累加,然而事实上确实累加了
            newG = nSucc->getG();

            // if successor not on open list or g value lower than before put it on open list
            if (!nodes2D[iSucc].isOpen() || newG < nodes2D[iSucc].getG()) {
              // calculate the H value
              nSucc->updateH(goal);
              // put successor on open list
              nSucc->open();
              nodes2D[iSucc] = *nSucc;
              O.push(&nodes2D[iSucc]);
              delete nSucc;
            } else { delete nSucc; }
          } else { delete nSucc; }
        }
      }
    }
  }

  return nullptr;
}


//###################################################
//                                         COST TO GO
//###################################################
void updateHFor3DNode(Node3D& start, const Node3D& goal, Node2D* nodes2D, float* dubinsLookup, int width, int height, 
  CollisionDetection& configurationSpace, Visualize& visualization) {
  
  float dubinsCost = 0;
  float reedsSheppCost = 0;
  float twoDCost = 0;
  float twoDoffset = 0;
  #ifdef DEBUG_TIME_UPDATEH
  double runAllTime = 0;
  double runReedSheppTime = 0;
  double runAstarForH = 0;
  auto startAllTime = std::chrono::high_resolution_clock::now();
  #endif
  // if dubins heuristic is activated calculate the shortest path
  // constrained without obstacles
  if (Constants::dubins) {
    ompl::base::DubinsStateSpace dubinsPath(Constants::r);
    State* dbStart = (State*)dubinsPath.allocState();
    State* dbEnd = (State*)dubinsPath.allocState();
    dbStart->setXY(start.getX(), start.getY());
    dbStart->setYaw(start.getT());
    dbEnd->setXY(goal.getX(), goal.getY());
    dbEnd->setYaw(goal.getT());
    dubinsCost = dubinsPath.distance(dbStart, dbEnd);
  }

  // if reversing is active use a  一般用这个reedsShepp，可以倒车
  #ifdef DEBUG_TIME_UPDATEH
  auto startReedSheppTime = std::chrono::high_resolution_clock::now();
  #endif
  if (Constants::reverse && !Constants::dubins) {
    ompl::base::ReedsSheppStateSpace reedsSheppPath(Constants::r);
    State* rsStart = (State*)reedsSheppPath.allocState();
    State* rsEnd = (State*)reedsSheppPath.allocState();
    rsStart->setXY(start.getX(), start.getY());
    rsStart->setYaw(start.getT());
    rsEnd->setXY(goal.getX(), goal.getY());
    rsEnd->setYaw(goal.getT());
    reedsSheppCost = reedsSheppPath.distance(rsStart, rsEnd);
  }
  #ifdef DEBUG_TIME_UPDATEH
  auto stopReedSheppTime = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsedReedShepp = stopReedSheppTime - startReedSheppTime;
  runReedSheppTime += elapsedReedShepp.count();
  #endif

  // if twoD heuristic is activated determine shortest path
  // unconstrained with obstacles
  if (Constants::twoD && !nodes2D[(int)start.getY() * width + (int)start.getX()].isDiscovered()) {
    //    ros::Time t0 = ros::Time::now();
    // create a 2d start node
    Node2D start2d(start.getX(), start.getY(), 0, 0, nullptr);
    // create a 2d goal node
    Node2D goal2d(goal.getX(), goal.getY(), 0, 0, nullptr);
    // run 2d astar and return the cost of the cheapest path for that node
    #ifdef DEBUG_TIME_UPDATEH
    auto startAstarForHTime = std::chrono::high_resolution_clock::now();
    #endif
    nodes2D[(int)start.getY() * width + (int)start.getX()].setG(
        aStar2DForUpdateHValueOf3DNode( goal2d,start2d, nodes2D, width, height, configurationSpace, visualization));
    #ifdef DEBUG_TIME_UPDATEH
    auto stopAstarForHTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsedAstarForH = stopAstarForHTime - startAstarForHTime;
    runAstarForH += elapsedAstarForH.count();
    #endif
  }

  if (Constants::twoD) {
    // offset for same node in cell
    twoDoffset = sqrt(((start.getX() - (long)start.getX()) - (goal.getX() - (long)goal.getX())) * ((start.getX() - (long)start.getX()) - (goal.getX() - (long)goal.getX())) +
                      ((start.getY() - (long)start.getY()) - (goal.getY() - (long)goal.getY())) * ((start.getY() - (long)start.getY()) - (goal.getY() - (long)goal.getY())));
    twoDCost = nodes2D[(int)start.getY() * width + (int)start.getX()].getG() - twoDoffset;

  }

  // return the maximum of the heuristics, making the heuristic admissable
  start.setH(std::max(reedsSheppCost, std::max(dubinsCost, twoDCost)));
  #ifdef DEBUG_TIME_UPDATEH
  auto stopAllTime = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsedAll = stopAllTime - startAllTime;
  runAllTime += elapsedAll.count();
  std::cout << "runReedSheppTime: " << runReedSheppTime << std::endl;
  std::cout << "runAstarForH: " << runAstarForH << std::endl;
  std::cout << "runAllTime: " << runAllTime << std::endl;
  #endif
}

//###################################################
//                                        DUBINS SHOT
//###################################################
Node3D* Algorithm::dubinsShot(Node3D& start, const Node3D& goal, CollisionDetection& configurationSpace) {
  // start
  double q0[] = { start.getX(), start.getY(), start.getT() };
  // goal
  double q1[] = { goal.getX(), goal.getY(), goal.getT() };
  // initialize the path
  DubinsPath path;
  // calculate the path
  dubins_init(q0, q1, Constants::r, &path);
  int primToInherit = start.getPrim();
  int i = 0;
  float x = 0.f;
  float length = dubins_path_length(&path);

  Node3D* dubinsNodes = new Node3D [(int)(length / Constants::dubinsStepSize) + 1];

  // avoid duplicate waypoint
  x += Constants::dubinsStepSize;
  while (x <  length) {
    double q[3];//q相当于是实际的点
    dubins_path_sample(&path, x, q);
    dubinsNodes[i].setX(q[0]);
    dubinsNodes[i].setY(q[1]);
    dubinsNodes[i].setT(Helper::normalizeHeadingRad(q[2]));
    dubinsNodes[i].setPrim(primToInherit);//为了能让prim正确继承是向前还是向后
    // collision check
    if (configurationSpace.isTraversable(&dubinsNodes[i])) {

      // set the predecessor to the previous step
      if (i > 0) {
        dubinsNodes[i].setPred(&dubinsNodes[i - 1]);
      } else {
        dubinsNodes[i].setPred(&start);
      }

      if (&dubinsNodes[i] == dubinsNodes[i].getPred()) {
        std::cout << "looping shot";
      }

      x += Constants::dubinsStepSize;
      i++;
    } else {
      // delete all nodes
      delete [] dubinsNodes;
      return nullptr;
    }
  }

  //  std::cout << "Dubins shot connected, returning the path" << "\n";
  return &dubinsNodes[i - 1];
}

Node3D* Algorithm::ArcShot(Node3D& start, const Node3D& goal, CollisionDetection& configurationSpace){

   // 获取起点和终点的坐标及方向
    Point_2 p1(start.getX(), start.getY()), p2(goal.getX(), goal.getY());
    double theta1 = start.getT(), theta2 = goal.getT();

    // 计算切线方向
    Vector_2 dir1(std::cos(theta1), std::sin(theta1)), dir2(std::cos(theta2), std::sin(theta2));

    // 构建切线
    Line_2 tangent1(p1, p1 + dir1), tangent2(p2, p2 + dir2);

    // 构建垂线
    Line_2 perp1 = tangent1.perpendicular(p1), perp2 = tangent2.perpendicular(p2);

    // 计算垂线交点（圆心）
    CGAL::Object result = CGAL::intersection(perp1, perp2);
    Point_2 center;
    if (const Point_2 *ipoint = CGAL::object_cast<Point_2>(&result)) {
        center = *ipoint;
    } else {
        // 垂线没有交点
        return nullptr;
    }
    // 计算半径的平方
    Kernel::FT square_radius = CGAL::squared_distance(center, p1);
    double radius = std::sqrt(CGAL::to_double(square_radius));
    Vector_2 radiusDirStart = p1 - center;
    Vector_2 radiusDirEnd = p2 - center;

    // 计算圆弧的起始角度和终止角度
    double startAngle = std::atan2(CGAL::to_double(radiusDirStart.y()), CGAL::to_double(radiusDirStart.x()));
    double endAngle = std::atan2(CGAL::to_double(radiusDirEnd.y()), CGAL::to_double(radiusDirEnd.x()));
    double angelFromRadiusToVehicle = theta1 - Helper::normalizeHeadingRad(startAngle);
    if(angelFromRadiusToVehicle > M_PI){
        angelFromRadiusToVehicle -= 2 * M_PI;
    } else if(angelFromRadiusToVehicle < -M_PI){
        angelFromRadiusToVehicle += 2 * M_PI;
    }
    double deltaAngle = endAngle - startAngle;
    if(deltaAngle > M_PI) {
        deltaAngle -= 2 * M_PI;
    } else if(deltaAngle < -M_PI) {
        deltaAngle += 2 * M_PI;
    }

    double eachAngle = Node3D::dx[0] / radius;
    int numPoints = static_cast<int>(std::abs(deltaAngle) / eachAngle) + 2; // +2 for start and goal
    Node3D* arcNodes = new Node3D[numPoints];
    double mul = deltaAngle > 0 ? 1.0 : -1.0;
    int i = 0;
    for (double angle = 0; angle <= std::abs(deltaAngle); angle += eachAngle) {
        double currentAngle = mul * angle+startAngle;
        double x = CGAL::to_double(center.x()) + radius * std::cos(currentAngle);
        double y = CGAL::to_double(center.y()) + radius * std::sin(currentAngle);
        arcNodes[i].setX(x);
        arcNodes[i].setY(y);
        arcNodes[i].setT(Helper::normalizeHeadingRad(currentAngle+angelFromRadiusToVehicle));

        // 设置前驱节点
        if (i > 0) {
            arcNodes[i].setPred(&arcNodes[i - 1]);
        } else {
            arcNodes[i].setPred(&start);
        }

        if (configurationSpace.isTraversable(&arcNodes[i])) {
            i++;
        } else {
            delete [] arcNodes; // 清理资源
            return nullptr;
        }
    }

    arcNodes[i] = goal; // 最后一个点是目标点
    arcNodes[i].setPred(&arcNodes[i - 1]);
    return &arcNodes[i];
}