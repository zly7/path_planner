#include "algorithm.h"
#include <chrono>
#include <boost/heap/binomial_heap.hpp>

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

  // PREDECESSOR AND SUCCESSOR INDEX
  int iPred, iSucc;
  float newG;
  // Number of possible directions, 3 for forward driving and an additional 3 for reversing
  int dir = Constants::reverse ? 6 : 3;
  // Number of iterations the algorithm has run for stopping based on Constants::iterations
  int iterations = 0;
  // VISUALIZATION DELAY
  ros::Duration d(0.003);

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

  // NODE POINTER
  Node3D* nPred;
  Node3D* nSucc;

  // float max = 0.f;
  int currentLoop = 0;
  double allRuningTime = 0;
  double allRuningTimeDubinsShot = 0;
  double allRuningTimeUpdateH = 0;
  double allRuningTimeisTraversal = 0;

  // continue until O empty
  while (!O.empty()) {
    auto startLoopTime = std::chrono::high_resolution_clock::now();
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
      // GOAL TEST
      for(auto &goal : goalSet.goals){
        if (*nPred == goal || iterations > Constants::iterations) {
          // DEBUG
          std::cout<< iterations<<std::endl;
          std::cout<<"npred "<<nPred->getX()<<" "<<nPred->getY()<<std::endl;
          std::cout<<"goal "<<goal.getX()<<" "<<goal.getY()<<std::endl; 
          if(iterations > Constants::iterations){
            std::cout<<"到达了最长的搜索迭代次数"<<std::endl;
          }
          return nPred;
        }
      }
      

      // ____________________
      // CONTINUE WITH SEARCH
      
      // _______________________
      // SEARCH WITH DUBINS SHOT
      if (Constants::dubinsShot && nPred->isInRange(goalSet.virtualCenterNode3D) && nPred->getPrim() < 3) {
        auto startDubinsShotTime = std::chrono::high_resolution_clock::now();
        for (auto &goal : goalSet.goals){
          nSucc = dubinsShot(*nPred, goal, configurationSpace);
          if (nSucc != nullptr && *nSucc == goal) {  // 这里的相等就很妙，就是整数相等，整数映射空间
          //DEBUG
          // std::cout << "max diff " << max << std::endl;
          std::cout<<"nSucc "<<nSucc->getX()<<" "<<nSucc->getY()<<std::endl;
          std::cout<<"goal "<<goal.getX()<<" "<<goal.getY()<<std::endl; 
          return nSucc;
          }
        }
        auto stopDubinsShotTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = stopDubinsShotTime - startDubinsShotTime;
        allRuningTimeDubinsShot += elapsed.count();

        
      }

      // ______________________________
      // SEARCH WITH FORWARD SIMULATION
      for (int i = 0; i < dir; i++) {  //dir 就是一个int，然后3 or 6
        // create possible successor
        nSucc = nPred->createSuccessor(i);
        // set index of the successor
        iSucc = nSucc->setIdx(width, height);

        // ensure successor is on grid and traversable
        auto startisTraversableTime = std::chrono::high_resolution_clock::now();
        bool isTraversable = configurationSpace.isTraversable(nSucc);
        auto stopisTraversableTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsedisTraversable = stopisTraversableTime - startisTraversableTime;
        allRuningTimeisTraversal += elapsedisTraversable.count();
        if (nSucc->isOnGrid(width, height) && isTraversable) {

          // ensure successor is not on closed list or it has the same index as the predecessor
          if (!nodes3D[iSucc].isClosed() || iPred == iSucc) {

            // calculate new G value
            nSucc->updateG();
            newG = nSucc->getG();

            // if successor not on open list or found a shorter way to the cell
            if (!nodes3D[iSucc].isOpen() || newG < nodes3D[iSucc].getG() || iPred == iSucc) {

              // calculate H value
              auto startupdateHTime = std::chrono::high_resolution_clock::now();
              updateHFor3DNode(*nSucc, goalSet.virtualCenterNode3D, nodes2D, dubinsLookup, width, height, configurationSpace, visualization);  //计算真实的花费
              auto stopupdateHTime = std::chrono::high_resolution_clock::now();
              std::chrono::duration<double> elapsedupdateH = stopupdateHTime - startupdateHTime;
              std::cout << "elapsedupdateH: " << elapsedupdateH.count() << std::endl;
              allRuningTimeUpdateH += elapsedupdateH.count();

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
  double runAllTime = 0;
  double runReedSheppTime = 0;
  double runAstarForH = 0;
  auto startAllTime = std::chrono::high_resolution_clock::now();
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
  auto startReedSheppTime = std::chrono::high_resolution_clock::now();
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
  auto stopReedSheppTime = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsedReedShepp = stopReedSheppTime - startReedSheppTime;
  runReedSheppTime += elapsedReedShepp.count();

  // if twoD heuristic is activated determine shortest path
  // unconstrained with obstacles
  if (Constants::twoD && !nodes2D[(int)start.getY() * width + (int)start.getX()].isDiscovered()) {
    //    ros::Time t0 = ros::Time::now();
    // create a 2d start node
    Node2D start2d(start.getX(), start.getY(), 0, 0, nullptr);
    // create a 2d goal node
    Node2D goal2d(goal.getX(), goal.getY(), 0, 0, nullptr);
    // run 2d astar and return the cost of the cheapest path for that node
    auto startAstarForHTime = std::chrono::high_resolution_clock::now();
    nodes2D[(int)start.getY() * width + (int)start.getX()].setG(
        aStar2DForUpdateHValueOf3DNode( goal2d,start2d, nodes2D, width, height, configurationSpace, visualization));
    auto stopAstarForHTime = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsedAstarForH = stopAstarForHTime - startAstarForHTime;
    runAstarForH += elapsedAstarForH.count();
  }

  if (Constants::twoD) {
    // offset for same node in cell
    twoDoffset = sqrt(((start.getX() - (long)start.getX()) - (goal.getX() - (long)goal.getX())) * ((start.getX() - (long)start.getX()) - (goal.getX() - (long)goal.getX())) +
                      ((start.getY() - (long)start.getY()) - (goal.getY() - (long)goal.getY())) * ((start.getY() - (long)start.getY()) - (goal.getY() - (long)goal.getY())));
    twoDCost = nodes2D[(int)start.getY() * width + (int)start.getX()].getG() - twoDoffset;

  }

  // return the maximum of the heuristics, making the heuristic admissable
  start.setH(std::max(reedsSheppCost, std::max(dubinsCost, twoDCost)));
  auto stopAllTime = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsedAll = stopAllTime - startAllTime;
  runAllTime += elapsedAll.count();
  std::cout << "runReedSheppTime: " << runReedSheppTime << std::endl;
  std::cout << "runAstarForH: " << runAstarForH << std::endl;
  std::cout << "runAllTime: " << runAllTime << std::endl;
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

  int i = 0;
  float x = 0.f;
  float length = dubins_path_length(&path);

  Node3D* dubinsNodes = new Node3D [(int)(length / Constants::dubinsStepSize) + 1];

  // avoid duplicate waypoint
  x += Constants::dubinsStepSize;
  while (x <  length) {
    double q[3];
    dubins_path_sample(&path, x, q);
    dubinsNodes[i].setX(q[0]);
    dubinsNodes[i].setY(q[1]);
    dubinsNodes[i].setT(Helper::normalizeHeadingRad(q[2]));

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
      //      std::cout << "Dubins shot collided, discarding the path" << "\n";
      // delete all nodes
      delete [] dubinsNodes;
      return nullptr;
    }
  }

  //  std::cout << "Dubins shot connected, returning the path" << "\n";
  return &dubinsNodes[i - 1];
}
