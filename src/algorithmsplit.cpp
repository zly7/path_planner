#include "algorithmsplit.h"

#include <boost/heap/binomial_heap.hpp>
// #define DEBUG_WHETHER_OUTPUT
using namespace HybridAStar;

void AlgorithmSplit::node2DToBox(std::vector<Node2D> &path2D, 
                  int width,  // grid info
                  int height, // grid info
                  CollisionDetection& configurationSpace, 
                  float deltaL){
  for (Node2D& node2d : path2D) {
    float x = node2d.getFloatX();
    float y = node2d.getFloatY();
    bool flag=false;
    while(true){
      float radius = node2d.getRadius()+deltaL; //next status
      if(y+radius>height||y-radius<0||x+radius>width||x-radius<0){
        break;
      }
      for(float count1=0;count1<2*M_PI;count1+=(2*M_PI/20)){//分成20个方向
        float node_x=x+radius*cos(count1);
        float node_y=y+radius*sin(count1);
        Node2D nNode =  Node2D(node_x, node_y);
        if(!configurationSpace.isObstacleThisPoint(&nNode)){
            flag=true;
            break;
        }
      }
      if(flag){
          break;
      }
      node2d.setRadius(radius);
    }
    #ifdef DEBUG_WHETHER_OUTPUT
    std::cout << "path2D_To_Box_Start_Search_Box "<< " x: " << x << " y: " << y << " r: " << node2d.getRadius() << std::endl;
    #endif
  }
}

std::vector<Node3D> AlgorithmSplit::findBou(Node3D& start,
                  const Node3D& goal,
                  std::vector<Node2D> &path2D,
                  float threshold){
  std::cout << "瓶颈宽度: " << threshold << std::endl;
  std::vector<Node3D> nodeBou;
  nodeBou.push_back(start);
  bool narrowFlag = false;
  bool wideFlag = true;
  int howManyContinueForBou=0;//让过度更加平滑，防止突然的波动，至少要连续3个点是狭窄或者宽阔
  for (size_t i = 0; i < path2D.size()-1; ++i) {
    float radius = path2D[i].getRadius();
    if(std::hypotf(path2D[i].getFloatX()-goal.getX(),path2D[i].getFloatY()-goal.getY())<Constants::length){
      if((wideFlag == true && narrowFlag == false) || (wideFlag == false && narrowFlag == true)){
        path2D[i].setBoundary(true);
        float nt=atan2f((path2D[i+1].getFloatY()-path2D[i].getFloatY()),(path2D[i+1].getFloatX()-path2D[i].getFloatX()));
        Node3D node3d(path2D[i].getFloatX(), path2D[i].getFloatY(), nt, 0, 0, nullptr);
        nodeBou.push_back(node3d);
        break;
      }else{
        std::cout << "findBou出现重大不符合预期失误" << std::endl;
        break;
      }
    }
    if(radius*2>=threshold){
      wideFlag=true;
      path2D[i].setWide(true);
      if(narrowFlag && howManyContinueForBou>3){
        narrowFlag=false;
        path2D[i].setBoundary(true);
        int indexForBou=i-howManyContinueForBou;
        float nt=atan2f((path2D[indexForBou+1].getFloatY()-path2D[indexForBou].getFloatY()),(path2D[indexForBou+1].getFloatX()-path2D[indexForBou].getFloatX()));
        nt=Helper::normalizeHeadingRad(nt);
        Node3D node3d(path2D[indexForBou].getFloatX(), path2D[indexForBou].getFloatY(), nt, 0, 0, nullptr);
        nodeBou.push_back(node3d);
        howManyContinueForBou=3;
        #ifdef DEBUG_WHETHER_OUTPUT
        std::cout << "从狭窄变动宽阔" << std::endl;
        std::cout << "x: " << path2D[i].getFloatX() << " y: " << path2D[i].getFloatY() << " t: " << nt << std::endl;
        #endif
      }else{
        howManyContinueForBou++;
      }
    }else{
      narrowFlag=true;
      if(wideFlag && howManyContinueForBou>3){
        wideFlag=false;
        path2D[i].setBoundary(true);
        int indexForBou=i-howManyContinueForBou;
        float nt=atan2f((path2D[indexForBou+1].getFloatY()-path2D[indexForBou].getFloatY()),(path2D[indexForBou+1].getFloatX()-path2D[indexForBou].getFloatX()));
        nt=Helper::normalizeHeadingRad(nt);
        Node3D node3d(path2D[i].getFloatX(), path2D[i].getFloatY(), nt, 0, 0, nullptr);
        nodeBou.push_back(node3d);
        howManyContinueForBou=3;
        #ifdef DEBUG_WHETHER_OUTPUT
        std::cout << "从宽阔变到狭窄" << std::endl;
        std::cout << "x: " << path2D[i].getFloatX() << " y: " << path2D[i].getFloatY() << " t: " << nt << std::endl;
        #endif
      }else{
        howManyContinueForBou++;
      }
    }
  }
  nodeBou.push_back(goal);
  std::cout << "nodeBou number" << nodeBou.size() << std::endl;
  for (size_t i = 0; i < nodeBou.size()-1; ++i) {
    // float nt=atan2f((nodeBou[i].getFloatX()-nodeBou[i-1].getFloatX()),(nodeBou[i].getFloatY()-nodeBou[i-1].getFloatY()));
    // nodeBou[i].setT(Helper::normalizeHeadingRad(nt));
    std::cout << i << " " << nodeBou[i].getX() << " "  << nodeBou[i].getY() << " "  << nodeBou[i].getT() << std::endl;
  }
  return nodeBou;
}

