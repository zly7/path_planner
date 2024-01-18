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
    std::cout << "path2D_To_Box_Start_Search_Box " << x << " " << y <<std::endl;
    bool flag=false;
    while(true){
      float radius = node2d.getRadius()+deltaL; //next status
      if(y+radius>height||y-radius<0||x+radius>width||x-radius<0){
        break;
      }
      // std::cout<<"number:"<<number<<std::endl;
      for(float count1=0;count1<2*M_PI;count1+=(2*M_PI/20)){
        float node_x=x+radius*cos(count1);
        float node_y=y+radius*sin(count1);
        Node2D nNode =  Node2D(node_x, node_y);
        nNode.setIdx(width);
        // std::cout<<"aa idx: "<<nNode.getIdx()<<" "<<!configurationSpace.isObstacleThisPoint(&nNode)<<std::endl;
        if(!configurationSpace.isObstacleThisPoint(&nNode)){
            flag=true;
            break;
        }
        
      }
      if(flag){
          break;
        }
      node2d.setRadius(radius);
      // std::cout << "radius " << radius <<std::endl;
    }
  }
}

std::vector<Node3D> AlgorithmSplit::findBou(Node3D& start,
                  const Node3D& goal,
                  std::vector<Node2D> &path2D,
                  float threshold){
  // std::vector<Node2D> nodeSeq;
  std::vector<Node3D> nodeBou;
  nodeBou.push_back(start);
  bool narrowFlag = false;
  bool wideFlag = true;
  #ifdef DEBUG_WHETHER_OUTPUT
    std::cout<<"path2D "<< path2D.size() <<std::endl;
    std::cout<<start.getT()<<" "<<goal.getT()<<std::endl;
  #endif

  
  for (size_t i = 0; i < path2D.size()-1; ++i) {
    float radius = path2D[i].getRadius();
    if(radius*2>=threshold){
      wideFlag=true;
      path2D[i].setWide(true);
      if(narrowFlag){
        narrowFlag=false;
        path2D[i].setBoundary(true);
        #ifdef DEBUG_WHETHER_OUTPUT
        std::cout<<"x "<<(path2D[i].getFloatX()-path2D[i+1].getFloatX())<<" y "<<(path2D[i].getFloatY()-path2D[i+1].getFloatY())<<std::endl;
        #endif
        float nt=atan2f((path2D[i+1].getFloatY()-path2D[i].getFloatY()),(path2D[i+1].getFloatX()-path2D[i].getFloatX()));
        nt=Helper::normalizeHeadingRad(nt);
        Node3D node3d(path2D[i].getFloatX(), path2D[i].getFloatX(), nt, 0, 0, nullptr);
        nodeBou.push_back(node3d);
        #ifdef DEBUG_WHETHER_OUTPUT
        std::cout << "bouBox" << " " << i+1  << " " << path2D[i+1].getIntX()  << " " <<path2D[i+1].getIntY() << std::endl;
        std::cout << "bouBox" << " " << i  << " " << radius << " " << path2D[i].getIntX()  << " " <<path2D[i].getIntY() << " " << node3d.getT() << std::endl;
        #endif
      }
    }else{
      narrowFlag=true;
      if(wideFlag){
        wideFlag=false;
        path2D[i].setBoundary(true);
        #ifdef DEBUG_WHETHER_OUTPUT
        std::cout<<"x "<<(path2D[i].getFloatX()-path2D[i+1].getFloatX())<<" y "<<(path2D[i].getFloatY()-path2D[i+1].getFloatY())<<std::endl;
        #endif
        float nt=atan2f((path2D[i+1].getFloatY()-path2D[i].getFloatY()),(path2D[i+1].getFloatX()-path2D[i].getFloatX()));
        nt=Helper::normalizeHeadingRad(nt);
        Node3D node3d(path2D[i].getFloatX(), path2D[i].getFloatY(), nt, 0, 0, nullptr);
        nodeBou.push_back(node3d);
        #ifdef DEBUG_WHETHER_OUTPUT
        std::cout << "bouBox" << " " << i-1  << " " << path2D[i+1].getIntX()  << " " <<path2D[i+1].getIntY() << std::endl;
        std::cout << "bouBox" << " " << i  << " " << radius << " " << path2D[i].getIntX()  << " " <<path2D[i].getIntY() << " " << node3d.getT() << std::endl;
        #endif
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

