#include "algorithmsplit.h"

#include <boost/heap/binomial_heap.hpp>

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
  nodeBou.push_back(goal);
  bool narrowFlag = false;
  bool wideFlag = false;
  std::cout<<"path2D "<< path2D.size() <<std::endl;
  std::cout<<start.getT()<<" "<<goal.getT()<<std::endl;
  
  for (size_t i = 0; i < path2D.size(); ++i) {
    float radius = path2D[i].getRadius();
    std::cout<< i << " " << path2D[i].getX() << " " << path2D[i].getY() <<std::endl;
    if(radius*2>=threshold){
      wideFlag=true;
      path2D[i].setWide(true);
      if(narrowFlag){
        narrowFlag=false;
        // nodeSeq.push_back(path2D[i-1]);
        // nodeSeq.push_back(path2D[i]);
        path2D[i].setBoundary(true);
        std::cout<<"x "<<(path2D[i].getX()-path2D[i+1].getX())<<" y "<<(path2D[i].getY()-path2D[i+1].getY())<<std::endl;
        float nt=atan2f((path2D[i].getY()-path2D[i+1].getY()),(path2D[i].getX()-path2D[i+1].getX()));
        nt=Helper::normalizeHeadingRad(nt);
        Node3D node3d(path2D[i].getIntX(), path2D[i].getIntY(), nt, 0, 0, nullptr);
        nodeBou.push_back(node3d);
        std::cout << "bouBox" << " " << i+1  << " " << path2D[i+1].getIntX()  << " " <<path2D[i+1].getIntY() << std::endl;
        std::cout << "bouBox" << " " << i  << " " << radius << " " << path2D[i].getIntX()  << " " <<path2D[i].getIntY() << " " << node3d.getT() << std::endl;
      }
    }else{
      narrowFlag=true;
      if(wideFlag){
        wideFlag=false;
        // nodeSeq.push_back(path2D[i-1]);
        // nodeSeq.push_back(path2D[i]);
        path2D[i].setBoundary(true);
        std::cout<<"x "<<(path2D[i].getX()-path2D[i+1].getX())<<" y "<<(path2D[i].getY()-path2D[i+1].getY())<<std::endl;
        float nt=atan2f((path2D[i].getY()-path2D[i+1].getY()),(path2D[i].getX()-path2D[i+1].getX()));
        nt=Helper::normalizeHeadingRad(nt);
        Node3D node3d(path2D[i].getIntX(), path2D[i].getIntY(), nt, 0, 0, nullptr);
        nodeBou.push_back(node3d);
        std::cout << "bouBox" << " " << i-1  << " " << path2D[i+1].getIntX()  << " " <<path2D[i+1].getIntY() << std::endl;
        std::cout << "bouBox" << " " << i  << " " << radius << " " << path2D[i].getIntX()  << " " <<path2D[i].getIntY() << " " << node3d.getT() << std::endl;
      }
    }
  }
  nodeBou.push_back(start);
  std::cout << "nodeBou number" << nodeBou.size() << std::endl;
  for (size_t i = 0; i < nodeBou.size()-1; ++i) {
    // float nt=atan2f((nodeBou[i].getX()-nodeBou[i-1].getX()),(nodeBou[i].getY()-nodeBou[i-1].getY()));
    // nodeBou[i].setT(Helper::normalizeHeadingRad(nt));
    std::cout << i << " " << nodeBou[i].getX() << " "  << nodeBou[i].getY() << " "  << nodeBou[i].getT() << std::endl;
  }
  return nodeBou;
}

