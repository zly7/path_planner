#include "rrtalgorithm.h"
#include <cstdlib>
using namespace HybridAStar;

RrtAlgorithm::RrtAlgorithm() {
}
Node2D* RrtAlgorithm::rrt(Node2D& start,
                          const Node2D& goal,
                          Node2D* nodes2D,
                          int width,
                          int height,
                          CollisionDetection& configurationSpace,
                          Visualize& visualization,
                          int maxIterations) {
      // 初始化 RRT 树
    std::vector<Node2D*> tree;
    tree.push_back(&start);

    Node2D* nearestNode;
    Node2D* newNode;

    for (int i = 0; i < maxIterations; ++i) {
        // 随机采样
        Node2D randomNode = sampleRandomNode(width, height);

        // 找到树中离随机点最近的节点
        nearestNode = findNearestNode(tree, randomNode);

        // 根据 nearestNode 和 randomNode 生成新的节点
        newNode = generateNewNode(nearestNode, randomNode);
        newNode->setPred(nearestNode);
        // 检查 newNode 是否与障碍物冲突
        if (configurationSpace.isTraversable(newNode)) {
            // 将 newNode 添加到树中
            tree.push_back(newNode);

            // 可视化
            if (Constants::visualization2D) {
                visualization.publishNode2DPoses(*newNode);
            }

            // 检查是否到达目标
            if (newNode->getIdx() == goal.getIdx()) {
                return newNode; 
            }
        }
    }

    return nullptr;
}
Node2D* RrtAlgorithm::rrtStar(Node2D& start,
                              const Node2D& goal,
                              Node2D* nodes2D,
                              int width,
                              int height,
                              CollisionDetection& configurationSpace,
                              Visualize& visualization,
                              int maxIterations) {
    // 初始化 RRT* 树
    std::vector<Node2D*> tree;
    tree.push_back(&start);
    start.setG(0);

    Node2D* nearestNode;
    Node2D* newNode;
    ros::Duration d(0.001);
    for (int i = 0; i < maxIterations; ++i) {
        // 随机采样
        Node2D randomNode = sampleRandomNode(width, height);
        nearestNode = findBestParent(tree, randomNode, configurationSpace);

        if (nearestNode == nullptr) {
            continue;
        }
        newNode = generateNewNode(nearestNode, randomNode);
        newNode->setPred(nearestNode);
        newNode->setG(nearestNode->getG() + distance(*nearestNode, *newNode));
        if (configurationSpace.isTraversable(newNode)) {
            rewire(tree, newNode, configurationSpace);
            tree.push_back(newNode);
            if (Constants::visualization2D) {
                visualization.publishNode2DPoses(*newNode);
            }
            if (newNode->getIdx() == goal.getIdx()) {
                return newNode; 
            }
        }
        std::cout<<"迭代生成新的点"<<std::endl;
    }

    return nullptr;
}

Node2D RrtAlgorithm::sampleRandomNode(int width, int height) {
    int x = std::rand() % width;
    int y = std::rand() % height;
    return Node2D(x, y);
}

Node2D* RrtAlgorithm::findNearestNode(const std::vector<Node2D*>& tree, const Node2D& randomNode) {
    Node2D* nearestNode = nullptr;
    float minDistance = std::numeric_limits<float>::max();

    for (auto& node : tree) {
        float distance = std::sqrt(std::pow(node->getX() - randomNode.getX(), 2) +
                                   std::pow(node->getY() - randomNode.getY(), 2));
        if (distance < minDistance) {
            minDistance = distance;
            nearestNode = node;
        }
    }

    return nearestNode;
}

Node2D* RrtAlgorithm::generateNewNode(const Node2D* nearestNode, const Node2D& randomNode) {
    // 假设的步长
    const float stepSize = 2*Constants::arcLengthForAstarSuccessor;

    // 计算方向
    float dx = randomNode.getX() - nearestNode->getX();
    float dy = randomNode.getY() - nearestNode->getY();
    float mag = sqrt(dx * dx + dy * dy);
    dx /= mag; dy /= mag;

    // 计算新节点的位置
    int newX = nearestNode->getX() + static_cast<int>(dx * stepSize);
    int newY = nearestNode->getY() + static_cast<int>(dy * stepSize);

    return new Node2D(newX, newY);
}

Node2D* RrtAlgorithm::findBestParent(std::vector<Node2D*>& tree, const Node2D& randomNode, CollisionDetection& configurationSpace) {
    std::vector<Node2D*> nearbyNodes = findNearbyNodes(tree, randomNode);
    Node2D* bestParent = nullptr;
    float minCost = std::numeric_limits<float>::max();

    for (Node2D* potentialParent : nearbyNodes) {
        float cost = potentialParent->getG() + distance(*potentialParent, randomNode);
        if (cost < minCost) {
            bestParent = potentialParent;
            minCost = cost;
        }
    }

    return bestParent;
}

void RrtAlgorithm::rewire(std::vector<Node2D*>& tree, Node2D* newNode, CollisionDetection& configurationSpace) {
    std::vector<Node2D*> nearbyNodes = findNearbyNodes(tree, *newNode);

    for (Node2D* potentialChild : nearbyNodes) {
        float newCost = newNode->getG() + distance(*newNode, *potentialChild);
        if (newCost < potentialChild->getG()) {
            potentialChild->setPred(newNode);
            potentialChild->setG(newCost);
        }
    }
}

std::vector<Node2D*> RrtAlgorithm::findNearbyNodes(const std::vector<Node2D*>& tree, const Node2D& node) {
    std::vector<Node2D*> nearbyNodes;
    float searchRadius = 5; // 根据需要设定搜索半径

    for (Node2D* potentialNode : tree) {
        if (distance(node, *potentialNode) <= searchRadius) {
            nearbyNodes.push_back(potentialNode);
        }
    }

    return nearbyNodes;
}

float RrtAlgorithm::distance(const Node2D& nodeA, const Node2D& nodeB) {
    return std::sqrt(std::pow(nodeA.getX() - nodeB.getX(), 2) +
                     std::pow(nodeA.getY() - nodeB.getY(), 2));
}
