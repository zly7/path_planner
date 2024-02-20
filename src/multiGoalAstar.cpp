#include "multiGoalAstar.h"


namespace HybridAStar {

multiGoalSet3D::multiGoalSet3D() {
    // Constructor implementation (if needed)
}

void multiGoalSet3D::addGoal(const Node3D& node) {
    this->goals.push_back(node);
    updateVirtualCenterNode();
}


void multiGoalSet3D::addGoals(const std::vector<Node3D>& nodes) {
    goals.insert(goals.end(), nodes.begin(), nodes.end());
    updateVirtualCenterNode();
}

void multiGoalSet3D::updateVirtualCenterNode() {
    if (goals.empty()) {
        return;
    }

    float sumX = std::accumulate(goals.begin(), goals.end(), 0.0f,
                                 [](float sum, const Node3D& node) { return sum + node.getX(); });
    float sumY = std::accumulate(goals.begin(), goals.end(), 0.0f,
                                 [](float sum, const Node3D& node) { return sum + node.getY(); });
    float avgX = sumX / goals.size();
    float avgY = sumY / goals.size();

    auto nearest = std::min_element(goals.begin(), goals.end(),
                                    [avgX, avgY](const Node3D& a, const Node3D& b) {
                                      float distA = (a.getX() - avgX) * (a.getX() - avgX) + (a.getY() - avgY) * (a.getY() - avgY);
                                      float distB = (b.getX() - avgX) * (b.getX() - avgX) + (b.getY() - avgY) * (b.getY() - avgY);
                                      return distA < distB;
                                    });

    virtualCenterNode3D.setX(avgX);
    virtualCenterNode3D.setY(avgY);
    virtualCenterNode3D.setT(nearest->getT());
}

multiGoalSet3D multiGoalSet3D::fuzzyOneNodeToSet(const CollisionDetection & collectionDection,const Node3D& node) {
    multiGoalSet3D returnSet;
    float centerX = node.getX();
    float centerY = node.getY();
    float vehicleAngle = node.getT();
    float deltaDistance = 1; // 这里是1最好是一个格子都不漏
    returnSet.addGoal(node);
    for(float currentOffset = deltaDistance;currentOffset<Constants::fuzzyLength;currentOffset+=deltaDistance){
        float currentX = centerX + currentOffset * cos(vehicleAngle);
        float currentY = centerY + currentOffset * sin(vehicleAngle);
        Node3D currentForwardNode(currentX,currentY,vehicleAngle);
        if( currentX>=0 && currentY >=0 && currentX < Node3D::widthForMap && currentY < Node3D::heightForMap && collectionDection.isTraversable(&currentForwardNode)){
            returnSet.addGoal(currentForwardNode);
        }
        currentX = centerX - currentOffset * cos(vehicleAngle);
        currentY = centerY - currentOffset * sin(vehicleAngle);
        Node3D currentBackwardNode(currentX,currentY,vehicleAngle);
        if( currentX>=0 && currentY >=0 && currentX < Node3D::widthForMap && currentY < Node3D::heightForMap &&collectionDection.isTraversable(&currentBackwardNode)){
            returnSet.addGoal(currentBackwardNode);
        }
    }
    return returnSet;
}

multiGoalSet3D multiGoalSet3D::fuzzyOneNodeToSetForSplitAstar(const CollisionDetection & collectionDetection,const Node3D& node) {
    multiGoalSet3D returnSet;
    float deltaDistance = Constants::each_meter_to_how_many_pixel * 0.2; 
    returnSet.addGoal(node);
    for(float angleOffset = -M_PI/2;angleOffset <= M_PI + 0.001;angleOffset += M_PI/2){
        float centerX = node.getX();
        float centerY = node.getY();
        float vehicleAngle = node.getT() + angleOffset; // 根据角度偏移调整

        for(float currentOffset = deltaDistance; currentOffset < Constants::fuzzyLength * 2; currentOffset += deltaDistance) {
            float currentX = centerX + currentOffset * cos(vehicleAngle);
            float currentY = centerY + currentOffset * sin(vehicleAngle);
            if(currentX < Node3D::widthForMap && currentY < Node3D::heightForMap){
                for(float deltaA = -5 * Constants::deltaHeadingRad;deltaA < 5 * Constants::deltaHeadingRad;deltaA += Constants::deltaHeadingRad) {
                    float angelVehicle = Helper::normalizeHeadingRad(node.getT() + deltaA);
                    Node3D currentNode(currentX, currentY, angelVehicle); // 使用原始节点的角度
                    if(currentX>=0 && currentY >=0 && currentX < Node3D::widthForMap && currentY < Node3D::heightForMap && collectionDetection.isTraversable(&currentNode)) {
                        returnSet.addGoal(currentNode);
                    }
                }   
            }
            
        }
    }
    if(returnSet.goals.size() == 1 && !collectionDetection.isTraversable(&(returnSet.goals[0]))){
        std::cout << "fuzzyOneNodeToSetForSplitAstar: 出现重大失误只有一个点，并且还不可以通行，会删掉这个集合" << std::endl;
        returnSet.goals.clear();
    }
    return returnSet;
}
Node3D multiGoalSet3D::getRandomGoal() {
    if (goals.empty()) {
        return Node3D();
    }
    int randomIndex = rand() % goals.size();
    return goals[randomIndex];   
}

void multiGoalSet3D::printMultiGoalsBou(const std::vector<multiGoalSet3D>& multiGoalsBou) {
    std::cout << "共有 " << multiGoalsBou.size() << " 个集合" << std::endl;
    // 遍历所有集合
    for (size_t i = 0; i < multiGoalsBou.size(); ++i) {
        const auto& set = multiGoalsBou[i];
        std::cout << "第 " << (i + 1) << " 个集合有 " << set.goals.size() << " 个点" << std::endl;
        // 遍历集合中的所有目标点
        for (size_t j = 0; j < set.goals.size(); ++j) {
            const auto& node = set.goals[j];
            std::cout << "点 " << (j + 1) << ": X = " << node.getX()
                    << ", Y = " << node.getY() << ", T = " << node.getT() << std::endl;
        }
        // 可以选择输出虚拟中心节点的信息
        std::cout << "虚拟中心节点: X = " << set.virtualCenterNode3D.getX()
                << ", Y = " << set.virtualCenterNode3D.getY()
                << ", T = " << set.virtualCenterNode3D.getT() << std::endl;
    }
}
    

}  // namespace HybridAStar
