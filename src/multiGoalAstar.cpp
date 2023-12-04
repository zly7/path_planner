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

}  // namespace HybridAStar
