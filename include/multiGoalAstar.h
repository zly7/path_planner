#ifndef MULTIGOALASTAR_H
#define MULTIGOALASTAR_H

#include <set>
#include <vector>
#include "node3d.h"
#include <numeric>  // For std::accumulate
#include <algorithm>  // For std::min_element
#include "collisiondetection.h"
namespace HybridAStar {


class multiGoalSet3D {
 public:
    std::vector<Node3D> goals;
    Node3D virtualCenterNode3D;
    
    multiGoalSet3D();
    void addGoal(const Node3D& node);
    void addGoals(const std::set<Node3D>& nodeset);
    void addGoals(const std::vector<Node3D>& nodes);
    static multiGoalSet3D fuzzyOneNodeToSet(const CollisionDetection & collectionDection,const Node3D& node);

private:
    void updateVirtualCenterNode();
};

}  // namespace HybridAStar


#endif // MULTIGOALASTAR_H
