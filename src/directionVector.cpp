#include "directionVector.h"

using namespace HybridAStar;

directionVector::directionVector(float x, float y) : x(x), y(y) {}

directionVector::directionVector() : x(0), y(0) {}

void directionVector::normalize() {
    float length = sqrt(x * x + y * y);
    if (length != 0) {
        x /= length;
        y /= length;
    }
}

directionVector directionVector::getReverseVector() {
    return directionVector(-x, -y);
}

directionVector directionVector::getUnitVectorFromNode3D(const Node3D* node) {
    return directionVector(cos(node->getT()), sin(node->getT()));
}
