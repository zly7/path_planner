#ifndef DIRECTIONVECTOR_H
#define DIRECTIONVECTOR_H

#include <cmath>
#include <node3d.h>

namespace HybridAStar {
  class directionVector {
  public:
    float x;
    float y;

    directionVector(float x, float y);
    directionVector();
    void normalize();
    directionVector getReverseVector();
    static directionVector getUnitVectorFromNode3D(const Node3D* node);
  };
}

#endif // DIRECTIONVECTOR_H
