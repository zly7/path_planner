/*!
   \file
   \brief This is a collection of helper functions that are used throughout the project.

*/
#ifndef HELPER
#define HELPER

#include <cmath>
#include <algorithm>
#include <node2d.h>
#include "constants.h"
namespace HybridAStar {
/*!
    \brief The namespace that wraps helper.h
    \namespace Helper
*/
namespace Helper {

/*!
   \fn static inline float normalizeHeading(float t)
   \brief Normalizes a heading given in degrees to (0,360]
   \param t heading in degrees
*/
static inline float normalizeHeading(float t) {
  if ((int)t <= 0 || (int)t >= 360) {
    if (t < -0.1) {
      t += 360.f;
    } else if ((int)t >= 360) {
      t -= 360.f;
    } else {
      t =  0;
    }
  }

  return t;
}

/*!
   \fn float normalizeHeadingRad(float t)
   \brief Normalizes a heading given in rad to (0,2PI]
   \param t heading in rad
*/
static inline float normalizeHeadingRad(float t) {
  if (t < 0) {
    t = t - 2.f * M_PI * (int)(t / (2.f * M_PI));
    return 2.f * M_PI + t;
  }

  return t - 2.f * M_PI * (int)(t / (2.f * M_PI));
}

/*!
   \fn float toDeg(float t)
   \brief Converts and normalizes a heading given in rad to deg
   \param t heading in deg
*/
static inline float toDeg(float t) {
  return normalizeHeadingRad(t) * 180.f / M_PI ;
}

/*!
   \fn float toRad(float t)
   \brief Converts and normalizes a heading given in deg to rad
   \param t heading in rad
*/
static inline float toRad(float t) {
  return normalizeHeadingRad(t / 180.f * M_PI);
}

/*!
   \fn float clamp(float n, float lower, float upper)
   \brief Clamps a number between a lower and an upper bound
   \param t heading in rad
*/
static inline float clamp(float n, float lower, float upper) {
  return std::max(lower, std::min(n, upper));
}

double crossProduct(Node2D* a, Node2D* b, Node2D* c) {
    double y1 = b->getFloatY() - a->getFloatY();
    double y2 = c->getFloatY() - a->getFloatY();
    double x1 = b->getFloatX() - a->getFloatX();
    double x2 = c->getFloatX() - a->getFloatX();
    return x1 * y2 - x2 * y1;
}

// 检查点C是否在线段AB的线性延长线上
bool onLine(Point a, Point b, Point c) {
    return std::min(a.x, b.x) <= c.x && c.x <= std::max(a.x, b.x) &&
           std::min(a.y, b.y) <= c.y && c.y <= std::max(a.y, b.y);
}

// 检查两线段是否相交
bool isIntersect(Point a, Point b, Point c, Point d) {
    double d1 = crossProduct(a, b, c);
    double d2 = crossProduct(a, b, d);
    double d3 = crossProduct(c, d, a);
    double d4 = crossProduct(c, d, b);

    // 如果每个线段的两个点分别位于另一个线段的两侧，则线段相交
    if (d1 * d2 < 0 && d3 * d4 < 0) return true;

    // 特殊情况：线段共线且重叠
    if (d1 == 0 && onLine(a, b, c)) return true;
    if (d2 == 0 && onLine(a, b, d)) return true;
    if (d3 == 0 && onLine(c, d, a)) return true;
    if (d4 == 0 && onLine(c, d, b)) return true;

    return false;
}

}
}

#endif // HELPER

