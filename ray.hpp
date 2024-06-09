#pragma once

#include "vector.hpp"


/*
Ray implementation
*/
struct Ray {
    // destination = origin + t*direction
    Vector3f origin;
    Vector3f direction;
    Vector3f directionInv;
    double t;

    Ray(const Vector3f &ori, const Vector3f &dir, const double _t = 0.0): origin(ori), direction(dir), t(_t) {
        directionInv.x = (fabs(direction.x) >= epsilon) ? 1.0 / direction.x : 0;
        directionInv.y = (fabs(direction.y) >= epsilon) ? 1.0 / direction.y : 0;
        directionInv.z = (fabs(direction.z) >= epsilon) ? 1.0 / direction.z : 0;
    }

    Vector3f operator()(double t) const { return origin + direction * t; }
};
