#ifndef RAYTRACERHOWTO_RAY_H
#define RAYTRACERHOWTO_RAY_H

#include "Vector.hpp"

struct Ray {
    // destination = origin + t*direction
    Vector3f origin;
    Vector3f direction;
    Vector3f direction_inv;
    double t;

    Ray(const Vector3f &ori, const Vector3f &dir, const double _t = 0.0): origin(ori), direction(dir), t(_t) {
        direction_inv.x = (fabs(direction.x) >= epsilon) ? 1.0 / direction.x : 0;
        direction_inv.y = (fabs(direction.y) >= epsilon) ? 1.0 / direction.y : 0;
        direction_inv.z = (fabs(direction.z) >= epsilon) ? 1.0 / direction.z : 0;
    }

    Vector3f operator()(double t) const { return origin + direction * t; }

    friend std::ostream &operator<<(std::ostream &os, const Ray &r) {
        os << "[origin:=" << r.origin << ", direction=" << r.direction << ", time=" << r.t << "]" << std::endl;
        return os;
    }
};

#endif // RAYTRACERHOWTO_RAY_H
