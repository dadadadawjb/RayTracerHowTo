#pragma once

#include <limits>
#include <array>

#include "vector.hpp"
#include "global.hpp"
#include "ray.hpp"


/*
Axis-Aligned Bounding Box implementation
CORE: 
- ray intersection with AABB
*/
class AABB {
public:
    Vector3f pMin, pMax;

public:
    AABB() {
        double minNum = std::numeric_limits<double>::lowest();
        double maxNum = std::numeric_limits<double>::max();
        pMax = Vector3f(minNum, minNum, minNum);
        pMin = Vector3f(maxNum, maxNum, maxNum);
    }
    AABB(const Vector3f p): pMin(p), pMax(p) {}
    AABB(const Vector3f p1, const Vector3f p2) {
        pMin = Vector3f(fmin(p1.x, p2.x), fmin(p1.y, p2.y), fmin(p1.z, p2.z));
        pMax = Vector3f(fmax(p1.x, p2.x), fmax(p1.y, p2.y), fmax(p1.z, p2.z));
    }

    const Vector3f &operator[](int i) const {
        return (i == 0) ? pMin : pMax;
    }

    Vector3f diagonal() const {
        // return the diagonal of the bounds
        return pMax - pMin;
    }
    int maxExtent() const {
        // return the coordinate-related shape
        Vector3f d = diagonal();
        if (d.x > d.y && d.x > d.z)
            return 0;
        else if (d.y > d.z)
            return 1;
        else
            return 2;
    }
    double surfaceArea() const {
        // return the area of the 6 surfaces
        Vector3f d = diagonal();
        return 2 * (d.x * d.y + d.x * d.z + d.y * d.z);
    }
    Vector3f centroid() const {
        // return the centroid of the AABB
        return 0.5 * pMin + 0.5 * pMax;
    }

    Vector3f offset(const Vector3f &p) const {
        // return the relative offset
        Vector3f o = p - pMin;
        if (pMax.x > pMin.x)
            o.x /= pMax.x - pMin.x;
        if (pMax.y > pMin.y)
            o.y /= pMax.y - pMin.y;
        if (pMax.z > pMin.z)
            o.z /= pMax.z - pMin.z;
        return o;
    }

    bool isIntersected(const Ray &ray, const std::array<int, 3> &dirIsNeg) const {
        // return whether the ray intersected with the AABB
        Vector3f tMin = (pMin - ray.origin) * ray.directionInv;
        Vector3f tMax = (pMax - ray.origin) * ray.directionInv;
        if (!dirIsNeg.at(0)) {
            float temp = tMin.x;
            tMin.x = tMax.x;
            tMax.x = temp;
        }
        if (!dirIsNeg.at(1)) {
            float temp = tMin.y;
            tMin.y = tMax.y;
            tMax.y = temp;
        }
        if (!dirIsNeg.at(2)) {
            float temp = tMin.z;
            tMin.z = tMax.z;
            tMax.z = temp;
        }

        float tEnter, tExit;
        if (ray.directionInv.x != 0 && ray.directionInv.y != 0 && ray.directionInv.z != 0) {
            tEnter = std::max(tMin.x, std::max(tMin.y, tMin.z));
            tExit = std::min(tMax.x, std::min(tMax.y, tMax.z));
        } else {
            // not beautiful
            if (ray.directionInv.x == 0) {
                if (ray.directionInv.y == 0) {
                    if (ray.directionInv.z == 0)
                        exit(-1);
                    else {
                        tEnter = tMin.z;
                        tExit = tMax.z;
                    }
                } else {
                    if (ray.directionInv.z == 0) {
                        tEnter = tMin.y;
                        tExit = tMax.y;
                    } else {
                        tEnter = std::max(tMin.y, tMin.z);
                        tExit = std::min(tMax.y, tMax.z);
                    }
                }
            } else {
                if (ray.directionInv.y == 0) {
                    if (ray.directionInv.z == 0) {
                        tEnter = tMin.x;
                        tExit = tMax.x;
                    } else {
                        tEnter = std::max(tMin.x, tMin.z);
                        tExit = std::min(tMax.x, tMax.z);
                    }
                } else {
                    if (ray.directionInv.z == 0) {
                        tEnter = std::max(tMin.x, tMin.y);
                        tExit = std::min(tMax.x, tMax.y);
                    } else 
                        exit(-2);
                }
            }
        }
        if ((tEnter <= tExit + epsilon2) && (tExit > epsilon2))
            return true;
        else
            return false;
    }
};


inline AABB unite(const AABB &b1, const AABB &b2) {
    AABB result;
    result.pMin = Vector3f::Min(b1.pMin, b2.pMin);
    result.pMax = Vector3f::Max(b1.pMax, b2.pMax);
    return result;
}
inline AABB unite(const AABB &b, const Vector3f &p) {
    AABB result;
    result.pMin = Vector3f::Min(b.pMin, p);
    result.pMax = Vector3f::Max(b.pMax, p);
    return result;
}

inline AABB intersect(const AABB &b1, const AABB &b2) {
    return AABB(Vector3f(fmax(b1.pMin.x, b2.pMin.x), fmax(b1.pMin.y, b2.pMin.y), fmax(b1.pMin.z, b2.pMin.z)),
                Vector3f(fmin(b1.pMax.x, b2.pMax.x), fmin(b1.pMax.y, b2.pMax.y), fmin(b1.pMax.z, b2.pMax.z)));
}

inline bool isOverlapped(const AABB &b1, const AABB &b2) {
    bool x = (b1.pMax.x >= b2.pMin.x) && (b1.pMin.x <= b2.pMax.x);
    bool y = (b1.pMax.y >= b2.pMin.y) && (b1.pMin.y <= b2.pMax.y);
    bool z = (b1.pMax.z >= b2.pMin.z) && (b1.pMin.z <= b2.pMax.z);
    return (x && y && z);
}

inline bool isInside(const Vector3f &p, const AABB &b) {
    return (p.x >= b.pMin.x && p.x <= b.pMax.x && p.y >= b.pMin.y &&
            p.y <= b.pMax.y && p.z >= b.pMin.z && p.z <= b.pMax.z);
}
