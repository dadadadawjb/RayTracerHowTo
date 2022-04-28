#ifndef RAYTRACERHOWTO_AABB_H
#define RAYTRACERHOWTO_AABB_H

#include <limits>
#include <array>

#include "Vector.hpp"
#include "global.hpp"
#include "Ray.hpp"

class AABB {
public:
    Vector3f pMin, pMax;    // two points to specify the bounding box

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
        Vector3f t_min = (pMin - ray.origin) * ray.direction_inv;
        Vector3f t_max = (pMax - ray.origin) * ray.direction_inv;
        if (!dirIsNeg.at(0)) {
            float temp = t_min.x;
            t_min.x = t_max.x;
            t_max.x = temp;
        }
        if (!dirIsNeg.at(1)) {
            float temp = t_min.y;
            t_min.y = t_max.y;
            t_max.y = temp;
        }
        if (!dirIsNeg.at(2)) {
            float temp = t_min.z;
            t_min.z = t_max.z;
            t_max.z = temp;
        }

        float t_enter, t_exit;
        if (ray.direction_inv.x != 0 && ray.direction_inv.y != 0 && ray.direction_inv.z != 0) {
            t_enter = std::max(t_min.x, std::max(t_min.y, t_min.z));
            t_exit = std::min(t_max.x, std::min(t_max.y, t_max.z));
        } else {
            // not beautiful
            if (ray.direction_inv.x == 0) {
                if (ray.direction_inv.y == 0) {
                    if (ray.direction_inv.z == 0)
                        exit(-1);
                    else {
                        t_enter = t_min.z;
                        t_exit = t_max.z;
                    }
                } else {
                    if (ray.direction_inv.z == 0) {
                        t_enter = t_min.y;
                        t_exit = t_max.y;
                    } else {
                        t_enter = std::max(t_min.y, t_min.z);
                        t_exit = std::min(t_max.y, t_max.z);
                    }
                }
            } else {
                if (ray.direction_inv.y == 0) {
                    if (ray.direction_inv.z == 0) {
                        t_enter = t_min.x;
                        t_exit = t_max.x;
                    } else {
                        t_enter = std::max(t_min.x, t_min.z);
                        t_exit = std::min(t_max.x, t_max.z);
                    }
                } else {
                    if (ray.direction_inv.z == 0) {
                        t_enter = std::max(t_min.x, t_min.y);
                        t_exit = std::min(t_max.x, t_max.y);
                    } else 
                        exit(-2);
                }
            }
        }
        if ((t_enter < t_exit || fabs(t_enter - t_exit) < epsilon) && (t_exit > 0 || fabs(t_exit - 0) < epsilon))
            return true;
        else
            return false;
    }
};

inline AABB Union(const AABB &b1, const AABB &b2) {
    AABB result;
    result.pMin = Vector3f::Min(b1.pMin, b2.pMin);
    result.pMax = Vector3f::Max(b1.pMax, b2.pMax);
    return result;
}
inline AABB Union(const AABB &b, const Vector3f &p) {
    AABB result;
    result.pMin = Vector3f::Min(b.pMin, p);
    result.pMax = Vector3f::Max(b.pMax, p);
    return result;
}

// inline AABB Intersection(const AABB &b1, const AABB &b2) {
//     return AABB(Vector3f(fmax(b1.pMin.x, b2.pMin.x), fmax(b1.pMin.y, b2.pMin.y), fmax(b1.pMin.z, b2.pMin.z)),
//                 Vector3f(fmin(b1.pMax.x, b2.pMax.x), fmin(b1.pMax.y, b2.pMax.y), fmin(b1.pMax.z, b2.pMax.z)));
// }

// bool isOverlapped(const AABB &b1, const AABB &b2) {
//     bool x = (b1.pMax.x >= b2.pMin.x) && (b1.pMin.x <= b2.pMax.x);
//     bool y = (b1.pMax.y >= b2.pMin.y) && (b1.pMin.y <= b2.pMax.y);
//     bool z = (b1.pMax.z >= b2.pMin.z) && (b1.pMin.z <= b2.pMax.z);
//     return (x && y && z);
// }

// bool isInside(const Vector3f &p, const AABB &b) {
//     return (p.x >= b.pMin.x && p.x <= b.pMax.x && p.y >= b.pMin.y &&
//             p.y <= b.pMax.y && p.z >= b.pMin.z && p.z <= b.pMax.z);
// }

#endif // RAYTRACERHOWTO_AABB_H
