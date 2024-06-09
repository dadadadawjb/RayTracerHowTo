#pragma once

#include "vector.hpp"
#include "global.hpp"
#include "aabb.hpp"
#include "bvh.hpp"
#include "ray.hpp"
#include "intersection.hpp"
#include "material.hpp"


class Object {
public:
    Material *material = nullptr;
    std::string name;

    Object() {}
    Object(Material *m, std::string n): material(m), name(n) {}
    virtual ~Object() {}

    virtual AABB getBoundingBox() = 0;
    virtual float getArea() = 0;
    virtual Intersection getIntersection(Ray ray) = 0;
    virtual void sample(Intersection &position, float &pdf) = 0;    // only needed by path tracing
};
