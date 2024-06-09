#pragma once

#include <vector>
#include <atomic>
#include <memory>
#include <ctime>

#include "vector.hpp"
#include "global.hpp"
#include "aabb.hpp"
#include "object.hpp"
#include "ray.hpp"
#include "intersection.hpp"


struct BVHBuildNode {
public:
    AABB boundingBox;
    Object *object;
    float area;             // only needed by path tracing
    BVHBuildNode *left;
    BVHBuildNode *right;

public:
    BVHBuildNode() {
        boundingBox = AABB(Vector3f(0, 0, 0), Vector3f(0, 0, 0));
        object = nullptr;
        area = 0;
        left = nullptr;
        right = nullptr;
    }
};

/*
Bounding Volume Hierarchy implementation
CORE: 
- BVH build (with Surface Area Heuristic)
- ray intersection with BVH
- sample point on BVH

NOTE: 
- MeshTriangle is also accelerated by BVH
*/
class BVH {
public:
    enum class SplitMethod { NAIVE, SAH };

private:
    const SplitMethod splitMethod;
    BVHBuildNode *root;

public:
    BVH(std::vector<Object *> _objects, SplitMethod _splitMethod = SplitMethod::NAIVE);
    ~BVH() {}

    Intersection intersect(const Ray &ray) const;

    void sample(Intersection &position, float &pdf);        // only needed by path tracing

private:
    BVHBuildNode *recursiveBuild(std::vector<Object *> objects);

    Intersection getIntersection(BVHBuildNode *node, const Ray &ray) const;

    void getSample(BVHBuildNode *node, float p, Intersection &position, float &pdf);    // only needed by path tracing
};
