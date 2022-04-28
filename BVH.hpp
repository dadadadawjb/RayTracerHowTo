#ifndef RAYTRACERHOWTO_BVH_H
#define RAYTRACERHOWTO_BVH_H

#include <vector>
#include <atomic>
#include <memory>
#include <ctime>

#include "Vector.hpp"
#include "global.hpp"
#include "AABB.hpp"
#include "Object.hpp"
#include "Ray.hpp"
#include "Intersection.hpp"

struct BVHBuildNode {
public:
    AABB bounding_box;      // the axis aligned bounding box
    Object *object;         // only leaf node has, and only one
    float area;             // area of all objects in it, needed by path tracing
    BVHBuildNode *left;
    BVHBuildNode *right;

public:
    BVHBuildNode() {
        bounding_box = AABB(Vector3f(0, 0, 0), Vector3f(0, 0, 0));
        object = nullptr;
        if (PATH)
            area = 0;
        left = nullptr;
        right = nullptr;
    }
};

class BVH {
public:
    enum class SplitMethod { NAIVE, SAH };

private:
    const SplitMethod splitMethod;
    BVHBuildNode *root;

public:
    BVH(std::vector<Object *> _objects, SplitMethod _splitMethod = SplitMethod::NAIVE);
    // set up BVH acceleration
    // only 1 object in one node
    ~BVH();

    Intersection intersect(const Ray &ray) const;
    // return the intersection

    void sample(Intersection &position, float &pdf);
    // needed by path tracing

private:
    BVHBuildNode *recursiveBuild(std::vector<Object *> objects);
    // private recursive set up

    Intersection getIntersection(BVHBuildNode *node, const Ray &ray) const;
    // private recursive return the intersection

    void getSample(BVHBuildNode *node, float p, Intersection &position, float &pdf);
    // needed by path tracing
};

#endif // RAYTRACERHOWTO_BVH_H
