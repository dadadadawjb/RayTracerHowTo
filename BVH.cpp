#include <algorithm>
#include <cassert>
#include <vector>
#include <map>

#include "Vector.hpp"
#include "BVH.hpp"
#include "Object.hpp"
#include "AABB.hpp"

BVH::BVH(std::vector<Object *> _objects, SplitMethod _splitMethod): splitMethod(_splitMethod) {
    // public build BVH
    if (_objects.empty())
        return;

    root = recursiveBuild(_objects);
}

BVHBuildNode *BVH::recursiveBuild(std::vector<Object *> objects) {
    // private recursively build BVH
    BVHBuildNode *node = new BVHBuildNode();
    
    if (objects.size() == 1) {
        node->bounding_box = objects[0]->getBoundingBox();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        if (PATH)
            node->area = objects[0]->getArea();
        return node;
    } else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector<Object *>{objects[0]});
        node->right = recursiveBuild(std::vector<Object *>{objects[1]});

        node->bounding_box = Union(node->left->bounding_box, node->right->bounding_box);
        if (PATH)
            node->area = node->left->area + node->right->area;
        return node;
    } else {
        AABB centroid_AABB;
        for (int i = 0; i < objects.size(); ++i)
            centroid_AABB = Union(centroid_AABB, objects[i]->getBoundingBox().centroid());
        
        std::vector<Object *> left_objects, right_objects;
        switch (splitMethod) {
            case SplitMethod::NAIVE:
            {
                int dim = centroid_AABB.maxExtent();
                switch (dim) {
                case 0:
                    std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                        return f1->getBoundingBox().centroid().x < f2->getBoundingBox().centroid().x;
                    });
                    break;
                case 1:
                    std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                        return f1->getBoundingBox().centroid().y < f2->getBoundingBox().centroid().y;
                    });
                    break;
                case 2:
                    std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                        return f1->getBoundingBox().centroid().z < f2->getBoundingBox().centroid().z;
                    });
                    break;
                }

                auto beginning = objects.begin();
                auto middling = objects.begin() + (objects.size() / 2);
                auto ending = objects.end();

                left_objects = std::vector<Object *>(beginning, middling);
                right_objects = std::vector<Object *>(middling, ending);
            } break;
            case SplitMethod::SAH:
            {
                double SN = centroid_AABB.surfaceArea();
                int minCostDimension = 0;
                int minCostIndex = 0;
                double minCost = std::numeric_limits<double>::infinity();
                std::map<int, std::map<int, int>> indexMap;

                for (int dim = 0; dim < 3; ++dim) {
                    // each axis
                    constexpr int bucketCount = 12;
                    constexpr double C_isect = 1.0;
                    constexpr double C_trav = 0.125;
                    std::vector<AABB> boundsBuckets;
                    std::vector<int> countBuckets;
                    std::map<int, int> objectsMap;
                    for (int i = 0; i < bucketCount; ++i) {
                        boundsBuckets.push_back(AABB());
                        countBuckets.push_back(0);
                    }

                    for (int i = 0; i < objects.size(); ++i) {
                        float offset;
                        switch (dim) {
                            case 0: offset = centroid_AABB.offset(objects[i]->getBoundingBox().centroid()).x; break;
                            case 1: offset = centroid_AABB.offset(objects[i]->getBoundingBox().centroid()).y; break;
                            case 2: offset = centroid_AABB.offset(objects[i]->getBoundingBox().centroid()).z; break;
                            default: break;
                        }
                        int bucketIdx = offset;
                        // combine the last region directly
                        if (bucketIdx > bucketCount - 1)
                            bucketIdx = bucketCount - 1;
                        
                        boundsBuckets[bucketIdx] = Union(boundsBuckets[bucketIdx], objects[i]->getBoundingBox().centroid());
                        countBuckets[bucketIdx] += 1;
                        objectsMap.insert(std::make_pair(i, bucketIdx));
                    }

                    indexMap.insert(std::make_pair(dim, objectsMap));

                    for (int i = 1; i < bucketCount; ++i) {
                        AABB A, B;
                        int countA = 0, countB = 0;
                        for (int k = 0; k < i; ++k) {
                            A = Union(A, boundsBuckets[k]);
                            countA += countBuckets[k];
                        }
                        for (int k = i; k < bucketCount; ++k) {
                            B = Union(B, boundsBuckets[k]);
                            countB += countBuckets[k];
                        }
                        double cost = C_trav + A.surfaceArea() / SN * countA * C_isect + B.surfaceArea() / SN * countB * C_isect;
                        if (cost < minCost) {
                            minCost = cost;
                            minCostIndex = i;
                            minCostDimension = dim;
                        }
                    }
                }

                for(int i = 0; i < objects.size(); ++i) {
                    if (indexMap[minCostDimension][i] < minCostIndex)
                        left_objects.push_back(objects[i]);
                    else
                        right_objects.push_back(objects[i]);
                }
            } break;
            default: break;
        }

        assert(objects.size() == (left_objects.size() + right_objects.size()));

        if (left_objects.size() == 0) {
            for (int i = 0; i < right_objects.size() / 2; ++i) {
                left_objects.push_back(right_objects.back());
                right_objects.pop_back();
            }
        } else if (right_objects.size() == 0) {
            for (int i = 0; i < left_objects.size() / 2; ++i) {
                right_objects.push_back(left_objects.back());
                left_objects.pop_back();
            }
        }

        node->left = recursiveBuild(left_objects);
        node->right = recursiveBuild(right_objects);

        node->bounding_box = Union(node->left->bounding_box, node->right->bounding_box);
        if (PATH)
            node->area = node->left->area + node->right->area;
    }

    return node;
}

Intersection BVH::intersect(const Ray &ray) const {
    // public get the ray intersection with BVH
    Intersection intersection;
    if (!root)
        return intersection;
    intersection = getIntersection(root, ray);
    return intersection;
}

Intersection BVH::getIntersection(BVHBuildNode *node, const Ray &ray) const {
    // private recursively get the ray intersection with BVH
    Intersection intersection;
    std::array<int, 3> dirIsNeg = {int(ray.direction.x>0), int(ray.direction.y>0), int(ray.direction.z>0)};
    if (!node->bounding_box.isIntersected(ray, dirIsNeg))
        return intersection;

    // leaf node
    if (node->left == nullptr && node->right == nullptr)
        return node->object->getIntersection(ray);
    
    Intersection hit1 = getIntersection(node->left, ray);
    Intersection hit2 = getIntersection(node->right, ray);
    if (hit1.happened == false)
        return hit2;
    else if (hit2.happened == false)
        return hit1;
    else
        return (hit1.distance < hit2.distance) ? hit1 : hit2;
}

void BVH::sample(Intersection &position, float &pdf) {
    // public sample a point at any object of BVH
    float p = std::sqrt(get_random_float()) * root->area;
    getSample(root, p, position, pdf);
    pdf /= root->area;
}

void BVH::getSample(BVHBuildNode *node, float p, Intersection &position, float &pdf) {
    // private recursively sample a point at any object of BVH
    if (node->left == nullptr || node->right == nullptr) {
        node->object->sample(position, pdf);
        pdf *= node->area;
        return;
    }
    if (p < node->left->area)
        getSample(node->left, p, position, pdf);
    else
        getSample(node->right, p - node->left->area, position, pdf);
}