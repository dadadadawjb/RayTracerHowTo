#include <algorithm>
#include <cassert>
#include <vector>
#include <map>

#include "bvh.hpp"


BVH::BVH(std::vector<Object *> _objects, SplitMethod _splitMethod): splitMethod(_splitMethod) {
    if (_objects.empty())
        return;

    root = recursiveBuild(_objects);
}

BVHBuildNode *BVH::recursiveBuild(std::vector<Object *> objects) {
    BVHBuildNode *node = new BVHBuildNode();
    
    if (objects.size() == 1) {
        node->boundingBox = objects[0]->getBoundingBox();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        if (IS_PATH)
            node->area = objects[0]->getArea();
        return node;
    } else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector<Object *>{objects[0]});
        node->right = recursiveBuild(std::vector<Object *>{objects[1]});

        node->boundingBox = unite(node->left->boundingBox, node->right->boundingBox);
        if (IS_PATH)
            node->area = node->left->area + node->right->area;
        return node;
    } else {
        AABB centroidAABB;
        for (int i = 0; i < objects.size(); ++i)
            centroidAABB = unite(centroidAABB, objects[i]->getBoundingBox().centroid());
        
        std::vector<Object *> leftObjects, rightObjects;
        switch (splitMethod) {
            case SplitMethod::NAIVE:
            {
                int dim = centroidAABB.maxExtent();
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

                leftObjects = std::vector<Object *>(beginning, middling);
                rightObjects = std::vector<Object *>(middling, ending);
            } break;
            case SplitMethod::SAH:
            {
                double SN = centroidAABB.surfaceArea();
                int minCostDimension = 0;
                int minCostIndex = 0;
                double minCost = std::numeric_limits<double>::infinity();
                std::map<int, std::map<int, int>> indexMap;

                for (int dim = 0; dim < 3; ++dim) {
                    // each axis
                    std::vector<AABB> boundsBuckets;
                    std::vector<int> countBuckets;
                    std::map<int, int> objectsMap;
                    for (int i = 0; i < SAH_BUCKET_COUNT; ++i) {
                        boundsBuckets.push_back(AABB());
                        countBuckets.push_back(0);
                    }

                    for (int i = 0; i < objects.size(); ++i) {
                        float offset;
                        switch (dim) {
                            case 0: offset = centroidAABB.offset(objects[i]->getBoundingBox().centroid()).x; break;
                            case 1: offset = centroidAABB.offset(objects[i]->getBoundingBox().centroid()).y; break;
                            case 2: offset = centroidAABB.offset(objects[i]->getBoundingBox().centroid()).z; break;
                            default: break;
                        }
                        int bucketIdx = offset;
                        // combine the last region directly
                        if (bucketIdx > SAH_BUCKET_COUNT - 1)
                            bucketIdx = SAH_BUCKET_COUNT - 1;
                        
                        boundsBuckets[bucketIdx] = unite(boundsBuckets[bucketIdx], objects[i]->getBoundingBox().centroid());
                        countBuckets[bucketIdx] += 1;
                        objectsMap.insert(std::make_pair(i, bucketIdx));
                    }

                    indexMap.insert(std::make_pair(dim, objectsMap));

                    for (int i = 1; i < SAH_BUCKET_COUNT; ++i) {
                        AABB A, B;
                        int countA = 0, countB = 0;
                        for (int k = 0; k < i; ++k) {
                            A = unite(A, boundsBuckets[k]);
                            countA += countBuckets[k];
                        }
                        for (int k = i; k < SAH_BUCKET_COUNT; ++k) {
                            B = unite(B, boundsBuckets[k]);
                            countB += countBuckets[k];
                        }
                        double cost = SAH_COST_TRAVERSE + A.surfaceArea() / SN * countA * SAH_COST_INTERSECT + B.surfaceArea() / SN * countB * SAH_COST_INTERSECT;
                        if (cost < minCost) {
                            minCost = cost;
                            minCostIndex = i;
                            minCostDimension = dim;
                        }
                    }
                }

                for(int i = 0; i < objects.size(); ++i) {
                    if (indexMap[minCostDimension][i] < minCostIndex)
                        leftObjects.push_back(objects[i]);
                    else
                        rightObjects.push_back(objects[i]);
                }
            } break;
            default: break;
        }

        assert(objects.size() == (leftObjects.size() + rightObjects.size()));

        if (leftObjects.size() == 0) {
            for (int i = 0; i < rightObjects.size() / 2; ++i) {
                leftObjects.push_back(rightObjects.back());
                rightObjects.pop_back();
            }
        } else if (rightObjects.size() == 0) {
            for (int i = 0; i < leftObjects.size() / 2; ++i) {
                rightObjects.push_back(leftObjects.back());
                leftObjects.pop_back();
            }
        }

        node->left = recursiveBuild(leftObjects);
        node->right = recursiveBuild(rightObjects);

        node->boundingBox = unite(node->left->boundingBox, node->right->boundingBox);
        if (IS_PATH)
            node->area = node->left->area + node->right->area;
    }

    return node;
}

Intersection BVH::intersect(const Ray &ray) const {
    Intersection intersection;
    if (!root)
        return intersection;
    intersection = getIntersection(root, ray);
    return intersection;
}

Intersection BVH::getIntersection(BVHBuildNode *node, const Ray &ray) const {
    Intersection intersection;
    std::array<int, 3> dirIsNeg = {int(ray.direction.x>0), int(ray.direction.y>0), int(ray.direction.z>0)};
    if (!node->boundingBox.isIntersected(ray, dirIsNeg))
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
    float p = getRandomFloat() * root->area;
    getSample(root, p, position, pdf);
    pdf /= root->area;
}

void BVH::getSample(BVHBuildNode *node, float p, Intersection &position, float &pdf) {
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
