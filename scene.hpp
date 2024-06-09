#pragma once

#include <vector>

#include "vector.hpp"
#include "global.hpp"
#include "object.hpp"
#include "light.hpp"
#include "bvh.hpp"
#include "ray.hpp"
#include "intersection.hpp"


/*
Scene implementation
CORE: 
- Ray Tracing (Whitted-style or Path Tracing)
*/
class Scene {
private:
    Vector3f backgroundColor = Vector3f(BACKGROUND_R, BACKGROUND_G, BACKGROUND_B);
    Vector3f ambientIntensity = Vector3f(AMBIENT_R, AMBIENT_G, AMBIENT_B);

    std::vector<Object *> objects;
    std::vector<Light *> lights;

    BVH *bvh;               // only needed by BVH acceleration

public:
    Scene(): bvh(nullptr) {}

    void add(Object *object) { objects.push_back(object); }
    void add(Light *light) { lights.push_back(std::move(light)); }

    const std::vector<Object *> &getObjects() const { return objects; }
    const std::vector<Light *> &getLights() const { return lights; }

    void buildBVH();        // only needed by BVH acceleration
    
    Vector3f castRay(const Ray &ray, int depth) const;

private:
    Intersection intersect(const Ray &ray) const;

    void sampleLight(Intersection &position, float &pdf) const; // only needed by path tracing
};
