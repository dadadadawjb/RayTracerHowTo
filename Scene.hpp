#pragma once

#include <vector>

#include "Vector.hpp"
#include "global.hpp"
#include "Object.hpp"
#include "Light.hpp"
#include "BVH.hpp"
#include "Ray.hpp"
#include "Intersection.hpp"

class Scene {
public:
    int width = WIDTH1;
    int height = HEIGHT1;
    double fov = FOV1;
    Vector3f backgroundColor = Vector3f(BACKGROUND_R, BACKGROUND_G, BACKGROUND_B);
    int maxDepth = DEPTH1;
    
    // only in path tracing
    float RussianRoulette = PATH_RR;

private:
    std::vector<Object *> objects;
    std::vector<std::unique_ptr<Light>> lights;

    // only in acceleration
    BVH *bvh;

public:
    Scene(int w, int h): width(w), height(h) {}

    // only in acceleration
    void buildBVH();

    void Add(Object *object) { objects.push_back(object); }
    void Add(std::unique_ptr<Light> light) { lights.push_back(std::move(light)); }

    const std::vector<Object *> &get_objects() const { return objects; }
    const std::vector<std::unique_ptr<Light>> &get_lights() const { return lights; }

    Intersection intersect(const Ray &ray) const;
    // return the ray intersection with the scene
    
    Vector3f castRay(const Ray &ray, int depth) const;
    // return the irradiance along this ray direction

private:
    void sampleLight(Intersection &position, float &pdf) const;
    // only in path tracing, sample among all the objects with material having emission and get randomly point
};