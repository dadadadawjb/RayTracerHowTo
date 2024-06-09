#pragma once

#include "object.hpp"


/*
Cylinder implementation
CORE: 
- ray intersection with cylinder
- sample point on cylinder
*/
class Cylinder: public Object {
private:
    Vector3f center;
    Vector3f direction;
    float radius, height;
    float area;

public:
    Cylinder(Vector3f _center, Vector3f _direction, float _radius, float _height, Material *_m = new Material(), std::string _name="cylinder")
        : Object(_m, _name), center(_center), direction(_direction), radius(_radius), height(_height) {
        area = 2 * MY_PI * radius * radius + 2 * MY_PI * radius * height;
    }

    AABB getBoundingBox() override {
        Vector3f p1 = center + height / 2.0 * direction;
        Vector3f p2 = center - height / 2.0 * direction;
        return AABB(Vector3f::Min(p1 - Vector3f(radius, radius, radius), p2 - Vector3f(radius, radius, radius)), 
                    Vector3f::Max(p1 + Vector3f(radius, radius, radius), p2 + Vector3f(radius, radius, radius)));
    }
    float getArea() override {
        return area;
    }

    Intersection getIntersection(Ray ray) override {
        // first circle
        float t1 = dotProduct((center + height / 2.0 * direction - ray.origin), direction) 
                    / dotProduct(direction, ray.direction);
        Vector3f p1 = ray(t1);
        bool hit1 = (p1 - center - height / 2.0 * direction).norm() <= radius;
        hit1 = hit1 && t1 >= 0;
        // second circle
        float t2 = dotProduct((center - height / 2.0 * direction - ray.origin), direction) 
                    / dotProduct(direction, ray.direction);
        Vector3f p2 = ray(t2);
        bool hit2 = (p2 - center + height / 2.0 * direction).norm() <= radius;
        hit2 = hit2 && t2 >= 0;
        // cylinder
        Vector3f L = ray.origin - center;
        float ll = dotProduct(L, L);
        float ld = dotProduct(L, direction);
        float lr = dotProduct(L, ray.direction);
        float rr = dotProduct(ray.direction, ray.direction);
        float rd = dotProduct(ray.direction, direction);
        float dd = dotProduct(direction, direction);
        float a1 = rr;
        float b1 = 2.0 * lr;
        float c1 = ll - radius * radius;
        float a23 = (dd - 2.0) * rd * rd;
        float b23 = (dd - 2.0) * 2.0 * rd * ld;
        float c23 = (dd - 2.0) * ld * ld;
        float t31, t32;
        bool hit3 = solveQuadratic(a1 + a23, b1 + b23, c1 + c23, t31, t32);
        if (hit3) {
            if (t31 < 0)
                t31 = t32;
            if (t31 < 0)
                hit3 = false;
        }
        Vector3f p3 = ray(t31);
        hit3 = hit3 && (fabs(dotProduct(p3 - center, direction)) < height / 2.0);

        Intersection result;
        result.happened = false;
        if (hit1 || hit2 || hit3) {
            result.happened = true;
            if (hit1 && (!hit2 || t1 < t2) && (!hit3 || t1 < t31)) {
                // first circle
                result.coordinate = p1;
                result.normal = direction;
                result.distance = t1;
            } else if (hit2 && (!hit1 || t2 < t1) && (!hit3 || t2 < t31)) {
                // second circle
                result.coordinate = p2;
                result.normal = -direction;
                result.distance = t2;
            } else {
                // cylinder
                result.coordinate = p3;
                result.normal = (p3 - center - dotProduct(p3 - center, direction) * direction).normalized();
                result.distance = t31;
            }
            result.material = this->material;
            result.object = this;
        }
        return result;
    }

    void sample(Intersection &position, float &pdf) override {
        float p = getRandomFloat() * area;
        if (p < MY_PI * radius * radius) {
            // first circle
            float theta = 2.0 * MY_PI * getRandomFloat();
            float r = radius * std::sqrt(getRandomFloat());
            position.coordinate = center + height / 2.0 * direction + r * toWorld(Vector3f(cos(theta), sin(theta), 0), direction);
            position.normal = direction;
        } else if (p < 2 * MY_PI * radius * radius) {
            // second circle
            float theta = 2.0 * MY_PI * getRandomFloat();
            float r = radius * std::sqrt(getRandomFloat());
            position.coordinate = center - height / 2.0 * direction + r * toWorld(Vector3f(cos(theta), sin(theta), 0), -direction);
            position.normal = -direction;
        } else {
            // cylinder
            float theta = 2.0 * MY_PI * getRandomFloat();
            float h = height * getRandomFloat() - height / 2.0;
            position.coordinate = center + h * direction + radius * toWorld(Vector3f(cos(theta), sin(theta), 0), direction);
            position.normal = toWorld(Vector3f(cos(theta), sin(theta), 0), direction);
        }
        position.material = material;
        position.object = this;
        pdf = 1.0f / area;
    }
};
