#pragma once

#include "object.hpp"


/*
Cone implementation
CORE: 
- ray intersection with cone
- sample point on cone
*/
class Cone: public Object {
private:
    Vector3f center;
    Vector3f direction;
    float radius, height;
    float cosine;
    float area;

public:
    Cone(Vector3f _center, Vector3f _direction, float _radius, float _height, Material *_m = new Material(), std::string _name="cone")
        : Object(_m, _name), center(_center), direction(_direction), radius(_radius), height(_height) {
        area = MY_PI * radius * radius + MY_PI * radius * std::sqrt(radius * radius + height * height);
        cosine = height / std::sqrt(radius * radius + height * height);
    }

    AABB getBoundingBox() override {
        Vector3f p1 = center + height * direction;
        Vector3f p2 = center;
        return AABB(Vector3f::Min(p1 - Vector3f(radius, radius, radius), p2 - Vector3f(radius, radius, radius)), 
                    Vector3f::Max(p1 + Vector3f(radius, radius, radius), p2 + Vector3f(radius, radius, radius)));
    }
    float getArea() override {
        return area;
    }

    Intersection getIntersection(Ray ray) override {
        // circle
        float t1 = dotProduct((center - ray.origin), direction) / dotProduct(direction, ray.direction);
        Vector3f p1 = ray(t1);
        bool hit1 = (p1 - center).norm() <= radius;
        hit1 = hit1 && t1 >= 0;
        // cone
        float height2 = height * height, radius2 = radius * radius;
        Vector3f L = ray.origin - center - height * direction;
        float ll = dotProduct(L, L);
        float ld = dotProduct(L, direction);
        float lr = dotProduct(L, ray.direction);
        float rr = dotProduct(ray.direction, ray.direction);
        float rd = dotProduct(ray.direction, direction);
        float dd = dotProduct(direction, direction);
        float a1 = height2 * rr * rr;
        float b1 = height2 * 2.0 * lr;
        float c1 = height2 * ll;
        float a2 = -1.0 * (height2 + radius2) * rd * rd;
        float b2 = -1.0 * (height2 + radius2) * 2.0 * ld * rd;
        float c2 = -1.0 * (height2 + radius2) * ld * ld;
        float t21, t22;
        bool hit2 = solveQuadratic(a1 + a2, b1 + b2, c1 + c2, t21, t22);
        if (hit2) {
            if (t21 < 0)
                t21 = t22;
            if (t21 < 0)
                hit2 = false;
        }
        Vector3f p2 = ray(t21);
        float h = height - dotProduct(p2 - center - height * direction, -direction);
        hit2 = hit2 && (h > 0) && (h <= height);

        Intersection result;
        result.happened = false;
        if (hit1 || hit2) {
            result.happened = true;
            if (hit1 && (!hit2 || t1 < t21)) {
                // circle
                result.coordinate = p1;
                result.normal = -direction;
                result.distance = t1;
            } else {
                // cone
                result.coordinate = p2;
                float hPrime = height - (p2 - center - height * direction).norm() / cosine;
                result.normal = (p2 - center - hPrime * direction).normalized();
                result.distance = t21;
            }
            result.material = this->material;
            result.object = this;
        }
        return result;
    }

    void sample(Intersection &position, float &pdf) override {
        float p = getRandomFloat() * area;
        if (p < MY_PI * radius * radius) {
            // circle
            float theta = 2.0 * MY_PI * getRandomFloat();
            float r = radius * std::sqrt(getRandomFloat());
            position.coordinate = center + r * toWorld(Vector3f(cos(theta), sin(theta), 0), -direction);
            position.normal = -direction;
        } else {
            // cone
            float theta = 2.0 * MY_PI * getRandomFloat();
            float l = std::sqrt(height * height + radius * radius) * std::sqrt(getRandomFloat());
            Vector3f cPrime = center + height * direction - l * cosine * direction;
            float rPrime = radius * l / std::sqrt(height * height + radius * radius);
            position.coordinate = cPrime + rPrime * toWorld(Vector3f(cos(theta), sin(theta), 0), direction);
            position.normal = (position.coordinate - center - (height - l / cosine) * direction).normalized();
        }
        position.material = material;
        position.object = this;
        pdf = 1.0f / area;
    }
};
