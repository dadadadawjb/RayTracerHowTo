#ifndef RAYTRACERHOWTO_SPHERE_H
#define RAYTRACERHOWTO_SPHERE_H

#include "Vector.hpp"
#include "Object.hpp"
#include "AABB.hpp"
#include "Material.hpp"

class Sphere: public Object {
private:
    Vector3f center;
    float radius, radius2;
    Material *material;
    float area;

public:
    Sphere(const Vector3f &c, const float &r, Material *m = new Material()): center(c), radius(r), radius2(r * r), material(m), area(4 * MY_PI *r *r) {}
    
    bool isIntersected(const Ray &ray) const override {
        // never used actually
        Vector3f L = ray.origin - center;
        float a = dotProduct(ray.direction, ray.direction);
        float b = 2 * dotProduct(ray.direction, L);
        float c = dotProduct(L, L) - radius2;
        float t0, t1;
        float area = 4 * MY_PI * radius2;
        if (!solveQuadratic(a, b, c, t0, t1))
            return false;
        if (t0 < 0)
            t0 = t1;
        if (t0 < 0)
            return false;
        return true;
    }
    bool isIntersected(const Ray &ray, float &t_near, uint32_t &index, Vector2f &) const override {
        // never used actually
        Vector3f L = ray.origin - center;
        float a = dotProduct(ray.direction, ray.direction);
        float b = 2 * dotProduct(ray.direction, L);
        float c = dotProduct(L, L) - radius2;
        float t0, t1;
        if (!solveQuadratic(a, b, c, t0, t1))
            return false;
        if (t0 < 0)
            t0 = t1;
        if (t0 < 0)
            return false;
        t_near = t0;

        return true;
    }
    Intersection getIntersection(Ray ray) override {
        Intersection result;
        result.happened = false;
        Vector3f L = ray.origin - center;
        float a = dotProduct(ray.direction, ray.direction);
        float b = 2 * dotProduct(ray.direction, L);
        float c = dotProduct(L, L) - radius2;
        float t0, t1;
        if (!solveQuadratic(a, b, c, t0, t1))
            return result;
        if (t0 < 0)
            t0 = t1;
        if (t0 < 0)
            return result;
        result.happened = true;

        result.coordinates = Vector3f(ray.origin + ray.direction * t0);
        result.normal = normalize(Vector3f(result.coordinates - center));
        result.material = this->material;
        result.object = this;
        result.distance = t0;
        return result;

    }

    AABB getBoundingBox() override {
        return AABB(Vector3f(center.x-radius, center.y-radius, center.z-radius),
                    Vector3f(center.x+radius, center.y+radius, center.z+radius));
    }
    float getArea() override {
        return area;
    }
    bool hasEmission() override {
        return material->hasEmission();
    }

    void getSurfaceProperties(const Vector3f &P, const Vector3f &, const uint32_t &, 
                                const Vector2f &, Vector3f &N, Vector2f &) const override {
        N = normalize(P - center);
    }
    Vector3f evalDiffuseColor(const Vector2f &) const override {
        return material->getColor();
    }
    
    void sample(Intersection &position, float &pdf) override {
        float theta = 2.0 * MY_PI * get_random_float(), phi = MY_PI * get_random_float();
        Vector3f dir(std::cos(phi), std::sin(phi)*std::cos(theta), std::sin(phi)*std::sin(theta));
        position.coordinates = center + radius * dir;
        position.normal = dir;
        position.material = material;
        position.object = this;
        // position.happened;
        // position.distance;
        pdf = 1.0f / area;
    }
};

#endif // RAYTRACERHOWTO_SPHERE_H
