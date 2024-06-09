#pragma onace

#include "object.hpp"


/*
Sphere implementation
CORE: 
- ray intersection with sphere
- sample point on sphere
*/
class Sphere: public Object {
private:
    Vector3f center;
    float radius, radius2;
    float area;

public:
    Sphere(const Vector3f &c, const float &r, Material *m = new Material(), std::string _name="sphere")
        : Object(m, _name), center(c), radius(r), radius2(r * r), area(4 * MY_PI *r *r) {}
    
    AABB getBoundingBox() override {
        return AABB(Vector3f(center.x-radius, center.y-radius, center.z-radius),
                    Vector3f(center.x+radius, center.y+radius, center.z+radius));
    }
    float getArea() override {
        return area;
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
        result.coordinate = Vector3f(ray.origin + ray.direction * t0);
        result.normal = normalize(Vector3f(result.coordinate - center));
        result.material = this->material;
        result.object = this;
        result.distance = t0;
        return result;
    }
    
    void sample(Intersection &position, float &pdf) override {
        float theta = 2.0 * MY_PI * getRandomFloat(), phi = MY_PI * getRandomFloat();
        Vector3f dir(std::cos(phi), std::sin(phi)*std::cos(theta), std::sin(phi)*std::sin(theta));
        position.coordinate = center + radius * dir;
        position.normal = dir;
        position.material = material;
        position.object = this;
        pdf = 1.0f / area;
    }
};
