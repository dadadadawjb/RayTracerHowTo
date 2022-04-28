#ifndef RAYTRACERHOWTO_OBJECT_H
#define RAYTRACERHOWTO_OBJECT_H

#include "Vector.hpp"
#include "global.hpp"
#include "AABB.hpp"
#include "Ray.hpp"
#include "Intersection.hpp"

class Object {
public:
    Object() {}
    virtual ~Object() {}
    virtual bool isIntersected(const Ray &ray) const = 0;
    // return whether ray intersected with the object, never used actually
    virtual bool isIntersected(const Ray &ray, float &t_near, uint32_t &index, Vector2f &uv) const = 0;
    // return whether ray intersected with the object, only used in Meshtriangle and get the corresponding index and uv, never used actually
    virtual Intersection getIntersection(Ray ray) = 0;
    // return the intersection ray intersected with the object
    virtual AABB getBoundingBox() = 0;
    // return the bounding box of the object
    virtual float getArea() = 0;
    // return the area of the object
    virtual bool hasEmission() = 0;
    // return whether the object has emission, actually whether its material has emission
    virtual void getSurfaceProperties(const Vector3f &P, const Vector3f &I, const uint32_t &index, const Vector2f &uv, Vector3f &N, Vector2f &st) const = 0;
    // P: specific point, index: specific triangle, uv: triangle's barycentric coordinates, N: normal at specific point, st: mesh's plane coordinates
    virtual Vector3f evalDiffuseColor(const Vector2f &st) const =0;
    // st: used for determining a beautiful pattern of Meshtriangle, only used in Whitted-style ray tracing, return the object's diffuse color
    virtual void sample(Intersection &position, float &pdf) = 0;
    // only in acceleration, get a randomly chosen point at the area of the object
};

#endif // RAYTRACERHOWTO_OBJECT_H
