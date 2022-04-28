#ifndef RAYTRACERHOWTO_INTERSECTION_H
#define RAYTRACERHOWTO_INTERSECTION_H

#include "Vector.hpp"
#include "Material.hpp"

class Object;
class Sphere;

// present a point at some object with additional information
struct Intersection {
    bool happened;                  // only valid under intersection meaning, determine whether intersected
    Vector3f coordinates;           // the coordinates of that point
    Vector3f normal;                // the normal of that point at that object
    double distance;                // only valid under intersection meaning, determine the ray time if intersected
    Object *object;                 // the object the point belonging to
    Material *material;             // the material of the object the point belonging to
    Vector2f uv;                    // only valid when the object is triangle, determine the barycentric coordinates

    Intersection() {
        happened = false;
        coordinates = Vector3f();
        normal = Vector3f();
        distance = std::numeric_limits<double>::max();
        object = nullptr;
        material = nullptr;
    }
};

#endif // RAYTRACERHOWTO_INTERSECTION_H
