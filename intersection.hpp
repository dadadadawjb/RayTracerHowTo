#pragma once

#include "vector.hpp"
#include "material.hpp"
#include "object.hpp"

class Object;
class Material;


/*
Intersection implementation
*/
struct Intersection {
    bool happened;                  // only valid under intersection meaning, determine whether intersected
    Vector3f coordinate;            // the coordinate of that point
    Vector3f normal;                // the normal of that point at that object
    double distance;                // only valid under intersection meaning, determine the ray time if intersected
    Object *object;                 // the object the point belonging to
    Material *material;             // the material of the object the point belonging to
    Vector2f uv;                    // only valid when the object is mesh/triangle, determine the barycentric coordinates
    uint32_t index;                 // only valid when the object is mesh, determine the index of the triangle

    Intersection() {
        happened = false;
        coordinate = Vector3f();
        normal = Vector3f();
        distance = std::numeric_limits<double>::max();
        object = nullptr;
        material = nullptr;
        uv = Vector2f();
        index = 0;
    }
};
