#pragma once

#include "vector.hpp"
#include "global.hpp"


/*
Point Light implementation
CORE: 
- reflection
- refraction
- fresnel

NOTE: 
- the area light is presented as a object with material with emission
*/
class Light {
public:
    Vector3f position;
    Vector3f intensity;

public:
    Light(const Vector3f &p, const Vector3f &i): position(p), intensity(i) {}
    virtual ~Light() = default;
};

// light-related methods
Vector3f reflect(const Vector3f &I, const Vector3f &N);
Vector3f refract(const Vector3f &I, const Vector3f &N, const float &ior);
float fresnel(const Vector3f &I, const Vector3f &N, const float &ior);
