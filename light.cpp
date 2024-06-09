#include "light.hpp"


// return the reflection direction
Vector3f reflect(const Vector3f &I, const Vector3f &N) {
    // (ray is outside the object) I points in, N points out, R points out
    // (ray is inside the object) I points out, N points out, R points in
    return I - 2 * dotProduct(I, N) * N;
}

// return the refraction direction
Vector3f refract(const Vector3f &I, const Vector3f &N, const float &ior) {
    // (ray is outside the object) I points in, N points out, R points in
    // (ray is inside the object) I points out, N points out, R points out
    float cosi = clamp(-1, 1, dotProduct(I, N));
    float etai = 1, etat = ior;
    Vector3f n = N;
    if (cosi < 0) {
        // ray is outside the object
        cosi = -cosi;
    } else {
        // ray is inside the object
        std::swap(etai, etat);
        n = -N;
    }
    float eta = etai / etat;
    float k = 1 - eta * eta * (1 - cosi * cosi);
    return (k < 0) ? (I - 2 * dotProduct(I, n) * n) : (eta * I + (eta * cosi - sqrtf(k)) * n);
}

// return the amount of light reflected
float fresnel(const Vector3f &I, const Vector3f &N, const float &ior) {
    float cosi = clamp(-1, 1, dotProduct(I, N));
    float etai = 1, etat = ior;
    if (cosi > 0)
        std::swap(etai, etat);
    float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
    if (sint >= 1) {
        return 1;
    }
    else {
        float cost = sqrtf(std::max(0.f, 1 - sint * sint));
        cosi = fabsf(cosi);
        float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
        float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
        return (Rs * Rs + Rp * Rp) / 2;
    }
}
