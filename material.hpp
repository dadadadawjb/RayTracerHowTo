#pragma once

#include "vector.hpp"
#include "light.hpp"


enum MaterialType { DIFFUSE, REFLECTION, REFRACTION, REFLECTION_AND_REFRACTION, EMISSION };

/*
Material implementation
CORE: 
- BRDF
*/
class Material {
public:
    MaterialType materialType;
    Vector3f Ka, Kd, Ks;            // diffuse only
    float specularExponent;         // diffuse only
    float ior;                      // refraction only
    Vector3f intensity;             // emission only

public:
    inline Material(MaterialType t=DIFFUSE): materialType(t) {}
    inline Material(MaterialType t, const Vector3f &_ka, const Vector3f &_kd, const Vector3f &_ks, float _se): materialType(t) {
        Ka = _ka;
        Kd = _kd;
        Ks = _ks;
        specularExponent = _se;
    }
    inline Material(MaterialType t, float _ior): materialType(t) {
        ior = _ior;
    }
    inline Material(MaterialType t, const Vector3f &_intensity): materialType(t) {
        intensity = _intensity;
    }

    inline MaterialType getType() const;

    inline Vector3f sample(const Vector3f &wi, const Vector3f &N);
    inline float pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);
    inline Vector3f brdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);
};

inline MaterialType Material::getType() const {
    return materialType;
}

inline Vector3f Material::sample(const Vector3f &wi, const Vector3f &N) {
    // uniform sample on the hemisphere, return a sampled ray
    float x_1 = getRandomFloat(), x_2 = getRandomFloat();
    float z = std::fabs(1.0f - 2.0f * x_1);
    float r = std::sqrt(1.0f - z * z), phi = 2 * MY_PI * x_2;
    Vector3f localRay(r * std::cos(phi), r * std::sin(phi), z);
    return toWorld(localRay, N);
}

inline float Material::pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N) {
    // uniform sample probability 1 / (2 * PI)
    if (dotProduct(wo, N) > 0.0f)
        return 0.5f / MY_PI;
    else
        return 0.0f;
}

inline Vector3f Material::brdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N) {
    // return the BRDF
    // not real BRDF since we only model diffuse BRDF, others are ideal by directly programming in castRay
    switch(materialType) {
        case DIFFUSE:
        {
            float cosalpha = dotProduct(N, wo);
            if (cosalpha > 0.0f) {
                Vector3f diffuse = Kd / MY_PI;
                return diffuse;
            }
            else
                return Vector3f(0.0f);
        } break;
        case REFLECTION:
        {
            // never reach here actually
            return Vector3f(1.0f);
        } break;
        case REFRACTION:
        {
            // never reach here actually
            return Vector3f(1.0f);
        } break;
        case REFLECTION_AND_REFRACTION:
        {
            // never reach here actually
            return Vector3f(1.0f);
        } break;
        case EMISSION:
        {
            // never reach here actually
            return Vector3f(1.0f);
        } break;
        default:
        {
            throw std::runtime_error("Unsupported material type.");
        }
    }
    // never reach here actually
    return Vector3f(0, 0, 0);
}
