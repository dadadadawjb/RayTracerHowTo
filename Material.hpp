#ifndef RAYTRACERHOWTO_MATERIAL_H
#define RAYTRACERHOWTO_MATERIAL_H

#include "Vector.hpp"
#include "Light.hpp"

enum MaterialType { REFLECTION_AND_REFRACTION, REFLECTION, DIFFUSE, MICROFACET };

class Material {
public:
    MaterialType material_type;
    Vector3f material_emission;
    Vector3f material_color;
    float ior;
    Vector3f Kd, Ks;
    float specularExponent;

public:
    inline Material(MaterialType t=DIFFUSE, Vector3f e=Vector3f(0, 0, 0), Vector3f c=Vector3f(1, 1, 1));

    inline MaterialType getType() const;
    inline Vector3f getEmission() const;
    inline Vector3f getColor() const;
    inline bool hasEmission() const;

    inline Vector3f sample(const Vector3f &wi, const Vector3f &N);
    // sample a ray by Material properties
    inline float pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);
    // given a ray, calculate the pdf of this ray
    inline Vector3f evaluate(const Vector3f &wi, const Vector3f &wo, const Vector3f &N);
    // given a ray, calculate the contribution of this ray

private:
    Vector3f _toWorld(const Vector3f &a, const Vector3f &N) {
        Vector3f B, C;
        if (std::fabs(N.x) > std::fabs(N.y)) {
            float invLen = 1.0f / std::sqrt(N.x * N.x + N.z * N.z);
            C = Vector3f(N.z * invLen, 0.0f, -N.x *invLen);
        } else {
            float invLen = 1.0f / std::sqrt(N.y * N.y + N.z * N.z);
            C = Vector3f(0.0f, N.z * invLen, -N.y *invLen);
        }
        B = crossProduct(C, N);
        return a.x * B + a.y * C + a.z * N;
    }

    float distributionGGX(Vector3f N, Vector3f H, float roughness) {
        float a = roughness * roughness;
        float a2 = a * a;
        float NdotH = std::max(dotProduct(N, H), 0.0f);
        float NdotH2 = NdotH * NdotH;

        float nom = a2;
        float denom = (NdotH2 * (a2 - 1.0) + 1.0);
        denom = MY_PI * denom * denom;

        return nom / std::max((double)denom, epsilon);
    }

    float geometrySchlickGGX(float NdotV, float roughness) {
        float r = (roughness + 1.0);
        float k = (r * r) / 8.0;

        float nom = NdotV;
        float denom = NdotV * (1.0 - k) + k;

        return nom / denom;
    }

    float geometrySmith(const Vector3f &N, const Vector3f &V, const Vector3f &L, float roughness) {
        float NdotV = std::max(dotProduct(N, V), 0.0f);
        float NdotL = std::max(dotProduct(N, L), 0.0f);
        float ggx2 = geometrySchlickGGX(NdotV, roughness);
        float ggx1 = geometrySchlickGGX(NdotL, roughness);

        return ggx1 * ggx2;
    }
};

inline Material::Material(MaterialType t, Vector3f e, Vector3f c) {
    material_type = t;
    material_emission = e;
    material_color = c;
}

inline MaterialType Material::getType() const {
    return material_type;
}
inline Vector3f Material::getEmission() const {
    return material_emission;
}
inline Vector3f Material::getColor() const {
    return material_color;
}
inline bool Material::hasEmission() const {
    if (material_emission.norm() > epsilon)
        return true;
    else
        return false;
}

inline Vector3f Material::sample(const Vector3f &wi, const Vector3f &N) {
    // uniform sample on the hemisphere, return a sampled ray
    float x_1 = get_random_float(), x_2 = get_random_float();
    float z = std::fabs(1.0f - 2.0f * x_1);
    float r = std::sqrt(1.0f - z * z), phi = 2 * MY_PI * x_2;
    Vector3f localRay(r * std::cos(phi), r * std::sin(phi), z);
    return _toWorld(localRay, N);
}

inline float Material::pdf(const Vector3f &wi, const Vector3f &wo, const Vector3f &N) {
    // uniform sample probability 1 / (2 * PI)
    if (dotProduct(wo, N) > 0.0f)
        return 0.5f / MY_PI;
    else
        return 0.0f;
}

inline Vector3f Material::evaluate(const Vector3f &wi, const Vector3f &wo, const Vector3f &N) {
    // only used in acceleration
    // return the BRDF
    switch(material_type) {
        case DIFFUSE:
        {
            float cosalpha = dotProduct(N, wo);
            if (cosalpha > 0.0f) {
                Vector3f diffuse = Kd / MY_PI;
                return diffuse;
            }
            else
                return Vector3f(0.0f);
            break;
        }
        case MICROFACET:
        {
            // TODO: not understand yet
            float cosalpha = dotProduct(N, wo);
            if (cosalpha > 0.0f) {
                float roughness = MICROFACET_ROUGHNESS;
                Vector3f V = -wi;
                Vector3f L = wo;
                Vector3f H = normalize(V + L);

                float D = distributionGGX(N, H, roughness);
                float G = geometrySmith(N, V, L, roughness);
                float F = fresnel(wi, N, MICROFACET_IOR);

                Vector3f nominator = D * G * F;
                float denominator = 4 * std::max(dotProduct(N, V), 0.0f) * std::max(dotProduct(N, L), 0.0f);
                Vector3f specular = nominator / std::max(double(denominator), epsilon);

                Vector3f diffuse = 1.0f / MY_PI;

                float ks_ = F;
                float kd_ = 1.0f - ks_;
                // no need for ks_ since specular with F
                return Ks * specular + kd_ * Kd * diffuse;
            } else {
                return Vector3f(0.0f);
            }
            break;
        }
        default:
        {
            throw std::runtime_error("Path tracing only support DIFFUSE and MICROFACET material type!");
        }
    }
    // never reach here actually
    return Vector3f(0, 0, 0);
}

#endif // RAYTRACERHOWTO_MATERIAL_H
