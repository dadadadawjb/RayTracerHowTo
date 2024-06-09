#include "scene.hpp"


void Scene::buildBVH() {
    if (!IS_SAH)
        bvh = new BVH(objects, BVH::SplitMethod::NAIVE);
    else
        bvh = new BVH(objects, BVH::SplitMethod::SAH);
}

Intersection Scene::intersect(const Ray &ray) const {
    if (!IS_BVH) {
        Intersection intersection;
        float tNear = kInfinity;
        for (const auto &object: objects) {
            Intersection intersectionTemp = object->getIntersection(ray);
            if (intersectionTemp.happened == true && intersectionTemp.distance < tNear) {
                intersection = intersectionTemp;
                tNear = intersectionTemp.distance;
            }
        }

        return intersection;
    } else {
        return bvh->intersect(ray);
    }
}

Vector3f Scene::castRay(const Ray &ray, int depth) const {
    if (IS_PATH) {
        // path tracing
        if (depth > MAX_DEPTH)
            return Vector3f(0.0, 0.0, 0.0);
        
        Vector3f hitColor = backgroundColor;
        Intersection intersection = intersect(ray);
        Material *material = intersection.material;
        Object *hitObject = intersection.object;
        if (intersection.happened) {
            Vector3f hitCoordinate = intersection.coordinate;
            Vector3f hitNormal = intersection.normal;
            switch (material->getType()) {
                case DIFFUSE:
                {
                    Vector3f LDir = {0.0, 0.0, 0.0};
                    Vector3f LIndir = {0.0, 0.0, 0.0};

                    Vector3f hitPointOrig = (dotProduct(ray.direction, hitNormal) < 0) ?
                                            hitCoordinate + hitNormal * epsilon2 :
                                            hitCoordinate - hitNormal * epsilon2;

                    // sample on light
                    bool isPointLight = getRandomFloat() <= POINT_LIGHT_RATIO;
                    if (isPointLight && lights.size() > 0) {
                        // point light
                        int lightIndex = getRandomInt(0, lights.size() - 1);
                        auto &light = lights[lightIndex];
                        Vector3f lightPosition = light->position;
                        Vector3f lightIntensity = light->intensity;
                        Vector3f lightDir = lightPosition - hitPointOrig;
                        float lightDistance2 = dotProduct(lightDir, lightDir);
                        lightDir = normalize(lightDir);
                        Intersection intersection2 = intersect(Ray(hitPointOrig, lightDir));
                        bool isDir = (!intersection2.happened) || (intersection2.happened && intersection2.distance >= std::sqrt(lightDistance2) - epsilon2);
                        if (isDir) {
                            LDir = lightIntensity * material->brdf(ray.direction, lightDir, hitNormal) 
                                    * dotProduct(lightDir, hitNormal) / lightDistance2;
                        }
                    } else {
                        // area light
                        Intersection lightSample;
                        float lightPdf = 0.0;
                        sampleLight(lightSample, lightPdf);
                        Vector3f lightPosition = lightSample.coordinate;
                        Vector3f lightNormal = lightSample.normal;
                        Vector3f lightIntensity = lightSample.material->intensity;
                        // shoot a ray from hit point to light
                        Vector3f lightDir = lightPosition - hitPointOrig;
                        float lightDistance2 = dotProduct(lightDir, lightDir);
                        lightDir = normalize(lightDir);
                        // direct illumination
                        Intersection intersection2 = intersect(Ray(hitPointOrig, lightDir));
                        bool isDir = (!intersection2.happened) || (intersection2.happened && intersection2.distance >= std::sqrt(lightDistance2) - epsilon2);
                        if (isDir) {
                            LDir = lightIntensity * material->brdf(ray.direction, lightDir, hitNormal) 
                                    * dotProduct(lightDir, hitNormal) * dotProduct(-lightDir, lightNormal) 
                                    / lightDistance2 / lightPdf;
                        }
                    }
                    
                    // russian roulette
                    bool isIndir = getRandomFloat() <= PATH_RR;
                    if (isIndir) {
                        // sample on hemisphere
                        Vector3f wi = material->sample(ray.direction, hitNormal).normalized();
                        // shoot a ray from hit point to wi direction
                        Ray rayIndir(hitPointOrig, wi);
                        Intersection interIndir = intersect(rayIndir);
                        if (interIndir.happened && interIndir.material->getType() != EMISSION) {
                            LIndir = castRay(rayIndir, depth + 1) * material->brdf(ray.direction, wi, hitNormal) 
                                        * dotProduct(wi, hitNormal) / material->pdf(ray.direction, wi, hitNormal) / PATH_RR;
                        }
                    }
                    hitColor = LDir + LIndir;
                } break;
                case REFLECTION:
                {
                    Vector3f reflectionDirection = normalize(reflect(ray.direction, hitNormal));
                    Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, hitNormal) < 0) ?
                                                hitCoordinate - hitNormal * epsilon2 :
                                                hitCoordinate + hitNormal * epsilon2;
                    hitColor = castRay(Ray(reflectionRayOrig, reflectionDirection), depth + 1);
                } break;
                case REFRACTION:
                {
                    Vector3f refractionDirection = normalize(refract(ray.direction, hitNormal, material->ior));
                    Vector3f refractionRayOrig = (dotProduct(refractionDirection, hitNormal) < 0) ?
                                                hitCoordinate - hitNormal * epsilon2 :
                                                hitCoordinate + hitNormal * epsilon2;
                    hitColor = castRay(Ray(refractionRayOrig, refractionDirection), depth + 1);
                } break;
                case REFLECTION_AND_REFRACTION:
                {
                    Vector3f reflectionDirection = normalize(reflect(ray.direction, hitNormal));
                    Vector3f refractionDirection = normalize(refract(ray.direction, hitNormal, material->ior));
                    Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, hitNormal) < 0) ?
                                                hitCoordinate - hitNormal * epsilon2 :
                                                hitCoordinate + hitNormal * epsilon2;
                    Vector3f refractionRayOrig = (dotProduct(refractionDirection, hitNormal) < 0) ?
                                                hitCoordinate - hitNormal * epsilon2 :
                                                hitCoordinate + hitNormal * epsilon2;
                    Vector3f reflectionColor = castRay(Ray(reflectionRayOrig, reflectionDirection), depth + 1);
                    Vector3f refractionColor = castRay(Ray(refractionRayOrig, refractionDirection), depth + 1);
                    float kr = fresnel(ray.direction, hitNormal, material->ior);
                    hitColor = reflectionColor * kr + refractionColor * (1 - kr);
                } break;
                case EMISSION:
                {
                    hitColor = material->intensity;
                } break;
                default:
                {
                    throw std::runtime_error("Unsupported material type.");
                }
            }
        }
        return hitColor;
    } else {
        // Whitted-style ray tracing
        if (depth > MAX_DEPTH)
            return Vector3f(0.0, 0.0, 0.0);

        Vector3f hitColor = backgroundColor;
        Intersection intersection = intersect(ray);
        Material *material = intersection.material;
        Object *hitObject = intersection.object;
        if (intersection.happened) {
            Vector3f hitCoordinate = intersection.coordinate;
            Vector3f hitNormal = intersection.normal;
            switch (material->getType()) {
                case DIFFUSE:
                {
                    // Blinn-Phong model
                    Vector3f ambientColor = 0, diffuseColor = 0, specularColor = 0;
                    bool outside = dotProduct(ray.direction, hitNormal) < 0;
                    Vector3f shadowPointOrig = outside ?
                                            hitCoordinate + hitNormal * epsilon2 :
                                            hitCoordinate - hitNormal * epsilon2;
                    // ambient
                    ambientColor += outside ? 0 : ambientIntensity;
                    // point light
                    for (auto &light: lights) {
                        Vector3f lightDir = light->position - shadowPointOrig;
                        float lightDistance2 = dotProduct(lightDir, lightDir);
                        lightDir = normalize(lightDir);
                        float LdotN = std::max(0.f, dotProduct(lightDir, hitNormal));
                        // hard shadow
                        Intersection intersection2 = intersect(Ray(shadowPointOrig, lightDir));
                        bool inShadow = intersection2.happened && (intersection2.distance < std::sqrt(lightDistance2) - epsilon2);
                        // diffuse
                        diffuseColor += inShadow ? 0 : light->intensity * LdotN / lightDistance2;
                        // specular
                        Vector3f halfVector = normalize(lightDir - ray.direction);
                        specularColor += inShadow ? 0 : light->intensity / lightDistance2 * 
                            powf(std::max(0.f, dotProduct(hitNormal, halfVector)), material->specularExponent);
                    }
                    // area light
                    for (auto &object: objects) {
                        if (object->material->getType() == EMISSION) {
                            for (int i = 0; i < AREA2POINT_NUM; ++i) {
                                // sample on area light
                                Intersection lightSample;
                                float lightPdf = 0.0;
                                object->sample(lightSample, lightPdf);
                                // similar to point light
                                Vector3f lightDir = lightSample.coordinate - shadowPointOrig;
                                float lightDistance2 = dotProduct(lightDir, lightDir);
                                lightDir = normalize(lightDir);
                                float LdotN = std::max(0.f, dotProduct(lightDir, hitNormal));
                                // hard shadow
                                Intersection intersection2 = intersect(Ray(shadowPointOrig, lightDir));
                                bool inShadow = intersection2.happened && (intersection2.distance < std::sqrt(lightDistance2) - epsilon2);
                                // diffuse
                                diffuseColor += inShadow ? 0 : object->material->intensity * object->getArea() / AREA2POINT_NUM * LdotN / lightDistance2;
                                // specular
                                Vector3f halfVector = normalize(lightDir - ray.direction);
                                specularColor += inShadow ? 0 : object->material->intensity * object->getArea() / AREA2POINT_NUM / lightDistance2 * 
                                    powf(std::max(0.f, dotProduct(hitNormal, halfVector)), material->specularExponent);
                            }
                        }
                    }

                    hitColor = material->Ka * ambientColor + material->Kd * diffuseColor + material->Ks * specularColor;
                } break;
                case REFLECTION:
                {
                    Vector3f reflectionDirection = normalize(reflect(ray.direction, hitNormal));
                    Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, hitNormal) < 0) ?
                                                hitCoordinate - hitNormal * epsilon2 :
                                                hitCoordinate + hitNormal * epsilon2;
                    hitColor = castRay(Ray(reflectionRayOrig, reflectionDirection), depth + 1);
                } break;
                case REFRACTION:
                {
                    Vector3f refractionDirection = normalize(refract(ray.direction, hitNormal, material->ior));
                    Vector3f refractionRayOrig = (dotProduct(refractionDirection, hitNormal) < 0) ?
                                                hitCoordinate - hitNormal * epsilon2 :
                                                hitCoordinate + hitNormal * epsilon2;
                    hitColor = castRay(Ray(refractionRayOrig, refractionDirection), depth + 1);
                } break;
                case REFLECTION_AND_REFRACTION:
                {
                    Vector3f reflectionDirection = normalize(reflect(ray.direction, hitNormal));
                    Vector3f refractionDirection = normalize(refract(ray.direction, hitNormal, material->ior));
                    Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, hitNormal) < 0) ?
                                                hitCoordinate - hitNormal * epsilon2 :
                                                hitCoordinate + hitNormal * epsilon2;
                    Vector3f refractionRayOrig = (dotProduct(refractionDirection, hitNormal) < 0) ?
                                                hitCoordinate - hitNormal * epsilon2 :
                                                hitCoordinate + hitNormal * epsilon2;
                    Vector3f reflectionColor = castRay(Ray(reflectionRayOrig, reflectionDirection), depth + 1);
                    Vector3f refractionColor = castRay(Ray(refractionRayOrig, refractionDirection), depth + 1);
                    float kr = fresnel(ray.direction, hitNormal, material->ior);
                    hitColor = reflectionColor * kr + refractionColor * (1 - kr);
                } break;
                case EMISSION:
                {
                    hitColor = material->intensity;
                } break;
                default:
                {
                    throw std::runtime_error("Unsupported material type.");
                }
            }
        }
        return hitColor;
    }
    // never reach here actually
    return Vector3f(0, 0, 0);
}

void Scene::sampleLight(Intersection &position, float &pdf) const {
    float emissionAreaSum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->material->getType() == EMISSION){
            emissionAreaSum += objects[k]->getArea();
        }
    }
    float p = getRandomFloat() * emissionAreaSum;
    emissionAreaSum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->material->getType() == EMISSION) {
            emissionAreaSum += objects[k]->getArea();
            if (p <= emissionAreaSum) {
                objects[k]->sample(position, pdf);
                break;
            }
        }
    }
}
