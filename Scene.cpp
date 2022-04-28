#include "Scene.hpp"

void Scene::buildBVH() {
    if (!SAH_ACC)
        bvh = new BVH(objects, BVH::SplitMethod::NAIVE);
    else
        bvh = new BVH(objects, BVH::SplitMethod::SAH);
}

Intersection Scene::intersect(const Ray &ray) const {
    if (!ACCELERATION) {
        Intersection intersection;
        float tNear = kInfinity;
        for (const auto &object: objects) {
            Intersection intersection_temp = object->getIntersection(ray);
            if (intersection_temp.happened == true && intersection_temp.distance < tNear) {
                intersection = intersection_temp;
                tNear = intersection_temp.distance;
            }
        }

        return intersection;
    } else {
        return bvh->intersect(ray);
    }
}

Vector3f Scene::castRay(const Ray &ray, int depth) const {
    if (PATH) {
        // path tracing
        if (depth > maxDepth)
            return Vector3f(0.0, 0.0, 0.0);
        
        Intersection intersection = intersect(ray);
        if (intersection.happened) {
            Vector3f hitPoint = intersection.coordinates;
            Vector3f N = intersection.normal;
            Material *material = intersection.material;
            Object *hitObject = intersection.object;
            // when hit directly light
            if (material->hasEmission())
                return material->getEmission();
            
            // otherwise, hit normal object
            switch (material->getType()) {
                case DIFFUSE:
                case MICROFACET:
                {
                    Vector3f L_dir = {0.0, 0.0, 0.0};
                    Vector3f L_indir = {0.0, 0.0, 0.0};
                    //////////---------- contribution from the light source ----------//////////
                    // sample on light
                    Intersection light_sample;
                    float pdf_light = 0.0;
                    sampleLight(light_sample, pdf_light);
                    if (light_sample.material == nullptr) {
                        throw std::runtime_error("no object having material with emission.");
                    }
                    Vector3f light_x = light_sample.coordinates;
                    Vector3f NN = light_sample.normal;
                    Vector3f light_emission = light_sample.material->getEmission();
                    // shoot a ray from hit point to light
                    Ray ray_p_light(hitPoint, normalize(light_x - hitPoint));
                    if (Scene::intersect(ray_p_light).distance - (light_x - hitPoint).norm() > -epsilon)
                        L_dir = light_emission * material->evaluate(ray.direction, ray_p_light.direction, N) 
                                * dotProduct(ray_p_light.direction, N) * dotProduct(-ray_p_light.direction, NN) 
                                / (light_x - hitPoint).norm() / (light_x - hitPoint).norm() / pdf_light;
                    
                    //////////---------- contribution from other reflectors ----------//////////
                    // russian roulette
                    if (get_random_float() > RussianRoulette)
                        return L_dir;
                    // sample on hemisphere
                    Vector3f wi = material->sample(ray.direction, N).normalized();
                    // shoot a ray from hit point to wi direction
                    Ray ray_wi(hitPoint, wi);
                    Intersection inter_wi = Scene::intersect(ray_wi);
                    if (inter_wi.happened) {
                        if (!inter_wi.material->hasEmission())
                            L_indir = castRay(ray_wi, depth + 1) * material->evaluate(ray.direction, wi, N) 
                                        * dotProduct(wi, N) / material->pdf(ray.direction, wi, N) / RussianRoulette;
                    }
                    return L_dir + L_indir;
                } break;
                default:
                {
                    throw std::runtime_error("Path tracing only support DIFFUSE and MICROFACET material type!");
                }
            }
        } else {
            return Vector3f(0.0, 0.0, 0.0);
        }
    } else {
        // Whitted-style light transport algorithm
        if (depth > maxDepth)
            return Vector3f(0.0, 0.0, 0.0);

        Vector3f hitColor = backgroundColor;
        // use BVH to get intersection
        // if no BVH, something is repeated
        Intersection intersection = intersect(ray);
        Material *material = intersection.material;
        Object *hitObject = intersection.object;
        uint32_t index = 0;
        if (intersection.happened) {
            Vector3f hitPoint = intersection.coordinates;
            Vector3f N = intersection.normal;
            Vector2f st;
            hitObject->getSurfaceProperties(hitPoint, ray.direction, index, intersection.uv, N, st);
            switch (material->getType()) {
                case REFLECTION_AND_REFRACTION:
                {
                    Vector3f reflectionDirection = normalize(reflect(ray.direction, N));
                    Vector3f refractionDirection = normalize(refract(ray.direction, N, material->ior));
                    Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ?
                                                hitPoint - N * epsilon :
                                                hitPoint + N * epsilon;
                    Vector3f refractionRayOrig = (dotProduct(refractionDirection, N) < 0) ?
                                                hitPoint - N * epsilon :
                                                hitPoint + N * epsilon;
                    Vector3f reflectionColor = castRay(Ray(reflectionRayOrig, reflectionDirection), depth + 1);
                    Vector3f refractionColor = castRay(Ray(refractionRayOrig, refractionDirection), depth + 1);
                    float kr = fresnel(ray.direction, N, material->ior);
                    hitColor = reflectionColor * kr + refractionColor * (1 - kr);
                    break;
                }
                case REFLECTION:
                {
                    float kr = fresnel(ray.direction, N, material->ior);
                    Vector3f reflectionDirection = normalize(reflect(ray.direction, N));
                    Vector3f reflectionRayOrig = (dotProduct(reflectionDirection, N) < 0) ?
                                                hitPoint - N * epsilon :
                                                hitPoint + N * epsilon;
                    hitColor = castRay(Ray(reflectionRayOrig, reflectionDirection), depth + 1) * kr;
                    break;
                }
                default:    // DIFFUSE, MICROFACET taken the same as DIFFUSE
                {
                    // Phong model, not Blinn-Phong model
                    Vector3f diffuseColor = 0, specularColor = 0;
                    Vector3f shadowPointOrig = (dotProduct(ray.direction, N) < 0) ?
                                            hitPoint + N * epsilon :
                                            hitPoint - N * epsilon;
                    for (auto &light: get_lights()) {
                        Vector3f lightDir = light->position - hitPoint;
                        float lightDistance2 = dotProduct(lightDir, lightDir);
                        lightDir = normalize(lightDir);
                        float LdotN = std::max(0.f, dotProduct(lightDir, N));
                        // hard shadow
                        Intersection intersection2 = intersect(Ray(shadowPointOrig, lightDir));
                        bool inShadow = intersection2.happened && (intersection2.distance * intersection2.distance < lightDistance2);
                        // diffuse
                        diffuseColor += inShadow ? 0 : light->intensity * LdotN;
                        // specular
                        Vector3f reflectionDirection = reflect(-lightDir, N);
                        specularColor += light->intensity * 
                            powf(std::max(0.f, -dotProduct(reflectionDirection, ray.direction)), material->specularExponent);
                    }

                    hitColor = material->Kd * diffuseColor * hitObject->evalDiffuseColor(st) + material->Ks * specularColor;
                    break;
                }
            }
        }
        return hitColor;
    }
    // never reach here actually
    return Vector3f(0, 0, 0);
}

void Scene::sampleLight(Intersection &position, float &pdf) const {
    float emission_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmission()){
            emission_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emission_area_sum;
    emission_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmission()) {
            emission_area_sum += objects[k]->getArea();
            if (p <= emission_area_sum){
                objects[k]->sample(position, pdf);
                break;
            }
        }
    }
}
