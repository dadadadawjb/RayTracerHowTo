#pragma once

#include <cstring>
#include <cassert>
#include <array>

#include "object.hpp"
#include "OBJ_loader.hpp"


bool rayTriangleIntersect(const Vector3f &v0, const Vector3f &v1, const Vector3f &v2, 
                          const Vector3f &orig, const Vector3f &dir, float &t_near, float &u, float &v) {
    // Möller Trumbore algorithm
    Vector3f edge1 = v1 - v0;
    Vector3f edge2 = v2 - v0;
    Vector3f pvec = crossProduct(dir, edge2);
    float det = dotProduct(edge1, pvec);
    if (fabs(det) < epsilon || det < 0)
        return false;

    Vector3f tvec = orig - v0;
    u = dotProduct(tvec, pvec);
    if (fabs(u) < epsilon || u < 0 || fabs(u - det) < epsilon || u > det)
        return false;

    Vector3f qvec = crossProduct(tvec, edge1);
    v = dotProduct(dir, qvec);
    if (fabs(v) < epsilon || v < 0 || fabs(u + v - det) < epsilon || u + v > det)
        return false;

    float invDet = 1 / det;

    t_near = dotProduct(edge2, qvec) * invDet;
    if (fabs(t_near) < epsilon || t_near < 0)
        return false;
    
    u *= invDet;
    v *= invDet;

    return true;
}

/*
Triangle implementation
CORE: 
- ray intersection with triangle (by Möller Trumbore algorithm)
- sample point on triangle
*/
class Triangle: public Object {
private:
    Vector3f v0, v1, v2; // vertices A, B ,C , counter-clockwise order
    Vector3f e1, e2;     // 2 edges v0-v1, v0-v2;
    Vector3f normal;
    float area;

public:
    Triangle(Vector3f _v0, Vector3f _v1, Vector3f _v2, Material *_m = new Material(), std::string _name="triangle")
        : Object(_m, _name), v0(_v0), v1(_v1), v2(_v2) {
        e1 = v1 - v0;
        e2 = v2 - v0;
        normal = normalize(crossProduct(e1, e2));
        area = crossProduct(e1, e2).norm() * 0.5f;
    }

    AABB getBoundingBox() override {
        return unite(AABB(v0, v1), v2);
    }
    float getArea() override {
        return area;
    }

    Intersection getIntersection(Ray ray) override {
        Intersection intersection;
        if (dotProduct(ray.direction, normal) > 0)
            return intersection;
        
        float u, v, t_near;
        intersection.happened = rayTriangleIntersect(v0, v1, v2, ray.origin, ray.direction, t_near, u, v);
        if (intersection.happened) {
            intersection.coordinate = ray(t_near);
            intersection.normal = normal;
            intersection.distance = t_near;
            intersection.object = this;
            intersection.material = this->material;
            intersection.uv = Vector2f(u, v);
        }
        return intersection;
    }

    void sample(Intersection &position, float &pdf) override {
        float x = std::sqrt(getRandomFloat()), y = getRandomFloat();
        position.coordinate = v0 * (1.0f - x) + v1 * (x * (1.0f - y)) + v2 * (x * y);
        position.normal = this->normal;
        position.material = material;
        position.object = this;
        pdf = 1.0f / area;
    }

friend class MeshTriangle;
};

/*
MeshTriangle implementation
CORE: 
- ray intersection with multiple triangles
- sample point on multiple triangles
*/
class MeshTriangle: public Object {
private:
    std::vector<Triangle> triangles;
    float area;
    AABB boundingBox;

    BVH *bvh;                           // only needed by BVH acceleration

public:
    MeshTriangle(const std::string &filename, Material *m = new Material(), std::string _name="mesh"): Object(m, _name) {
        objl::Loader loader;
        loader.LoadFile(filename);

        area = 0;

        assert(loader.LoadedMeshes.size() == 1);
        auto mesh = loader.LoadedMeshes[0];

        Vector3f min_vert = Vector3f{std::numeric_limits<float>::infinity(),
                                     std::numeric_limits<float>::infinity(),
                                     std::numeric_limits<float>::infinity()};
        Vector3f max_vert = Vector3f{-std::numeric_limits<float>::infinity(),
                                     -std::numeric_limits<float>::infinity(),
                                     -std::numeric_limits<float>::infinity()};
        for (int i = 0; i < mesh.Vertices.size(); i += 3) {
            std::array<Vector3f, 3> face_vertices;

            for (int j = 0; j < 3; j++) {
                auto vert = Vector3f(mesh.Vertices[i + j].Position.X,
                                     mesh.Vertices[i + j].Position.Y,
                                     mesh.Vertices[i + j].Position.Z);
                face_vertices[j] = vert;

                min_vert = Vector3f(std::min(min_vert.x, vert.x),
                                    std::min(min_vert.y, vert.y),
                                    std::min(min_vert.z, vert.z));
                max_vert = Vector3f(std::max(max_vert.x, vert.x),
                                    std::max(max_vert.y, vert.y),
                                    std::max(max_vert.z, vert.z));
            }

            triangles.emplace_back(face_vertices[0], face_vertices[1],
                                   face_vertices[2], m, _name + "_triangle_" + std::to_string(i / 3));
        }

        boundingBox = AABB(min_vert, max_vert);

        std::vector<Object *> ptrs;
        for (auto &tri: triangles) {
            ptrs.push_back(&tri);
            area += tri.area;
        }
        if (!IS_BVH) {
            bvh = nullptr;
        } else {
            if (!IS_SAH)
                bvh = new BVH(ptrs, BVH::SplitMethod::NAIVE);
            else
                bvh = new BVH(ptrs, BVH::SplitMethod::SAH);
        }
    }

    AABB getBoundingBox() override {
        return boundingBox;
    }
    float getArea() override {
        return area;
    }

    Intersection getIntersection(Ray ray) override {
        Intersection intersection;
        if (!IS_BVH) {
            float tNear = kInfinity;
            for (uint32_t k = 0; k < triangles.size(); ++k) {
                Vector3f v0 = triangles[k].v0;
                Vector3f v1 = triangles[k].v1;
                Vector3f v2 = triangles[k].v2;
                float t, u, v;
                if (rayTriangleIntersect(v0, v1, v2, ray.origin, ray.direction, t, u, v) && t < tNear) {
                    intersection.happened = true;
                    intersection.coordinate = ray(t);
                    intersection.normal = triangles[k].normal;
                    intersection.object = this;
                    intersection.distance = t;
                    intersection.material = material;
                    intersection.uv = Vector2f(u, v);
                    intersection.index = k;
                    tNear = t;
                }
            }
        } else {
            intersection = bvh->intersect(ray);
        }

        return intersection;
    }
    
    void sample(Intersection &position, float &pdf) override {
        if (!IS_BVH) {
            float p = getRandomFloat() * area;
            float emissionAreaSum = 0;
            for (uint32_t k = 0; k < triangles.size(); ++k) {
                emissionAreaSum += triangles[k].area;
                if (p <= emissionAreaSum) {
                    triangles[k].sample(position, pdf);
                    pdf *= triangles[k].area;
                    break;
                }
            }
            pdf /= area;
        } else {
            bvh->sample(position, pdf);
        }
    }
};
