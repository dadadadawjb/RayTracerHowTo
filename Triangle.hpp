#pragma once

#include <cstring>
#include <cassert>
#include <array>

#include "Vector.hpp"
#include "Object.hpp"
#include "OBJ_Loader.hpp"
#include "BVH.hpp"
#include "Ray.hpp"
#include "Intersection.hpp"
#include "Material.hpp"

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

class Triangle: public Object {
public:
    Vector3f v0, v1, v2; // vertices A, B ,C , counter-clockwise order
    Vector3f e1, e2;     // 2 edges v1-v0, v2-v0;
    Vector3f t0, t1, t2; // texture coords
    Vector3f normal;
    float area;
    Material *material;

    Triangle(Vector3f _v0, Vector3f _v1, Vector3f _v2, Material *_m = new Material())
        : v0(_v0), v1(_v1), v2(_v2), material(_m) {
        e1 = v1 - v0;
        e2 = v2 - v0;
        normal = normalize(crossProduct(e1, e2));
        area = crossProduct(e1, e2).norm() * 0.5f;
    }

    bool isIntersected(const Ray &ray) const override {
        // never used actually
        if (dotProduct(ray.direction, normal) > 0)
            return false;
        
        float u, v, t_near;
        return rayTriangleIntersect(v0, v1, v2, ray.origin, ray.direction, t_near, u, v);
    }
    bool isIntersected(const Ray &ray, float &t_near, uint32_t &, Vector2f &uv) const override {
        // never used actually
        if (dotProduct(ray.direction, normal) > 0)
            return false;
        
        float u, v;
        bool flag = rayTriangleIntersect(v0, v1, v2, ray.origin, ray.direction, t_near, u, v);
        uv = Vector2f(u, v);
        return flag;
    }
    Intersection getIntersection(Ray ray) override {
        Intersection intersection;
        if (dotProduct(ray.direction, normal) > 0)
            return intersection;
        
        float u, v, t_near;
        intersection.happened = rayTriangleIntersect(v0, v1, v2, ray.origin, ray.direction, t_near, u, v);
        if (intersection.happened) {
            intersection.coordinates = ray(t_near);
            intersection.normal = normal;
            intersection.distance = t_near;
            intersection.object = this;
            intersection.material = this->material;
            intersection.uv = Vector2f(u, v);
        }
        return intersection;
    }

    AABB getBoundingBox() override {
        return Union(AABB(v0, v1), v2);
    }
    float getArea() override {
        return area;
    }
    bool hasEmission() override {
        return material->hasEmission();
    }

    void getSurfaceProperties(const Vector3f &, const Vector3f &, const uint32_t &, const Vector2f &,
                              Vector3f &N, Vector2f &) const override {
        N = normal;
    }
    Vector3f evalDiffuseColor(const Vector2f &) const override {
        // never used actually
        return Vector3f(TRIS2_DIFFUSE_R, TRIS2_DIFFUSE_G, TRIS2_DIFFUSE_B);
    }

    void sample(Intersection &position, float &pdf) override {
        float x = std::sqrt(get_random_float()), y = get_random_float();
        position.coordinates = v0 * (1.0f - x) + v1 * (x * (1.0f - y)) + v2 * (x * y);
        position.normal = this->normal;
        position.material = material;
        position.object = this;
        // position.happened;
        // position.distance;
        pdf = 1.0f / area;
    }
};

class MeshTriangle: public Object {
private:
    AABB bounding_box;
    uint32_t numTriangles;
    std::unique_ptr<Vector3f[]> vertices;
    std::unique_ptr<uint32_t[]> vertexIndex;
    std::unique_ptr<Vector2f[]> stCoordinates;

    std::vector<Triangle> triangles;
    float area;
    Material *material;

    // only in acceleration
    BVH *bvh;

public:
    MeshTriangle(const std::string &filename, Material *m = new Material()) {
        objl::Loader loader;
        loader.LoadFile(filename);

        area = 0;
        material = m;

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
                                     mesh.Vertices[i + j].Position.Z) * ((BUNNY) ? BUNNY_SCALE : 1);
                face_vertices[j] = vert;

                min_vert = Vector3f(std::min(min_vert.x, vert.x),
                                    std::min(min_vert.y, vert.y),
                                    std::min(min_vert.z, vert.z));
                max_vert = Vector3f(std::max(max_vert.x, vert.x),
                                    std::max(max_vert.y, vert.y),
                                    std::max(max_vert.z, vert.z));
            }

            triangles.emplace_back(face_vertices[0], face_vertices[1],
                                   face_vertices[2], m);
        }

        bounding_box = AABB(min_vert, max_vert);

        std::vector<Object *> ptrs;
        for (auto &tri: triangles) {
            ptrs.push_back(&tri);
            area += tri.area;
        }
        if (!SAH_ACC)
            bvh = new BVH(ptrs, BVH::SplitMethod::NAIVE);
        else
            bvh = new BVH(ptrs, BVH::SplitMethod::SAH);
    }
    MeshTriangle(const Vector3f *verts, const uint32_t *vertsIndex, const uint32_t &numTris, const Vector2f *st, Material *m = new Material()) {
        area = 0;
        material = m;
        uint32_t maxIndex = 0;
        for (uint32_t i = 0; i < numTris * 3; ++i)
            if (vertsIndex[i] > maxIndex)
                maxIndex = vertsIndex[i];
        maxIndex += 1;
        vertices = std::unique_ptr<Vector3f[]>(new Vector3f[maxIndex]);
        memcpy(vertices.get(), verts, sizeof(Vector3f) * maxIndex);
        vertexIndex = std::unique_ptr<uint32_t[]>(new uint32_t[numTris * 3]);
        memcpy(vertexIndex.get(), vertsIndex, sizeof(uint32_t) * numTris * 3);
        numTriangles = numTris;
        stCoordinates = std::unique_ptr<Vector2f[]>(new Vector2f[maxIndex]);
        memcpy(stCoordinates.get(), st, sizeof(Vector2f) * maxIndex);
    }

    bool isIntersected(const Ray &ray) const override {
        // never used actually
        bool intersected = false;
        float tNear = kInfinity;
        if (!ACCELERATION) {
            for (uint32_t k = 0; k < numTriangles; ++k) {
                const Vector3f &v0 = vertices[vertexIndex[k * 3]];
                const Vector3f &v1 = vertices[vertexIndex[k * 3 + 1]];
                const Vector3f &v2 = vertices[vertexIndex[k * 3 + 2]];
                float t, u, v;
                if (rayTriangleIntersect(v0, v1, v2, ray.origin, ray.direction, t, u, v)  && t < tNear) {
                    intersected |= true;
                    tNear = t;
                }
            }
        } else if (bvh) {
            intersected = bvh->intersect(ray).happened;
        }

        return intersected;
    }
    bool isIntersected(const Ray &ray, float &t_near, uint32_t &index, Vector2f &uv) const override {
        // never used actually
        bool intersected = false;
        if (!ACCELERATION) {
            for (uint32_t k = 0; k < numTriangles; ++k) {
                const Vector3f &v0 = vertices[vertexIndex[k * 3]];
                const Vector3f &v1 = vertices[vertexIndex[k * 3 + 1]];
                const Vector3f &v2 = vertices[vertexIndex[k * 3 + 2]];
                float t, u, v;
                if (rayTriangleIntersect(v0, v1, v2, ray.origin, ray.direction, t, u, v) 
                    && t < t_near) {
                    t_near = t;
                    index = k;
                    uv.x = u;
                    uv.y = v;
                    intersected |= true;
                }
            }
        } else if (bvh) {
            intersected = bvh->intersect(ray).happened;
        }

        return intersected;
    }

    Intersection getIntersection(Ray ray) override {
        Intersection intersection;
        if (!ACCELERATION) {
            float tNear = kInfinity;
            for (uint32_t k = 0; k < numTriangles; ++k) {
                const Vector3f &v0 = vertices[vertexIndex[k * 3]];
                const Vector3f &v1 = vertices[vertexIndex[k * 3 + 1]];
                const Vector3f &v2 = vertices[vertexIndex[k * 3 + 2]];
                float t, u, v;
                if (rayTriangleIntersect(v0, v1, v2, ray.origin, ray.direction, t, u, v) && t < tNear) {
                    intersection.happened = true;
                    intersection.object = this;
                    intersection.uv = Vector2f(u, v);
                    intersection.distance = t;
                    intersection.normal = normalize(crossProduct(v1 - v0, v2 - v0));
                    intersection.material = material;
                    intersection.coordinates = ray(t);
                    tNear = t;
                }
            }
        } else if (bvh) {
            intersection = bvh->intersect(ray);
        }

        return intersection;
    }

    AABB getBoundingBox() override {
        return bounding_box;
    }
    float getArea() override {
        return area;
    }
    bool hasEmission() override {
        return material->hasEmission();
    }

    void getSurfaceProperties(const Vector3f &, const Vector3f &, const uint32_t &index, const Vector2f &uv,
                              Vector3f &N, Vector2f &st) const override {
        const Vector3f &v0 = vertices[vertexIndex[index * 3]];
        const Vector3f &v1 = vertices[vertexIndex[index * 3 + 1]];
        const Vector3f &v2 = vertices[vertexIndex[index * 3 + 2]];
        Vector3f e0 = normalize(v1 - v0);
        Vector3f e1 = normalize(v2 - v1);
        N = normalize(crossProduct(e0, e1));
        const Vector2f &st0 = stCoordinates[vertexIndex[index * 3]];
        const Vector2f &st1 = stCoordinates[vertexIndex[index * 3 + 1]];
        const Vector2f &st2 = stCoordinates[vertexIndex[index * 3 + 2]];
        st = st0 * (1 - uv.x - uv.y) + st1 * uv.x + st2 * uv.y;
    }
    Vector3f evalDiffuseColor(const Vector2f &st) const override {
        // TODO: hard coded for DEMO
        float scale = 5;
        float pattern = (fmodf(st.x * scale, 1) > 0.5) ^ (fmodf(st.y * scale, 1) > 0.5);
        return lerp(Vector3f(0.815, 0.235, 0.031), Vector3f(0.937, 0.937, 0.231), pattern);
    }
    
    void sample(Intersection &position, float &pdf) override {
        bvh->sample(position, pdf);
    }
};
