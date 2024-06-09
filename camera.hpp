#pragma once

#include "vector.hpp"
#include "global.hpp"
#include "ray.hpp"


/*
Camera implementation
CORE: 
- Camera intrinsics and extrinsics
- ray casting
*/
class Camera {
public:
    int width = WIDTH;
    int height = HEIGHT;
    double fov = FOV;
    Vector3f eye;
    Vector3f front, up, right;

    Camera(int w, int h, double f, Vector3f _eye, Vector3f _front, Vector3f _up): width(w), height(h), fov(f) {
        eye = _eye;
        front = _front;
        up = _up;
        right = normalize(crossProduct(front, up));
        up = normalize(crossProduct(right, front));
    }

    Ray generateRay(int x, int y) const {
        // inverse viewport transformation
        double x_ = (2.0 * (x + 0.5) / (double)width - 1);
        double y_ = (1 - 2.0 * (y + 0.5) / (double)height);
        // inverse perspective transformation
        double scale = tan(deg2rad(fov * 0.5));
        double imageAspectRatio = width / (double)height;
        x_ *= scale * imageAspectRatio;
        y_ *= scale;
        Vector3f dir = normalize(x_ * right + y_ * up + front);
        return Ray(eye, dir);
    }
};
