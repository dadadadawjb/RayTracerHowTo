#pragma once

#include <eigen3/Eigen/Eigen>

#include "vector.hpp"
#include "global.hpp"
#include "scene.hpp"
#include "camera.hpp"


/*
RayTracer implementation
CORE: 
- ray tracing
- GAMMA correction
*/
class RayTracer {
private:
    std::vector<Eigen::Vector3f> frameBuffer;

public:
    void render(const Scene &scene, const Camera &camera);

    std::vector<Eigen::Vector3f> &capture() {
        uint32_t frameSize = frameBuffer.size();
        if (!IS_GAMMA) {
            for (uint32_t i = 0; i < frameSize; ++i) {
                frameBuffer[i] = Eigen::Vector3f(255 * clamp(0, 1, frameBuffer[i].x()), 
                                                 255 * clamp(0, 1, frameBuffer[i].y()), 
                                                 255 * clamp(0, 1, frameBuffer[i].z()));
            }
        } else {
            for (uint32_t i = 0; i < frameSize; ++i) {
                frameBuffer[i] = Eigen::Vector3f(255 * std::pow(clamp(0, 1, frameBuffer[i].x()), GAMMA_VALUE_R), 
                                                 255 * std::pow(clamp(0, 1, frameBuffer[i].y()), GAMMA_VALUE_G), 
                                                 255 * std::pow(clamp(0, 1, frameBuffer[i].z()), GAMMA_VALUE_B));
            }
        }
        return frameBuffer;
    }
};
