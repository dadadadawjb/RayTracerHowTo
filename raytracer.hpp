#pragma once

#include <eigen3/Eigen/Eigen>

#include "Vector.hpp"
#include "Scene.hpp"

class RayTracer {
private:
    std::vector<Eigen::Vector3f> frame_buffer;

public:
    void RayTracing(const Scene &scene, const Vector3f &eye_position);

    std::vector<Eigen::Vector3f> &get_frame_buffer() {
        uint32_t frame_size = frame_buffer.size();
        if (!GAMMA) {
            for (uint32_t i = 0; i < frame_size; ++i) {
                frame_buffer[i] = Eigen::Vector3f(255 * clamp(0, 1, frame_buffer[i].x()), 
                                                  255 * clamp(0, 1, frame_buffer[i].y()), 
                                                  255 * clamp(0, 1, frame_buffer[i].z()));
            }
        } else {
            for (uint32_t i = 0; i < frame_size; ++i) {
                frame_buffer[i] = Eigen::Vector3f(255 * std::pow(clamp(0, 1, frame_buffer[i].x()), GAMMA_VALUE_R), 
                                                  255 * std::pow(clamp(0, 1, frame_buffer[i].y()), GAMMA_VALUE_G), 
                                                  255 * std::pow(clamp(0, 1, frame_buffer[i].z()), GAMMA_VALUE_B));
            }
        }
        return frame_buffer;
    }
};
