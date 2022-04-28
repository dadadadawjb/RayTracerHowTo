#include <fstream>
#include <thread>
#include <mutex>

#include "Vector.hpp"
#include "global.hpp"
#include "raytracer.hpp"
#include "Scene.hpp"

void RayTracer::RayTracing(const Scene &scene, const Vector3f &eye_position) {
    frame_buffer.resize(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;

    if (!MULTITHREADING) {
        for (uint32_t j = 0; j < scene.height; ++j) {
            for (uint32_t i = 0; i < scene.width; ++i) {
                // generate primary ray direction
                float x;
                float y;
                // inverse viewport transformation
                x = 2.0 / float(scene.width) * (i + 0.5) - 1;
                y = 2.0 / float(scene.height) * (scene.height - (j + 0.5)) - 1;
                // inverse perspective transformation
                x *= scale * imageAspectRatio;
                y *= scale;

                // TODO: not understand yet why
                Vector3f dir;
                if (BUNNY || DEMO)
                    dir = normalize(Vector3f(x, y, -1));
                else if (CORNELLBOX)
                    dir = normalize(Vector3f(-x, y, 1));
                else
                    throw std::runtime_error("Determine your cast ray direction under your configuration.");
                if (!PATH) {
                    Vector3f irradiance = scene.castRay(Ray(eye_position, dir), 0);
                    frame_buffer[j * scene.width + i] = Eigen::Vector3f(irradiance.x, irradiance.y, irradiance.z);
                } else {
                    Vector3f irradiance(0);
                    for (int k = 0; k < PATH_SAMPLES; ++k) {
                        irradiance += scene.castRay(Ray(eye_position, dir), 0) / PATH_SAMPLES;
                    }
                    frame_buffer[j * scene.width + i] = Eigen::Vector3f(irradiance.x, irradiance.y, irradiance.z);
                }
            }
            updateProgress(j / (float)scene.height);
        }
        updateProgress(1.f);
    } else {
        std::mutex my_mutex;
        int processed = 0;
        auto castRayMultiThreading = [&](uint32_t rowStart, uint32_t rowEnd, uint32_t colStart, uint32_t colEnd) {
            for (uint32_t j = rowStart; j < rowEnd; ++j) {
                for (uint32_t i = colStart; i < colEnd; ++i) {
                    // generate primary ray direction
                    float x = (2 * (i + 0.5) / (float)scene.width - 1) *
                        imageAspectRatio * scale;
                    float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

                    // TODO: not understand yet why
                    Vector3f dir;
                    if (BUNNY || DEMO)
                        dir = normalize(Vector3f(x, y, -1));
                    else if (CORNELLBOX)
                        dir = normalize(Vector3f(-x, y, 1));
                    else
                        throw std::runtime_error("Determine your cast ray direction under your configuration.");
                    if (!PATH) {
                        Vector3f irradiance = scene.castRay(Ray(eye_position, dir), 0);
                        frame_buffer[j * scene.width + i] = Eigen::Vector3f(irradiance.x, irradiance.y, irradiance.z);
                    } else {
                        Vector3f irradiance(0);
                        for (int k = 0; k < PATH_SAMPLES; ++k) {
                            irradiance += scene.castRay(Ray(eye_position, dir), 0) / PATH_SAMPLES;
                        }
                        frame_buffer[j * scene.width + i] = Eigen::Vector3f(irradiance.x, irradiance.y, irradiance.z);
                    }
                    my_mutex.lock();
                    ++processed;
                    my_mutex.unlock();
                }

                my_mutex.lock();
                updateProgress(processed / (float)(scene.width * scene.height));
                my_mutex.unlock();
            }
        };

        int id = 0;
        std::thread my_threads[THREADS_X * THREADS_Y];
        int strideX = (scene.width + 1) / THREADS_X;
        int strideY = (scene.height + 1) / THREADS_Y;
        for (uint32_t j = 0; j < scene.height; j += strideY) {
            for (uint32_t i = 0; i < scene.width; i += strideX) {
                my_threads[id] = std::thread(castRayMultiThreading, j, std::min(j + strideY, uint32_t(scene.height)), i, std::min(i + strideX, uint32_t(scene.width)));
                ++id;
            }
        }

        for (int i = 0; i < THREADS_X * THREADS_Y; ++i)
            my_threads[i].join();
        updateProgress(1.f);
    }
}
