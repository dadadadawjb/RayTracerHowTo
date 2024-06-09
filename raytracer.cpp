#include <fstream>
#include <thread>
#include <mutex>

#include "raytracer.hpp"


void RayTracer::render(const Scene &scene, const Camera &camera) {
    frameBuffer.resize(camera.width * camera.height);

    if (!IS_MULTITHREADING) {
        for (uint32_t j = 0; j < camera.height; ++j) {
            for (uint32_t i = 0; i < camera.width; ++i) {
                Ray ray = camera.generateRay(i, j);
                if (!IS_PATH) {
                    Vector3f irradiance = scene.castRay(ray, 0);
                    frameBuffer[j * camera.width + i] = Eigen::Vector3f(irradiance.x, irradiance.y, irradiance.z);
                } else {
                    Vector3f irradiance(0);
                    for (int k = 0; k < PATH_SAMPLES; ++k) {
                        irradiance += scene.castRay(ray, 0) / PATH_SAMPLES;
                    }
                    frameBuffer[j * camera.width + i] = Eigen::Vector3f(irradiance.x, irradiance.y, irradiance.z);
                }
            }
            updateProgress(j / (float)camera.height);
        }
        updateProgress(1.f);
    } else {
        std::mutex myMutex;
        int processed = 0;
        auto castRayMultiThreading = [&](uint32_t rowStart, uint32_t rowEnd, uint32_t colStart, uint32_t colEnd) {
            for (uint32_t j = rowStart; j < rowEnd; ++j) {
                for (uint32_t i = colStart; i < colEnd; ++i) {
                    Ray ray = camera.generateRay(i, j);
                    if (!IS_PATH) {
                        Vector3f irradiance = scene.castRay(ray, 0);
                        frameBuffer[j * camera.width + i] = Eigen::Vector3f(irradiance.x, irradiance.y, irradiance.z);
                    } else {
                        Vector3f irradiance(0);
                        for (int k = 0; k < PATH_SAMPLES; ++k) {
                            irradiance += scene.castRay(ray, 0) / PATH_SAMPLES;
                        }
                        frameBuffer[j * camera.width + i] = Eigen::Vector3f(irradiance.x, irradiance.y, irradiance.z);
                    }
                    myMutex.lock();
                    ++processed;
                    myMutex.unlock();
                }

                myMutex.lock();
                updateProgress(processed / (float)(camera.width * camera.height));
                myMutex.unlock();
            }
        };

        int id = 0;
        std::thread myThreads[THREADS_X * THREADS_Y];
        int strideX = (camera.width + 1) / THREADS_X;
        int strideY = (camera.height + 1) / THREADS_Y;
        for (uint32_t j = 0; j < camera.height; j += strideY) {
            for (uint32_t i = 0; i < camera.width; i += strideX) {
                myThreads[id] = std::thread(castRayMultiThreading, j, std::min(j + strideY, uint32_t(camera.height)), i, std::min(i + strideX, uint32_t(camera.width)));
                ++id;
            }
        }

        for (int i = 0; i < THREADS_X * THREADS_Y; ++i)
            myThreads[i].join();
        updateProgress(1.f);
    }
}
