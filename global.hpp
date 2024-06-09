#pragma once

#include <iostream>
#include <cmath>
#include <random>


constexpr double MY_PI = 3.1415926535;
constexpr float kInfinity = std::numeric_limits<float>::max();
constexpr double epsilon = 0.00001;
constexpr double epsilon2 = 0.01;

#define FILENAME "output.png"
#define CORNELLBOX_FLOOR_OBJ "../models/cornellbox/floor.obj"
#define CORNELLBOX_LEFTWALL_OBJ "../models/cornellbox/left.obj"
#define CORNELLBOX_RIGHTWALL_OBJ "../models/cornellbox/right.obj"
#define CORNELLBOX_SHORTBOX_OBJ "../models/cornellbox/shortbox.obj"
#define CORNELLBOX_TALLBOX_OBJ "../models/cornellbox/tallbox.obj"
#define CORNELLBOX_LIGHT_OBJ "../models/cornellbox/light.obj"

#define IS_PATH true
#define PATH_SAMPLES 32
#define PATH_RR 0.8
#define IS_MULTITHREADING true
#define THREADS_X 8
#define THREADS_Y 8
#define IS_BVH true
#define IS_SAH true
#define SAH_BUCKET_COUNT 12
#define SAH_COST_INTERSECT 1.0
#define SAH_COST_TRAVERSE 0.125
#define IS_GAMMA true
#define GAMMA_VALUE_R 0.6
#define GAMMA_VALUE_G 0.6
#define GAMMA_VALUE_B 0.6
#define AREA2POINT_NUM 16
#define POINT_LIGHT_RATIO 0.1
#define WIDTH 1024
#define HEIGHT 1024
#define FOV 40
#define MAX_DEPTH 16
#define BACKGROUND_R 0.0
#define BACKGROUND_G 0.0
#define BACKGROUND_B 0.0
#define AMBIENT_R 0.0
#define AMBIENT_G 0.0
#define AMBIENT_B 0.0
#define EYE_POS_X 278
#define EYE_POS_Y 273
#define EYE_POS_Z -800
#define EYE_FRONT_X 0
#define EYE_FRONT_Y 0
#define EYE_FRONT_Z 1
#define EYE_UP_X 0
#define EYE_UP_Y 1
#define EYE_UP_Z 0

#define RED_KA_R 0.0
#define RED_KA_G 0.0
#define RED_KA_B 0.0
#define RED_KD_R 0.63
#define RED_KD_G 0.065
#define RED_KD_B 0.05
#define RED_KS_R 0.0
#define RED_KS_G 0.0
#define RED_KS_B 0.0
#define RED_SE 0.0
#define GREEN_KA_R 0.0
#define GREEN_KA_G 0.0
#define GREEN_KA_B 0.0
#define GREEN_KD_R 0.14
#define GREEN_KD_G 0.45
#define GREEN_KD_B 0.091
#define GREEN_KS_R 0.0
#define GREEN_KS_G 0.0
#define GREEN_KS_B 0.0
#define GREEN_SE 0.0
#define WHITE_KA_R 0.0
#define WHITE_KA_G 0.0
#define WHITE_KA_B 0.0
#define WHITE_KD_R 0.725
#define WHITE_KD_G 0.71
#define WHITE_KD_B 0.68
#define WHITE_KS_R 0.0
#define WHITE_KS_G 0.0
#define WHITE_KS_B 0.0
#define WHITE_SE 0.0
#define REFRACTION_IOR 3.0
#define REFLECTION_REFRACTION_IOR 1.5
#define SPHERE_POS_X 370
#define SPHERE_POS_Y 405.2
#define SPHERE_POS_Z 350
#define SPHERE_RADIUS 75
#define CYLINDER_POS_X 150
#define CYLINDER_POS_Y 75.2
#define CYLINDER_POS_Z 450
#define CYLINDER_DIR_X 0
#define CYLINDER_DIR_Y 1
#define CYLINDER_DIR_Z 0
#define CYLINDER_RADIUS 50
#define CYLINDER_HEIGHT 250
#define CONE_POS_X 450
#define CONE_POS_Y 0.2
#define CONE_POS_Z 100
#define CONE_DIR_X 0
#define CONE_DIR_Y 1
#define CONE_DIR_Z 0
#define CONE_RADIUS 50
#define CONE_HEIGHT 150
#define LIGHT_INTENSITY_R1 47.8348
#define LIGHT_INTENSITY_G1 38.5664
#define LIGHT_INTENSITY_B1 31.0808
#define LIGHT_POX_X2 545
#define LIGHT_POX_Y2 545
#define LIGHT_POX_Z2 555
#define LIGHT_INTENSITY_R2 0.0
#define LIGHT_INTENSITY_G2 0.0
#define LIGHT_INTENSITY_B2 4000000.0

inline float deg2rad(const float &deg) {
    return deg * MY_PI / 180.0;
}

inline float clamp(const float &low, const float &high, const float &v) {
    return std::max(low, std::min(high, v));
}

inline bool solveQuadratic(const float &a, const float &b, const float &c, float &x0, float &x1) {
    float delta = b * b - 4 * a * c;
    if (delta < 0)
        return false;
    else if (delta == 0)
        x0 = x1 = - 0.5 * b / a;
    else {
        float q = (b > 0) ? -0.5 * (b + sqrt(delta)) : -0.5 * (b - sqrt(delta));
        x0 = q / a;
        x1 = c / q;
    }
    if (x0 > x1)
        std::swap(x0, x1);
    return true;
}

inline Vector3f toWorld(const Vector3f &vecLocal, const Vector3f &normalWorld) {
    Vector3f B, C;
    if (std::fabs(normalWorld.x) > std::fabs(normalWorld.y)) {
        float invLen = 1.0f / std::sqrt(normalWorld.x * normalWorld.x + normalWorld.z * normalWorld.z);
        C = Vector3f(normalWorld.z * invLen, 0.0f, -normalWorld.x *invLen);
    } else {
        float invLen = 1.0f / std::sqrt(normalWorld.y * normalWorld.y + normalWorld.z * normalWorld.z);
        C = Vector3f(0.0f, normalWorld.z * invLen, -normalWorld.y *invLen);
    }
    B = crossProduct(C, normalWorld);
    return vecLocal.x * B + vecLocal.y * C + vecLocal.z * normalWorld;
}

inline float getRandomFloat() {
    // return random number in [0.0, 1.0)
    static std::random_device dev;
    static std::mt19937 rng(dev());
    static std::uniform_real_distribution<float> dist(0.f, 1.f);

    return dist(rng);
}

inline int getRandomInt(int low, int high) {
    // return random number in [low, high]
    static std::random_device dev;
    static std::mt19937 rng(dev());
    static std::uniform_int_distribution<int> dist(low, high);

    return dist(rng);
}

inline void updateProgress(float progress) {
    int barWidth = 70;

    std::cout << "[";
    int pos = barWidth * progress;
    for (int i = 0; i < barWidth; ++i) {
        if (i < pos)
            std::cout << "=";
        else if (i == pos)
            std::cout << ">";
        else
            std::cout << " ";
    }
    std::cout << "] " << int(progress * 100.0) << " %\r";
    std::cout.flush();
};
