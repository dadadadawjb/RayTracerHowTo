#pragma once

#include <iostream>
#include <cmath>
#include <random>

constexpr double MY_PI = 3.1415926535;
constexpr float kInfinity = std::numeric_limits<float>::max();
constexpr double epsilon = 0.00001;

#define FILENAME "output.png"
#define BUNNY_OBJ "../models/bunny/bunny.obj"
#define CORNELLBOX_FLOOR_OBJ "../models/cornellbox/floor.obj"
#define CORNELLBOX_SHORTBOX_OBJ "../models/cornellbox/shortbox.obj"
#define CORNELLBOX_TALLBOX_OBJ "../models/cornellbox/tallbox.obj"
#define CORNELLBOX_LEFTWALL_OBJ "../models/cornellbox/left.obj"
#define CORNELLBOX_RIGHTWALL_OBJ "../models/cornellbox/right.obj"
#define CORNELLBOX_LIGHT_OBJ "../models/cornellbox/light.obj"
#define BUNNY_SCALE 60
// allowed configuration: 
// (DEMO=true, BUNNY=any(no influence), CORNELLBOX=any(no influence), PATH=false, ACCELERATION=false, MULTITHREADING=any(influence speed, recommended for false), SAH_ACC=any(influence speed, recommended for false), GAMMA=any(influence color at last, recommended for false))
// (DEMO=false, BUNNY=true, CORNELLBOX=false, PATH=false, ACCELERATION=true, MULTITHREADING=any(influence speed, recommended for true), SAH_ACC=any(influence speed, recommended for false), GAMMA=any(influence color at last, recommended for true))
// (DEMO=false, BUNNY=false, CORNELLBOX=true, PATH=true, ACCELERATION=true, MULTITHREADING=any(influence speed, recommended for true), SAH_ACC=any(influence speed, recommended for true), GAMMA=any(influence color at last, recommended for true))
// why DEMO=true then PATH=true cannot: no object having emission as arealight
// why DEMO=true then ACCELERATION=true cannot: Meshtriangle vertices load no supoort BVH build right now
// why BUNNY=true then PATH=true cannot: no object having emission as arealight
// why CORNELLBOX=true then PATH=false cannot: no light
// why BUNNY=true or CORNELLBOX=true then ACCELERATION=false cannot: Meshtriangle obj load no support vertices store right now
// why PATH=true then ACCELERATION=false cannot: no support sample based on no BVH right now
#define DEMO false
#define BUNNY false
#define CORNELLBOX true
#define PATH true
#define ACCELERATION true
#define MULTITHREADING true
#define SAH_ACC true
#define GAMMA true
#define GAMMA_VALUE_R 0.6
#define GAMMA_VALUE_G 0.6
#define GAMMA_VALUE_B 0.6
#define THREADS_X 5
#define THREADS_Y 5
#define WIDTH1 1280
#define HEIGHT1 960
#define FOV1 90
#define DEPTH1 5
#define WIDTH2 1024
#define HEIGHT2 1024
// #define WIDTH2 1920
// #define HEIGHT2 1920
#define FOV2 40
#define DEPTH2 1
// #define DEPTH2 5
#define BACKGROUND_R 0.235294
#define BACKGROUND_G 0.67451
#define BACKGROUND_B 0.843137
#define EYE_POS_X_1 0
#define EYE_POS_Y_1 0
#define EYE_POS_Z_1 0
#define EYE_POS_X_2 -1
#define EYE_POS_Y_2 5
#define EYE_POS_Z_2 10
#define EYE_POS_X_3 278
#define EYE_POS_Y_3 273
#define EYE_POS_Z_3 -800
#define PATH_SAMPLES 16
// #define PATH_SAMPLES 64
#define PATH_RR 0.8
#define RED_KD_R 0.63
#define RED_KD_G 0.065
#define RED_KD_B 0.05
#define GREEN_KD_R 0.14
#define GREEN_KD_G 0.45
#define GREEN_KD_B 0.091
#define WHITE_KD_R 0.725
#define WHITE_KD_G 0.71
#define WHITE_KD_B 0.68
#define LIGHT_KD_R 0.65
#define LIGHT_KD_G 0.65
#define LIGHT_KD_B 0.65
#define LIGHT_EMISSION_R 47.8348
#define LIGHT_EMISSION_G 38.5664
#define LIGHT_EMISSION_B 31.0808
#define MICROFACET_ROUGHNESS 0.35
#define MICROFACET_IOR 1.85
#define LIGHT1_X -20
#define LIGHT1_Y 70
#define LIGHT1_Z 20
#define LIGHT1_R 0.5
#define LIGHT1_G 0.5
#define LIGHT1_B 0.5
#define LIGHT2_X 30
#define LIGHT2_Y 50
#define LIGHT2_Z -12
#define LIGHT2_R 0.5
#define LIGHT2_G 0.5
#define LIGHT2_B 0.5
#define LIGHT3_X 20
#define LIGHT3_Y 70
#define LIGHT3_Z 20
#define LIGHT3_R 1
#define LIGHT3_G 1
#define LIGHT3_B 1
#define TRIANGLE_NUM 2
#define TRIANGLE_V_NUM 4
#define TRIANGLE_V1_X -5
#define TRIANGLE_V1_Y -3
#define TRIANGLE_V1_Z -6
#define TRIANGLE_V2_X 5
#define TRIANGLE_V2_Y -3
#define TRIANGLE_V2_Z -6
#define TRIANGLE_V3_X 5
#define TRIANGLE_V3_Y -3
#define TRIANGLE_V3_Z -16
#define TRIANGLE_V4_X -5
#define TRIANGLE_V4_Y -3
#define TRIANGLE_V4_Z -16
#define TRIS1_DIFFUSE_R 0.2
#define TRIS1_DIFFUSE_G 0.2
#define TRIS1_DIFFUSE_B 0.2
#define TRIS1_EMISSION_R 0
#define TRIS1_EMISSION_G 0
#define TRIS1_EMISSION_B 0
#define TRIS1_IOR 1.3
#define TRIS1_KD 0.8
#define TRIS1_KS 0.2
#define TRIS1_P 25
#define TRIS2_DIFFUSE_R 0.5
#define TRIS2_DIFFUSE_G 0.5
#define TRIS2_DIFFUSE_B 0.5
#define TRIS2_EMISSION_R 0
#define TRIS2_EMISSION_G 0
#define TRIS2_EMISSION_B 0
#define TRIS2_KD 0.6
#define TRIS2_KS 0
#define TRIS2_P 0
#define SPHERE1_C_X -1
#define SPHERE1_C_Y 0
#define SPHERE1_C_Z -12
#define SPHERE1_R 2
#define SPHERE1_IOR 1.3
#define SPHERE1_KD 0.8
#define SPHERE1_KS 0.2
#define SPHERE1_P 25
#define SPHERE1_DIFFUSE_R 0.6
#define SPHERE1_DIFFUSE_G 0.7
#define SPHERE1_DIFFUSE_B 0.8
#define SPHERE1_EMISSION_R 0
#define SPHERE1_EMISSION_G 0
#define SPHERE1_EMISSION_B 0
#define SPHERE2_C_X 0.5
#define SPHERE2_C_Y -0.5
#define SPHERE2_C_Z -8
#define SPHERE2_R 1.5
#define SPHERE2_IOR 1.5
#define SPHERE2_KD 0.8
#define SPHERE2_KS 0.2
#define SPHERE2_P 25
#define SPHERE2_DIFFUSE_R 0.2
#define SPHERE2_DIFFUSE_G 0.2
#define SPHERE2_DIFFUSE_B 0.2
#define SPHERE2_EMISSION_R 0
#define SPHERE2_EMISSION_G 0
#define SPHERE2_EMISSION_B 0

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

inline float get_random_float() {
    // return random number in [0.0, 1.0)
    static std::random_device dev;
    static std::mt19937 rng(dev());
    static std::uniform_real_distribution<float> dist(0.f, 1.f);

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
