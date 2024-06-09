#include <chrono>

#include <eigen3/Eigen/Eigen>
#include <opencv4/opencv2/opencv.hpp>

#include "vector.hpp"
#include "global.hpp"
#include "scene.hpp"
#include "camera.hpp"
#include "raytracer.hpp"
#include "material.hpp"
#include "sphere.hpp"
#include "cylinder.hpp"
#include "cone.hpp"
#include "triangle.hpp"
#include "light.hpp"


int main(int argc, char **argv) {
    // initialize scene
    Scene scene;

    // add primitives
    Material *whiteMat = new Material(MaterialType::DIFFUSE, Vector3f(WHITE_KA_R, WHITE_KA_G, WHITE_KA_B), 
                                      Vector3f(WHITE_KD_R, WHITE_KD_G, WHITE_KD_B), Vector3f(WHITE_KS_R, WHITE_KS_G, WHITE_KS_B), RED_SE);
    Material *redMat = new Material(MaterialType::DIFFUSE, Vector3f(RED_KA_R, RED_KA_G, RED_KA_B), 
                                    Vector3f(RED_KD_R, RED_KD_G, RED_KD_B), Vector3f(RED_KS_R, RED_KS_G, RED_KS_B), RED_SE);
    Material *greenMat = new Material(MaterialType::DIFFUSE, Vector3f(GREEN_KA_R, GREEN_KA_G, GREEN_KA_B), 
                                      Vector3f(GREEN_KD_R, GREEN_KD_G, GREEN_KD_B), Vector3f(GREEN_KS_R, GREEN_KS_G, GREEN_KS_B), GREEN_SE);
    Material *reflectMat = new Material(MaterialType::REFLECTION);
    Material *refractMat = new Material(MaterialType::REFRACTION, REFRACTION_IOR);
    Material *reflectRefractMat = new Material(MaterialType::REFLECTION_AND_REFRACTION, REFLECTION_REFRACTION_IOR);
    Material *lightMat1 = new Material(MaterialType::EMISSION, Vector3f(LIGHT_INTENSITY_R1, LIGHT_INTENSITY_G1, LIGHT_INTENSITY_B1));

    MeshTriangle floor(CORNELLBOX_FLOOR_OBJ, whiteMat, "floor");
    MeshTriangle left(CORNELLBOX_LEFTWALL_OBJ, redMat, "left");
    MeshTriangle right(CORNELLBOX_RIGHTWALL_OBJ, greenMat, "right");
    MeshTriangle shortbox(CORNELLBOX_SHORTBOX_OBJ, reflectRefractMat, "shortbox");
    MeshTriangle tallbox(CORNELLBOX_TALLBOX_OBJ, reflectMat, "tallbox");
    Sphere sphere(Vector3f(SPHERE_POS_X, SPHERE_POS_Y, SPHERE_POS_Z), SPHERE_RADIUS, refractMat, "sphere");
    Cylinder cylinder(Vector3f(CYLINDER_POS_X, CYLINDER_POS_Y, CYLINDER_POS_Z), 
                      Vector3f(CYLINDER_DIR_X, CYLINDER_DIR_Y, CYLINDER_DIR_Z), 
                      CYLINDER_RADIUS, CYLINDER_HEIGHT, whiteMat, "cylinder");
    Cone cone(Vector3f(CONE_POS_X, CONE_POS_Y, CONE_POS_Z), 
              Vector3f(CYLINDER_DIR_X, CYLINDER_DIR_Y, CYLINDER_DIR_Z), 
              CONE_RADIUS, CONE_HEIGHT, whiteMat, "cone");
    MeshTriangle light1(CORNELLBOX_LIGHT_OBJ, lightMat1, "light1");
    Light light2(Vector3f(LIGHT_POX_X2, LIGHT_POX_Y2, LIGHT_POX_Z2), Vector3f(LIGHT_INTENSITY_R2, LIGHT_INTENSITY_G2, LIGHT_INTENSITY_B2));

    scene.add(&floor);
    scene.add(&left);
    scene.add(&right);
    scene.add(&shortbox);
    scene.add(&tallbox);
    scene.add(&sphere);
    scene.add(&cylinder);
    scene.add(&cone);
    scene.add(&light1);
    scene.add(&light2);

    // accelerate
    if (IS_BVH)
        scene.buildBVH();
    
    // set camera
    Camera camera(WIDTH, HEIGHT, FOV, Vector3f(EYE_POS_X, EYE_POS_Y, EYE_POS_Z), 
                    Vector3f(EYE_FRONT_X, EYE_FRONT_Y, EYE_FRONT_Z), 
                    Vector3f(EYE_UP_X, EYE_UP_Y, EYE_UP_Z));

    // ray tracing
    RayTracer r;
    auto start = std::chrono::system_clock::now();
    r.render(scene, camera);
    cv::Mat image(HEIGHT, WIDTH, CV_32FC3, r.capture().data());
    image.convertTo(image, CV_8UC3, 1.0f);
    cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
    cv::imwrite(FILENAME, image);
    auto stop = std::chrono::system_clock::now();

    std::cout << "Render complete: " << std::endl;
    std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::hours>(stop - start).count() << " hours" << std::endl;
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::minutes>(stop - start).count() << " minutes" << std::endl;
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() << " seconds" << std::endl;

    return 0;
}
