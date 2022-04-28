#include <chrono>

#include <eigen3/Eigen/Eigen>
#include <opencv4/opencv2/opencv.hpp>

#include "Vector.hpp"
#include "global.hpp"
#include "Scene.hpp"
#include "Triangle.hpp"
#include "Sphere.hpp"
#include "Material.hpp"
#include "raytracer.hpp"


int main(int argc, char **argv) {
    if (DEMO) {
        // initialize scene
        Scene scene(WIDTH1, HEIGHT1);

        // add primitives
        Material *material1 = new Material(MaterialType::DIFFUSE, 
                                            Vector3f(SPHERE1_EMISSION_R, SPHERE1_EMISSION_G, SPHERE1_EMISSION_B), 
                                            Vector3f(SPHERE1_DIFFUSE_R, SPHERE1_DIFFUSE_G, SPHERE1_DIFFUSE_B));
        material1->ior = SPHERE1_IOR;
        material1->Kd = SPHERE1_KD;
        material1->Ks = SPHERE1_KS;
        material1->specularExponent = SPHERE1_P;
        Sphere *sph1 = new Sphere(Vector3f(SPHERE1_C_X, SPHERE1_C_Y, SPHERE1_C_Z), SPHERE1_R, material1);

        Material *material2 = new Material(MaterialType::REFLECTION_AND_REFRACTION, 
                                            Vector3f(SPHERE2_EMISSION_R, SPHERE2_EMISSION_G, SPHERE2_EMISSION_B), 
                                            Vector3f(SPHERE2_DIFFUSE_R, SPHERE2_DIFFUSE_G, SPHERE2_DIFFUSE_B));
        material2->ior = SPHERE2_IOR;
        material2->Kd = SPHERE2_KD;
        material2->Ks = SPHERE2_KS;
        material2->specularExponent = SPHERE2_P;
        Sphere *sph2 = new Sphere(Vector3f(SPHERE2_C_X, SPHERE2_C_Y, SPHERE2_C_Z), SPHERE2_R, material2);

        scene.Add(sph1);
        scene.Add(sph2);

        Material *material3 = new Material(MaterialType::DIFFUSE, 
                                            Vector3f(TRIS1_EMISSION_R, TRIS1_EMISSION_G, TRIS1_EMISSION_B), 
                                            Vector3f(TRIS1_DIFFUSE_R, TRIS1_DIFFUSE_G, TRIS1_DIFFUSE_B));
        material3->ior = TRIS1_IOR;
        material3->Kd = TRIS1_KD;
        material3->Ks = TRIS1_KS;
        material3->specularExponent = TRIS1_P;
        Vector3f verts[TRIANGLE_V_NUM] = {{TRIANGLE_V1_X, TRIANGLE_V1_Y, TRIANGLE_V1_Z}, 
                                          {TRIANGLE_V2_X, TRIANGLE_V2_Y, TRIANGLE_V2_Z}, 
                                          {TRIANGLE_V3_X, TRIANGLE_V3_Y, TRIANGLE_V3_Z}, 
                                          {TRIANGLE_V4_X, TRIANGLE_V4_Y, TRIANGLE_V4_Z}};
        uint32_t vertIndex[TRIANGLE_NUM * 3] = {0, 1, 3, 1, 2, 3};
        Vector2f st[TRIANGLE_V_NUM] = {{0, 0}, {1, 0}, {1, 1}, {0, 1}};
        MeshTriangle *mesh = new MeshTriangle(verts, vertIndex, TRIANGLE_NUM, st, material3);
        scene.Add(mesh);

        scene.Add(std::make_unique<Light>(Vector3f(LIGHT1_X, LIGHT1_Y, LIGHT1_Z), 
                                          Vector3f(LIGHT1_R, LIGHT1_G, LIGHT1_B)));
        scene.Add(std::make_unique<Light>(Vector3f(LIGHT2_X, LIGHT2_Y, LIGHT2_Z), 
                                          Vector3f(LIGHT2_R, LIGHT2_G, LIGHT2_B)));

        // accelerate
        if (ACCELERATION)
            scene.buildBVH();

        // set eye position
        Vector3f eye_position(EYE_POS_X_1, EYE_POS_Y_1, EYE_POS_Z_1);

        // ray tracing
        RayTracer r;
        auto start = std::chrono::system_clock::now();
        r.RayTracing(scene, eye_position);
        cv::Mat image(scene.height, scene.width, CV_32FC3, r.get_frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
        cv::imwrite(FILENAME, image);
        auto stop = std::chrono::system_clock::now();

        std::cout << "Render complete: " << std::endl;
        std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::hours>(stop - start).count() << " hours" << std::endl;
        std::cout << "          : " << std::chrono::duration_cast<std::chrono::minutes>(stop - start).count() << " minutes" << std::endl;
        std::cout << "          : " << std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() << " seconds" << std::endl;
    } else {
        if (BUNNY) {
            // initialize scene
            Scene scene(WIDTH1, HEIGHT1);

            // add primitives
            Material *triangles = new Material(MaterialType::DIFFUSE, 
                                                Vector3f(TRIS2_EMISSION_R, TRIS2_EMISSION_G, TRIS2_EMISSION_B), 
                                                Vector3f(TRIS2_DIFFUSE_R, TRIS2_DIFFUSE_G, TRIS2_DIFFUSE_B));
            triangles->Kd = TRIS2_KD;
            triangles->Ks = TRIS2_KS;
            triangles->specularExponent = TRIS2_P;
            MeshTriangle bunny(BUNNY_OBJ, triangles);
            scene.Add(&bunny);

            scene.Add(std::make_unique<Light>(Vector3f(LIGHT1_X, LIGHT1_Y, LIGHT1_Z), 
                                              Vector3f(LIGHT1_R, LIGHT1_G, LIGHT1_B)));
            scene.Add(std::make_unique<Light>(Vector3f(LIGHT3_X, LIGHT3_Y, LIGHT3_Z), 
                                              Vector3f(LIGHT3_R, LIGHT3_G, LIGHT3_B)));

            // accelerate
            if (ACCELERATION)
                scene.buildBVH();
            
            // set eye position
            Vector3f eye_position(EYE_POS_X_2, EYE_POS_Y_2, EYE_POS_Z_2);

            // ray tracing
            RayTracer r;
            auto start = std::chrono::system_clock::now();
            r.RayTracing(scene, eye_position);
            cv::Mat image(scene.height, scene.width, CV_32FC3, r.get_frame_buffer().data());
            image.convertTo(image, CV_8UC3, 1.0f);
            cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
            cv::imwrite(FILENAME, image);
            auto stop = std::chrono::system_clock::now();

            std::cout << "Render complete: " << std::endl;
            std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::hours>(stop - start).count() << " hours" << std::endl;
            std::cout << "          : " << std::chrono::duration_cast<std::chrono::minutes>(stop - start).count() << " minutes" << std::endl;
            std::cout << "          : " << std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() << " seconds" << std::endl;
        } else if (CORNELLBOX) {
            // initialize scene
            Scene scene(WIDTH2, HEIGHT2);
            scene.fov = FOV2;
            scene.maxDepth = DEPTH2;

            // add primitives
            Material *red = new Material(MaterialType::DIFFUSE);
            red->Kd = Vector3f(RED_KD_R, RED_KD_G, RED_KD_B);
            Material *green = new Material(MaterialType::DIFFUSE);
            green->Kd = Vector3f(GREEN_KD_R, GREEN_KD_G, GREEN_KD_B);
            Material *white = new Material(MaterialType::DIFFUSE);
            white->Kd = Vector3f(WHITE_KD_R, WHITE_KD_G, WHITE_KD_B);
            Material *light = new Material(MaterialType::DIFFUSE, 
                                            Vector3f(LIGHT_EMISSION_R, LIGHT_EMISSION_G, LIGHT_EMISSION_B));
            light->Kd = Vector3f(LIGHT_KD_R, LIGHT_KD_G, LIGHT_KD_B);

            MeshTriangle floor(CORNELLBOX_FLOOR_OBJ, white);
            MeshTriangle shortbox(CORNELLBOX_SHORTBOX_OBJ, white);
            MeshTriangle tallbox(CORNELLBOX_TALLBOX_OBJ, white);
            MeshTriangle left(CORNELLBOX_LEFTWALL_OBJ, red);
            MeshTriangle right(CORNELLBOX_RIGHTWALL_OBJ, green);
            MeshTriangle light_(CORNELLBOX_LIGHT_OBJ, light);

            scene.Add(&floor);
            scene.Add(&shortbox);
            scene.Add(&tallbox);
            scene.Add(&left);
            scene.Add(&right);
            scene.Add(&light_);

            // accelerate
            if (ACCELERATION)
                scene.buildBVH();
            
            // set eye position
            Vector3f eye_position(EYE_POS_X_3, EYE_POS_Y_3, EYE_POS_Z_3);

            // ray tracing
            RayTracer r;
            auto start = std::chrono::system_clock::now();
            r.RayTracing(scene, eye_position);
            cv::Mat image(scene.height, scene.width, CV_32FC3, r.get_frame_buffer().data());
            image.convertTo(image, CV_8UC3, 1.0f);
            cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
            cv::imwrite(FILENAME, image);
            auto stop = std::chrono::system_clock::now();

            std::cout << "Render complete: " << std::endl;
            std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::hours>(stop - start).count() << " hours" << std::endl;
            std::cout << "          : " << std::chrono::duration_cast<std::chrono::minutes>(stop - start).count() << " minutes" << std::endl;
            std::cout << "          : " << std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() << " seconds" << std::endl;
        }
    }

    return 0;
}