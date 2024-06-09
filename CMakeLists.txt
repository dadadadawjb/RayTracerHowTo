cmake_minimum_required(VERSION 3.10)
project(RayTracerHowTo)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

include_directories(/usr/local/include)

add_executable(RayTracerHowTo main.cpp vector.hpp global.hpp scene.hpp scene.cpp 
        camera.hpp aabb.hpp bvh.hpp bvh.cpp intersection.hpp light.hpp light.cpp 
        material.hpp ray.hpp raytracer.hpp raytracer.cpp object.hpp OBJ_loader.hpp 
        triangle.hpp sphere.hpp)
target_link_libraries(RayTracerHowTo ${OpenCV_LIBRARIES})
