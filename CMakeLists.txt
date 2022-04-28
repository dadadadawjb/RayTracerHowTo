cmake_minimum_required(VERSION 3.10)
project(RayTracerHowTo)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

include_directories(/usr/local/include)

add_executable(RayTracerHowTo main.cpp Vector.hpp global.hpp OBJ_Loader.hpp 
        Scene.hpp Scene.cpp Object.hpp Sphere.hpp Triangle.hpp Light.hpp Light.cpp 
        BVH.hpp BVH.cpp AABB.hpp Ray.hpp Material.hpp Intersection.hpp 
        raytracer.hpp raytracer.cpp)
target_link_libraries(RayTracerHowTo ${OpenCV_LIBRARIES})
