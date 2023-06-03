cmake_minimum_required(VERSION 3.10)
project(Extraction)

FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)

ADD_EXECUTABLE(Extraction main.cpp array.hpp camera.hpp curve.hpp global.hpp image_process.hpp mat.hpp net.hpp quat.hpp vec.hpp)
target_link_libraries(Extraction ${OpenCV_LIBRARIES})
