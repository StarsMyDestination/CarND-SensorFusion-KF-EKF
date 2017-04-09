cmake_minimum_required(VERSION 3.7)
project(EKF)

set(CMAKE_CXX_STANDARD 11)

include_directories(
        include/
)

set(SOURCE_FILES
        include/FusionEKF.h
        include/ground_truth_package.h
        include/kalman_filter.h
        include/measurement_package.h
        include/tools.h
        src/FusionEKF.cpp
        src/kalman_filter.cpp
        src/main.cpp
        src/tools.cpp)

add_executable(EKF ${SOURCE_FILES})