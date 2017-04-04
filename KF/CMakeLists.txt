cmake_minimum_required(VERSION 3.7)
project(KF)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES
        include/kf_2d.h
        include/measurement_package.h
        include/tracking.h
        src/kf_2d.cpp
        src/main.cpp
        src/tracking.cpp)

include_directories(
        include/
)

add_executable(KF ${SOURCE_FILES})