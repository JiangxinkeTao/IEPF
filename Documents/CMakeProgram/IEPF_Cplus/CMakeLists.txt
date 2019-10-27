
cmake_minimum_required(VERSION 3.15)
project(IEPF_Cplus)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(include/)
set(SRC_PATH src/)

add_library(libsrc
        ${SRC_PATH}/test.cpp
        ${SRC_PATH}/vehiclemodel.cpp)

#include_directories(src/)
link_directories(lib)
add_executable(IEPF_Cplus src/test.cpp)

target_link_libraries(IEPF_Cplus dispoline.a)
target_link_libraries(IEPF_Cplus libsrc)
