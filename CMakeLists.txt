# cmake version
cmake_minimum_required(VERSION 4.0)

# project name
project(PP_mid_assignment)

# CXX settings
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp") # try also to compile and execute without: -fopenmp

# source files
set(SOURCE_SEQ
        seq/main_seq.cpp
        seq/boids_seq.cpp
        seq/boids_seq.h
)

set(SOURCE_OMP
        omp/main_omp.cpp
        omp/boids_omp.cpp
        omp/boids_omp.h
        omp/spatial_grid.h
)

# executables
add_executable(PP_mid_assignment_seq ${SOURCE_SEQ})
target_link_libraries(PP_mid_assignment_seq sfml-graphics sfml-window sfml-system)

add_executable(PP_mid_assignment_omp ${SOURCE_OMP})
target_link_libraries(PP_mid_assignment_omp sfml-graphics sfml-window sfml-system)
