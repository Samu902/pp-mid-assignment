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

set(SOURCE_OMP_AOS
        omp_aos/main_omp_aos.cpp
        omp_aos/boids_omp_aos.cpp
        omp_aos/boids_omp_aos.h
        omp_aos/spatial_grid.h
)

set(SOURCE_OMP_SOA
        omp_soa/main_omp_soa.cpp
        omp_soa/boids_omp_soa.cpp
        omp_soa/boids_omp_soa.h
        omp_soa/spatial_grid.h
)

# executables
add_executable(PP_mid_assignment_seq ${SOURCE_SEQ})
target_link_libraries(PP_mid_assignment_seq sfml-graphics sfml-window sfml-system)

add_executable(PP_mid_assignment_omp_aos ${SOURCE_OMP_AOS})
target_link_libraries(PP_mid_assignment_omp_aos sfml-graphics sfml-window sfml-system)

add_executable(PP_mid_assignment_omp_soa ${SOURCE_OMP_SOA})
target_link_libraries(PP_mid_assignment_omp_soa sfml-graphics sfml-window sfml-system)
