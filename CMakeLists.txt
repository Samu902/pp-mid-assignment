# cmake version
cmake_minimum_required(VERSION 4.0)

# project name
project(PP_mid_assignment)

# CXX settings
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp") # try also to compile and execute without: -fopenmp

# source files
set(SOURCE_FILES main.cpp
        boids.cpp
        boids.h
)
add_executable(PP_mid_assignment ${SOURCE_FILES})

# external libs
target_link_libraries(${PROJECT_NAME} sfml-graphics sfml-window sfml-system)
