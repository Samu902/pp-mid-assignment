cmake_minimum_required(VERSION 4.0)
project(PP_mid_assignment)

set(CMAKE_CXX_STANDARD 20)

# try also to compile and execute without: -fopenmp
# in Linux just use -fopenmp, without -lomp and -Xpreprocessor
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Xpreprocessor -fopenmp -lomp") #commentiamo
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp") #commentiamo
# the following 2 lines are needed on macOS. Check the position of headers in libraries (e.g. Macports vs. Homebrew)
include_directories(/usr/local/include)
link_directories(/usr/local/lib)

set(SOURCE_FILES main.cpp)
add_executable(PP_mid_assignment ${SOURCE_FILES})