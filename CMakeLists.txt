cmake_minimum_required(VERSION 3.10)
project(pcl_example)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PCL 1.9.1 REQUIRED COMPONENTS common io geometry kdtree search features surface visualization filters octree)

# include dir
include_directories(${PCL_INCLUDE_DIRS})
include_directories(include)

# library dir
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# src file
add_executable(pcl_example src/main.cpp src/GaussianCodec.cpp)

# link
target_link_libraries(pcl_example ${PCL_LIBRARIES})

set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")  # -g: 디버그 정보 포함, -O0: 최적화 없음
set(CMAKE_CXX_FLAGS_RELEASE "-O3")  # -O3: 최적화 수준 3

add_definitions(-DDEBUG)

if(CMAKE_BUILD_TYPE MATCHES Debug)
    message("Building in debug mode.")
    add_definitions(-DDEBUG_MODE)
endif()

set(CMAKE_BUILD_TYPE Debug)