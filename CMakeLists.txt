cmake_minimum_required(VERSION 3.20)
project(boids)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package (Eigen3 REQUIRED NO_MODULE)
find_package (OpenMP)

add_library(game_engine INTERFACE)
target_include_directories(game_engine INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(main main.cpp)

target_link_libraries(main PRIVATE game_engine Eigen3::Eigen)

if(OpenMP_CXX_FOUND)
    target_link_libraries(main PRIVATE OpenMP::OpenMP_CXX)
endif()

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_libraries(main PRIVATE X11 GL pthread png asound)
endif()
