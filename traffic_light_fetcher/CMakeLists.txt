cmake_minimum_required(VERSION 3.0.0)
project(traffic_light_fetcher VERSION 1.0.0)
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Werror -Wextra)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

set(TRAFFIC_LIGHT_DETECTOR traffic_light_detector)
file(GLOB TRAFFIC_LIGHT_DETECTOR_SRC CONFIGURE_DEPENDS src/${TRAFFIC_LIGHT_DETECTOR}/*.cpp)

add_library(${TRAFFIC_LIGHT_DETECTOR} SHARED
    ${TRAFFIC_LIGHT_DETECTOR_SRC}
)

target_link_libraries(${TRAFFIC_LIGHT_DETECTOR} PUBLIC
    Eigen3::Eigen
    ${OpenCV_LIBRARIES}
)

target_include_directories(${TRAFFIC_LIGHT_DETECTOR} PUBLIC
    include/${TRAFFIC_LIGHT_DETECTOR}
)

add_subdirectory(test)