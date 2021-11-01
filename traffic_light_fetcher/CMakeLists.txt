cmake_minimum_required(VERSION 3.0.0)
project(traffic_light_fetcher VERSION 1.0.0)
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Werror -Wextra)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
)

catkin_package()

set(TRAFFIC_LIGHT_DETECTOR traffic_light_detector)
file(GLOB TRAFFIC_LIGHT_DETECTOR_SRC CONFIGURE_DEPENDS src/${TRAFFIC_LIGHT_DETECTOR}/*.cpp)

add_library(${TRAFFIC_LIGHT_DETECTOR} SHARED
    ${TRAFFIC_LIGHT_DETECTOR_SRC}
)

target_link_libraries(${TRAFFIC_LIGHT_DETECTOR} PUBLIC
    ${OpenCV_LIBRARIES}
)

target_include_directories(${TRAFFIC_LIGHT_DETECTOR} PUBLIC
    include/${TRAFFIC_LIGHT_DETECTOR}
)

set(TRAFFIC_LIGHT_FETCHER traffic_light_fetcher)
file(GLOB TRAFFIC_LIGHT_FETCHER_SRC CONFIGURE_DEPENDS src/${TRAFFIC_LIGHT_FETCHER}/*.cpp)

add_library(${TRAFFIC_LIGHT_FETCHER} SHARED
    ${TRAFFIC_LIGHT_FETCHER_SRC}
)

target_link_libraries(${TRAFFIC_LIGHT_FETCHER} PUBLIC
    ${catkin_LIBRARIES}    
    ${TRAFFIC_LIGHT_DETECTOR}
)

target_include_directories(${TRAFFIC_LIGHT_FETCHER} PUBLIC
    ${catkin_INCLUDE_DIRS}   
    include/${TRAFFIC_LIGHT_FETCHER}
)

add_executable(traffic_light_fetcher_node
    src/main.cpp
)

target_link_libraries(traffic_light_fetcher_node PRIVATE
    ${catkin_LIBRARIES}    
    ${TRAFFIC_LIGHT_FETCHER}
)

target_include_directories(traffic_light_fetcher_node PUBLIC
    ${catkin_INCLUDE_DIRS}   
)

if (CATKIN_ENABLE_TESTING)
    add_subdirectory(test)
endif()