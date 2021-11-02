cmake_minimum_required(VERSION 3.0.0)
project(traffic_light_analysis VERSION 1.0.0)
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Werror -Wextra)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
)

catkin_package()

set(TRAFFIC_LIGHT_ANALYSIS traffic_light_analysis)
file(GLOB TRAFFIC_LIGHT_ANALYSIS_SRC CONFIGURE_DEPENDS src/${TRAFFIC_LIGHT_ANALYSIS}/*.cpp)

add_library(${TRAFFIC_LIGHT_ANALYSIS} SHARED
    ${TRAFFIC_LIGHT_ANALYSIS_SRC}
)

target_link_libraries(${TRAFFIC_LIGHT_ANALYSIS} PUBLIC
    ${catkin_LIBRARIES}
)

target_include_directories(${TRAFFIC_LIGHT_ANALYSIS} PUBLIC
    include/${TRAFFIC_LIGHT_ANALYSIS}
    ${catkin_INCLUDE_DIRS}
)

add_executable(traffic_light_analysis_node
    src/main.cpp
)

target_link_libraries(traffic_light_analysis_node PRIVATE
    ${catkin_LIBRARIES}
    ${TRAFFIC_LIGHT_ANALYSIS}
)
