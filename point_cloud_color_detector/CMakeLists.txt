cmake_minimum_required(VERSION 3.0.2)
project(point_cloud_color_detector)

add_compile_options(-O3)
add_compile_options(-mtune=native)
add_compile_options(-march=native)
add_compile_options(-mfpmath=both)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    pcl_ros
    color_detector_msgs
)

catkin_package(
    INCLUDE_DIRS include
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(
    point_cloud_color_detector src/point_cloud_color_detector.cpp
)

add_dependencies(
    point_cloud_color_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(point_cloud_color_detector
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)