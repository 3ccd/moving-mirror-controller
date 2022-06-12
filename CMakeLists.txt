cmake_minimum_required(VERSION 3.22)
project(moving_mirror_controller)

set(CMAKE_CXX_STANDARD 14)

add_executable(moving_mirror_controller src/main.cpp src/transform.cpp include/transform.h)

target_link_libraries(moving_mirror_controller opencv_core opencv_imgproc opencv_highgui opencv_imgcodecs)
