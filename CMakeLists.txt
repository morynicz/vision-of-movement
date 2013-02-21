cmake_minimum_required(VERSION 2.8)
project(VisualOdometry)
add_definitions(-Wall -pedantic -g)
find_package(OpenCV REQUIRED)

#add_executable(visualOdometry src/visualOdometry.cpp)
#target_link_libraries(visualOdometry opencv_core opencv_highgui opencv_imgproc opencv_calib3d opencv_video)