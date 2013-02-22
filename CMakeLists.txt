cmake_minimum_required(VERSION 2.8)
project(VisualOdometry)
add_definitions(-Wall -pedantic -g)
find_package(OpenCV REQUIRED)

set(CMAKE_BUILD_TYPE Debug)

include_directories(inc)

add_executable(visualOdometer src/OdometerMain.cpp src/VisualOdometer.cpp)
add_library(featureFilters src/FeatureFilter.cpp src/SmoothFilter.cpp src/ImageEdgeFilter.cpp)
add_library(translationReaders src/TranslationReader.cpp src/BirdsEyeTranslationReader.cpp)
add_library(rotationReaders src/RotationReader.cpp src/TangentRotationReader.cpp)
add_library(featureExtractors src/FeatureExtractor.cpp src/ShiThomasFeatureExtractor.cpp)
add_library(featureTrackers src/FeatureTracker.cpp src/LucasCandaePyramidTracker.cpp)

target_link_libraries(featureExtractors opencv_core opencv_video opencv_imgproc)
target_link_libraries(featureTrackers opencv_core opencv_video)

target_link_libraries(visualOdometer opencv_core opencv_highgui opencv_imgproc opencv_calib3d opencv_video featureFilters featureExtractors featureTrackers translationReaders rotationReaders)