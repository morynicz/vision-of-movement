cmake_minimum_required(VERSION 2.8)
project(VisualOdometry C CXX)
add_definitions(-Wall -pedantic -g)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED thread program_options chrono filesystem)
find_package(Threads)
set(CMAKE_BUILD_TYPE Debug)

include_directories(inc)

add_executable(visualOdometer src/OdometerMain.cpp src/VisualOdometer.cpp src/PreparationFunctions.cpp src/DrawingFunctions.cpp src/ConvenienceFunctions.cpp src/CvTypesIo.cpp src/CommandlineParameters.cpp) 
add_library(featureFilters src/FeatureFilter.cpp src/SmoothFilter.cpp src/ImageEdgeFilter.cpp src/TrimHistoryFilter.cpp src/ConvenienceFunctions.cpp)
add_library(translationReaders src/TranslationReader.cpp src/BirdsEyeTranslationReader.cpp)
add_library(rotationReaders src/RotationReader.cpp src/TangentRotationReader.cpp)
add_library(movementReaders src/MovementReader.cpp)
add_library(featureExtractors src/FeatureExtractor.cpp src/ShiThomasiFeatureExtractor.cpp src/ConvenienceFunctions.cpp)
add_library(featureTrackers src/FeatureTracker.cpp src/LucasKanadePyramidTracker.cpp)
add_library(capturing src/Catcher.cpp)

target_link_libraries(featureExtractors opencv_core opencv_video opencv_imgproc)
target_link_libraries(featureTrackers opencv_core opencv_video)
target_link_libraries(translationReaders featureFilters movementReaders)
target_link_libraries(rotationReaders movementReaders)
target_link_libraries(capturing boost_thread)

target_link_libraries(visualOdometer opencv_core opencv_highgui opencv_imgproc opencv_calib3d opencv_video featureFilters featureExtractors featureTrackers translationReaders rotationReaders capturing boost_program_options)

add_executable(cameraCalibration src/calibration.cpp)
target_link_libraries(cameraCalibration opencv_core opencv_highgui opencv_calib3d opencv_imgproc opencv_features2d)

add_executable(odometryTests src/visualOdometry.cpp)
target_link_libraries(odometryTests opencv_core opencv_highgui opencv_imgproc opencv_calib3d opencv_video featureFilters)

add_executable(serial src/serialPortTest.cpp)
target_link_libraries(serial ${CMAKE_THREAD_LIBS_INIT})
set_target_properties(serial PROPERTIES COMPILE_FLAGS -std=c++11)
#set_target_properties(serial PROPERTIES COMPILE_FLAGS -pthread)
#g++ -o serial ../MGR/src/SerialPortTest.cpp -lboost_thread -g -std=c++11 -pthread

#-- Add an Option to toggle the generation of the API documentation
option(BUILD_DOCUMENTATION "Use Doxygen to create the HTML based API documentation" ON)
if(BUILD_DOCUMENTATION)
  FIND_PACKAGE(Doxygen)
  if (NOT DOXYGEN_FOUND)
    message(FATAL_ERROR
      "Doxygen is needed to build the documentation. Please install it correctly")
  endif()
  #-- Configure the Template Doxyfile for our specific project
  configure_file(Doxyfile.in
                 ${PROJECT_BINARY_DIR}/Doxyfile  @ONLY IMMEDIATE)
  #-- Add a custom target to run Doxygen when ever the project is built
  add_custom_target (doc #ALL
    COMMAND ${DOXYGEN_EXECUTABLE} ${PROJECT_BINARY_DIR}/Doxyfile
    SOURCES ${PROJECT_BINARY_DIR}/Doxyfile)
  # IF you do NOT want the documentation to be generated EVERY time you build the project
  # then leave out the 'ALL' keyword from the above command.
endif()

configure_file(config/parameters.yml ${PROJECT_BINARY_DIR}/defaultParameters.yml COPYONLY)