/**
 * \file
 * \date 2.06.2013
 * \author Michał Orynicz
 */

#ifndef COMMANDLINEPARAMETERS_HPP_
#define COMMANDLINEPARAMETERS_HPP_

#include "ShiThomasiFeatureExtractor.hpp"
#include "LucasKanadePyramidTracker.hpp"
#include "PreparationFunctions.hpp"

#include <string>

/// Method saves parameters given to a YAML file

/**
 * Method writes structures passed as arguments to a YAML file
 * @param fileName - name of the file in which parameters shall be saved
 * @param shiThomasi - parameters of ShiThomasiFeatureExtractor
 * @param lucasKanade - parameters of LucasKanadeFeatureTracker
 * @param chessboard - chessboard parameters
 * @param upperMaxFeatures - max features in upper part of the image
 * @param lowerMaxFeatures - max features in the lower part of the image
 * @param maxHistoryLength - max previous positions remembered by movement
 * readers
 * @param upperMargin - margin (in px) from image edges in which features are
 * not tracked in lower part of the image
 * @param lowerMargin - margin (in px) from image edges in which features are
 * not tracked in upper part of the image
 */
void saveParameters(const std::string &fileName,
        ShiThomasiParameters &shiThomasi,
        LucasKanadeParameters &lucasKanade,
        ChessboardParameters &chessboard, int &upperMaxFeatures,
        int &lowerMaxFeatures, int &maxHistoryLength,
        double &upperMargin, double &lowerMargin);

/// Method reads parameters from YAML file

/**
 * Method reads values of provided structures from YAML file
 * @param fileName
 * @param shiThomasi - parameters of ShiThomasiFeatureExtractor
 * @param lucasKanade - parameters of LucasKanadeFeatureTracker
 * @param chessboard - chessboard parameters
 * @param upperMaxFeatures - max features in upper part of the image
 * @param lowerMaxFeatures - max features in the lower part of the image
 * @param maxHistoryLength - max previous positions remembered by movement
 * readers
 * @param upperMargin - margin (in px) from image edges in which features are
 * not tracked in lower part of the image
 * @param lowerMargin - margin (in px) from image edges in which features are
 * not tracked in upper part of the image
 */
void readParameters(const std::string &fileName,
        ShiThomasiParameters &shiThomasi,
        LucasKanadeParameters &lucasKanade,
        ChessboardParameters &chessboard, int &upperMaxFeatures,
        int &lowerMaxFeatures, int &maxHistoryLength,
        double &upperMargin, double &lowerMargin);

/// Function parses command line arguments and sets values to it's arguments
/// accordingly. Version for "live" camera.

/**
 * Function parsing parameters passed through command line and setting
 * values to structures passed as arguments. Version for "live" camera.
 * @param argc - number of strings passed to command line
 * @param argv - array of strings passed through command line
 * @param shiThomasi - parameters of ShiThomasiFeatureExtractor
 * @param lucasKanade - parameters of LucasKanadeFeatureTracker
 * @param chessboard - chessboard parameters
 * @param spatialParameters - spatial parameters of camera (world coordinates,
 * horizon and dead zone values and birds eye transform matrix)
 * @param upperMaxFeatures - max features in upper part of the image
 * @param lowerMaxFeatures - max features in the lower part of the image
 * @param maxHistoryLength - max previous positions remembered by movement
 * readers
 * @param upperMargin - margin (in px) from image edges in which features are
 * not tracked in lower part of the image
 * @param lowerMargin - margin (in px) from image edges in which features are
 * not tracked in upper part of the image* @param captureDeviceNumber
 * @param captureDeviceNumber - number of the device used to get images
 * @param verbosity parameter sets how verbose the program should be
 * @param cameraParametersFilename - filename from which parameters of camera
 * (camera matrix, distortion coefficients) should be read
 * @param matlabFileName - name of file into which the calculated positions
 * will be written
 * @param cameraPositionFilename - name of file into which parameters like
 * birds eye view homography and horizon height will be written.
 */
void readParametersCommandLine(const int &argc, char **argv,
        ShiThomasiParameters &shiThomasi,
        LucasKanadeParameters &lucasKanade,
        ChessboardParameters &chessboard,
        CameraSpatialParameters &spatialParameters,
        int &upperMaxFeatures, int &lowerMaxFeatures,
        int &maxHistoryLength, double &upperMargin,
        double &lowerMargin, int &captureDeviceNumber, int &verbosity,
        std::string &cameraParametersFilename,
        std::string &matlabFileName,
        std::string &cameraPositionFilename);


/// Function parses command line arguments and sets values to it's arguments
/// accordingly. Version for video file.

/**
 * Function parsing parameters passed through command line and setting
 * values to structures passed as arguments. Version for video file.
 * @param argc - number of strings passed to command line
 * @param argv - array of strings passed through command line
 * @param shiThomasi - parameters of ShiThomasiFeatureExtractor
 * @param lucasKanade - parameters of LucasKanadeFeatureTracker
 * @param chessboard - chessboard parameters
 * @param spatialParameters - spatial parameters of camera (world coordinates,
 * horizon and dead zone values and birds eye transform matrix)
 * @param upperMaxFeatures - max features in upper part of the image
 * @param lowerMaxFeatures - max features in the lower part of the image
 * @param maxHistoryLength - max previous positions remembered by movement
 * readers
 * @param upperMargin - margin (in px) from image edges in which features are
 * not tracked in lower part of the image
 * @param lowerMargin - margin (in px) from image edges in which features are
 * not tracked in upper part of the image* @param captureDeviceNumber
 * @param videoFile - name of video file from which image frames will be drawn
 * @param verbosity parameter sets how verbose the program should be
 * @param cameraParametersFilename - filename from which parameters of camera
 * (camera matrix, distortion coefficients) should be read
 * @param matlabFileName - name of file into which the calculated positions
 * will be written
 * @param cameraPositionFilename - name of file into which parameters like
 * birds eye view homography and horizon height will be written.
 */
void readParametersCommandLine(const int &argc, char **argv,
        ShiThomasiParameters &shiThomasi,
        LucasKanadeParameters &lucasKanade,
        ChessboardParameters &chessboard,
        CameraSpatialParameters &spatialParameters,
        int &upperMaxFeatures, int &lowerMaxFeatures,
        int &maxHistoryLength, double &upperMargin,
        double &lowerMargin, std::string &videoFile, int &verbosity,
        std::string &cameraParametersFilename,
        std::string &matlabFileName,
        std::string &cameraPositionFilename) ;
#endif /* COMMANDLINEPARAMETERS_HPP_ */
