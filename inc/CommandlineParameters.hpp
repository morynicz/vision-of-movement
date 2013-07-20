/*
 * CommandlineParameters.hpp
 *
 *  Created on: Jun 2, 2013
 *      Author: link
 */

#ifndef COMMANDLINEPARAMETERS_HPP_
#define COMMANDLINEPARAMETERS_HPP_

#include "ShiThomasiFeatureExtractor.hpp"
#include "LucasKanadePyramidTracker.hpp"
#include "PreparationFunctions.hpp"

#include <string>

void saveParameters(const std::string &fileName,
        ShiThomasiParameters &shiThomasi,
        LucasKanadeParameters &lucasKanade,
        ChessboardParameters &chessboard, int &upperMaxFeatures,
        int &lowerMaxFeatures, int &maxHistoryLength,
        double &upperMargin, double &lowerMargin);

void readParameters(const std::string &fileName,
        ShiThomasiParameters &shiThomasi,
        LucasKanadeParameters &lucasKanade,
        ChessboardParameters &chessboard, int &upperMaxFeatures,
        int &lowerMaxFeatures, int &maxHistoryLength,
        double &upperMargin, double &lowerMargin);

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
