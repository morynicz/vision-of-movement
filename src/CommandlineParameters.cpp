/*
 * CommandlineParameters.cpp
 *
 *  Created on: Jun 2, 2013
 *      Author: link
 */

#include "CommandlineParameters.hpp"
#include <boost/program_options.hpp>

void readParameters(const std::string &fileName,
        ShiThomasiParameters &shiThomasi,
        LucasKanadeParameters &lucasKanade,
        ChessboardParameters &chessboard, int &upperMaxFeatures,
        int &lowerMaxFeatures, int &maxHistoryLength,
        double &upperMargin, double &lowerMargin) {

    cv::FileStorage fs(fileName, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        cv::Exception ex(-1, "Could not open FileStorage", __func__,
                __FILE__, __LINE__);
        throw ex;
    }

    fs["ShiThomasiParameters"] >> shiThomasi;
    fs["LucasKanadeParameters"] >> lucasKanade;
    fs["ChessboardParameters"] >> chessboard;

    fs["upperMaxFeatures"] >> upperMaxFeatures;
    fs["lowerMaxFeatures"] >> lowerMaxFeatures;
    fs["maxHistoryLength"] >> maxHistoryLength;
    fs["upperMargin"] >> upperMargin;
    fs["lowerMargin"] >> lowerMargin;
    fs.release();
}

void saveParameters(const std::string &fileName,
        ShiThomasiParameters &shiThomasi,
        LucasKanadeParameters &lucasKanade,
        ChessboardParameters &chessboard, int &upperMaxFeatures,
        int &lowerMaxFeatures, int &maxHistoryLength,
        double &upperMargin, double &lowerMargin) {
    cv::FileStorage fs(fileName, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        cv::Exception ex(-1, "Could not open FileStorage", __func__,
                __FILE__, __LINE__);
        throw ex;
    }
    fs << "ShiThomasiParameters" << shiThomasi;
    fs << "LucasKanadeParameters" << lucasKanade;
    fs << "ChessboardParameters" << chessboard;

    fs << "upperMaxFeatures" << upperMaxFeatures;
    fs << "lowerMaxFeatures" << lowerMaxFeatures;
    fs << "maxHistoryLength" << maxHistoryLength;
    fs << "upperMargin" << upperMargin;
    fs << "lowerMargin" << lowerMargin;
    fs.release();
}

void readParametersCommandLine(const int &argc, char **argv,
        boost::program_options::options_description &desc,
        boost::program_options::variables_map &vm,
        ShiThomasiParameters &shiThomasi,
        LucasKanadeParameters &lucasKanade,
        ChessboardParameters &chessboard,
        CameraSpatialParameters &spatialParameters,
        int &upperMaxFeatures, int &lowerMaxFeatures,
        int &maxHistoryLength, double &upperMargin,
        double &lowerMargin, int &verbosity,
        std::string &cameraParametersFilename,
        std::string &matlabFileName,
        std::string &cameraPositionFilename) {

    desc.add_options()("help,h", "this help message")(
            "parameters_file,p",
            boost::program_options::value<std::string>(),
            "the file containing program parameters")(
            "camera_parameters,c",
            boost::program_options::value<std::string>(),
            "camera parameters file")("verbosity,v",
            boost::program_options::value<int>(&verbosity)->default_value(
                    0),
            "Program verbosity level.\n 0 - default, returns current "
                    "position to standard output\n 1 - instead of numeric values,"
                    " draws map. Values are printed at end of execution\n 2 - "
                    "shows video output from camera and tracked features")(
            "matlab_data,m",
            boost::program_options::value<std::string>(),
            "matlab/octave m-file, in which the route will be stored")(
            "camera_position,P",
            boost::program_options::value<std::string>(),
            "camera position parameters filename")(
            "camera_position_output",
            boost::program_options::value<std::string>(),
            "filename to which the position parameters shall be written to");

    boost::program_options::store(
            boost::program_options::parse_command_line(argc, argv,
                    desc), vm);
    boost::program_options::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
    }

    if (vm.count("verbosity")) {
        if (vm["verbosity"].as<int>() > 2
                || vm["verbosity"].as<int>() < 0) {
            cv::Exception ex(-1, "Bad verbosity value", __func__,
                    __FILE__, __LINE__);
            throw ex;
        }
    }

    if (vm.count("parameters_file")) {
        readParameters(vm["parameters_file"].as<std::string>(),
                shiThomasi, lucasKanade, chessboard, upperMaxFeatures,
                lowerMaxFeatures, maxHistoryLength, upperMargin,
                lowerMargin);
    } else {
        cv::Exception ex(-2, "Program parameters file not supplied",
                __func__, __FILE__, __LINE__);
        throw ex;
    }

    if (vm.count("camera_parameters")) {
        cameraParametersFilename = vm["camera_parameters"].as<
                std::string>();

    } else {
        cv::Exception ex(-2, "camera parameters file not supplied",
                __func__, __FILE__, __LINE__);
        throw ex;
    }

    if (vm.count("matlab_data")) {
        matlabFileName = vm["matlab_data"].as<std::string>();
    } else {
        matlabFileName = "";
    }

    if (vm.count("camera_position")) {
        cv::FileStorage fs(vm["camera_position"].as<std::string>(),
                cv::FileStorage::READ);
        if (!fs.isOpened()) {
            cv::Exception ex(-1, "Could not open FileStorage",
                    __func__, __FILE__, __LINE__);
            throw ex;
        }
        fs["cameraSpatialParameters"] >> spatialParameters;
        fs.release();
    }

    if (vm.count("camera_position_output")) {
        cameraPositionFilename = vm["camera_position_output"].as<
                std::string>();
    }

}

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
        std::string &cameraPositionFilename) {

    boost::program_options::options_description desc(
            "Allowed options");
    boost::program_options::variables_map vm;
    desc.add_options()("device,d",
            boost::program_options::value<int>(&captureDeviceNumber)->default_value(
                    0), "input device number");

    readParametersCommandLine(argc, argv, desc, vm, shiThomasi,
            lucasKanade, chessboard, spatialParameters,
            upperMaxFeatures, lowerMaxFeatures, maxHistoryLength,
            upperMargin, lowerMargin, verbosity,
            cameraParametersFilename, matlabFileName,
            cameraPositionFilename);
}

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
        std::string &cameraPositionFilename) {

    boost::program_options::options_description desc(
            "Allowed options");
    boost::program_options::variables_map vm;
    desc.add_options()("file_name,f",
            boost::program_options::value<std::string>(),
            "input video file");
    readParametersCommandLine(argc, argv, desc, vm, shiThomasi,
            lucasKanade, chessboard, spatialParameters,
            upperMaxFeatures, lowerMaxFeatures, maxHistoryLength,
            upperMargin, lowerMargin, verbosity,
            cameraParametersFilename, matlabFileName,
            cameraPositionFilename);

    if (vm.count("file_name")) {
        videoFile = vm["file_name"].as<std::string>();
    } else {
        cv::Exception ex(-2, "Movie file not supplied", __func__,
                __FILE__, __LINE__);
        throw ex;
    }
}

void readParametersCommandLine(const int &argc, char **argv,
        ShiThomasiParameters &shiThomasi,
        LucasKanadeParameters &lucasKanade,
        ChessboardParameters &chessboard, int &upperMaxFeatures,
        int &lowerMaxFeatures, int &maxHistoryLength,
        double &upperMargin, double &lowerMargin,
        std::string &captureFileName, int &verbosity,
        std::string &cameraParametersFilename,
        std::string &matlabFileName) {

    boost::program_options::options_description desc(
            "Allowed options");
    boost::program_options::variables_map vm;
    desc.add_options()("help,h", "this help message")("file_name,f",
            boost::program_options::value<std::string>(),
            "input device number")("parameters_file,p",
            boost::program_options::value<std::string>(),
            "the file containing program parameters")(
            "camera_parameters,c",
            boost::program_options::value<std::string>(),
            "camera parameters file")("verbosity,v",
            boost::program_options::value<int>(&verbosity)->default_value(
                    0),
            "Program verbosity level.\n 0 - default, returns current "
                    "position to standard output\n 1 - instead of numeric values,"
                    " draws map. Values are printed at end of execution\n 2 - "
                    "shows video output from camera and tracked features")(
            "matlab_data,m",
            boost::program_options::value<std::string>(),
            "matlab/octave m-file, in which the route will be stored")(
            "frame_rate,r", boost::program_options::value<int>(),
            "movie file framerate");

    boost::program_options::store(
            boost::program_options::parse_command_line(argc, argv,
                    desc), vm);
    boost::program_options::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << std::endl;
    }

    if (vm.count("file_name")) {
        captureFileName = vm["file_name"].as<std::string>();
    } else {
        cv::Exception ex(-2, "Movie file not supplied", __func__,
                __FILE__, __LINE__);
        throw ex;
    }

    if (vm.count("verbosity")) {
        if (vm["verbosity"].as<int>() > 2
                || vm["verbosity"].as<int>() < 0) {
            cv::Exception ex(-1, "Bad verbosity value", __func__,
                    __FILE__, __LINE__);
            throw ex;
        }
    }

    if (vm.count("parameters_file")) {
        readParameters(vm["parameters_file"].as<std::string>(),
                shiThomasi, lucasKanade, chessboard, upperMaxFeatures,
                lowerMaxFeatures, maxHistoryLength, upperMargin,
                lowerMargin);
    } else {
        cv::Exception ex(-2, "Program parameters file not supplied",
                __func__, __FILE__, __LINE__);
        throw ex;
    }

    if (vm.count("camera_parameters")) {
        cameraParametersFilename = vm["camera_parameters"].as<
                std::string>();

    } else {
        cv::Exception ex(-2, "camera parameters file not supplied",
                __func__, __FILE__, __LINE__);
        throw ex;
    }

    if (vm.count("matlab_data")) {
        matlabFileName = vm["matlab_data"].as<std::string>();
    } else {
        matlabFileName = "";
    }

}

