/// @author Michał Orynicz

#include "VisualOdometer.hpp"
#include "BirdsEyeTranslationReader.hpp"
#include "TangentRotationReader.hpp"
#include "SmoothFilter.hpp"
#include "TrimHistoryFilter.hpp"
#include "LucasCandaePyramidTracker.hpp"
#include "ShiThomasFeatureExtractor.hpp"
#include "PreparationFunctions.hpp"
#include "ConvenienceFunctions.hpp"
#include "ErrorCodes.hpp"
#include "DrawingFunctions.hpp"
#include "Catcher.hpp"
#include "CvTypesIo.hpp"

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <string>
#include <vector>
#include <ctime>

using std::cerr;
using std::endl;

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
        ChessboardParameters &chessboard, int &upperMaxFeatures,
        int &lowerMaxFeatures, int &maxHistoryLength,
        double &upperMargin, double &lowerMargin,
        int &captureDeviceNumber, int &verbosity,
        std::string &cameraParametersFilename);

int main(int argc, char **argv) {

    ShiThomasiParameters shiThomasi;
    LucasKanadeParameters lucasKanade;
    ChessboardParameters chessboard;

    shiThomasi.qualityLevel = 0.005;
    shiThomasi.minDistance = 30;
    shiThomasi.blockSize = 5;
    shiThomasi.winSize = cv::Size(5, 5);
    shiThomasi.zeroZone = cv::Size(-1, -1);
    shiThomasi.termCrit = cv::TermCriteria(
            CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.0001);

    lucasKanade.winSize = cv::Size(21, 21);
    lucasKanade.maxLevel = 4;
    lucasKanade.flags = 0;
    lucasKanade.minEigenvalueThresh = 1e-4;
    lucasKanade.maxErrorValue = 500;
    lucasKanade.termCrit = cv::TermCriteria(
            CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.0001);

    chessboard.winSize = cv::Size(5, 5);
    chessboard.zeroZone = cv::Size(-1, -1);
    chessboard.squareSize = 25;
    chessboard.size = cv::Size(9, 6);
    chessboard.height = 10;
    chessboard.termCrit = cv::TermCriteria(
            CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.0001);

    int maxFeaturesUpper = 100;
    int maxFeaturesLower = 100;
    int maxHistoryLength = 8;

    double maxDeviation = CV_PI / 16;
    double maxDeviants = 0.9;
    double minLength = 10;
    double maxLength = 50;

    double upperMargin = 30;
    double lowerMargin = 30;

    int captureDeviceNumber = 1;
    int verbosity = 0;
    std::string cameraParametersFilename;

    readParametersCommandLine(argc, argv, shiThomasi, lucasKanade,
            chessboard, maxFeaturesUpper, maxFeaturesLower,
            maxHistoryLength, upperMargin, lowerMargin,
            captureDeviceNumber, verbosity, cameraParametersFilename);

    cv::Mat homography;

    Catcher capture(captureDeviceNumber);

    ShiThomasFeatureExtractor extractor(shiThomasi.qualityLevel,
            shiThomasi.minDistance, shiThomasi.blockSize,
            shiThomasi.winSize, shiThomasi.zeroZone,
            shiThomasi.termCrit);
    LucasCandaePyramidTracker tracker(lucasKanade.winSize,
            lucasKanade.maxLevel, lucasKanade.flags,
            lucasKanade.termCrit, lucasKanade.minEigenvalueThresh,
            lucasKanade.maxErrorValue);

    std::list<FeatureFilter*> filters;

    filters.push_back(new TrimHistoryFilter(maxHistoryLength));
    filters.push_back(
            new SmoothFilter(maxDeviation, maxDeviants, minLength,
                    maxLength));

    int horizon;
    int deadZone;

    std::vector<cv::Mat> rectifyMaps;
    cv::Size imageSize;

    std::list<cv::Point3f> positions;
    cv::Point2d rotationCenter;
    cv::Mat rtMatrix;
    cv::Point3f currentPosition(0, 0, 0);

    positions.push_back(currentPosition);


    try {
        cv::Mat cameraMatrix, distortionCoefficients;
        getCameraParameters(cameraParametersFilename, rectifyMaps,
                cameraMatrix, distortionCoefficients, imageSize);

        capture.set(CV_CAP_PROP_FRAME_WIDTH, imageSize.width);
        capture.set(CV_CAP_PROP_FRAME_HEIGHT, imageSize.height);
        horizon = imageSize.height / 2;
        deadZone = 0;
        calibrateParameters(capture, rectifyMaps, horizon, deadZone,
                imageSize);

        std::vector<cv::Point2f> corners = getChessboardCorners(
                capture, rectifyMaps, horizon, deadZone,
                chessboard.size, imageSize, chessboard.winSize,
                chessboard.zeroZone, chessboard.termCrit);
        cerr << "got corners" << endl;
        getHomographyRtMatrixAndRotationCenter(corners, imageSize,
                chessboard.size, chessboard.squareSize, horizon,
                deadZone, cameraMatrix, distortionCoefficients,
                chessboard.height, homography, rotationCenter,
                rtMatrix);
        std::cerr << "rt: " << rtMatrix << std::endl;
        cerr << "got transform and rt" << endl;
        TangentRotationReader rotationReader(tracker, extractor,
                filters, maxFeaturesUpper,
                cameraMatrix.at<double>(0, 0),
                cv::Size(imageSize.width, horizon - deadZone),
                upperMargin);
        BirdsEyeTranslationReader transReader(homography, extractor,
                tracker, maxFeaturesLower, filters, rotationCenter,
                cv::Size(imageSize.width,
                        imageSize.height - horizon - deadZone),
                lowerMargin);
        cerr << "got birdy" << endl;
        VisualOdometer odo(rotationReader, transReader, horizon,
                deadZone);
        cerr << "got VO" << endl;
        char control = ' ';
        cv::Mat input;
        cv::Mat undistorted;
        cv::Mat grey;
        cv::Mat map;
        if (verbosity > 1) {
            cv::namedWindow("main", CV_WINDOW_KEEPRATIO);
        }
        if (verbosity > 0) {
            cv::namedWindow("map", CV_WINDOW_KEEPRATIO);
        }
        std::vector<std::list<cv::Point2f> > featuresRot;
        std::vector<std::list<cv::Point2f> > featuresGround;

        do {
            cv::Point3f displacement;
            capture >> input;
            if (input.empty()) {
                continue;
            }

            cv::remap(input, undistorted, rectifyMaps[0],
                    rectifyMaps[1], cv::INTER_LINEAR);
            cv::cvtColor(undistorted, grey, CV_RGB2GRAY);
            displacement = odo.calculateDisplacement(grey);
            currentPosition = currentPosition + displacement;
            positions.push_front(currentPosition);

            if (verbosity > 0) {
                map = drawTraveledRoute(positions);
                cv::imshow("map", map);
            }
            if (verbosity > 1) {
                featuresRot = odo.getRotationFeatures();
                featuresGround = odo.getTranslationFeatures();

                drawFeaturesUpperAndLower(undistorted, featuresRot,
                        cv::Mat(), featuresGround, homography,
                        horizon, deadZone);
                cv::line(input, cv::Point(imageSize.width / 2, 0),
                        cv::Point(imageSize.width / 2,
                                imageSize.height), CV_RGB(255,0,0), 1,
                        8, 0);
                cv::imshow("main", undistorted);
            }
            control = cv::waitKey(1);
        } while ('q' != control);
        cv::waitKey(0);
        for (std::list<cv::Point3f>::iterator it = positions.begin();
                positions.end() != it; ++it) {
            std::cerr << *it << std::endl;
        }
    } catch (cv::Exception &ex) {
        if (USER_TRIGGERED_EXIT == ex.code) {
            return 0;
        } else {
            std::cerr << "EXCEPTION!!!!!!!!!!!" << std::endl
                    << ex.what() << std::endl;
            return 1;
        }
    }
    return 0;
}

void checkPnPMethodStats(Catcher &capture,
        const std::vector<cv::Mat> &rectifyMaps, const int &horizon,
        const int &deadZone, const cv::Size &boardSize,
        const cv::Size &imageSize, const cv::Size &winSize,
        const cv::Size &zeroZone, const cv::TermCriteria &termCrit,
        const double &squareSize, const cv::Mat &cameraMatrix,
        const cv::Mat & distortionCoefficients,
        const double &chessboardHeight, const int & iterations,
        cv::Point3d & mean, cv::Point3d & variance,
        cv::Point3d & median, cv::Point3d &stdDev) {
    std::vector<cv::Point3d> coords(iterations);
    cv::Mat homography, rtMatrix;
    cv::Point2d rotationCenter;
    for (unsigned int i = 0; i < coords.size(); ++i) {
        std::vector<cv::Point2f> corners = getChessboardCorners(
                capture, rectifyMaps, horizon, deadZone, boardSize,
                imageSize, winSize, zeroZone, termCrit);
        getHomographyRtMatrixAndRotationCenter(corners, imageSize,
                boardSize, squareSize, horizon, deadZone,
                cameraMatrix, distortionCoefficients,
                chessboardHeight, homography, rotationCenter,
                rtMatrix);

        coords[i] = cv::Point3d(rtMatrix.at<double>(0, 3),
                rtMatrix.at<double>(1, 3), rtMatrix.at<double>(2, 3));
    }
    pointMeanAndVariance(coords, mean, variance);
    stdDev = cv::Point3d(sqrt(variance.x), sqrt(variance.y),
            sqrt(variance.y));

}

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
        ShiThomasiParameters &shiThomasi,
        LucasKanadeParameters &lucasKanade,
        ChessboardParameters &chessboard, int &upperMaxFeatures,
        int &lowerMaxFeatures, int &maxHistoryLength,
        double &upperMargin, double &lowerMargin,
        int &captureDeviceNumber, int &verbosity,
        std::string &cameraParametersFilename) {

    boost::program_options::options_description desc(
            "Allowed options");
    boost::program_options::variables_map vm;
    desc.add_options()("help,h", "this help message")("device,d",
            boost::program_options::value<int>(&captureDeviceNumber)->default_value(
                    0), "input device number")("parameters_file,p",
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
                    "shows video output from camera and tracked features");

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

}
