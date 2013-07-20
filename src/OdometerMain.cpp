/// @author Michał Orynicz

#include "VisualOdometer.hpp"
#include "BirdsEyeTranslationReader.hpp"
#include "TangentRotationReader.hpp"
#include "SmoothFilter.hpp"
#include "TrimHistoryFilter.hpp"
#include "LucasCandaePyramidTracker.hpp"
#include "ShiThomasiFeatureExtractor.hpp"
#include "PreparationFunctions.hpp"
#include "ConvenienceFunctions.hpp"
#include "ErrorCodes.hpp"
#include "DrawingFunctions.hpp"
#include "Catcher.hpp"
#include "CvTypesIo.hpp"
#include "CommandlineParameters.hpp"

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <boost/program_options.hpp>
#include <boost/date_time.hpp>

#include <string>
#include <vector>
#include <ctime>

using std::cerr;
using std::endl;

void processInput(const cv::Mat &input,
        const std::vector<cv::Mat> &rectifyMaps,
        std::list<cv::Point3f> &positions, VisualOdometer &odo,
        std::list<long long> &timestamps,
        const boost::posix_time::ptime &milenium,
        const int &verbosity, const CameraSpatialParameters &spatial,
        const cv::Size &imageSize, const std::string &mainWindowName,
        const std::string &mapWindowName);

VisualOdometer getVisualOdometer(cv::VideoCapture &capture,
        const ShiThomasiParameters &shiThomasi,
        const LucasKanadeParameters &lucasKanade,
        const ChessboardParameters &chessboard,
        CameraSpatialParameters &spatial,
        const std::string &cameraParametersFilename,
        const std::string &cameraSpatialFilename,
        const int &maxHistoryLength, const int &maxDeviation,
        const int &maxDeviants, const int &minLength,
        const int &maxLength, const int &upperMargin,
        const int &lowerMargin, const int & maxFeaturesUpper,
        const int & maxFeaturesLower,
        std::vector<cv::Mat> &rectifyMaps, cv::Size &imageSize,
        std::list<cv::Point3f> &positions,
        const cv::Mat &cameraMatrix,
        const cv::Mat &distortionCoefficients,
        void (*parametersCalibration)(cv::VideoCapture &capture,
                std::vector<cv::Mat> rectifyMaps, int &horizon,
                int &deadZone, const cv::Size &imageSize));

void printTrajectory(const std::string &matlabFilename,
        const std::list<long long> &timestamps,
        const std::list<cv::Point3f> &positions);

int main(int argc, char **argv) {

    ShiThomasiParameters shiThomasi;
    LucasKanadeParameters lucasKanade;
    ChessboardParameters chessboard;
    CameraSpatialParameters spatial;

    int maxFeaturesUpper = 100;
    int maxFeaturesLower = 100;
    int maxHistoryLength = 8;

    double maxDeviation = CV_PI / 16;
    double maxDeviants = 0.9;
    double minLength = 10;
    double maxLength = 50;

    double upperMargin = 30;
    double lowerMargin = 30;

    spatial.horizon = -1;
    spatial.deadZone = -1;

    int captureDeviceNumber = 1;
    int verbosity = 0;
    std::string cameraParametersFilename;
    std::string matlabFilename, cameraSpatialFilename;

    std::vector<cv::Mat> rectifyMaps;
    cv::Size imageSize;

    std::list<cv::Point3f> positions;
    std::list<long long> timestamps;
    cv::Point3f currentPosition(0, 0, 0);

    readParametersCommandLine(argc, argv, shiThomasi, lucasKanade,
            chessboard, spatial, maxFeaturesUpper, maxFeaturesLower,
            maxHistoryLength, upperMargin, lowerMargin,
            captureDeviceNumber, verbosity, cameraParametersFilename,
            matlabFilename, cameraSpatialFilename);

    Catcher capture;
    capture.open(captureDeviceNumber);
    cv::Mat cameraMatrix, distortionCoefficients;
    getCameraParameters(cameraParametersFilename, rectifyMaps,
            cameraMatrix, distortionCoefficients, imageSize);

    capture.set(CV_CAP_PROP_FRAME_WIDTH, imageSize.width);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, imageSize.height);

    try {
        VisualOdometer odo = getVisualOdometer(capture, shiThomasi,
                lucasKanade, chessboard, spatial,
                cameraParametersFilename, cameraSpatialFilename,
                maxHistoryLength, maxDeviation, maxDeviants,
                minLength, maxLength, upperMargin, lowerMargin,
                maxFeaturesUpper, maxFeaturesLower, rectifyMaps,
                imageSize, positions, cameraMatrix,
                distortionCoefficients, calibrateParameters);
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
        boost::posix_time::ptime milenium(
                boost::gregorian::date(1970, 1, 1));

        for (int i = 0; i < 5; ++i) { //a couple of dry cycles to warm up
            cv::Point3f displacement;
            capture >> input;
            if (input.empty()) {
                continue;
            }

            cv::remap(input, undistorted, rectifyMaps[0],
                    rectifyMaps[1], cv::INTER_LINEAR);
            cv::cvtColor(undistorted, grey, CV_RGB2GRAY);
            displacement = odo.calculateDisplacement(grey);
        }

        do {
            cv::Point3f displacement;
            capture >> input;
            if (input.empty()) {
                continue;
            }
            processInput(input, rectifyMaps, positions, odo,
                    timestamps, milenium, verbosity, spatial,
                    imageSize,"main","map");

            if (1 < verbosity && 'p' == control) {
                std::string fileName;
                std::stringstream buff;
                buff << "screen" << timestamps.front() << ".png";
                buff >> fileName;
                cv::imwrite(fileName, undistorted);

            }
            control = cv::waitKey(1);
        } while ('q' != control);
        cv::waitKey(0);
        printTrajectory(matlabFilename, timestamps, positions);

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

void printTrajectory(const std::string &matlabFilename,
        const std::list<long long> &timestamps,
        const std::list<cv::Point3f> &positions) {
    std::list<long long>::const_iterator iTime = timestamps.begin();
    std::ofstream out;
    if (!matlabFilename.empty()) {
        out.open(matlabFilename.c_str(), std::ofstream::out);
        if (!out.is_open()) {
            std::cerr << "WARNING: could not open file "
                    << matlabFilename << " for write" << std::endl;
        } else {
            out << "t=[" << std::endl;
        }
    }
    for (std::list<cv::Point3f>::const_iterator iPos =
            positions.begin();
            positions.end() != iPos && timestamps.end() != iTime;
            ++iPos, ++iTime) {
        std::cerr << *iPos << std::endl;
        if (!matlabFilename.empty()) {
            out << iPos->x << "," << iPos->y << "," << iPos->z << ","
                    << *iTime << std::endl;
        }
    }
    out << "];" << std::endl;
    out.close();
}

void processInput(const cv::Mat &input,
        const std::vector<cv::Mat> &rectifyMaps,
        std::list<cv::Point3f> &positions, VisualOdometer &odo,
        std::list<long long> &timestamps,
        const boost::posix_time::ptime &milenium,
        const int &verbosity, const CameraSpatialParameters &spatial,
        const cv::Size &imageSize, const std::string &mainWindowName,
        const std::string &mapWindowName) {
    cv::Mat undistorted;
    cv::Mat grey;
    cv::Point3d currentPosition = positions.front();
    cv::Point3d displacement;
    cv::Mat map;
    cv::remap(input, undistorted, rectifyMaps[0], rectifyMaps[1],
            cv::INTER_LINEAR);
    cv::cvtColor(undistorted, grey, CV_RGB2GRAY);
    displacement = odo.calculateDisplacement(grey);
    currentPosition = currentPosition + displacement;
    positions.push_front(currentPosition);

    boost::posix_time::ptime pTime =
            boost::posix_time::microsec_clock::universal_time();
    boost::posix_time::time_duration duration(pTime - milenium);
    long long miliseconds = duration.total_milliseconds();
    timestamps.push_front(miliseconds);
    std::cout << currentPosition << " " << miliseconds << std::endl;

    if (verbosity > 0) {
        map = drawTraveledRoute(positions);
        cv::imshow(mapWindowName, map);
    }
    if (verbosity > 1) {
        std::vector<std::list<cv::Point2f> > featuresRot =
                odo.getRotationFeatures();
        std::vector<std::list<cv::Point2f> > featuresGround =
                odo.getTranslationFeatures();

        drawFeaturesUpperAndLower(undistorted, featuresRot, cv::Mat(),
                featuresGround, spatial.homography, spatial.horizon,
                spatial.deadZone);
        cv::line(undistorted, cv::Point(imageSize.width / 2, 0),
                cv::Point(imageSize.width / 2, imageSize.height),
                CV_RGB(255,0,0), 1, 8, 0);
        cv::imshow(mainWindowName, undistorted);

    }
}

VisualOdometer getVisualOdometer(cv::VideoCapture &capture,
        const ShiThomasiParameters &shiThomasi,
        const LucasKanadeParameters &lucasKanade,
        const ChessboardParameters &chessboard,
        CameraSpatialParameters &spatial,
        const std::string &cameraParametersFilename,
        const std::string &cameraSpatialFilename,
        const int &maxHistoryLength, const int &maxDeviation,
        const int &maxDeviants, const int &minLength,
        const int &maxLength, const int &upperMargin,
        const int &lowerMargin, const int & maxFeaturesUpper,
        const int & maxFeaturesLower,
        std::vector<cv::Mat> &rectifyMaps, cv::Size &imageSize,
        std::list<cv::Point3f> &positions,
        const cv::Mat &cameraMatrix,
        const cv::Mat &distortionCoefficients,
        void (*parametersCalibration)(cv::VideoCapture &capture,
                std::vector<cv::Mat> rectifyMaps, int &horizon,
                int &deadZone, const cv::Size &imageSize)) {

    ShiThomasiFeatureExtractor extractor(shiThomasi.qualityLevel,
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

    cv::Point3f currentPosition(0, 0, 0);

    positions.push_back(currentPosition);

    if (0 >= spatial.horizon || 0 > spatial.deadZone) {
        spatial.horizon = imageSize.height / 2;
        spatial.deadZone = 0;
        parametersCalibration(capture, rectifyMaps, spatial.horizon,
                spatial.deadZone, imageSize);

        std::vector<cv::Point2f> corners = getChessboardCorners(
                capture, rectifyMaps, spatial.horizon,
                spatial.deadZone, chessboard.size, imageSize,
                chessboard.winSize, chessboard.zeroZone,
                chessboard.termCrit);
        cerr << "got corners" << endl;
        getHomographyRtMatrixAndRotationCenter(corners, imageSize,
                chessboard.size, chessboard.squareSize,
                spatial.horizon, spatial.deadZone, cameraMatrix,
                distortionCoefficients, chessboard.height,
                spatial.homography, spatial.rotationCenter,
                spatial.rtMatrix);
        if (!cameraSpatialFilename.empty()) {
            cv::FileStorage fs(cameraSpatialFilename,
                    cv::FileStorage::WRITE);
            if (!fs.isOpened()) {
                cv::Exception ex(-1, "Could not open FileStorage",
                        __func__, __FILE__, __LINE__);
                throw ex;
            }
            fs << "cameraSpatialParameters" << spatial;
            fs.release();
        }

    }
//        std::cerr << "rt: " << spatial.rtMatrix << std::endl;
//        std::cerr << "rot center: " << spatial.rotationCenter
//                << std::endl;
    cerr << "got transform and rt" << endl;
    TangentRotationReader rotationReader(tracker, extractor, filters,
            maxFeaturesUpper, cameraMatrix.at<double>(0, 0),
            cv::Size(imageSize.width,
                    spatial.horizon - spatial.deadZone), upperMargin);
    BirdsEyeTranslationReader transReader(spatial.homography,
            extractor, tracker, maxFeaturesLower, filters,
            spatial.rotationCenter,
            cv::Size(imageSize.width,
                    imageSize.height - spatial.horizon
                            - spatial.deadZone), lowerMargin);
    cerr << "got birdy" << endl;
    VisualOdometer odo(rotationReader, transReader, spatial.horizon,
            spatial.deadZone);
    cerr << "got VO" << endl;
    return odo;

}
