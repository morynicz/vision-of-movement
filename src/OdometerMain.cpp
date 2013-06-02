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


int main(int argc, char **argv) {

    ShiThomasiParameters shiThomasi;
    LucasKanadeParameters lucasKanade;
    ChessboardParameters chessboard;
    CameraSpatialParameters spatial;

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

    spatial.horizon = -1;
    spatial.deadZone = -1;

    int captureDeviceNumber = 1;
    int verbosity = 0;
    std::string cameraParametersFilename;
    std::string matlabFilename, cameraSpatialFilename;

    readParametersCommandLine(argc, argv, shiThomasi, lucasKanade,
            chessboard, spatial, maxFeaturesUpper, maxFeaturesLower,
            maxHistoryLength, upperMargin, lowerMargin,
            captureDeviceNumber, verbosity, cameraParametersFilename,
            matlabFilename, cameraSpatialFilename);

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

    std::vector<cv::Mat> rectifyMaps;
    cv::Size imageSize;

    std::list<cv::Point3f> positions;
    std::list<long long> timestamps;
    cv::Point3f currentPosition(0, 0, 0);

    positions.push_back(currentPosition);
    try {
        cv::Mat cameraMatrix, distortionCoefficients;
        getCameraParameters(cameraParametersFilename, rectifyMaps,
                cameraMatrix, distortionCoefficients, imageSize);

        capture.set(CV_CAP_PROP_FRAME_WIDTH, imageSize.width);
        capture.set(CV_CAP_PROP_FRAME_HEIGHT, imageSize.height);

        if (0 >= spatial.horizon || 0 > spatial.deadZone) {
            spatial.horizon = imageSize.height / 2;
            spatial.deadZone = 0;
            calibrateParameters(capture, rectifyMaps, spatial.horizon,
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
        TangentRotationReader rotationReader(tracker, extractor,
                filters, maxFeaturesUpper,
                cameraMatrix.at<double>(0, 0),
                cv::Size(imageSize.width,
                        spatial.horizon - spatial.deadZone),
                upperMargin);
        BirdsEyeTranslationReader transReader(spatial.homography,
                extractor, tracker, maxFeaturesLower, filters,
                spatial.rotationCenter,
                cv::Size(imageSize.width,
                        imageSize.height - spatial.horizon
                                - spatial.deadZone), lowerMargin);
        cerr << "got birdy" << endl;
        VisualOdometer odo(rotationReader, transReader,
                spatial.horizon, spatial.deadZone);
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
        boost::posix_time::ptime milenium(
                boost::gregorian::date(2000, 1, 1));

        for(int i=0;i<5;++i){//a couple of dry cycles to warm up
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

            cv::remap(input, undistorted, rectifyMaps[0],
                    rectifyMaps[1], cv::INTER_LINEAR);
            cv::cvtColor(undistorted, grey, CV_RGB2GRAY);
            displacement = odo.calculateDisplacement(grey);
            currentPosition = currentPosition + displacement;
            positions.push_front(currentPosition);

            //setting up timestamps
//            boost::posix_time::ptime pTime =
//                    boost::posix_time::microsec_clock::universal_time();
//            boost::posix_time::time_duration duration(
//                    pTime.time_of_day());
//            boost::gregorian::date date = pTime.date();
//
//            std::cout << currentPosition << " " << date.year() << ":"
//                    << date.month() << ":" << date.day() << ":"
//                    << duration << std::endl;
            boost::posix_time::ptime pTime =
                    boost::posix_time::microsec_clock::universal_time();
            boost::posix_time::time_duration duration(
                    pTime - milenium);
            long long miliseconds = duration.total_milliseconds();
            timestamps.push_front(miliseconds);
            std::cout << currentPosition << " " << miliseconds
                    << std::endl;

            if (verbosity > 0) {
                map = drawTraveledRoute(positions);
                cv::imshow("map", map);
            }
            if (verbosity > 1) {
                featuresRot = odo.getRotationFeatures();
                featuresGround = odo.getTranslationFeatures();

                drawFeaturesUpperAndLower(undistorted, featuresRot,
                        cv::Mat(), featuresGround, spatial.homography,
                        spatial.horizon, spatial.deadZone);
                cv::line(input, cv::Point(imageSize.width / 2, 0),
                        cv::Point(imageSize.width / 2,
                                imageSize.height), CV_RGB(255,0,0), 1,
                        8, 0);
                cv::imshow("main", undistorted);
                if ('p' == control) {
                    std::string fileName;
                    std::stringstream buff;
                    buff << "screen" << timestamps.front() << ".png";
                    buff >> fileName;
                    cv::imwrite(fileName, undistorted);
                }
            }
            control = cv::waitKey(1);
        } while ('q' != control);
        cv::waitKey(0);
        std::list<long long>::iterator iTime = timestamps.begin();
        std::ofstream out;
        if (!matlabFilename.empty()) {
            out.open(matlabFilename.c_str(), std::ofstream::out);
            if (!out.is_open()) {
                std::cerr << "WARNING: could not open file "
                        << matlabFilename << " for write"
                        << std::endl;
            } else {
                out << "t=[" << std::endl;
            }
        }
        for (std::list<cv::Point3f>::iterator iPos =
                positions.begin();
                positions.end() != iPos && timestamps.end() != iTime;
                ++iPos, ++iTime) {
            std::cerr << *iPos << std::endl;
            if (!matlabFilename.empty()) {
                out << iPos->x << "," << iPos->y << "," << iPos->z
                        << "," << *iTime << std::endl;
            }
        }
        out << "];" << std::endl;
        out.close();
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
