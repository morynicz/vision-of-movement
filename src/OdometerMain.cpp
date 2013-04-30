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

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <string>
#include <vector>

using std::cerr;
using std::endl;

int main() {

    double qualityLevel = 0.01;
    double minDistance = 10;
    int blockSize = 5;
    cv::Size winSizeShi(5, 5);
    cv::Size zeroZone(-1, -1);
    cv::TermCriteria termCritShi(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER,
            40, 0.0001);

    cv::Size winSizeLuc(21, 21);
    unsigned int maxLevel = 4;
    int flagsLuc = 0;
    cv::TermCriteria termCritLuc(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER,
            40, 0.0001);
    double minEigThr = 1e-4;
    double maxErrVal = 500;

    cv::Mat homography;

    //cv::Size boardSize(10, 7);
    cv::Size boardSize(9, 6);
    cv::VideoCapture capture(1);

    ShiThomasFeatureExtractor extractor(qualityLevel, minDistance,
            blockSize, winSizeShi, zeroZone, termCritShi);
    LucasCandaePyramidTracker tracker(winSizeLuc, maxLevel, flagsLuc,
            termCritLuc, minEigThr, maxErrVal);

    unsigned int maxFeatures = 500;
    int maxHistoryLength = 9;
    double maxDeviation = CV_PI / 16;
    double maxDeviants = maxFeatures / 4;
    double minLength = 10;
    std::list<FeatureFilter*> filters;

    filters.push_back(new TrimHistoryFilter(maxHistoryLength));
    filters.push_back(
            new SmoothFilter(maxDeviation, maxDeviants, minLength));

    int horizon;
    int deadZone;
    int featuresNumber;

    std::vector<cv::Mat> rectifyMaps;
    cv::Size imageSize;

    cv::Size winSizeHom(5, 5);
    cv::Size zeroZoneHom(-1, -1);
    cv::TermCriteria termCritHom(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER,
            40, 0.001);
    float squareSize = 25;
    std::list<cv::Point3f> positions;
    cv::Point2f rotationCenter;
    cv::Mat rtMatrix;
    cv::Point3f currentPosition(0, 0, 0);
    double margin = 5;

    positions.push_back(currentPosition);
    try {
        cv::Mat cameraMatrix, distortionCoefficients;
        getCameraParameters("../sphereAF.yml", rectifyMaps,
                cameraMatrix, distortionCoefficients, imageSize);
        capture.set(CV_CAP_PROP_FRAME_WIDTH, imageSize.width);
        capture.set(CV_CAP_PROP_FRAME_HEIGHT, imageSize.height);
        horizon = imageSize.height / 2;
        deadZone = 0;
        calibrateParameters(capture, rectifyMaps, horizon, deadZone,
                imageSize);
        std::vector<cv::Point2f> corners = getChessboardCorners(
                capture, rectifyMaps, horizon, deadZone, boardSize,
                imageSize, winSizeHom, zeroZoneHom, termCritHom);
        cerr << "got corners" << endl;
        getHomographyAndRtMatrix(corners, imageSize, boardSize,
                squareSize, horizon, deadZone, cameraMatrix,
                distortionCoefficients, homography, rotationCenter,
                rtMatrix);
        cerr << "got transform and rt" << endl;
        printMatrix(cameraMatrix, true);
        TangentRotationReader rotationReader(tracker, extractor,
                filters, maxFeatures, cameraMatrix.at<double>(0, 0));
        BirdsEyeTranslationReader transReader(homography, extractor,
                tracker, maxFeatures, filters, rotationCenter,
                imageSize, margin);
        cerr << "got birdy" << endl;
        VisualOdometer odo(rotationReader, transReader, horizon,
                deadZone, featuresNumber);
        cerr << "got VO" << endl;
        char control = ' ';
        cv::Mat input;
        cv::Mat undistorted;
        cv::Mat grey;
        cv::namedWindow("main", CV_WINDOW_KEEPRATIO);
        cv::namedWindow("map", CV_WINDOW_KEEPRATIO);
        std::vector<std::list<cv::Point2f> > featuresRot;
        std::vector<std::list<cv::Point2f> > featuresGround;
        do {
            std::cerr << "p" << std::endl;
            cv::Point3f displacement;
            capture >> input;

            cv::remap(input, undistorted, rectifyMaps[0],
                    rectifyMaps[1], cv::INTER_LINEAR);
            cv::cvtColor(undistorted, grey, CV_RGB2GRAY);
            displacement = odo.calculateDisplacement(grey);
            currentPosition = currentPosition + displacement;
            positions.push_front(currentPosition);
           std::cerr << "d: " << displacement << std::endl;
            cv::Mat map = drawTraveledRoute(positions);
            //printMatrix(map);
            std::cerr << currentPosition << std::endl << map.size()
                    << std::endl;

            featuresRot = odo.getRotationFeatures();
            featuresGround = odo.getTranslationFeatures();

            drawFeaturesUpperAndLower(input, featuresRot, cv::Mat(),
                    featuresGround, homography, horizon, deadZone);

            cv::imshow("map", map);
            cv::imshow("main", input);
            control = cv::waitKey(1);
            std::cerr << "k" << std::endl;
        } while ('q' != control);

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
