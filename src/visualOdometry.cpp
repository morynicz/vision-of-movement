#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <algorithm>
#include <list>
#include <cmath>

#include "SmoothFilter.hpp"
#include "ImageEdgeFilter.hpp"

using std::vector;
using std::string;
using namespace cv;

cv::Mat getR(const cv::Mat &rT) {
    return rT(cv::Rect(0, 0, 3, 3));
}

cv::Mat getT(const cv::Mat &rT) {
    return rT(cv::Range(0, 3), cv::Range(3, 4));
}

cv::Mat rtTransformMatrix(const cv::Mat &rot, const cv::Mat &trans) {
    cv::Mat r;
    if (1 == rot.cols || 1 == rot.rows) {
        cv::Rodrigues(rot, r);
    } else {
        r = rot;
    }
    cv::Mat result = cv::Mat::eye(4, 4, trans.depth());
    cv::Mat rotation = result(cv::Rect(0, 0, 3, 3));
    cv::Mat translation = result(cv::Range(0, 3), cv::Range(3, 4));
    r.copyTo(rotation);
    trans.copyTo(translation);
    return result;
}

cv::Mat invRtTransformMatrix(const cv::Mat& rot, const cv::Mat& trans) {
    cv::Mat result = cv::Mat::eye(4, 4, trans.depth());
    cv::Mat invR = rot.t();
    cv::Mat invT = -invR * trans;
    invR.copyTo(result(cv::Range(0, 3), cv::Range(0, 3)));
    invT.copyTo(result(cv::Range(0, 3), cv::Range(3, 4)));
    return result;
}

cv::Mat invRtTransformMatrix(const cv::Mat& rtTransformMatrix) {
    cv::Mat r = rtTransformMatrix(cv::Range(0, 3), cv::Range(0, 3));
    cv::Mat t = rtTransformMatrix(cv::Range(0, 3), cv::Range(3, 4));
    return invRtTransformMatrix(r, t);
}

string num2Str(const double &number) {
    std::string output;
    std::stringstream stream;
    stream << number;
    stream >> output;
    return output;
}

bool comparePoints(Point2f p1, Point2f p2) {
    if (p1.x != p2.x) {
        return p1.x < p2.x;
    } else {
        return p1.y < p2.y;
    }
    return false;
}

bool compareListPoints(std::list<Point2f> l1, std::list<Point2f> l2) {
    Point2f p1 = l1.front();
    Point2f p2 = l2.front();

    if (p1.x != p2.x) {
        return p1.x < p2.x;
    } else if (p1.y != p2.y) {
        return p1.y < p2.y;
    } else if (l1.size() != l2.size()) {
        return l1.size() > l2.size();
    }
    return false;
}

bool listPointsEqual(std::list<Point2f> l1, std::list<Point2f> l2) {
    return l1.front() == l2.front();
}

void drawDeadZoneHorizon(cv::Mat image, const int &horizon,
        const int& deadZone) {
    line(image, Point2f(0, horizon + deadZone),
            Point2f(image.cols, horizon + deadZone), CV_RGB(0,255,0), 2, CV_AA);
    line(image, Point2f(0, horizon - deadZone),
            Point2f(image.cols, horizon - deadZone), CV_RGB(0,255,0), 2, CV_AA);

}

void markFeatureMovement(cv::Mat &newImage, Point2f& oldFeature,
        const Point2f& newFeature) {
    int radius = 5;
    circle(newImage, newFeature, radius, Scalar(100, 0, 100), -1, 8, 0);
    line(newImage, oldFeature, newFeature, CV_RGB(100,0,0), 1, CV_AA);

}

void drawFeatureHistory(cv::Mat &newImage, std::list<Point2f> featureHistory) {
    int radius = 5;
    circle(newImage, featureHistory.front(), radius, Scalar(100, 0, 100), -1, 8,
            0);
    std::list<Point2f>::iterator itb = featureHistory.begin();
    std::list<Point2f>::iterator ite = itb;
    if (!featureHistory.size() < 2) {
        for (ite++; ite != featureHistory.end(); ++ite, ++itb) {
            line(newImage, *ite, *itb, CV_RGB(100,0,0), 1, CV_AA);
        }
    }
}

void trimFeatureHistory(std::vector<std::list<Point2f> > &featureHistory,
        const unsigned &maxLength) {
    for (unsigned int i = 0; i < featureHistory.size(); ++i) {
        if (featureHistory[i].size() > maxLength) {
            featureHistory[i].resize(maxLength);
        }
    }
}

std::vector<std::list<Point2f> > transformFeatureHistory(
        const std::vector<std::list<Point2f> > &input,
        cv::Mat transformMatrix) {
    std::vector<std::list<Point2f> > result(input.size());
    for (unsigned int i = 0; i < input.size(); ++i) {
        std::vector<Point2f> tmp(input[i].begin(), input[i].end());
        std::vector<Point2f> out;
        perspectiveTransform(tmp, out, transformMatrix);
        result[i] = std::list<Point2f>(out.begin(), out.end());
    }
    return result;
}

void printMatrix(const cv::Mat &arg) {
    std::cerr << "dims: " << arg.cols << ' ' << arg.rows << std::endl;
    std::cerr << "chans: " << arg.channels() << std::endl;
    int type = arg.depth();
    {
        std::string depth;
        switch (type) {
            case CV_8U:
                depth = "CV_8U";
                break;
            case CV_8S:
                depth = "CV_8S";
                break;
            case CV_16U:
                depth = "CV_16U";
                break;
            case CV_16S:
                depth = "CV_16S";
                break;
            case CV_32S:
                depth = "CV_32S";
                break;
            case CV_32F:
                depth = "CV_32F";
                break;
            case CV_64F:
                depth = "CV_64F";
                break;
        }
        std::cerr << "depth: " << depth << std::endl;
    }
    int channels = arg.channels();/*
     for (int i = 0; i < arg.rows; ++i) {
     for (int j = 0; j < arg.cols; ++j) {
     switch (type) {
     case CV_8U:
     std::cerr << (int) arg.at<unsigned char>(i, j);
     break;
     case CV_8S:
     std::cerr << arg.at<char>(i, j);
     break;
     case CV_16U:
     std::cerr << arg.at<unsigned int>(i, j);
     break;
     case CV_16S:
     std::cerr << arg.at<int>(i, j);
     break;
     case CV_32S:
     std::cerr << arg.at<long int>(i, j);
     break;
     case CV_32F:
     if (2 == channels) {
     std::cerr << arg.at<cv::Point2f>(i, j);
     } else if (3 == channels) {
     std::cerr << arg.at<cv::Point3f>(i, j);
     } else {
     std::cerr << arg.at<float>(i, j);
     }
     break;
     case CV_64F:
     if (2 == channels) {
     std::cerr << arg.at<cv::Point2d>(i, j);
     } else if (3 == channels) {
     std::cerr << arg.at<cv::Point3d>(i, j);
     } else {
     std::cerr << arg.at<double>(i, j);
     }
     break;
     }
     std::cerr << " ";
     }
     std::cerr << std::endl;
     }*/
}

void extractFeatures(const Mat &input, const int& maxCorners,
        vector<Point2f>& corners);

void refillFeatures(const cv::Mat& oldIn, const unsigned &maxCorners,
        const double &threshold,
        std::vector<std::list<Point2f> > &featureHistory);

void pyrLK(const cv::Mat &oldInput, const cv::Mat &newInput,
        std::vector<std::list<Point2f> > &featureHistory);

std::vector<Point2f> getTranslationVectors(
        const std::vector<Point2f> &oldPositions,
        const std::vector<Point2f> &newPositions) {
    std::vector<Point2f> result(oldPositions.size());
    for (unsigned int i = 0; i < result.size(); ++i) {
        result[i] = newPositions[i] - oldPositions[i];
    }
    return result;
}

void getNewAndOldFeaturesFromHistory(
        const std::vector<std::list<cv::Point2f> > &featuresHistory,
        std::vector<cv::Point2f> &newFeatures,
        std::vector<cv::Point2f> &oldFeatures) {
    newFeatures.resize(featuresHistory.size());
    oldFeatures.resize(featuresHistory.size());
    for (unsigned int i = 0; i < featuresHistory.size(); ++i) {
        std::list<cv::Point2f>::const_iterator it =
                featuresHistory.at(i).begin();
        newFeatures[i] = *it;
        ++it;
        oldFeatures[i] = *it;
    }
}

std::vector<cv::Point2f> getLatestFeaturesFromHistory(
        const std::vector<std::list<cv::Point2f> > &featuresHistory) {
    std::vector<cv::Point2f> result(featuresHistory.size());
    for (unsigned int i = 0; i < featuresHistory.size(); ++i) {
        result[i] = featuresHistory[i].front();
    }
    return result;
}

std::vector<cv::Point2f> getTranslationVectors(
        const std::vector<std::list<cv::Point2f> > &featuresHistory) {
    std::vector<cv::Point2f> newFeatures(featuresHistory.size()), oldFeatures(
            featuresHistory.size());
    getNewAndOldFeaturesFromHistory(featuresHistory, newFeatures, oldFeatures);
    return getTranslationVectors(oldFeatures, newFeatures);
}

std::vector<float> getRotationAngles(const std::vector<Point2f> &oldPositions,
        const std::vector<Point2f> &newPositions, const double &distance,
        const Point2f &imgCenter) {
    std::vector<float> result(oldPositions.size());
    for (unsigned int i = 0; i < result.size(); ++i) {
        result[i] = atan2(newPositions[i].x - imgCenter.x, distance)
                - atan2(oldPositions[i].x - imgCenter.x, distance);
    }
    return result;
}

std::vector<float> getRotationAngles(
        const std::vector<std::list<cv::Point2f> > &featuresHistory,
        const double &distance, const cv::Point2f &imageCenter) {
    std::vector<cv::Point2f> newFeatures(featuresHistory.size()), oldFeatures(
            featuresHistory.size());
    getNewAndOldFeaturesFromHistory(featuresHistory, newFeatures, oldFeatures);
    return getRotationAngles(oldFeatures, newFeatures, distance, imageCenter);

}

bool horizontalPoint2Compare(Point2f p1, Point2f p2) {
    return p1.x < p2.x;
}

bool verticalPoint2Compare(Point2f p1, Point2f p2) {
    return p1.y < p2.y;
}

cv::Mat createTranslationHistogram(
        const std::vector<Point2f>& featureTranslations, const int &bins,
        const float &maxSize, const bool &vertical) {
    int histSize[] = { bins };
    float range[] = { -maxSize, maxSize };
    const float *ranges[] = { range };
    cv::Mat result;
    int histDims = 1;
    int channels[] = { 0 };
    std::vector<float> translations(featureTranslations.size());

    for (unsigned int i = 0; i < translations.size(); ++i) {
        if (vertical) {
            translations[i] = featureTranslations[i].x;
        } else {
            translations[i] = featureTranslations[i].y;
        }
    }

    cv::Mat translationMat(translations);
    calcHist(&translationMat, 1, channels, cv::Mat(), result, histDims,
            histSize, ranges, true, false);
    return result;
}

cv::Mat createRotationHistogram(const std::vector<float>& featureRotations,
        const int &bins) {
    cv::Mat result;
    float range[] = { -CV_PI, CV_PI };
    int channels[] = { 0 };
    int histSize[1] = { bins };
    int histDims = 1;
    const float *ranges[] = { range };
    cv::Mat rotMat(featureRotations);
    calcHist(&rotMat, 1, channels, cv::Mat(), result, histDims, histSize,
            ranges, true, false);
    return result;
}

cv::Mat getDrawableHistogram1D(const cv::Mat &histogram,
        const cv::Size &scale) {
    double maxValue;
    int rows = 10;
    int cols = histogram.rows;
    cv::Mat result = cv::Mat::zeros(rows * scale.width, cols * scale.height,
            CV_8UC3);
    cv::minMaxLoc(histogram, 0, &maxValue, 0, 0);

    if (0 == maxValue) {
        cv::Exception ex;
        ex.err = "maxValue == 0";
        throw ex;
    }

    for (int i = 0; i < cols - 1; ++i) {
        float histValue = histogram.at<float>(i, 0);

        Point p1 = Point(i * scale.width, rows * scale.height);
        Point p2 = Point((i + 1) * scale.width, rows * scale.height);
        Point p3 = Point((i + 1) * scale.width,
                (rows - histValue * rows / maxValue) * scale.height);
        Point p4 = Point(i * scale.width,
                (rows - histValue * rows / maxValue) * scale.height);

        int numPts = 5;
        Point pts[] = { p1, p2, p3, p4, p1 };
        fillConvexPoly(result, pts, numPts, Scalar::all(255));
    }
    return result;
}

cv::Mat getDrawableHistogram2D(const cv::Mat &histogram, const int &scale,
        const int &bins) {
    cv::Mat result;
    cv::Mat th = cv::Mat::zeros(histogram.size(), CV_8UC1);
    int sum = (cv::sum(histogram))[0];
    for (int i = 0; i < histogram.rows; ++i) {
        for (int j = 0; j < histogram.cols; ++j) {
            th.at<unsigned char>(i, j) = 255 * histogram.at<float>(i, j) / sum;
        }
    }
    cv::resize(th, result, Size(th.rows * 100, th.cols * 100));
    for (int i = 1; i < result.rows; i = i + result.rows / bins) {
        cv::line(result, Point(i, 0), Point(i, result.rows), Scalar::all(255),
                1, CV_AA);
        cv::line(result, Point(0, i), Point(result.cols, i), Scalar::all(255),
                1, CV_AA);
    }
    return result;
}

void testFunction(cv::InputArray ip, cv::InputArray op) {
    cv::Mat img = ip.getMat();
    cv::Mat obj = op.getMat();

    printMatrix(img);
    printMatrix(obj);

//	std::cerr<<img.checkVector()<<std::endl;

}

int main(int argc, char **argv) {
    String fileName = "../sphereAF.yml";
//String fileName = "../logitech.yaml";
//String fileName = "../creativeDom.yml";
    cv::FileStorage fs(fileName, FileStorage::READ);
    cv::Mat intrinistics;
    cv::Mat cameraMatrix;
    cv::Mat distortionCoeffs;
    cv::Mat map1, map2;
    double squareSize = 24;
    cv::Size birdSize(1000, 1000);

    cv::Size boardSize(9, 6);
//cv::Size boardSize(10, 7);
    Mat newIn, oldIn;
    Mat greyNewIn;
    int maxCorners = 200;
    int maxTrackbar = 1000;
    int maxHistoryLength = 10;
    Size imageSize;

    if (!fs.isOpened()) {
        return 1;
    }

    fs["camera_matrix"] >> intrinistics;
    fs["distortion_coefficients"] >> distortionCoeffs;
    fs["image_width"] >> imageSize.width;
    fs["image_height"] >> imageSize.height;

    imageSize.width = 640;
    imageSize.height = 480;

    cameraMatrix = cv::getOptimalNewCameraMatrix(intrinistics, distortionCoeffs,
            Size(640, 480), 1);
    cv::initUndistortRectifyMap(intrinistics, distortionCoeffs,
            Mat::eye(3, 3, CV_32F), cameraMatrix, imageSize, CV_32FC1, map1,
            map2);

    string mainWinName = "mainWindow";

    VideoCapture capture(1);
    capture.set(CV_CAP_PROP_FRAME_WIDTH, imageSize.width);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, imageSize.height);

    char control = 'a';
    namedWindow(mainWinName.c_str(), CV_WINDOW_AUTOSIZE);
    createTrackbar("Max corners", mainWinName.c_str(), &maxCorners,
            maxTrackbar);
//    vector<Point2f> lowNewFeatures;
//    vector<Point2f> lowOldFeatures;
//    vector<Point2f> highNewFeatures;
//    vector<Point2f> highOldFeatures;

//initial step
    capture >> newIn;
    cvtColor(newIn, greyNewIn, CV_RGB2GRAY);
    cvtColor(newIn, oldIn, CV_RGB2GRAY);
    bool found = false;
    vector<Point2f> corners;
    int horizon = newIn.rows / 2;
    int deadZone = 0;
    int height = 1;
    Mat perspTrans;
    namedWindow("trackbars", CV_WINDOW_AUTOSIZE);
    createTrackbar("horizon", "trackbars", &horizon, newIn.rows);
    createTrackbar("dead zone", "trackbars", &deadZone, newIn.rows / 2);
//createTrackbar("height", mainWinName.c_str(), &height, 1000);

    namedWindow("ground", CV_WINDOW_NORMAL);
    namedWindow("far", CV_WINDOW_NORMAL);
    Mat shown;
    Rect lowerRoi;
    Rect upperRoi;
    Mat ground;
    Mat far;
    do {
        capture >> newIn;
        Mat undistorted;
        cv::remap(newIn, undistorted, map1, map2, INTER_LINEAR);
        cvtColor(undistorted, greyNewIn, CV_RGB2GRAY);
        shown = undistorted.clone();
        lowerRoi = Rect(Point2f(0, horizon + deadZone),
                Size(shown.cols, shown.rows - horizon - deadZone));
        upperRoi = Rect(Point2f(0, 0), Size(shown.cols, horizon - deadZone));
        ground = shown(lowerRoi);
        far = shown(upperRoi);
        drawDeadZoneHorizon(shown, horizon, deadZone);
        imshow("ground", ground);
        imshow("far", far);
        imshow(mainWinName.c_str(), shown);
        control = waitKey(1);
    } while ('s' != control && 'q' != control);

    if ('q' == control)
        return 0;
    Mat undistorted;
    cv::remap(greyNewIn, undistorted, map1, map2, INTER_LINEAR);
    far = undistorted(lowerRoi);
    ground = undistorted(upperRoi);
    std::cerr << "searching" << std::endl;
    do {
        capture >> newIn;
        cv::remap(newIn, undistorted, map1, map2, INTER_LINEAR);
        cvtColor(undistorted, greyNewIn, CV_RGB2GRAY);
        far = greyNewIn(upperRoi);
        ground = greyNewIn(lowerRoi);

        found = findChessboardCorners(ground, boardSize, corners,
                CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_ADAPTIVE_THRESH
                        | CV_CALIB_CB_NORMALIZE_IMAGE);
        shown = undistorted.clone();
        if (corners.size() > 0) {
            drawChessboardCorners(shown(lowerRoi), boardSize, corners, found);
            drawChessboardCorners(ground, boardSize, corners, found);
            std::cerr << corners.size() << std::endl;
        }
        drawDeadZoneHorizon(shown, horizon, deadZone);
        imshow("ground", ground);
        imshow("far", far);
        imshow(mainWinName.c_str(), shown);
        control = waitKey(1);
    } while (!found && 'q' != control);

    cv::Point2f rotationCentre;

    if ('q' == control)
        return 0;
    else {

        shown = undistorted.clone();
        Mat shownGround = shown(lowerRoi);
        drawChessboardCorners(shownGround, boardSize, corners, found);
        imshow(mainWinName.c_str(), shown);
        waitKey(0);
        Size winSize = Size(5, 5);
        Size zeroZone(-1, -1);
        TermCriteria termCrit = TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER,
                40, 0.001);
        cornerSubPix(ground, corners, winSize, zeroZone, termCrit);
        vector<Point2f> objPts(4), imgPts(4);
        cv::Size imageBoardSize((boardSize.width - 1) * squareSize,
                (boardSize.height - 1) * squareSize);
        //cv::Size imageSize = shown.size();
        objPts[0].x = birdSize.width / 2 - imageBoardSize.width / 2;
        objPts[0].y = birdSize.height - horizon - deadZone
                - imageBoardSize.height;
        objPts[1].x = birdSize.width / 2 + imageBoardSize.width / 2;
        objPts[1].y = birdSize.height - horizon - deadZone
                - imageBoardSize.height;
        objPts[2].x = birdSize.width / 2 - imageBoardSize.width / 2;
        objPts[2].y = birdSize.height - horizon - deadZone;
        objPts[3].x = birdSize.width / 2 + imageBoardSize.width / 2;
        objPts[3].y = birdSize.height - horizon - deadZone;
        imgPts[0] = corners[0];
        imgPts[1] = corners[boardSize.width - 1];
        imgPts[2] = corners[(boardSize.height - 1) * boardSize.width];
        imgPts[3] = corners[(boardSize.height) * boardSize.width - 1];


        perspTrans = getPerspectiveTransform(objPts, imgPts);
        perspTrans.at<double>(Point(2, 2)) = height;

        cv::Mat zeroBird;
        warpPerspective(greyNewIn(lowerRoi), zeroBird, perspTrans,
                greyNewIn.size(), INTER_LINEAR | WARP_INVERSE_MAP);
        cv::Mat rvec, tvec;

        std::vector<Point3f> mCorners;
        for (int i = 0; i < boardSize.height; ++i) {
            for (int j = 0; j < boardSize.width; ++j) {
                mCorners.push_back(
                        Point3f(i * squareSize, j * squareSize, 0.9));
                std::cerr << mCorners.back() << std::endl;
            }
        }

        cv::solvePnP(mCorners, corners, cameraMatrix, distortionCoeffs, rvec,
                tvec, false, CV_ITERATIVE);
        cv::Mat rot;
        cv::Rodrigues(rvec, rot);
        printMatrix(rvec);
        printMatrix(rot);
        printMatrix(tvec);

        printMatrix(rtTransformMatrix(rot, tvec));
        printMatrix(invRtTransformMatrix(rot, tvec));
        cv::Mat invRt = invRtTransformMatrix(rot, tvec);
        rotationCentre = objPts[0]
                + cv::Point2f(invRt.at<float>(3, 0), invRt.at<float>(3, 1));
        std::cerr << invRt << std::endl;
        {
            cv::Mat invT = getT(invRt);

            cv::Mat z = invRt(cv::Range(0, 3), cv::Range(2, 3));
            std::cerr << z << std::endl << cv::Vec3d(z);
            z.at<double>(2) = 0;
            cv::Mat zN = z / norm(cv::Vec3d(z));

            std::cerr << zN << std::endl;

            double gamma = atan2(zN.at<double>(1), zN.at<double>(0))
                    - atan2(0, 1);
            std::cerr << gamma << std::endl;
            gamma *= 180 / CV_PI;
            cv::Mat rMatN = cv::getRotationMatrix2D(objPts[0], -gamma, 1);

            cv::Mat homN;

            std::vector<cv::Point2f> rPointsN;
            cv::transform(objPts, rPointsN, rMatN);
            homN = cv::getPerspectiveTransform(rPointsN, imgPts);

            std::cerr << gamma << std::endl;

            cv::Mat nBird;
           warpPerspective(greyNewIn(lowerRoi), nBird, homN, greyNewIn.size(),
                    INTER_LINEAR | WARP_INVERSE_MAP);
            perspTrans = homN.clone();
        }
    }
    namedWindow("birdsEye", CV_WINDOW_NORMAL);
    Mat newBirdGround;
    Mat oldBirdGround;
    Mat oldFar;
    Mat newFar;
    {
        Mat newGround = greyNewIn(lowerRoi);
        warpPerspective(newGround, newBirdGround, perspTrans, greyNewIn.size(),
                INTER_LINEAR | WARP_INVERSE_MAP);
        warpPerspective(newGround, oldBirdGround, perspTrans, greyNewIn.size(),
                INTER_LINEAR | WARP_INVERSE_MAP);
    }
    Mat newGround = greyNewIn(lowerRoi);
    Mat oldGround = oldIn(lowerRoi);

    std::vector<std::list<Point2f> > lowFeatureHistory;
    std::vector<std::list<Point2f> > highFeatureHistory;
    std::vector<float> rotations;

    namedWindow("trHistVe", CV_WINDOW_NORMAL);
    namedWindow("trHistHo", CV_WINDOW_NORMAL);
    namedWindow("roHist", CV_WINDOW_NORMAL);

    SmoothFilter filter(CV_PI / 16, 10,5,30);
    ImageEdgeFilter edgeFilter(perspTrans, newGround.size(), 60);
    do {
        //perspTrans.at<double>(Point(2, 2)) = height;

        capture >> newIn;
        cv::remap(newIn, undistorted, map1, map2, INTER_LINEAR);

        cvtColor(undistorted, greyNewIn, CV_RGB2GRAY);

        oldFar = oldIn(upperRoi);
        newFar = greyNewIn(upperRoi);

        newGround = greyNewIn(lowerRoi);
        oldGround = oldIn(lowerRoi);

        shown = greyNewIn.clone();
        Mat lowerShown = shown(lowerRoi);
        Mat upperShown = shown(upperRoi);
        Mat showBird;

        warpPerspective(newGround, newBirdGround, perspTrans, birdSize,
                INTER_LINEAR | WARP_INVERSE_MAP);
        warpPerspective(oldGround, oldBirdGround, perspTrans, birdSize,
                INTER_LINEAR | WARP_INVERSE_MAP);
        showBird = newBirdGround.clone();

        refillFeatures(oldBirdGround, maxCorners, 0.9, lowFeatureHistory);
        if (!lowFeatureHistory.empty()) {
            pyrLK(oldBirdGround, newBirdGround, lowFeatureHistory);
            trimFeatureHistory(lowFeatureHistory, maxHistoryLength);
        }
        refillFeatures(oldFar, maxCorners, 0.9, highFeatureHistory);
        if (!highFeatureHistory.empty()) {
            pyrLK(oldFar, newFar, highFeatureHistory);
            trimFeatureHistory(highFeatureHistory, maxHistoryLength);
        }

        lowFeatureHistory = filter.filterFeatures(lowFeatureHistory);
        highFeatureHistory = filter.filterFeatures(highFeatureHistory);
        lowFeatureHistory = edgeFilter.filterFeatures(lowFeatureHistory);

        if (!lowFeatureHistory.empty()) {
            vector<std::list<Point2f> > trHist = transformFeatureHistory(
                    lowFeatureHistory, perspTrans);
            for (unsigned int h = 0; h < trHist.size(); ++h) {
                drawFeatureHistory(lowerShown, trHist[h]);
            }
        }
        std::cerr << lowFeatureHistory.size() << " "
                << highFeatureHistory.size() << std::endl;
        try {
            for (unsigned int i = 0; i < lowFeatureHistory.size(); ++i) {
                drawFeatureHistory(showBird, lowFeatureHistory[i]);
            }
            for (unsigned int j = 0; j < highFeatureHistory.size(); ++j) {
                drawFeatureHistory(upperShown, highFeatureHistory[j]);
            }

        } catch (cv::Exception &ex) {
            std::cerr << ex.what() << std::endl;
        }
        std::cerr << "ple!" << std::endl;

        std::vector<cv::Point2f> translations = getTranslationVectors(
                lowFeatureHistory);
        std::cerr << "pu" << std::endl;

        std::vector<float> rotations = getRotationAngles(highFeatureHistory,
                3.22, Point2f(greyNewIn.cols / 2, greyNewIn.rows / 2));
        std::cerr << "fu" << std::endl;
        /*
         cv::Mat verTransHist;
         cv::Mat horTransHist;
         cv::Mat rotHist;
         cv::Mat showTrVe;
         cv::Mat showTrHo;
         cv::Mat showRo;
         //----------------------------------------------------------------
         if (!translations.empty()) {
         //std::cerr << "raz" << std::endl;
         verTransHist = createTranslationHistogram(translations, 101, 480, true);
         //std::cerr << "dwa" << std::endl;
         horTransHist = createTranslationHistogram(translations, 101, 640, true);
         //std::cerr << "czy" << std::endl;
         showTrVe = getDrawableHistogram1D(verTransHist, Size(10, 10));
         //std::cerr << "cztery" << std::endl;
         showTrHo = getDrawableHistogram1D(horTransHist, Size(10, 10));
         imshow("trHistVe", showTrVe);
         imshow("trHistHo", showTrHo);
         }
         if (!rotations.empty()) {
         //std::cerr << "piec" << std::endl;
         rotHist = createRotationHistogram(rotations, 101);
         //std::cerr << "szesc" << std::endl;
         showRo = getDrawableHistogram1D(rotHist, Size(10, 10));
         //std::cerr << "siedem" << std::endl;
         imshow("roHist", showRo);
         }
         //----------------------------------------------------------------
         */
        //
        putText(shown, num2Str(lowFeatureHistory.size()), Point(0, shown.rows),
                cv::FONT_HERSHEY_SIMPLEX, 2, Scalar::all(255), 1, true);
        imshow("birdsEye", showBird);

        drawDeadZoneHorizon(shown, horizon, deadZone);
        imshow(mainWinName.c_str(), shown);
        imshow("ground", lowerShown);
        imshow("far", upperShown);
        if ('s' == control) {
            cv::Mat colorBird;
            cv::Mat colorGround = undistorted(lowerRoi);
            warpPerspective(colorGround, colorBird, perspTrans,
                    colorGround.size(), INTER_LINEAR | WARP_INVERSE_MAP);

            //	cv::imwrite("before.png", colorGround);
            //	cv::imwrite("after.png", colorBird);
            //	std::cerr << "saved!" << std::endl;
        }

        oldIn = greyNewIn.clone();
        //  lowOldFeatures = lowNewFeatures;
        //  highOldFeatures = highNewFeatures;
        control = waitKey(1);
    } while ('q' != control);
}

void extractFeatures(const Mat &input, const int& maxCorners,
        vector<Point2f>& corners) {
    double qualityLevel = 0.1;
    double minDistance = 10;
    int blockSize = 5;
    bool useHarrisDetector = false;
    double k = 0.04;

    Size winSize = Size(5, 5);
    Size zeroZone(-1, -1);
    TermCriteria termCrit = TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40,
            0.001);
    printMatrix(input);
    goodFeaturesToTrack(input, corners, maxCorners, qualityLevel, minDistance,
            Mat(), blockSize, useHarrisDetector, k);
    if (!corners.empty()) {
        cornerSubPix(input, corners, winSize, zeroZone, termCrit);
    }
}

void pyrLK(const cv::Mat &oldInput, const cv::Mat &newInput,
        std::vector<std::list<Point2f> > &featureHistory) {
    std::vector<cv::Point2f> newFeatures, oldFeatures;
    oldFeatures = getLatestFeaturesFromHistory(featureHistory);
    vector<uchar> status;
    vector<float> err; //thou shall not use double instead of float!!
    TermCriteria termCrit = TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40,
            0.001);
    Size windowSize = Size(21, 21);
    unsigned int maxLevel = 3;
    int flags = 0;
    double minEigThresh = 1e-4;
    calcOpticalFlowPyrLK(oldInput, newInput, oldFeatures, newFeatures, status,
            err, windowSize, maxLevel, termCrit, flags, minEigThresh);
    std::cerr << "pyr " << oldFeatures.size() << " " << newFeatures.size()
            << std::endl;
    for (unsigned i = 0; i < newFeatures.size(); ++i) {
        if (!status[i] || (err[i] > 500)) {
            newFeatures.erase(newFeatures.begin() + i);
            oldFeatures.erase(oldFeatures.begin() + i);
            status.erase(status.begin() + i);
            err.erase(err.begin() + i);
            featureHistory.erase(featureHistory.begin() + i);
            --i;
        } else {
            if (featureHistory[i].front() != oldFeatures[i]) {
                std::cerr << "skucha" << std::endl;
            }
            featureHistory[i].push_front(newFeatures[i]);
        }
    }
}

void refillFeatures(const cv::Mat& oldIn, const unsigned &maxCorners,
        const double &threshold,
        std::vector<std::list<Point2f> >& featureHistory) {

    std::vector<cv::Point2f> newFeatures(featureHistory.size()), oldFeatures(
            featureHistory.size());
    getNewAndOldFeaturesFromHistory(featureHistory, newFeatures, oldFeatures);

    if (featureHistory.size() > maxCorners) {
        featureHistory.resize(maxCorners);
    } else if (featureHistory.size() < threshold * maxCorners) {
        vector<Point2f> additionalFeatures;
        vector<std::list<Point2f> > additionalFeatureList;
        extractFeatures(oldIn, maxCorners - featureHistory.size(),
                additionalFeatures);
        additionalFeatureList.resize(additionalFeatures.size());
        for (unsigned int i = 0; i < additionalFeatures.size(); ++i) {
            additionalFeatureList[i].push_front(additionalFeatures[i]);
        }
        std::cerr << "New features added: " << additionalFeatures.size()
                << std::endl;
//    oldFeatures.insert(oldFeatures.end(), additionalFeatures.begin(),
//            additionalFeatures.end());
        featureHistory.insert(featureHistory.end(),
                additionalFeatureList.begin(), additionalFeatureList.end());
//    std::sort(oldFeatures.begin(), oldFeatures.end(), comparePoints);
        std::sort(featureHistory.begin(), featureHistory.end(),
                compareListPoints);
//    oldFeatures.erase(std::unique(oldFeatures.begin(), oldFeatures.end()));
        featureHistory.erase(
                std::unique(featureHistory.begin(), featureHistory.end(),
                        listPointsEqual));
    }
}

/*
 } catch (cv::Exception &ex) {
 std::cerr << ex.what() << std::endl;
 }
 */
