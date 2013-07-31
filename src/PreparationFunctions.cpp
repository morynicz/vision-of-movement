/*
 * PreparationFunctions.cpp
 *
 *  Created on: Apr 5, 2013
 *      Author: link
 */

#include "PreparationFunctions.hpp"
#include "ErrorCodes.hpp"
#include "DrawingFunctions.hpp"
#include "ConvenienceFunctions.hpp"

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include "opencv2/highgui/highgui.hpp"
#include <cmath>

#include <iostream>

void calibrateParameters(cv::VideoCapture &capture,
        std::vector<cv::Mat> rectifyMaps, int &horizon, int &deadZone,
        const cv::Size &imageSize) {
    cv::Mat newIn;
    cv::Mat undistorted;
    cv::Mat far;
    cv::Mat ground;
    char control = ' ';
    cv::Rect lowerRoi;
    cv::Rect upperRoi;

    cv::namedWindow("ground", CV_WINDOW_NORMAL);
    cv::namedWindow("far", CV_WINDOW_NORMAL);
    cv::namedWindow("main", CV_WINDOW_NORMAL);
    cv::namedWindow("trackbars", CV_WINDOW_NORMAL);

    cv::createTrackbar("horizon", "trackbars", &horizon,
            imageSize.height);
    cv::createTrackbar("dead zone", "trackbars", &deadZone,
            imageSize.height / 2);
    cv::Mat resizing;
    std::cerr << imageSize.width << " " << imageSize.height
            << std::endl;
    do {
        capture >> newIn;
        cv::remap(newIn, undistorted, rectifyMaps[0], rectifyMaps[1],
                cv::INTER_LINEAR);
        lowerRoi = cv::Rect(cv::Point2f(0, horizon + deadZone),
                cv::Size(undistorted.cols,
                        undistorted.rows - horizon - deadZone));
        upperRoi = cv::Rect(cv::Point2f(0, 0),
                cv::Size(undistorted.cols, horizon - deadZone));
        ground = undistorted(lowerRoi);
        far = undistorted(upperRoi);
        drawDeadZoneHorizon(undistorted, horizon, deadZone);
        cv::line(undistorted, cv::Point(400, 0), cv::Point(400, 600),
                CV_RGB(255,0,0), 1, 8, 0);
        imshow("ground", ground);
        imshow("far", far);
        imshow("main", undistorted);

        control = cv::waitKey(1);
    } while ('s' != control && 'q' != control);

    if ('q' == control) {
        cv::Exception ex(USER_TRIGGERED_EXIT, "user commanded exit",
                __func__, __FILE__, __LINE__);
        throw ex;
    }
    cv::destroyWindow("ground");
    cv::destroyWindow("far");
    cv::destroyWindow("main");
}

void calibrateParametersSingleImage(cv::VideoCapture &capture,
        std::vector<cv::Mat> rectifyMaps, int &horizon, int &deadZone,
        const cv::Size &imageSize) {
    cv::Mat newIn;
    cv::Mat undistorted;
    cv::Mat far;
    cv::Mat ground;
    char control = ' ';
    cv::Rect lowerRoi;
    cv::Rect upperRoi;

    cv::namedWindow("ground", CV_WINDOW_NORMAL);
    cv::namedWindow("far", CV_WINDOW_NORMAL);
    cv::namedWindow("main", CV_WINDOW_NORMAL);
    cv::namedWindow("trackbars", CV_WINDOW_NORMAL);

    cv::createTrackbar("horizon", "trackbars", &horizon,
            imageSize.height);
    cv::createTrackbar("dead zone", "trackbars", &deadZone,
            imageSize.height / 2);
    cv::Mat resizing;
    do {
        capture >> newIn;
        cv::waitKey(1);
    } while (newIn.empty());
    cv::resize(newIn, resizing, imageSize);
    cv::remap(resizing, undistorted, rectifyMaps[0], rectifyMaps[1],
            cv::INTER_LINEAR);
    do {
        cv::Mat copy = undistorted.clone();

        lowerRoi = cv::Rect(cv::Point2f(0, horizon + deadZone),
                cv::Size(undistorted.cols,
                        undistorted.rows - horizon - deadZone));
        upperRoi = cv::Rect(cv::Point2f(0, 0),
                cv::Size(undistorted.cols, horizon - deadZone));
        ground = copy(lowerRoi);
        far = copy(upperRoi);
        drawDeadZoneHorizon(copy, horizon, deadZone);
        cv::line(copy, cv::Point(400, 0), cv::Point(400, 600),
                CV_RGB(255,0,0), 1, 8, 0);
        imshow("ground", ground);
        imshow("far", far);
        imshow("main", copy);

        control = cv::waitKey(0);
    } while ('s' != control && 'q' != control);

    if ('q' == control) {
        cv::Exception ex(USER_TRIGGERED_EXIT, "user commanded exit",
                __func__, __FILE__, __LINE__);
        throw ex;
    }
    cv::destroyWindow("ground");
    cv::destroyWindow("far");
    cv::destroyWindow("main");
}

std::vector<cv::Point2f> getChessboardCorners(
        cv::VideoCapture &capture,
        const std::vector<cv::Mat> &rectificationMaps,
        const int &horizon, const int &deadZone,
        const cv::Size &boardSize, const cv::Size &imageSize,
        const cv::Size &winSize, const cv::Size &zeroZone,
        const cv::TermCriteria &termCriteria) {
    cv::Mat newIn;
    cv::Mat undistorted;
    cv::Mat greyNewIn;
    cv::Mat ground;

    cv::Rect lowerRoi(cv::Point2f(0, horizon + deadZone),
            cv::Size(imageSize.width,
                    imageSize.height - horizon - deadZone));
    bool found = false;
    std::vector<cv::Point2f> corners;
    cv::namedWindow("ground", CV_WINDOW_NORMAL);
    char control = ' ';
    cv::Mat resizing;
    do {
        capture >> newIn;
        if (!newIn.empty()) {
            cv::resize(newIn, resizing, imageSize);
            cv::remap(resizing, undistorted, rectificationMaps[0],
                    rectificationMaps[1], cv::INTER_LINEAR);
            cv::cvtColor(undistorted, greyNewIn, CV_RGB2GRAY);
            ground = greyNewIn(lowerRoi);
            found = cv::findChessboardCorners(ground, boardSize,
                    corners,
                    CV_CALIB_CB_FAST_CHECK
                            | CV_CALIB_CB_ADAPTIVE_THRESH
                            | CV_CALIB_CB_NORMALIZE_IMAGE);
            if (corners.size() > 0) {
                drawChessboardCorners(undistorted(lowerRoi),
                        boardSize, corners, found);
                drawChessboardCorners(ground, boardSize, corners,
                        found);
                std::cerr << corners.size() << std::endl;
            }
            drawDeadZoneHorizon(undistorted, horizon, deadZone);
            imshow("ground", ground);
            imshow("main", undistorted);
        }
        control = cv::waitKey(1);
    } while (!found && 'q' != control);

    if ('q' == control) {
        cv::Exception ex(USER_TRIGGERED_EXIT, "user requested exit",
                __func__, __FILE__, __LINE__);
        throw ex;
    }
    cv::cornerSubPix(ground, corners, winSize, zeroZone,
            termCriteria);
    cv::imwrite("chessboard.jpeg", undistorted);
    cv::drawChessboardCorners(undistorted(lowerRoi), boardSize,
            corners, found);
    cv::imshow("main", undistorted);

    cv::destroyWindow("ground");
    cv::destroyWindow("main");

    return corners;
}

std::vector<cv::Point2f> getChessboardCornersFileStream(
        cv::VideoCapture &capture,
        const std::vector<cv::Mat> &rectificationMaps,
        const int &horizon, const int &deadZone,
        const cv::Size &boardSize, const cv::Size &imageSize,
        const cv::Size &winSize, const cv::Size &zeroZone,
        const cv::TermCriteria &termCriteria) {
    cv::Mat newIn;
    cv::Mat undistorted;
    cv::Mat greyNewIn;
    cv::Mat ground;

    cv::Rect lowerRoi(cv::Point2f(0, horizon + deadZone),
            cv::Size(imageSize.width,
                    imageSize.height - horizon - deadZone));
    bool found = false;
    std::vector<cv::Point2f> corners;
    cv::namedWindow("ground", CV_WINDOW_NORMAL);
    char control = ' ';
    cv::Mat resizing;
    do {
        capture >> newIn;
        cv::resize(newIn, resizing, imageSize);
        cv::remap(resizing, undistorted, rectificationMaps[0],
                rectificationMaps[1], cv::INTER_LINEAR);
        cv::cvtColor(undistorted, greyNewIn, CV_RGB2GRAY);
        ground = greyNewIn(lowerRoi);
        found = cv::findChessboardCorners(ground, boardSize, corners,
                CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_ADAPTIVE_THRESH
                        | CV_CALIB_CB_NORMALIZE_IMAGE);
        if (corners.size() > 0) {
            drawChessboardCorners(undistorted(lowerRoi), boardSize,
                    corners, found);
            drawChessboardCorners(ground, boardSize, corners, found);
            std::cerr << corners.size() << std::endl;
        }
        drawDeadZoneHorizon(undistorted, horizon, deadZone);
        imshow("ground", ground);
        imshow("main", undistorted);
        control = cv::waitKey(1);
    } while (!found && 'q' != control);

    if ('q' == control) {
        cv::Exception ex(USER_TRIGGERED_EXIT, "user requested exit",
                __func__, __FILE__, __LINE__);
        throw ex;
    }
    cv::cornerSubPix(ground, corners, winSize, zeroZone,
            termCriteria);
    cv::imwrite("chessboard.jpeg", undistorted);
    cv::drawChessboardCorners(undistorted(lowerRoi), boardSize,
            corners, found);
    cv::imshow("main", undistorted);

    cv::destroyWindow("ground");
    cv::destroyWindow("main");

    return corners;
}

template<class T>
cv::Mat_<T> matrixFromPoint(cv::Point3_<T> pt) {
    return (cv::Mat_<T>(3, 1) << pt.x, pt.y, pt.z);
}

const int WRONG_TYPE = -2;

cv::Point3d pointFromMatrix(const cv::Mat &mat) {
    if (CV_64F == mat.depth()) {
        return cv::Point3d(mat.at<double>(0), mat.at<double>(1),
                mat.at<double>(2));
    } else {
        cv::Exception ex(WRONG_TYPE, "wrong type, should be CV_64F",
                __func__, __FILE__, __LINE__);
        throw ex;
    }
}

cv::Mat getR(const cv::Mat &rT) {
    return rT(cv::Rect(0, 0, 3, 3));
}

cv::Mat getT(const cv::Mat &rT) {
    return rT(cv::Range(0, 3), cv::Range(3, 4));
}

void getHomographyRtMatrixAndRotationCenter(
        std::vector<cv::Point2f> corners, const cv::Size& imageSize,
        const cv::Size& boardSize, const double& squareSize,
        const int &horizon, const int &deadZone,
        const cv::Mat& cameraMatrix,
        const cv::Mat& distortionCoefficients,
        const double &chessboardHeight, cv::Mat& homography,
        cv::Point2d &rotationCenter, cv::Mat &rtMatrix) {
//The size of chessboard on the image
    cv::Size imageBoardSize((boardSize.width - 1) * squareSize,
            (boardSize.height - 1) * squareSize);

    std::vector<cv::Point2f> objPts(4), imgPts(4);
//Describe where the chessboardcorners are on the outcome image
    objPts[0].x = imageSize.width / 2 - imageBoardSize.width / 2;
    objPts[0].y = imageSize.height - horizon - deadZone
            - imageBoardSize.height;
    objPts[1].x = imageSize.width / 2 + imageBoardSize.width / 2;
    objPts[1].y = imageSize.height - horizon - deadZone
            - imageBoardSize.height;
    objPts[2].x = imageSize.width / 2 - imageBoardSize.width / 2;
    objPts[2].y = imageSize.height - horizon - deadZone;
    objPts[3].x = imageSize.width / 2 + imageBoardSize.width / 2;
    objPts[3].y = imageSize.height - horizon - deadZone;
//The only four corners used to get homography
    imgPts[0] = corners[0];
    imgPts[1] = corners[boardSize.width - 1];
    imgPts[2] = corners[(boardSize.height - 1) * boardSize.width];
    imgPts[3] = corners[(boardSize.height) * boardSize.width - 1];

    cv::Mat rvec, tvec;
//Creation of 3D chessboard model
    std::vector<cv::Point3d> mCorners;
    for (int i = 0; i < boardSize.height; ++i) {
        for (int j = 0; j < boardSize.width; ++j) {
            mCorners.push_back(
                    cv::Point3f(i * squareSize, j * squareSize,
                            chessboardHeight));
        }
    }
    std::vector<cv::Point3d> p3pcorn(4);
    p3pcorn[0] = mCorners[0];
    p3pcorn[1] = mCorners[boardSize.width - 1];
    p3pcorn[2] = mCorners[(boardSize.height - 1) * boardSize.width];
    p3pcorn[3] = mCorners[(boardSize.height) * boardSize.width - 1];
//Get rvec and tvec describing where the chessboards first corner lays in cameras coordinate system
    cv::solvePnP(p3pcorn, imgPts, cameraMatrix,
            distortionCoefficients, rvec, tvec, false, /*CV_ITERATIVE*/
            CV_ITERATIVE);

    cv::Mat rot;
    cv::Rodrigues(rvec, rot);

    rtMatrix = invRtTransformMatrix(rot, tvec);

    {
        cv::Mat invT = getT(rtMatrix);

        cv::Mat z = rtMatrix(cv::Range(0, 3), cv::Range(2, 3));

        z.at<double>(2) = 0;
        cv::Mat zN = z / norm(cv::Vec3d(z));

        double gamma = atan2(zN.at<double>(1), zN.at<double>(0))
                - atan2(0, 1);
        gamma *= 180 / CV_PI;
        cv::Mat rMatN = cv::getRotationMatrix2D(objPts[0], -gamma, 1);
        cv::Mat rMatP = cv::getRotationMatrix2D(objPts[0], gamma, 1);

        cv::Mat homN;

        std::vector<cv::Point2f> rPointsN;
        cv::transform(objPts, rPointsN, rMatN);
        homN = cv::getPerspectiveTransform(rPointsN, imgPts);

// VERY IMPORTANT: in bird's eye view, the x and y axes are swapped!
        rotationCenter = rPointsN[0]
                + cv::Point2f(rtMatrix.at<double>(1, 3),
                        rtMatrix.at<double>(0, 3));
        std::vector<cv::Point2f> rcenter, rrcenter;
        rcenter.push_back(rotationCenter);
        cv::transform(rcenter, rrcenter, rMatN);
        rotationCenter = rrcenter.front();

        homography = homN.clone();

    }

}

void getRAndT(const cv::Mat &rT, cv::Mat &r, cv::Mat &t) {
    r = rT(cv::Rect(0, 0, 3, 3));
    t = rT(cv::Range(0, 3), cv::Range(3, 4));
}

cv::Mat rtTransformMatrix(const cv::Mat &rot, const cv::Mat &trans) {
    cv::Mat r;
    if (1 == rot.cols || 1 == rot.rows) {
        cv::Rodrigues(rot, r);
    } else {
        r = rot;
    }
    cv::Mat result = cv::Mat::eye(4, 4, trans.depth());
    cv::Mat rotation; // = result(cv::Rect(0, 0, 3, 3));
    cv::Mat translation; // = result(cv::Range(0, 3), cv::Range(3, 4));
    getRAndT(result, rotation, translation);
    r.copyTo(rotation);
    trans.copyTo(translation);
    return result;
}

cv::Mat invRtTransformMatrix(const cv::Mat& rot,
        const cv::Mat& trans) {
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

void getCameraParameters(const std::string &fileName,
        std::vector<cv::Mat> &undistortionMaps, cv::Mat& cameraMatrix,
        cv::Mat& distortionCoefficients, cv::Size &imageSize) {
    cv::FileStorage fs(fileName, cv::FileStorage::READ);
    cv::Mat intrinistics;

    if (!fs.isOpened()) {
        cv::Exception ex(READ_CAMERA_PARAMETERS_FAILED,
                "could not read file: " + fileName, __func__,
                __FILE__, __LINE__);
        throw ex;
    }

    undistortionMaps.resize(2);

    fs["camera_matrix"] >> intrinistics;
    fs["distortion_coefficients"] >> distortionCoefficients;
    fs["image_width"] >> imageSize.width;
    fs["image_height"] >> imageSize.height;

    cameraMatrix = intrinistics;
    cv::initUndistortRectifyMap(intrinistics, distortionCoefficients,
            cv::Mat::eye(3, 3, CV_32F), cameraMatrix, imageSize,
            CV_32FC1, undistortionMaps[0], undistortionMaps[1]);

}

cv::FileStorage &operator<<(cv::FileStorage &fs,
        const ChessboardParameters &chessboard) {
    fs << "{" << "winSize" << chessboard.winSize << "zeroZone"
            << chessboard.zeroZone << "squareSize"
            << chessboard.squareSize << "height" << chessboard.height
            << "size" << chessboard.size << "termCrit"
            << chessboard.termCrit << "}";
    return fs;
}

void operator>>(const cv::FileNode &node,
        ChessboardParameters &chessboard) {
    node["winSize"] >> chessboard.winSize;
    node["zeroZone"] >> chessboard.zeroZone;
    node["squareSize"] >> chessboard.squareSize;
    node["height"] >> chessboard.height;
    node["size"] >> chessboard.size;
    node["termCrit"] >> chessboard.termCrit;

}

cv::FileStorage &operator<<(cv::FileStorage &fs,
        const CameraSpatialParameters &camera) {
    fs << "{" << "homography" << camera.homography << "rtMatrix"
            << camera.rtMatrix << "rotationCenter"
            << camera.rotationCenter << "horizon" << camera.horizon
            << "deadZone" << camera.deadZone << "}";
    return fs;
}

void operator>>(const cv::FileNode &node,
        CameraSpatialParameters &camera) {
    node["rtMatrix"] >> camera.rtMatrix;
    node["homography"] >> camera.homography;
    node["rotationCenter"] >> camera.rotationCenter;
    node["horizon"] >> camera.horizon;
    node["deadZone"] >> camera.deadZone;
}

