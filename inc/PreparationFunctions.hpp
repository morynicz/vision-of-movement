#ifndef PREPARATION_FUNCTIONS_HPP
#define PREPARATION_FUNCTIONS_HPP

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <list>
#include <vector>

#include "Catcher.hpp"
#include "CvTypesIo.hpp"

class ChessboardParameters {
    public:
        cv::Size winSize;
        cv::Size zeroZone;
        float squareSize;
        double height;
        cv::Size size;
        cv::TermCriteria termCrit;
};

class CameraSpatialParameters{
    public:
        cv::Mat homography;
        cv::Mat rtMatrix;
        cv::Point2d rotationCenter;
        int horizon;
        int deadZone;
};

void calibrateParameters(cv::VideoCapture &capture,
        std::vector<cv::Mat> rectifyMaps, int &horizon, int &deadZone,
        const cv::Size &imageSize);

void calibrateParametersSingleImage(cv::VideoCapture &capture,
        std::vector<cv::Mat> rectifyMaps, int &horizon, int &deadZone,
        const cv::Size &imageSize);

std::vector<cv::Point2f> getChessboardCorners(cv::VideoCapture &capture,
        const std::vector<cv::Mat> &rectificationMaps,
        const int &horizon, const int &deadZone,
        const cv::Size &boardSize, const cv::Size &imageSize,
        const cv::Size &winSize, const cv::Size &zeroZone,
        const cv::TermCriteria &termCriteria);



void getHomographyRtMatrixAndRotationCenter(
        std::vector<cv::Point2f> corners, const cv::Size& imageSize,
        const cv::Size& boardSize, const double& squareSize,
        const int &horizon, const int &deadZone,
        const cv::Mat& cameraMatrix,
        const cv::Mat& distortionCoefficients,
        const double &chessboardHeight, cv::Mat& homography,
        cv::Point2d &rotationCenter, cv::Mat &rtMatrix);

cv::Mat rtTransformMatrix(const cv::Mat &rot, const cv::Mat &trans);

cv::Mat invRtTransformMatrix(const cv::Mat& rot,
        const cv::Mat& trans);

cv::Mat invRtTransformMatrix(const cv::Mat& rtTransformMatrix);

void getCameraParameters(const std::string &fileName,
        std::vector<cv::Mat> &undistortionMaps, cv::Mat& cameraMatrix,
        cv::Mat& distortionCoefficients, cv::Size &imageSize);

cv::FileStorage &operator<<(cv::FileStorage &fs,
        const ChessboardParameters &chessboard);

void operator>>(const cv::FileNode &node,
        ChessboardParameters &chessboard);

cv::FileStorage &operator<<(cv::FileStorage &fs,
        const CameraSpatialParameters &camera);

void operator>>(const cv::FileNode &node,
        CameraSpatialParameters &camera);

#endif
