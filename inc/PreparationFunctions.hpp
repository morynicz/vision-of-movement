#ifndef PREPARATION_FUNCTIONS_HPP
#define PREPARATION_FUNCTIONS_HPP

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <list>
#include <vector>

void calibrateParameters(cv::VideoCapture &capture,
		std::vector<cv::Mat> rectifyMaps, int &horizon, int &deadZone,
		const cv::Size &imageSize);

std::vector<cv::Point2f> getChessboardCorners(cv::VideoCapture &capture,
		const std::vector<cv::Mat> &rectificationMaps, const int &horizon,
		const int &deadZone, cv::Size &boardSize, cv::Size &imageSize,
		cv::Size &winSize, cv::Size &zeroZone, cv::TermCriteria termCriteria);

void getHomographyAndRtMatrix(std::vector<cv::Point2f> corners,
		const cv::Size& imageSize, const cv::Size& boardSize,
		const double& squareSize, const int &horizon, const int &deadZone,
		const cv::Mat& cameraMatrix, const cv::Mat& distortionCoefficients,
		cv::Mat& homography, cv::Point2f rotationCenter, cv::Mat &rtMatrix);

cv::Mat rtTransformMatrix(const cv::Mat &rot, const cv::Mat &trans);

cv::Mat invRtTransformMatrix(const cv::Mat& rot, const cv::Mat& trans);

cv::Mat invRtTransformMatrix(const cv::Mat& rtTransformMatrix);

void getCameraParameters(const std::string &fileName,
		std::vector<cv::Mat> &undistortionMaps, cv::Mat& cameraMatrix,
		cv::Mat& distortionCoefficients, cv::Size &imageSize);

#endif
