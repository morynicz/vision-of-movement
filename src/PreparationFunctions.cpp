/*
 * PreparationFunctions.cpp
 *
 *  Created on: Apr 5, 2013
 *      Author: link
 */

#include "PreparationFunctions.hpp"
#include "ErrorCodes.hpp"
#include "DrawingFunctions.hpp"

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"

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

	cv::createTrackbar("horizon", "trackbars", &horizon, imageSize.height);
	cv::createTrackbar("dead zone", "trackbars", &deadZone,
			imageSize.height / 2);
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

		imshow("ground", ground);
		imshow("far", far);
		imshow("main", undistorted);

		control = cv::waitKey(1);
	} while ('s' != control && 'q' != control);

	if ('q' == control) {
		cv::Exception ex(USER_TRIGGERED_EXIT, "user commanded exit", __func__,
				__FILE__, __LINE__);
		throw ex;
	}

}

std::vector<cv::Point2f> getChessboardCorners(cv::VideoCapture &capture,
		const std::vector<cv::Mat> &rectificationMaps, const int &horizon,
		const int &deadZone, cv::Size &boardSize, cv::Size &imageSize,
		cv::Size &winSize, cv::Size &zeroZone, cv::TermCriteria termCriteria) {
	cv::Mat newIn;
	cv::Mat undistorted;
	cv::Mat greyNewIn;
	cv::Mat ground;

	cv::Rect lowerRoi(cv::Point2f(0, horizon + deadZone),
			cv::Size(imageSize.width, imageSize.height - horizon - deadZone));
	bool found = false;
	std::vector<cv::Point2f> corners;
	int cornerCount = 0;
	cv::namedWindow("ground", CV_WINDOW_NORMAL);
	char control = ' ';
	do {
		capture >> newIn;
		cv::remap(newIn, undistorted, rectificationMaps[0],
				rectificationMaps[1], cv::INTER_LINEAR);
		cv::cvtColor(undistorted, greyNewIn, CV_RGB2GRAY);
		ground = greyNewIn(lowerRoi);
		found = cv::findChessboardCorners(ground, boardSize, corners,
				CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_ADAPTIVE_THRESH
						| CV_CALIB_CB_NORMALIZE_IMAGE);
		//shown = undistorted.clone();
		if (cornerCount > 0) {
			drawChessboardCorners(undistorted, boardSize, corners, found);
		}
		drawDeadZoneHorizon(undistorted, horizon, deadZone);
		imshow("ground", ground);
		//imshow("main"), shown);
		control = cv::waitKey(1);
	} while (!found && 'q' != control);

	if ('q' == control) {
		cv::Exception ex(USER_TRIGGERED_EXIT, "user requested exit", __func__,
				__FILE__, __LINE__);
		throw ex;
	}
	cv::cornerSubPix(ground, corners, winSize, zeroZone, termCriteria);
	cv::imwrite("chessboard.jpeg", undistorted);
	cv::drawChessboardCorners(undistorted(lowerRoi), boardSize, corners, found);
	cv::imshow("main", undistorted);
	cv::waitKey(0);
	return corners;
}

void getHomographyAndRtMatrix(std::vector<cv::Point2f> corners,
		const cv::Size& imageSize, const cv::Size& boardSize,
		const double& squareSize, const int &horizon, const int &deadZone,
		const cv::Mat& cameraMatrix, const cv::Mat& distortionCoefficients,
		cv::Mat& homography, cv::Point2f rotationCenter, cv::Mat &rtMatrix) {

	cv::Size imageBoardSize((boardSize.width - 1) * squareSize,
			(boardSize.height - 1) * squareSize);

	std::vector<cv::Point2f> objPts(4), imgPts(4);

	objPts[0].x = imageSize.width / 2 - imageBoardSize.width / 2;
	objPts[0].y = imageSize.height - horizon - deadZone - imageBoardSize.height;
	objPts[1].x = imageSize.width / 2 + imageBoardSize.width / 2;
	objPts[1].y = imageSize.height - horizon - deadZone - imageBoardSize.height;
	objPts[2].x = imageSize.width / 2 - imageBoardSize.width / 2;
	objPts[2].y = imageSize.height - horizon - deadZone;
	objPts[3].x = imageSize.width / 2 + imageBoardSize.width / 2;
	objPts[3].y = imageSize.height - horizon - deadZone;

	imgPts[0] = corners[0];
	imgPts[1] = corners[boardSize.width - 1];
	imgPts[2] = corners[(boardSize.height - 1) * boardSize.width];
	imgPts[3] = corners[(boardSize.height) * boardSize.width - 1];

	homography = cv::getPerspectiveTransform(objPts, imgPts);

	cv::Mat rvec, tvec;

	std::vector<cv::Point3f> mCorners;
	for (int i = 0; i < boardSize.height; ++i) {
		for (int j = 0; j < boardSize.width; ++j) {
			mCorners.push_back(
					cv::Point3f(j * squareSize, i * squareSize, 0.9));
		}
	}

	cv::solvePnP(mCorners, corners, cameraMatrix, distortionCoefficients, rvec,
			tvec, false, CV_ITERATIVE);
	cv::Mat rot;
	cv::Rodrigues(rvec, rot);

	rtMatrix = invRtTransformMatrix(rot, tvec);
	rotationCenter = objPts[0]
			+ cv::Point2f(rtMatrix.at<float>(3, 0), rtMatrix.at<float>(3, 1));

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

void getCameraParameters(const std::string &fileName,
		std::vector<cv::Mat> &undistortionMaps, cv::Mat& cameraMatrix,
		cv::Mat& distortionCoefficients, cv::Size &imageSize) {
	cv::FileStorage fs(fileName, cv::FileStorage::READ);
	cv::Mat intrinistics;

	if (!fs.isOpened()) {
		cv::Exception ex(READ_CAMERA_PARAMETERS_FAILED,
				"could not read file: " + fileName, __func__, __FILE__,
				__LINE__);
		throw ex;
	}

	undistortionMaps.resize(2);

	fs["camera_matrix"] >> intrinistics;
	fs["distortion_coefficients"] >> distortionCoefficients;
	fs["image_width"] >> imageSize.width;
	fs["image_height"] >> imageSize.height;

	cameraMatrix = cv::getOptimalNewCameraMatrix(intrinistics,
			distortionCoefficients, imageSize, 1);
	cv::initUndistortRectifyMap(intrinistics, distortionCoefficients,
			cv::Mat::eye(3, 3, CV_32F), cameraMatrix, imageSize, CV_32FC1,
			undistortionMaps[0], undistortionMaps[1]);

}