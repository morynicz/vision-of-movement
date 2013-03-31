/// @author Michał Orynicz

#include "VisualOdometer.hpp"
#include "BirdsEyeTranslationReader.hpp"
#include "TangentRotationReader.hpp"
#include "SmoothFilter.hpp"
#include "LucasCandaePyramidTracker.hpp"
#include "ShiThomasFeatureExtractor.hpp"

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <string>
#include <vector>

void printMatrix(const cv::Mat &arg) {
	std::cerr << "dims: " << arg.cols << ' ' << arg.rows << std::endl;
	std::cerr << "chans: " << arg.channels() << std::endl;
	int type = arg.depth();
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
				std::cerr << arg.at<float>(i, j);
				break;
			case CV_64F:
				std::cerr << arg.at<double>(i, j);
				break;
			}
			std::cerr << " ";
		}
		std::cerr << std::endl;
	}
}

void drawDeadZoneHorizon(cv::Mat image, const int &horizon,
		const int& deadZone) {
	cv::line(image, cv::Point2f(0, horizon + deadZone),
			cv::Point2f(image.cols, horizon + deadZone), CV_RGB(0,255,0), 2,
			CV_AA);
	cv::line(image, cv::Point2f(0, horizon - deadZone),
			cv::Point2f(image.cols, horizon - deadZone), CV_RGB(0,255,0), 2,
			CV_AA);

}

const int USER_TRIGGERED_EXIT = 1;

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

const int READ_CAMERA_PARAMETERS_FAILED = -1;

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
			cv::Mat::eye(3, 3, CV_32F), cameraMatrix, cv::Size(640, 480),
			CV_32FC1, undistortionMaps[0], undistortionMaps[1]);

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
				CV_CALIB_CB_FILTER_QUADS | CV_CALIB_CB_ADAPTIVE_THRESH
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
	cv::drawChessboardCorners(undistorted(lowerRoi), boardSize, corners, found);
	cv::imshow("main", undistorted);
	cv::waitKey(0);
	return corners;
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

void getHomographyAndRtMatrix(std::vector<cv::Point2f> corners,
		const cv::Size& imageSize, const cv::Size& boardSize,
		const double& squareSize, const int &horizon, const int &deadZone,
		const cv::Mat& cameraMatrix, const cv::Mat& distortionCoefficients,
		cv::Mat& homography, cv::Point2f rotationCenter, cv::Mat &rtMatrix) {
	cv::Mat result;

	cv::Mat newIn;
	cv::Mat undistorted;
	cv::Mat greyNewIn;
	cv::Mat ground;

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

bool horizontalPoint3Compare(cv::Point3f p1, cv::Point3f p2) {
	return p1.x < p2.x;
}

bool verticalPoint3Compare(cv::Point3f p1, cv::Point3f p2) {
	return p1.y < p2.y;
}

cv::Mat drawTraveledRoute(const std::list<cv::Point3f> &route) {
	cv::Mat result;
	cv::Point3f extremes[4];

	extremes[0] = *std::min_element(route.begin(), route.end(),
			horizontalPoint3Compare);
	extremes[1] = *std::max_element(route.begin(), route.end(),
			horizontalPoint3Compare);

	extremes[2] = *std::min_element(route.begin(), route.end(),
			verticalPoint3Compare);
	extremes[3] = *std::max_element(route.begin(), route.end(),
			verticalPoint3Compare);

	cv::Size mapSize(extremes[1].x - extremes[0].x,
			extremes[3].y - extremes[2].y);

	if (0 == mapSize.width) {
		mapSize.width = 1;
	}
	if (0 == mapSize.height) {
		mapSize.height = 1;
	}

	std::cerr << mapSize.width << " " << mapSize.height << std::endl;

	std::list<cv::Point3f>::const_iterator bg = route.begin();
	std::list<cv::Point3f>::const_iterator end = route.begin();

	result = cv::Mat(mapSize, CV_8UC3, cv::Scalar::all(255));

	for (++end; route.end() != end; ++end, ++bg) {
		cv::line(result,
				cv::Point2f(bg->x - extremes[0].x, bg->y - extremes[2].y),
				cv::Point2f(end->x - extremes[0].x, end->y - extremes[2].y),
				CV_RGB(0,0,255), 1, 8);

	}

	return result;
}

int main() {

	double qualityLevel = 0.01;
	double minDistance = 10;
	int blockSize = 5;
	cv::Size winSizeShi(5, 5);
	cv::Size zeroZone(-1, -1);
	cv::TermCriteria termCritShi(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40,
			0.0001);

	cv::Size winSizeLuc(21, 21);
	unsigned int maxLevel = 4;
	int flagsLuc = 0;
	cv::TermCriteria termCritLuc(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40,
			0.0001);
	double minEigThr = 1e-4;
	double maxErrVal = 500;

	cv::Mat homography;

	//cv::Size boardSize(10, 7);
	cv::Size boardSize(9, 6);
	cv::VideoCapture capture(0);

	ShiThomasFeatureExtractor extractor(qualityLevel, minDistance, blockSize,
			winSizeShi, zeroZone, termCritShi);
	LucasCandaePyramidTracker tracker(winSizeLuc, maxLevel, flagsLuc,
			termCritLuc, minEigThr, maxErrVal);

	unsigned int maxFeatures = 500;
	std::list<FeatureFilter*> filters;

	TangentRotationReader rotationReader(extractor, tracker, filters);

	int horizon;
	int deadZone;
	int featuresNumber;

	std::vector<cv::Mat> rectifyMaps;
	cv::Size imageSize;

	cv::Size winSizeHom(5, 5);
	cv::Size zeroZoneHom(-1, -1);
	cv::TermCriteria termCritHom(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001);
	float squareSize = 24;
	std::list<cv::Point3f> positions;
	cv::Point2f rotationCenter;
	cv::Mat rtMatrix;
	cv::Point3f currentPosition(0, 0, 0);
	positions.push_back(currentPosition);
	try {
		cv::Mat cameraMatrix, distortionCoefficients;
		getCameraParameters("../logitech.yaml", rectifyMaps, cameraMatrix,
				distortionCoefficients, imageSize);
		horizon = imageSize.height / 2;
		deadZone = 0;
		calibrateParameters(capture, rectifyMaps, horizon, deadZone, imageSize);
		std::vector<cv::Point2f> corners = getChessboardCorners(capture,
				rectifyMaps, horizon, deadZone, boardSize, imageSize,
				winSizeHom, zeroZoneHom, termCritHom);
		getHomographyAndRtMatrix(corners, imageSize, boardSize, squareSize,
				horizon, deadZone, cameraMatrix, distortionCoefficients,
				homography, rotationCenter, rtMatrix);

		BirdsEyeTranslationReader transReader(homography, extractor, tracker,
				maxFeatures, filters, rotationCenter);

		VisualOdometer odo(rotationReader, transReader, filters, horizon,
				deadZone, featuresNumber);

		char control = ' ';
		cv::Mat input;
		cv::Mat undistorted;
		cv::Mat grey;
		cv::namedWindow("main", CV_WINDOW_KEEPRATIO);
		cv::namedWindow("map", CV_WINDOW_KEEPRATIO);

		do {
			std::cerr << "p" << std::endl;
			cv::Point3f displacement;
			capture >> input;

			cv::remap(input, undistorted, rectifyMaps[0], rectifyMaps[1],
					cv::INTER_LINEAR);
			cv::cvtColor(undistorted, grey, CV_RGB2GRAY);
			displacement = odo.calculateDisplacement(grey);
			currentPosition = currentPosition + displacement;
			positions.push_front(currentPosition);
			std::cerr << "d\n" << displacement << std::endl;
			cv::Mat map = drawTraveledRoute(positions);
			//printMatrix(map);
			std::cerr << currentPosition << std::endl << map.size()
					<< std::endl;
			cv::imshow("map", map);
			cv::imshow("main", input);
			control = cv::waitKey(1);
			std::cerr << "k" << std::endl;
		} while ('q' != control);

	} catch (cv::Exception &ex) {
		if (USER_TRIGGERED_EXIT == ex.code) {
			return 0;
		} else {
			std::cerr << "EXCEPTION!!!!!!!!!!!" << std::endl << ex.what()
					<< std::endl;
			return 1;
		}
	}
	return 0;
}
