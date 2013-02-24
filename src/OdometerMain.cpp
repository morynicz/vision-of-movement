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
		std::vector<cv::Mat> &undistortionMaps, cv::Size &imageSize) {
	cv::FileStorage fs(fileName, cv::FileStorage::READ);
	cv::Mat intrinistics;
	cv::Mat cameraMatrix;
	cv::Mat distortionCoeffs;

	if (!fs.isOpened()) {
		cv::Exception ex(READ_CAMERA_PARAMETERS_FAILED,
				"could not read file: " + fileName, __func__, __FILE__,
				__LINE__);
		throw ex;
	}

	undistortionMaps.resize(2);

	fs["camera_matrix"] >> intrinistics;
	fs["distortion_coefficients"] >> distortionCoeffs;
	fs["image_width"] >> imageSize.width;
	fs["image_height"] >> imageSize.height;

	cameraMatrix = cv::getOptimalNewCameraMatrix(intrinistics, distortionCoeffs,
			cv::Size(640, 480), 1);
	cv::initUndistortRectifyMap(intrinistics, distortionCoeffs,
			cv::Mat::eye(3, 3, CV_32F), cameraMatrix, cv::Size(640, 480),
			CV_32FC1, undistortionMaps[0], undistortionMaps[1]);

}

cv::Mat getHomography(cv::VideoCapture &capture,
		const std::vector<cv::Mat> &rectificationMaps, const int &horizon,
		const int &deadZone, cv::Size &boardSize, cv::Size &imageSize,
		cv::Size &winSize, cv::Size &zeroZone, cv::TermCriteria termCriteria,
		const float &squareSize) {
	cv::Mat result;

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

	std::vector<cv::Point2f> objPts(4), imgPts(4);
	objPts[0].x = 0;
	objPts[0].y = 0;
	objPts[1].x = (boardSize.width - 1) * squareSize;
	objPts[1].y = 0;
	objPts[2].x = 0;
	objPts[2].y = (boardSize.height - 1) * squareSize;
	objPts[3].x = (boardSize.width - 1) * squareSize;
	objPts[3].y = (boardSize.height - 1) * squareSize;
	imgPts[0] = corners[0];
	imgPts[1] = corners[boardSize.width - 1];
	imgPts[2] = corners[(boardSize.height - 1) * boardSize.width];
	imgPts[3] = corners[(boardSize.height) * boardSize.width - 1];

	result = cv::getPerspectiveTransform(objPts, imgPts);
//	perspTrans.at<double>(Point(2, 2)) = height;

	return result;
}

bool horizontalPoint3Compare(cv::Point3f p1, cv::Point3f p2) {
	return p1.x < p2.x;
}

bool verticalPoint3Compare(cv::Point3f p1, cv::Point2f p2) {
	return p1.y < p2.y;
}

cv::Mat drawTraveledRoute(const std::list<cv::Point3f> &route) {
	cv::Mat result;
	cv::Point3f extremes[4];

	extremes[0] = std::min_element(route.begin(), route.end(),
			horizontalPoint3Compare);
	extremes[1] = std::max_element(route.begin(), route.end(),
			horizontalPoint3Compare);
	extremes[2] = std::min_element(route.begin(), route.end(),
			verticalPoint3Compare);
	extremes[3] = std::max_element(route.begin(), route.end(),
			verticalPoint3Compare);

	cv::Size mapSize(extremes[1].x - extremes[1].x,
			extremes[3].y - extremes[2].y);

	std::list<cv::Point3f>::const_iterator bg = route.begin();
	std::list<cv::Point3f>::const_iterator end = route.begin();

	result = cv::Mat(mapSize, CV_8UC3);

	for (++end; route.end() != end; ++end) {
		cv::line(result, cv::Point2f(bg->x, bg->y), cv::Point2f(end->x, end->y),
				CV_RGB(255,128,0), 1, CV_AA);
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

	cv::Size boardSize(9, 6);

	cv::VideoCapture capture(0);

	ShiThomasFeatureExtractor extractor(qualityLevel, minDistance, blockSize,
			winSizeShi, zeroZone, termCritShi);
	LucasCandaePyramidTracker tracker(winSizeLuc, maxLevel, flagsLuc,
			termCritLuc, minEigThr, maxErrVal);

	BirdsEyeTranslationReader transReader(homography, extractor, tracker);
	TangentRotationReader rotationReader(extractor, tracker);

	std::list<FeatureFilter*> filters;
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
	cv::Point3f currentPosition(0, 0, 0);
	try {
		getCameraParameters("../logitech.yaml", rectifyMaps, imageSize);
		horizon = imageSize.height / 2;
		deadZone = 0;
		std::cerr << "raz";
		calibrateParameters(capture, rectifyMaps, horizon, deadZone, imageSize);
		std::cerr << "dwa";
		homography = getHomography(capture, rectifyMaps, horizon, deadZone,
				boardSize, imageSize, winSizeHom, zeroZoneHom, termCritHom,
				squareSize);
		std::cerr << "trzy";
		VisualOdometer odo(rotationReader, transReader, filters, horizon,
				deadZone, featuresNumber);

		char control = ' ';
		cv::Mat input;
		cv::namedWindow("main",CV_WINDOW_KEEPRATIO);
		do {
			cv::Point3f displacement;
			capture >> input;
			displacement = odo.calculateDisplacement(input);
			currentPosition = currentPosition + displacement;
			positions.push_front(currentPosition);

			cv::Mat map = drawTraveledRoute(positions);
			cv::imshow("main",map);

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
