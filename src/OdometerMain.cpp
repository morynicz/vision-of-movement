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

//cv::Mat getHomography(cv::VideoCapture &capture,)

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

	cv::VideoCapture capture(1);

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
	try {
		getCameraParameters("../logitech.yaml", rectifyMaps, imageSize);
		horizon = imageSize.height / 2;
		deadZone = 0;
		std::cerr << "raz";
		calibrateParameters(capture, rectifyMaps, horizon, deadZone, imageSize);
		std::cerr << "dwa";
		VisualOdometer odo(rotationReader, transReader, filters, horizon,
				deadZone, featuresNumber);
	} catch (cv::Exception &ex) {
		if (USER_TRIGGERED_EXIT == ex.code) {
			return 0;
		} else {
			std::cerr << ex.what() << std::endl;
			return 1;
		}
	}
	return 0;
}
