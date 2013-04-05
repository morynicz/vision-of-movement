/// @author Michał Orynicz

#include "VisualOdometer.hpp"
#include "BirdsEyeTranslationReader.hpp"
#include "TangentRotationReader.hpp"
#include "SmoothFilter.hpp"
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
	double margin = 5;

	positions.push_back(currentPosition);
	try {
		cv::Mat cameraMatrix, distortionCoefficients;
		getCameraParameters("../logitech.yaml", rectifyMaps, cameraMatrix,
				distortionCoefficients, imageSize);
		capture.set(CV_CAP_PROP_FRAME_WIDTH, imageSize.width);
		capture.set(CV_CAP_PROP_FRAME_HEIGHT, imageSize.height);
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
				maxFeatures, filters, rotationCenter, imageSize, margin);
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
