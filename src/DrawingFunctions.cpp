/*
 * DrawingFunctions.cpp
 *
 *  Created on: Apr 5, 2013
 *      Author: link
 */
#include "DrawingFunctions.hpp"
#include "ErrorCodes.hpp"
#include "ConvenienceFunctions.hpp"


void drawDeadZoneHorizon(cv::Mat image, const int &horizon,
		const int& deadZone) {
	cv::line(image, cv::Point2f(0, horizon + deadZone),
			cv::Point2f(image.cols, horizon + deadZone), CV_RGB(0,255,0), 2,
			CV_AA);
	cv::line(image, cv::Point2f(0, horizon - deadZone),
			cv::Point2f(image.cols, horizon - deadZone), CV_RGB(0,255,0), 2,
			CV_AA);
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
