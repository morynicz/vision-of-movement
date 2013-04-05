/*
 * DrawingFunctions.hpp
 *
 *  Created on: Apr 5, 2013
 *      Author: link
 */

#ifndef DRAWINGFUNCTIONS_HPP_
#define DRAWINGFUNCTIONS_HPP_

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <list>
#include <vector>


void drawDeadZoneHorizon(cv::Mat image, const int &horizon,
		const int& deadZone);

cv::Mat drawTraveledRoute(const std::list<cv::Point3f> &route);
#endif /* DRAWINGFUNCTIONS_HPP_ */
