/*
 * ImageEdgeFilter.cpp
 *
 *  Created on: Feb 21, 2013
 *      Author: link
 */

#include "ImageEdgeFilter.hpp"

ImageEdgeFilter::ImageEdgeFilter(const cv::Mat &transformMatrix,
		const cv::Size &imageSize, const double &margin) :
		_margin(margin) {
	std::vector<cv::Point2f> tempCorners;
	tempCorners.push_back(cv::Point2f(0, 0));
	tempCorners.push_back(cv::Point2f(imageSize.width, 0));
	tempCorners.push_back(cv::Point2f(0, imageSize.height));
	tempCorners.push_back(cv::Point2f(imageSize.width, imageSize.height));
	cv::transform(tempCorners, corners, transformMatrix);

}

ImageEdgeFilter::~ImageEdgeFilter() {
	// TODO Auto-generated destructor stub
}

std::vector<std::list<cv::Point2f> > ImageEdgeFilter::filterFeatures(
		const std::vector<std::list<cv::Point2f> >& features) {
	std::vector<std::list<cv::Point2f> > result(features);
	for (int i = 0; i < result.size(); ++i) {

	}
	return result;
}
