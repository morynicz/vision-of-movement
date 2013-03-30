/*
 * ImageEdgeFilter.hpp
 *
 *  Created on: Feb 21, 2013
 *      Author: link
 */

#ifndef IMAGEEDGEFILTER_HPP_
#define IMAGEEDGEFILTER_HPP_

#include <opencv2/core/core.hpp>
#include "FeatureFilter.hpp"

class ImageEdgeFilter: public FeatureFilter {
	std::vector<cv::Point2f> corners;
	double _margin;
public:
	ImageEdgeFilter(const cv::Mat& transformMatrix, const cv::Size &imageSzie,
			const double &margin);
	std::vector<std::list<cv::Point2f> > filterFeatures(
				const std::vector<std::list<cv::Point2f> > &features);
	virtual ~ImageEdgeFilter();
};

#endif /* IMAGEEDGEFILTER_HPP_ */
