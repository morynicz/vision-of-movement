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
	std::vector<double> _a;
	std::vector<double> _c;
	double _margin;
public:
	ImageEdgeFilter(const cv::Mat& transformMatrix, const cv::Size &imageSzie,
			const double &margin);
	ImageEdgeFilter(const ImageEdgeFilter &toCopy);
	std::vector<std::list<cv::Point2f> > filterFeatures(
				const std::vector<std::list<cv::Point2f> > &features);
	FeatureFilter *constructCopy() const;
	virtual ~ImageEdgeFilter();
};

#endif /* IMAGEEDGEFILTER_HPP_ */
