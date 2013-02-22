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
	cv::Mat _homographyMatrix;
public:
	ImageEdgeFilter();
	void setHomographyMatrix(const cv::Mat &homographyMatrix) {
		_homographyMatrix = homographyMatrix;
	}
	;
	virtual ~ImageEdgeFilter();
};

#endif /* IMAGEEDGEFILTER_HPP_ */
