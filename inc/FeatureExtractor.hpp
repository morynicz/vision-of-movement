/*
 * FeatureExtractor.hpp
 *
 *  Created on: Feb 21, 2013
 *      Author: link
 */

#ifndef FEATUREEXTRACTOR_HPP_
#define FEATUREEXTRACTOR_HPP_

#include <vector>
#include <opencv2/core/core.hpp>

class FeatureExtractor {
public:
	FeatureExtractor();
	virtual FeatureExtractor *constructCopy() const =0;
	virtual std::vector<cv::Point2f> extractFeatures(const cv::Mat &input,
			const int & maxCorners) const=0;
	virtual ~FeatureExtractor();
};

#endif /* FEATUREEXTRACTOR_HPP_ */
