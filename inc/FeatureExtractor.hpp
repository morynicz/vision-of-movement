/*
 * FeatureExtractor.hpp
 *
 *  Created on: Feb 21, 2013
 *      Author: link
 */

#ifndef FEATUREEXTRACTOR_HPP_
#define FEATUREEXTRACTOR_HPP_

#include <vector>
#include <list>
#include <opencv2/core/core.hpp>

const int NO_FEATURES_FOUND = -10;

class FeatureExtractor {
public:
	FeatureExtractor();
	virtual FeatureExtractor *constructCopy() const =0;
	virtual std::vector<cv::Point2f> extractFeatures(const cv::Mat &input,
			const int & maxCorners) const=0;
	void refillFeatures(const cv::Mat& oldFrame,
			std::vector<std::list<cv::Point2f> >& features,
			const unsigned int& maxFeatures);
	virtual ~FeatureExtractor();
};

#endif /* FEATUREEXTRACTOR_HPP_ */
