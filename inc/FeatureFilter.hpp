/*
 * FeatureFilter.hpp
 *
 *  Created on: Feb 21, 2013
 *      Author: link
 */

#ifndef FEATUREFILTER_HPP_
#define FEATUREFILTER_HPP_

#include <vector>
#include <list>
#include <opencv2/core/core.hpp>

class FeatureFilter {
public:
	FeatureFilter();
	virtual FeatureFilter *constructCopy() const = 0;
	virtual std::vector<std::list<cv::Point2f> > filterFeatures(
			const std::vector<std::list<cv::Point2f> > &features)=0;
	virtual ~FeatureFilter();
};

#endif /* FEATUREFILTER_HPP_ */
