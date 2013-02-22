/*
 * FeatureTracker.hpp
 *
 *  Created on: Feb 21, 2013
 *      Author: link
 */

#ifndef FEATURETRACKER_HPP_
#define FEATURETRACKER_HPP_

#include <vector>
#include <list>
#include <opencv2/core/core.hpp>

class FeatureTracker {
public:
	FeatureTracker();
	virtual FeatureTracker *constructCopy() const = 0;
	virtual void trackFeatures(const cv::Mat &oldInput, const cv::Mat &newInput,
			std::vector<std::list<cv::Point2f> > &trackedFeatures)=0;
	virtual ~FeatureTracker();
};

#endif /* FEATURETRACKER_HPP_ */
