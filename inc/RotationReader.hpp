/*
 * RotationReader.hpp
 *
 *  Created on: Feb 21, 2013
 *      Author: link
 */

#ifndef ROTATIONREADER_HPP_
#define ROTATIONREADER_HPP_

#include <vector>
#include <list>
#include <opencv2/core/core.hpp>

#include "FeatureExtractor.hpp"
#include "FeatureTracker.hpp"
#include "FeatureFilter.hpp"

class RotationReader {
protected:
	std::vector<std::list<cv::Point2f> > _trackedFeatures;
	FeatureTracker *_tracker;
	FeatureExtractor *_extractor;
	cv::Mat _oldFrame;
	std::list<FeatureFilter*> _filters;
public:
	RotationReader();
	virtual RotationReader *constructCopy() const=0;
	virtual float readRotation(const cv::Mat &newFrame)=0;
	virtual ~RotationReader();
};

#endif /* ROTATIONREADER_HPP_ */
