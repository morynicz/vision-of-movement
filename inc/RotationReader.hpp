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
#include "MovementReader.hpp"

class RotationReader : public MovementReader{
protected:
	RotationReader(const FeatureTracker &tracker,
	                const FeatureExtractor &extractor,
	                const std::list<FeatureFilter*> &filters,
	                const unsigned int &maxFeatures,
	                const std::vector<std::list<cv::Point2f> > &trackedFeatures =
	                        std::vector<std::list<cv::Point2f> >());
	RotationReader(const RotationReader &toCopy);
public:
	virtual RotationReader *constructCopy() const=0;
	virtual float readRotation(const cv::Mat &newFrame)=0;
	virtual ~RotationReader();
};

#endif /* ROTATIONREADER_HPP_ */
