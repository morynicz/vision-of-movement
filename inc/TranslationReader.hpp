/*
 * TranslationReader.h
 *
 *  Created on: Feb 21, 2013
 *      Author: link
 */

#ifndef TRANSLATIONREADER_H_
#define TRANSLATIONREADER_H_

#include <vector>
#include <list>
#include <opencv2/core/core.hpp>

#include "FeatureExtractor.hpp"
#include "FeatureTracker.hpp"
#include "FeatureFilter.hpp"

class TranslationReader{
	protected:
	std::vector<std::list<cv::Point2f> > _trackedFeatures;
	FeatureTracker *_tracker;
	FeatureExtractor *_extractor;
	cv::Mat _oldFrame;
	std::list<FeatureFilter*> _filters;
public:
	TranslationReader();
	virtual TranslationReader *constructCopy() const =0;
	virtual cv::Point3f readTranslation(const cv::Mat &newFrame,
			const double& rotation) =0;
	virtual ~TranslationReader();
};

#endif /* TRANSLATIONREADER_H_ */
