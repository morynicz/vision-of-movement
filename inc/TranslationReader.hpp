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

#include "FeatureTracker.hpp"

class TranslationReader {
protected:
	std::vector<std::list<cv::Point2f> > _trackedFeatures;
	FeatureTracker *_tracker;
	cv::Mat _oldFrame;
public:
	TranslationReader();
	virtual TranslationReader *constructCopy() const =0;
	virtual cv::Point2f readTranslation(const cv::Mat &newFrame) =0;
	virtual ~TranslationReader();
};

#endif /* TRANSLATIONREADER_H_ */
