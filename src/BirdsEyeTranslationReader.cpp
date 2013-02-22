/*
 * BirdsEyeTranslationReader.cpp
 *
 *  Created on: Feb 21, 2013
 *      Author: link
 */

#include "BirdsEyeTranslationReader.hpp"

BirdsEyeTranslationReader::BirdsEyeTranslationReader(
		const cv::Mat &homography,const FeatureTracker &tracker) {
	_homography = homography.clone();
	_tracker = tracker.constructCopy();
}

BirdsEyeTranslationReader::BirdsEyeTranslationReader(
		const BirdsEyeTranslationReader &toCopy) {
	_homography = toCopy._homography;
	_translations = toCopy._translations;
	_trackedFeatures = toCopy._trackedFeatures;
}

BirdsEyeTranslationReader::~BirdsEyeTranslationReader() {
	if(NULL!=_tracker){
		delete _tracker;
	}
}

TranslationReader *BirdsEyeTranslationReader::constructCopy() const {
	return new BirdsEyeTranslationReader(*this);
}

cv::Point2f BirdsEyeTranslationReader::readTranslation(
		const cv::Mat &newFrame) {
	cv::Point2f result(0, 0);
	return result;
}
