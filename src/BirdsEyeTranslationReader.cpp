/*
 * BirdsEyeTranslationReader.cpp
 *
 *  Created on: Feb 21, 2013
 *      Author: link
 */

#include "BirdsEyeTranslationReader.hpp"
#include <opencv2/imgproc/imgproc.hpp>

bool horizontalPoint2Compare(cv::Point2f p1, cv::Point2f p2) {
	return p1.x < p2.x;
}

bool verticalPoint2Compare(cv::Point2f p1, cv::Point2f p2) {
	return p1.y < p2.y;
}

BirdsEyeTranslationReader::BirdsEyeTranslationReader(const cv::Mat &homography,
		const FeatureExtractor &extractor, const FeatureTracker &tracker,
		const unsigned int& maxFeatures,
		const std::list<FeatureFilter*>& filters) :
		_maxFeatures(maxFeatures) {
	_homography = homography.clone();
	_tracker = tracker.constructCopy();
	_extractor = extractor.constructCopy();
	for (std::list<FeatureFilter*>::const_iterator it = filters.begin();
			it != filters.end(); ++it) {
		_filters.push_back((*it)->constructCopy());
	}
}

BirdsEyeTranslationReader::BirdsEyeTranslationReader(
		const BirdsEyeTranslationReader &toCopy) :
		_homography(toCopy._homography), _translations(toCopy._translations), _maxFeatures(
				toCopy._maxFeatures) {
	_trackedFeatures = toCopy._trackedFeatures;
	_tracker = toCopy._tracker->constructCopy();
	_extractor = toCopy._extractor->constructCopy();
}

BirdsEyeTranslationReader::~BirdsEyeTranslationReader() {
	if (NULL != _tracker) {
		delete _tracker;
	}
	if (NULL != _extractor) {
		delete _extractor;
	}
	for (std::list<FeatureFilter*>::const_iterator it = _filters.begin();
			it != _filters.end(); ++it) {
		delete *it;
	}
}

TranslationReader *BirdsEyeTranslationReader::constructCopy() const {
	return new BirdsEyeTranslationReader(*this);
}

cv::Point2f BirdsEyeTranslationReader::readTranslation(const cv::Mat &newFrame,
		const double& rotation) {
	cv::Mat temp;
	cv::Point2f result(0, 0);
	int vectorHalf = 0;
	_extractor->refillFeatures(_oldFrame, _trackedFeatures, _maxFeatures);
	cv::warpPerspective(newFrame, temp, _homography, newFrame.size(),
			cv::INTER_LINEAR | cv::WARP_INVERSE_MAP);
	_tracker->trackFeatures(_oldFrame, temp, _trackedFeatures);
	std::vector<cv::Point2f> translations = computeTranslationVectors();
	vectorHalf = translations.size() / 2;
	std::nth_element(translations.begin(), translations.begin() + vectorHalf,
			translations.end(), horizontalPoint2Compare);
	result.x = translations[vectorHalf].x;
	std::nth_element(translations.begin(), translations.begin() + vectorHalf,
			translations.end(), verticalPoint2Compare);
	result.y = translations[vectorHalf].y;
	return result;
}

std::vector<cv::Point2f> BirdsEyeTranslationReader::computeTranslationVectors() {
	std::vector<cv::Point2f> result(_trackedFeatures.size());

	for (unsigned int i = 0; i < result.size(); ++i) { //Highly susceptible to bugs
		result[i] = _trackedFeatures[i].front()
				- *(++_trackedFeatures[i].begin());
	}
	return result;
}
