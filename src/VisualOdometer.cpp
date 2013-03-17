/*
 * VisualOdometer.cpp
 *
 *  Created on: Feb 21, 2013
 *      Author: Michał Orynicz
 */

#include "VisualOdometer.hpp"
#include "TangentRotationReader.hpp"
#include "ShiThomasFeatureExtractor.hpp"
#include "LucasCandaePyramidTracker.hpp"
#include "BirdsEyeTranslationReader.hpp"

const unsigned int DEFAULT_MAX_FEATURES = 500;

VisualOdometer::VisualOdometer() {
	_filters.clear();
	//TODO Add default smoothness filter instead of empty list.
	_translationReader = new BirdsEyeTranslationReader(
			cv::Mat::eye(3, 3, CV_32F), ShiThomasFeatureExtractor(),
			LucasCandaePyramidTracker(),DEFAULT_MAX_FEATURES,std::list<FeatureFilter*>());
	_rotationReader = new TangentRotationReader(ShiThomasFeatureExtractor(),
			LucasCandaePyramidTracker(),std::list<FeatureFilter*>());

	_deadZoneWidth = 0;
	_horizonHeight = 0.5;
	_featuresNumber = 100;

}

VisualOdometer::~VisualOdometer() {
	if (NULL != _translationReader) {
		delete _translationReader;
	}
	if (NULL != _rotationReader) {
		delete _rotationReader;
	}

	for (std::list<FeatureFilter*>::iterator it = _filters.begin();
			_filters.end() != it; ++it) {
		if (NULL != *it) {
			delete *it;
		}
	}
}

VisualOdometer::VisualOdometer(const RotationReader &rotationReader,
		const TranslationReader &translationReader,
		const std::list<FeatureFilter*> filters, const int &horizonHeight,
		const int &deadZoneWidth, const int &featuresNumber) :
		_featuresNumber(featuresNumber), _horizonHeight(horizonHeight), _deadZoneWidth(
				deadZoneWidth) {
	_translationReader = translationReader.constructCopy();
	_rotationReader = rotationReader.constructCopy();

	for (std::list<FeatureFilter*>::const_iterator it = filters.begin();
			filters.end() != it; ++it) {
		_filters.push_back(*it);
	}
}

cv::Point3f VisualOdometer::calculateDisplacement(const cv::Mat &newFrame) {
	cv::Point3f result(0, 0, 0);
	_rng.next();
	float min =-5;
	float max = 5;

	float dmin = 0;
	float dmax = 20;

	float alphaMin = -CV_PI/3;
	float alhaMax = CV_PI/3;

	float alpha = _rng.uniform(alphaMin,alhaMax);
	float d = _rng.uniform(dmin,dmax);

//	result.x = _rng.uniform(min, max);
//	result.y = _rng.uniform(min, max);
//	result.z = _rng.uniform(min, max);

	result.x = d*sin(alpha);
	result.y = d*cos(alpha);
	result.z = alpha;

	return result;
}
