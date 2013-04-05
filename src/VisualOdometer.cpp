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

	double alpha = _rotationReader->readRotation(newFrame);
	result = _translationReader->readTranslation(newFrame, alpha);

	return result;
}
