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


VisualOdometer::VisualOdometer() {
	_featureExtractor = new ShiThomasFeatureExtractor();
	_filters.clear();
	_translationReader = new BirdsEyeTranslationReader(cv::Mat::eye(3,3,CV_32F),LucasCandaePyramidTracker());
	_rotationReader = new TangentRotationReader(LucasCandaePyramidTracker());

	_deadZoneWidth = 0;
	_horizonHeight = 0.5;
	_featuresNumber = 100;

}

VisualOdometer::~VisualOdometer() {
	if (NULL != _featureExtractor) {
		delete _featureExtractor;
	}
	if (NULL != _translationReader) {
		delete _translationReader;
	}
	if (NULL != _rotationReader) {
		delete _rotationReader;
	}

	for (std::list<FeatureFilter*>::iterator it=_filters.begin();_filters.end()!=it;++it) {
		if (NULL != *it) {
			delete *it;
		}
	}
	// TODO Auto-generated destructor stub
}

